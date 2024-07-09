/****************************************************************************

   Thorlabs CCS Series Spectrometer - VISA instrument driver

   FOR DETAILED DESCRIPTION OF THE DRIVER FUNCTIONS SEE THE ONLINE HELP FILE
   AND THE PROGRAMMERS REFERENCE MANUAL.

   Copyright:  Copyright(c) 2008-2012 Thorlabs (www.thorlabs.com)
   Author:     Olaf Wohlmann (owohlmann@thorlabs.com)

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   Source file

   Date:          Mar-13-2013
   Built with:    NI LabWindows/CVI 2012SP1
   Version:       2.0.0

   Changelog:     see 'readme.rtf'

   TODO:          change FP settings for tlccs_setAmplitudeData(); and
                  for tlccs_setWavelengthData(); for some array parameters
                  from Output to Input for better automated LabView conversion

****************************************************************************/


/*#include <utility.h>*/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>

#ifdef _CVI_
   typedef unsigned char  uint8_t;
   typedef unsigned short uint16_t;
   typedef unsigned int   uint32_t;
#else
   #include <stdint.h> 
#endif

#define NIVISA_USB   // this is necessary to access NI-VISA-USB
#include "visa.h"

#include "TLCCS.h"



/*===========================================================================

   VERSION INFORMATION

===========================================================================*/

#define TLCCS_VER_MAJOR      2
#define TLCCS_VER_MINOR      0
#define TLCCS_VER_SUBMINOR   0

#define TLCCS_MAKE_REVISION(major, minor, subminor)  (((major & 0x00000FFF) << 20) | ((minor & 0x00000FFF) << 8) | (subminor & 0x000000FF))

#define TLCCS_VERSION        TLCCS_MAKE_REVISION(TLCCS_VER_MAJOR, TLCCS_VER_MINOR, TLCCS_VER_SUBMINOR)


/*===========================================================================
 
   Macros
 
===========================================================================*/
// Resource locking
#ifdef _CVI_DEBUG_
   // We are in a debugging session - do not lock resource
   #define TLCCS_LOCK_STATE            VI_NULL
#else
   #define TLCCS_LOCK_STATE            VI_EXCLUSIVE_LOCK
   //#define TLCCS_LOCK_STATE          VI_NULL
#endif

// Buffers
#define TLCCS_SERIAL_NO_LENGTH         24
#define TLCCS_NUM_POLY_POINTS          4
#define TLCCS_NUM_INTEG_CTRL_BYTES     6
#define TLCCS_NUM_VERSION_BYTES        3
#define TLCCS_NUM_FLAG_WORDS           1
#define TLCCS_NUM_CHECKSUMS            2

// Version
#define TLCCS_FIRMWARE_VERSION         0
#define TLCCS_HARDWARE_VERSION         1

// Calibration
#define TLCCS_ACOR_FACTORY             0
#define TLCCS_ACOR_USER                1

// Error query mode
#define TLCCS_DEFAULT_ERR_QUERY_MODE   VI_ON

// Range checking
#define INVAL_RANGE(val, min, max)     ( ((val) < (min)) || ((val) > (max)) )

// Logging
#define DEBUG_BUF_SIZE                 512

// Macros for Cypress USB chip
#define ENDPOINT_0_TRANSFERSIZE        64          // this is the max. size of bytes that can be transferred at once for Endpoint 0

// Analysis 
#define MATRIX_ROWS  4
#define MATRIX_COLS  4
   
/*===========================================================================
   EEPROM mapping
===========================================================================*/
// a 256kBit = 32kB EEPROM is used here   
#define EE_LENGTH_SERIAL_NO               ( sizeof(ViChar)                       * TLCCS_SERIAL_NO_LENGTH    )
#define EE_LENGTH_SW_VERSION              (4                                                                 )
#define EE_LENGTH_USER_LABEL              ( sizeof(ViChar)                       * TLCCS_MAX_USER_NAME_SIZE  )  
#define EE_LENGTH_FACT_CAL_COEF_FLAG      (2                                                                 )
#define EE_LENGTH_FACT_CAL_COEF_DATA      ( sizeof(ViReal64)                     * TLCCS_NUM_POLY_POINTS     )
#define EE_LENGTH_USER_CAL_COEF_FLAG      (2                                                                 )
#define EE_LENGTH_USER_CAL_COEF_DATA      ( sizeof(ViReal64)                     * TLCCS_NUM_POLY_POINTS     )
#define EE_LENGTH_USER_CAL_POINTS_CNT     (2                                                                 )
#define EE_LENGTH_USER_CAL_POINTS_DATA    ((sizeof(ViInt32) + sizeof(ViReal64))  * TLCCS_MAX_NUM_USR_ADJ     )
#define EE_LENGTH_OFFSET_MAX              (2                                                                 ) 
#define EE_LENGTH_ACOR                    ( sizeof(ViReal32)                     * TLCCS_NUM_PIXELS          )
#define EE_LENGTH_FLAGS                   ( sizeof(ViUInt32)                     * TLCCS_NUM_FLAG_WORDS      )
#define EE_LENGTH_CHECKSUMS               ( sizeof(ViUInt16)                     * TLCCS_NUM_CHECKSUMS       )

// eeprom sizes
#define EE_SIZE_CHECKSUM                  (2)
#define EE_SIZE_BOOT_CODE                 (1)
#define EE_SIZE_SERIAL_NO                 (EE_LENGTH_SERIAL_NO)         
#define EE_SIZE_SW_VERSION                (EE_LENGTH_SW_VERSION            + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_LABEL                (EE_LENGTH_USER_LABEL            + EE_SIZE_CHECKSUM)
#define EE_SIZE_FACT_CAL_COEF_FLAG        (EE_LENGTH_FACT_CAL_COEF_FLAG    + EE_SIZE_CHECKSUM)
#define EE_SIZE_FACT_CAL_COEF_DATA        (EE_LENGTH_FACT_CAL_COEF_DATA    + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_CAL_COEF_FLAG        (EE_LENGTH_USER_CAL_COEF_FLAG    + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_CAL_COEF_DATA        (EE_LENGTH_USER_CAL_COEF_DATA    + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_CAL_POINTS_CNT       (EE_LENGTH_USER_CAL_POINTS_CNT   + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_CAL_POINTS_DATA      (EE_LENGTH_USER_CAL_POINTS_DATA  + EE_SIZE_CHECKSUM)
#define EE_SIZE_OFFSET_MAX                (EE_LENGTH_OFFSET_MAX            + EE_SIZE_CHECKSUM)
#define EE_SIZE_ACOR                      (EE_LENGTH_ACOR                  + EE_SIZE_CHECKSUM)
#define EE_SIZE_FLAGS                     (EE_LENGTH_FLAGS                 + EE_SIZE_CHECKSUM)
#define EE_SIZE_CHECKSUMS                 (EE_LENGTH_CHECKSUMS             + EE_SIZE_CHECKSUM)

// eeprom adresses
#define EE_BOOT_CODE                      0
#define EE_VENDOR_ID                      1
#define EE_PRODUCT_ID                     3
#define EE_DEVICE_ID                      5
#define EE_SERIAL_NO                      8
#define EE_SW_VERSION                     (EE_SERIAL_NO              + EE_SIZE_SERIAL_NO              )  // software version
#define EE_USER_LABEL                     (EE_SW_VERSION             + EE_SIZE_SW_VERSION             )  // user label
#define EE_FACT_CAL_COEF_FLAG             (EE_USER_LABEL             + EE_SIZE_USER_LABEL             )  // factory calibration flags
#define EE_FACT_CAL_COEF_DATA             (EE_FACT_CAL_COEF_FLAG     + EE_SIZE_FACT_CAL_COEF_FLAG     )  // factory calibration coefficients
#define EE_USER_CAL_COEF_FLAG             (EE_FACT_CAL_COEF_DATA     + EE_SIZE_FACT_CAL_COEF_DATA     )  // user calibration flags
#define EE_USER_CAL_COEF_DATA             (EE_USER_CAL_COEF_FLAG     + EE_SIZE_USER_CAL_COEF_FLAG     )  // user calibration coefficients
#define EE_USER_CAL_POINTS_CNT            (EE_USER_CAL_COEF_DATA     + EE_SIZE_USER_CAL_COEF_DATA     )  // user calibration points count
#define EE_USER_CAL_POINTS_DATA           (EE_USER_CAL_POINTS_CNT    + EE_SIZE_USER_CAL_POINTS_CNT    )  // user calibration points
#define EE_EVEN_OFFSET_MAX                (EE_USER_CAL_POINTS_DATA   + EE_SIZE_USER_CAL_POINTS_DATA   )  // even offset max
#define EE_ODD_OFFSET_MAX                 (EE_EVEN_OFFSET_MAX        + EE_SIZE_OFFSET_MAX             )  // odd offset max
#define EE_ACOR_FACTORY                   (EE_ODD_OFFSET_MAX         + EE_SIZE_OFFSET_MAX             )  // amplitude correction, factory setting
#define EE_ACOR_USER                      (EE_ACOR_FACTORY           + EE_SIZE_ACOR                   )  // amplitude correction, factory setting
#define EE_FLAGS                          (EE_ACOR_USER              + EE_SIZE_ACOR                   )  // flags for e.g. user cal/factory cal
#define EE_CHECKSUMS                      (EE_FLAGS                  + EE_SIZE_FLAGS                  )  // checksums for amplitude correction arrays
#define EE_FREE                           (EE_CHECKSUMS              + EE_SIZE_CHECKSUMS              )  // free memory 

// Macros for error sources
#define TLCCS_ERR_SRC_EEPROM              1
#define TLCCS_ERR_SRC_FPGA                2
#define TLCCS_ERR_SRC_FIRMWARE            3

// Macros for CCS_SERIES commands
// reflect commands found in TLCCS_cmds.h of 8051 firmware project 08010
//    WRITE commands (OUT)
//#define TLCCS_WCMD_LOAD_FPGA            0x10
//#define TLCCS_WCMD_SET_VID_PID          0x11
//#define TLCCS_WCMD_SET_CONFIG           0x12
//#define TLCCS_WCMD_WRITE_I2C            0x20
#define TLCCS_WCMD_WRITE_EEPROM           0x21
#define TLCCS_WCMD_INTEGRATION_TIME       0x23
#define TLCCS_WCMD_MODUS                  0x24
#define TLCCS_WCMD_RESET                  0x26
//#define TLCCS_WCMD_ALWAYS_STALL         0x2F

//    READ commands (IN)
//#define TLCCS_RCMD_READ_I2C             0x20
#define TLCCS_RCMD_READ_EEPROM            0x21
#define TLCCS_RCMD_INTEGRATION_TIME       0x23
//#define TLCCS_RCMD_MODUS                0x24
#define TLCCS_RCMD_PRODUCT_INFO           0x25
//#define TLCCS_RCMD_ALWAYS_STALL         0x2F
#define TLCCS_RCMD_GET_STATUS             0x30
#define TLCCS_RCMD_GET_ERROR              0xFF

// Operation Modes
#define MODUS_INTERN_SINGLE_SHOT          0
#define MODUS_INTERN_CONTINUOUS           1
#define MODUS_EXTERN_SINGLE_SHOT          2
#define MODUS_EXTERN_CONTINUOUS           3

#define TLCCS_CALIB_VALID_FLAG            0x5A     // this is the value for check bytes
#define TLCCS_USERCAL_VALID_FLAG          0x5A     // user wavelenght adjustment data is valid

// strings
#define DEFAULT_USER_TEXT                 "My CCS Spectrometer"
   
/*===========================================================================
 Structures
===========================================================================*/
// static error list
typedef struct
{
   ViStatus err;
   ViString descr;
} tlccs_errDescrStat_t;


typedef struct
{
   ViUInt16       user_cal_node_cnt;                           // number of user-defined supporting points
   ViUInt32       user_cal_node_pixel[TLCCS_MAX_NUM_USR_ADJ];  //   pixel array of supporting points
   ViReal64       user_cal_node_wl[TLCCS_MAX_NUM_USR_ADJ];     // wavelength array of supporting points
   
} tlccs_usr_cal_pts_t; 


typedef struct
{
   ViReal64       poly[TLCCS_NUM_POLY_POINTS];  // polynomial coefficients for pixel - wavelength computation
   ViReal64       min;                          // lower wavelength limit
   ViReal64       max;
   ViReal64       wl[TLCCS_NUM_PIXELS];         // array of wavelengths according to pixel number
   ViUInt16       valid;                        // valid flag
} tlccs_wl_cal_t;

typedef struct
{
   ViReal32       acor[TLCCS_NUM_PIXELS];       // array of wavelengths according to pixel number
   ViUInt16       chksum;                       // CRC16 checksum for testing if data set is from user or from Thorlabs
} tlccs_acor_t;


typedef struct
{
   ViUInt8        major;
   ViUInt8        minor;                        
   ViUInt8        subminor;
} tlccs_version_t; 


// driver private data
typedef struct
{
   // common data
   ViSession                  instr;      // instrument handle
   ViBoolean                  errQuery;   // TRUE - query instruments error queue on every access
   ViUInt32                   timeout;    // given communication timeout in ms
   ViUInt16                   pid;        // USB PID value. used to determine the instrument type
   ViUInt16                   vid;        // USB VID value, used to determine the manufacturer
   
   // device information
   ViChar                     name[TLCCS_BUFFER_SIZE];
   ViChar                     serNr[TLCCS_BUFFER_SIZE];
   ViChar                     manu[TLCCS_BUFFER_SIZE];
   ViChar                     firm[TLCCS_BUFFER_SIZE];
   ViChar                     instrDrv[TLCCS_BUFFER_SIZE];
   ViAttrState                userData;   // user data - available via tlccs_setAttribute()/tlccs_getAttribute()
   
   // device settings
   ViReal64                   intTime;
   ViUInt16                   evenOffsetMax;
   ViUInt16                   oddOffsetMax;
   
   // device calibration
   tlccs_wl_cal_t             factory_cal;
   tlccs_wl_cal_t             user_cal;
   tlccs_usr_cal_pts_t        user_points;
   
   tlccs_acor_t               factory_acor_cal;
   tlccs_acor_t               user_acor_cal;
   ViUInt32                   cal_mode;
   
   // version
   tlccs_version_t            firmware_version;
   tlccs_version_t            hardware_version;


} tlccs_data_t;


/*===========================================================================
 Constants
===========================================================================*/
/*---------------------------------------------------------------------------
 Static error descriptions
---------------------------------------------------------------------------*/
static const tlccs_errDescrStat_t TLCCS_errDescrStat[] =
{
   {VI_ERROR_PARAMETER1,               "Parameter 1 out of range"                         },
   {VI_ERROR_PARAMETER2,               "Parameter 2 out of range"                         },
   {VI_ERROR_PARAMETER3,               "Parameter 3 out of range"                         },
   {VI_ERROR_PARAMETER4,               "Parameter 4 out of range"                         },
   {VI_ERROR_PARAMETER5,               "Parameter 5 out of range"                         },
   {VI_ERROR_PARAMETER6,               "Parameter 6 out of range"                         },
   {VI_ERROR_PARAMETER7,               "Parameter 7 out of range"                         },
   {VI_ERROR_PARAMETER8,               "Parameter 8 out of range"                         },
   {VI_ERROR_INV_RESPONSE,             "Errors occured interpreting instrument's response"},
   
   {VI_ERROR_NSUP_COMMAND,             "Command not supported by instrument"              },
   {VI_ERROR_TLCCS_UNKNOWN,            "Unknown CCS_SERIES error, please report this"     },
   {VI_ERROR_SCAN_DATA_INVALID,        "Scan data is invalid. Sensor may be overexposed"  },
   {VI_ERROR_XSVF_SIZE,                "XSVF stream size must be greater 0"               },
   {VI_ERROR_XSVF_MEMORY,              "Memory allocation for XSVF stream failed"         },
   {VI_ERROR_XSVF_FILE,                "Access to XSVF file failed"                       },

   {VI_ERROR_FIRMWARE_SIZE,            "Firmware size must be greater than 0"             },
   {VI_ERROR_FIRMWARE_MEMORY,          "Memory allocation for firmware data failed"       },
   {VI_ERROR_FIRMWARE_FILE,            "Access to firmware file failed"                   },
   {VI_ERROR_FIRMWARE_CHKSUM,          "Checksum error in firmware HEX-File"              },
   {VI_ERROR_FIRMWARE_BUFOFL,          "Given buffer is to small for firmware HEX-File"   },

   {VI_ERROR_CYEEPROM_SIZE,            "EEPROM size mismatch"                             },
   {VI_ERROR_CYEEPROM_MEMORY,          "Memory allocation for EEPROM data failed"         },
   {VI_ERROR_CYEEPROM_FILE,            "Access to EEPROM file failed"                     },
   {VI_ERROR_CYEEPROM_CHKSUM,          "Checksum error in EEPROM"                         },
   {VI_ERROR_CYEEPROM_BUFOVL,          "Given buffer is to small for EEPROM HEX-File"     },


   {VI_ERROR_TLCCS_ENDP0_SIZE,         "Attempt to send or receive more than MAX_EP0_TRANSACTION_SIZE (64) bytes at once over endpoint 0"},
   {VI_ERROR_TLCCS_EEPROM_ADR_TO_BIG,  "Given EEPROM address is to big"                   },

   // CCS_SERIES XSVF errors copied from 'xsvf.c'
   {VI_ERROR_TLCCS_XSVF_UNKNOWN,       "XSVF Error 1: unknown XSVF error"                 },
   {VI_ERROR_TLCCS_XSVF_TDOMISMATCH,   "XSVF Error 2: TDO mismatch"                       },
   {VI_ERROR_TLCCS_XSVF_MAXRETRIES,    "XSVF Error 3: TDO mismatch after max. retries"    },
   {VI_ERROR_TLCCS_XSVF_ILLEGALCMD,    "XSVF Error 4: illegal XSVF command"               },
   {VI_ERROR_TLCCS_XSVF_ILLEGALSTATE,  "XSVF Error 5: illegal TAP state"                  },
   {VI_ERROR_TLCCS_XSVF_DATAOVERFLOW,  "XSVF Error 6: XSVF record length too big"         },
   {VI_ERROR_TLCCS_I2C_NACK,           "CCS_SERIES I2C bus did not acknowledge"           },
   {VI_ERROR_TLCCS_I2C_ERR,            "CCS_SERIES I2C bus error"                         },

   {VI_ERROR_TLCCS_READ_INCOMPLETE,    "Data readout from spectrometer was incomplete"    },
   {VI_ERROR_TLCCS_NO_USER_DATA,       "No user wavelength adjustment data available"     },
   {VI_ERROR_TLCCS_INV_USER_DATA,      "Invalid user wavelength adjustment data"          },
   {VI_ERROR_TLCCS_INV_ADJ_DATA,       "Adjustment data invalid/corrupt"                  },
   
   {0 , VI_NULL}  // termination

};

/*===========================================================================
 Prototypes
===========================================================================*/
// Closing
static ViStatus tlccs_initClose (ViSession vi, ViStatus stat);

// I/O Communication
__attribute__((visibility("default"))) ViStatus tlccs_USB_out(ViSession vi, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer);
__attribute__((visibility("default"))) ViStatus tlccs_USB_in(ViSession vi, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer, ViPUInt16 Read_Bytes);

static ViStatus tlccs_USB_read(ViSession vi, unsigned char *ReceiveData, ViUInt32 Count, ViUInt32 *ReturnCount);
static ViStatus tlccs_USB_write(ViSession vi, ViBuf Buffer, ViUInt32 Count, ViUInt32 *ReturnCount);

static ViStatus tlccs_query(ViSession vi, ViBuf cmdBuf, ViUInt32 cmdLen, ViBuf rspBuf, ViUInt32 rspLen);
static ViStatus tlccs_write(ViSession vi, ViBuf cmdBuf, ViUInt32 cmdLen);

//  Calculate CRC-16 of given data.
#ifndef DO_NOT_COMPILE_DRIVER_HERE
   static uint16_t crc16_block(const void *dat, size_t len);
   //  Optimized CRC-16 calculation. Polynomial: x^16 + x^15 + x^2 + 1 (0xa001) .
   static uint16_t crc16_update(uint16_t crc, uint8_t a);
#else
   uint16_t crc16_block(const void *dat, size_t len);
   //  Optimized CRC-16 calculation. Polynomial: x^16 + x^15 + x^2 + 1 (0xa001) .
   uint16_t crc16_update(uint16_t crc, uint8_t a);
#endif

// analysis
static int LeastSquareInterpolation (int * PixelArray, double * WaveLengthArray, int iLength, double Coefficients[]);  
static double SpecSummation(double *pcorrel,int pwr,int values);
static double Spec2Summation(double *pcorrel,int pwr,int values);
static double Summation(double *pdata,int values);
static void MergeMC(double s[MATRIX_ROWS][MATRIX_COLS],double i[MATRIX_ROWS],double d[MATRIX_ROWS][MATRIX_COLS],int column);
static double Determinant(double mt[MATRIX_ROWS][MATRIX_COLS]);

// interpretes code as status and pops up an error screen if necessary, returns the code itself
#ifndef DO_NOT_COMPILE_DRIVER_HERE
   static ViStatus tlccs_checkErrorLevel(ViSession vi, ViStatus code);
#else
   ViStatus tlccs_checkErrorLevel(ViSession vi, ViStatus code);
#endif

static ViStatus tlccs_aquireRawScanData(ViSession vi, ViUInt16 raw[], ViReal64 data[]);
static ViStatus tlccs_getWavelengthParameters (ViSession vi);
static ViStatus tlccs_readEEFactoryPoly(ViSession vi, ViReal64 poly[]); 
static ViStatus tlccs_checkNodes(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt); 
static ViStatus tlccs_nodes2poly(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt, ViReal64 poly[]); 
static ViStatus tlccs_poly2wlArray(tlccs_wl_cal_t *wl);

static ViStatus tlccs_readEEFactoryFlag(ViSession vi, ViPUInt16 flag);
static ViStatus tlccs_writeEEFactoryFlag(ViSession vi, ViUInt16 flag);

static ViStatus tlccs_readEEUserFlag(ViSession vi, ViPUInt16 flag); 
static ViStatus tlccs_writeEEUserFlag(ViSession vi, ViUInt16 flag);

static ViStatus tlccs_readEEFactoryPoly(ViSession vi, ViReal64 poly[]);
static ViStatus tlccs_writeEEFactoryPoly(ViSession vi, ViReal64 poly[]);

static ViStatus tlccs_readEEUserPoly(ViSession vi, ViReal64 poly[]);
static ViStatus tlccs_writeEEUserPoly(ViSession vi, ViReal64 poly[]);

static ViStatus tlccs_readEEUserPoints(ViSession vi, ViUInt32 pixel[], ViReal64 wl[], ViPUInt16 cnt);
static ViStatus tlccs_writeEEUserPoints(ViSession vi, ViUInt32 pixel[], ViReal64 wl[], ViUInt16 cnt); 

static ViStatus tlccs_getFirmwareRevision(ViSession vi);  
static ViStatus tlccs_getHardwareRevision(ViSession vi);
static ViStatus tlccs_getAmplitudeCorrection(ViSession vi);
static ViStatus tlccs_getAmplitudeCorrectionArray(ViSession vi, tlccs_data_t *data, int what);

static ViStatus tlccs_getRawData(ViSession vi, ViUInt16 data[]);

__attribute__((visibility("default"))) ViStatus tlccs_setSerialNumber(ViSession vi, ViPChar serial);  

__attribute__((visibility("default"))) ViStatus tlccs_setDarkCurrentOffset(ViSession vi, ViUInt16 evenOffset, ViUInt16 oddOffset);
__attribute__((visibility("default"))) ViStatus tlccs_getDarkCurrentOffset(ViSession vi, ViPUInt16 even, ViPUInt16 odd); 

__attribute__((visibility("default"))) ViStatus tlccs_writeEEPROM(ViSession vi, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf, ViUInt16 *chksum);
__attribute__((visibility("default"))) ViStatus tlccs_writeEEPROM_wo_crc(ViSession vi, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf);
__attribute__((visibility("default"))) ViStatus tlccs_readEEPROM(ViSession vi, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf, ViUInt16* cnt);
__attribute__((visibility("default"))) ViStatus tlccs_readEEPROM_wo_crc(ViSession vi, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf, ViUInt16* cnt);

/*===========================================================================
 Global Variables
===========================================================================*/

/*===========================================================================



 USER-CALLABLE FUNCTIONS (Exportable Functions)   ---   START



===========================================================================*/


#ifndef DO_NOT_COMPILE_DRIVER_HERE



/*===========================================================================

 
 Init/close


===========================================================================*/

/*---------------------------------------------------------------------------
   Function:   Initialize
   Purpose:    This function initializes the instrument driver session and
               returns an instrument handle which is used in subsequent calls. 

   Parameters:
   
   ViRsrc rsrcName:        The visa resource string.
   ViBoolean IDQuery:      Boolean to query the ID or not.
   ViBoolean reset_instr:  Boolean to reset the device or not.
   ViPSession vi:          Instrument Handle (Pointer to session to device to be opened)
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_init (ViRsrc rsrscName, ViBoolean IDQuery, ViBoolean reset_instr, ViPSession vi)
{
   ViStatus       err;
   ViSession      rm = VI_NULL;
   ViUInt16       vid, pid;
   tlccs_data_t   *data;

   //Open instrument session and set 'user data' to 'VI_NULL'
   *vi = VI_NULL;
   if((err = viOpenDefaultRM(&rm))) return (err);
   if((err = viOpen(rm, rsrscName, TLCCS_LOCK_STATE, VI_NULL, vi)))
   {
      viClose(rm);
      return (err);
   }
   if((err = viSetAttribute(*vi, VI_ATTR_USER_DATA, (ViAttrState)VI_NULL)))
   {
      viClose(*vi);
      viClose(rm);
      return (err);
   }

   // Is it a Thorlabs CCS_SERIES
   if((err = viGetAttribute(*vi, VI_ATTR_MANF_ID,    &vid)))               return (tlccs_initClose(*vi, err));
   if((err = viGetAttribute(*vi, VI_ATTR_MODEL_CODE, &pid)))               return (tlccs_initClose(*vi, err));
   if(vid != TLCCS_VID)                                                    return (tlccs_initClose(*vi, VI_ERROR_FAIL_ID_QUERY));
   if((pid != CCS100_PID) && (pid != CCS125_PID) && 
      (pid != CCS150_PID) && (pid != CCS175_PID) &&
      (pid != CCS200_PID))                                                 return (tlccs_initClose(*vi, VI_ERROR_FAIL_ID_QUERY));

   // Communication buffers
   if((err = viFlush (*vi, VI_WRITE_BUF_DISCARD | VI_READ_BUF_DISCARD)))   return (tlccs_initClose(*vi, err));

   // Configure Session
   if ((err = viSetAttribute(*vi, VI_ATTR_TMO_VALUE, TLCCS_TIMEOUT_DEF)))  return (tlccs_initClose(*vi, err));


   // Private driver data
   if((data = (tlccs_data_t*)malloc(sizeof(tlccs_data_t))) == NULL)        return (tlccs_initClose(*vi, VI_ERROR_SYSTEM_ERROR));

   if((err = viSetAttribute(*vi, VI_ATTR_USER_DATA, (ViAttrState)data)))   return (tlccs_initClose(*vi, err));
   data->instr    = *vi;
   data->errQuery = VI_OFF;   // turn off auto-error-query
   data->pid      = pid;
   data->vid      = vid;
   data->timeout  = TLCCS_TIMEOUT_DEF;
   data->cal_mode = TLCCS_CAL_MODE_USER;      // this parameter will be updated in tlccs_getAmplitudeCorrection()


   viGetAttribute(*vi, VI_ATTR_MODEL_NAME,     data->name);
   viGetAttribute(*vi, VI_ATTR_MANF_NAME,      data->manu);
   viGetAttribute(*vi, VI_ATTR_USB_SERIAL_NUM, data->serNr);
   
   // Reset device
   if((err = viClear(*vi))) return (tlccs_initClose(*vi, err));

   // Error query
   data->errQuery = TLCCS_DEFAULT_ERR_QUERY_MODE;
   
   // Configure device
   // set the default integration time
   if((err = tlccs_setIntegrationTime (*vi, TLCCS_DEF_INT_TIME))) return tlccs_initClose(*vi, err);

   // get wavelength to pixel calculation parameters
   if((err = tlccs_getWavelengthParameters (*vi))) return tlccs_initClose(*vi, err);

   // get dark current offset values
   if((err = tlccs_getDarkCurrentOffset (*vi, VI_NULL, VI_NULL))) return tlccs_initClose(*vi, err);

   // get firmware revision
   if((err = tlccs_getFirmwareRevision (*vi))) return tlccs_initClose(*vi, err);

   // get hardware revision
   if((err = tlccs_getHardwareRevision (*vi))) return tlccs_initClose(*vi, err);
   
   // get amplitude correction
   if((err = tlccs_getAmplitudeCorrection (*vi))) return tlccs_initClose(*vi, err);
   
   //Ready
   return (VI_SUCCESS);
}


/*---------------------------------------------------------------------------
   Function:   Close
   Purpose:    This function close an instrument driver session. 

   Input Parameters:
   ViSession vi:              Instrument Handle
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_close (ViSession vi)
{
   return (tlccs_initClose(vi, VI_SUCCESS));
}


/*===========================================================================


 Class: Configuration Functions.


===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Set Integration Time
   Purpose:    This function set the optical integration time in seconds. 

   Parameters:
   
   ViSession vi:              The actual session to opened device.
   ViReal64 integrationTime:  The optical integration time. 
                              Min: TLCCS_MIN_INT_TIME (1.0E-5)
                              Max: TLCCS_MAX_INT_TIME (6.0E+1)
                              Default value: 1.0E-3. 
---------------------------------------------------------------------------*/
#define SH_PERCENT               16.5
#define MIN_SHUTTER_PULSES       ((int)((double)3695 * SH_PERCENT))
ViStatus _VI_FUNC tlccs_setIntegrationTime (ViSession vi, ViReal64 integrationTime) 
{
   ViStatus err = VI_SUCCESS;
   ViUInt8  data[TLCCS_NUM_INTEG_CTRL_BYTES];
   ViInt32 integ_us = 0;
   ViInt32 integ = 0;
   ViInt32 presc = 0;
   ViInt32 fill = 0;
   ViInt32 integ_max;
   ViInt32 diff;

   // check for time range
   if((integrationTime < TLCCS_MIN_INT_TIME) || (integrationTime > TLCCS_MAX_INT_TIME)) return VI_ERROR_PARAMETER2;
   
   // convert the integration time from seconds to micro seconds
   integ_us = (ViInt32)(integrationTime * 1000000.0);
   

   integ_max = (int)(4095.0 / (1.0 + 0.01 * SH_PERCENT));
   presc = 0;
   integ = integ_us;

   // calculate the prescaler value
   while((integ > integ_max) && (presc < 20))
   {
      integ >>= 1;
      presc++;
   };


   // calculate the filling value
   diff = 0;

   // integ counter has to be so big that CPLD internal counter has finifshed before integ counter becomes zero
   if(integ < (TLCCS_NUM_RAW_PIXELS >> presc))
   {
      diff = (TLCCS_NUM_RAW_PIXELS >> presc) - integ;
   }

   fill = (integ * SH_PERCENT) / 100 + diff;
   integ = integ - 8 + fill;
   // when the newly calculated value for integ leads to a possible overflow
   // increase the prescaler and decrease integ and fill accordingly
   if(integ > TLCCS_NUM_RAW_PIXELS)
   {
      integ >>= 1;
      integ  -= 4;
      fill  >>= 1;
      presc++;
   }
      

   data[0] = (ViUInt8)((presc & 0xFF00) >> 8);
   data[1] = (ViUInt8)((presc & 0x00FF));
   
   data[2] = (ViUInt8)((fill & 0xFF00) >> 8);
   data[3] = (ViUInt8)((fill & 0x00FF));
   
   data[4] = (ViUInt8)(((integ) & 0xFF00) >> 8);
   data[5] = (ViUInt8)(((integ) & 0x00FF));
   
   // set address masking bits
   data[0] |= 0x00;  // prescaler address
   data[2] |= 0x10;  // filling timer address
   data[4] |= 0x20;  // integration timer address

   // the transfer to device
   err = tlccs_USB_out(vi, TLCCS_WCMD_INTEGRATION_TIME, 0, 0, TLCCS_NUM_INTEG_CTRL_BYTES, (ViBuf)data);

   // error check and log
   err = tlccs_checkErrorLevel(vi, err);
   
   return err; 
}


/*---------------------------------------------------------------------------
   Function:   Get Integration Time
   Purpose:    This function returns the optical integration time in seconds. 

   Parameters:
   
   ViSession vi:              The actual session to opened device.
   ViPReal64 integrationTime: The optical integration time. 
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getIntegrationTime (ViSession vi, ViPReal64 integrationTime)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViUInt8 data[TLCCS_NUM_INTEG_CTRL_BYTES];
   ViInt32 integ = 0;
   ViInt32 presc = 0;
   ViInt32 fill = 0;
   
   // request the data
   err = tlccs_USB_in(vi, TLCCS_RCMD_INTEGRATION_TIME, 0, 0, (ViUInt16)(TLCCS_NUM_INTEG_CTRL_BYTES * sizeof(ViUInt8)), (ViBuf)data, &read_bytes);
   
   // error mapping
   if((read_bytes != (TLCCS_NUM_INTEG_CTRL_BYTES * sizeof(ViUInt8))) && (!err))   err = VI_ERROR_TLCCS_READ_INCOMPLETE; 
   
   // check for errors
   if(err = tlccs_checkErrorLevel(vi, err))  return (err);
   
   // decode the response
   presc = ((data[0] << 8) + data[1]) & 0x0FFF;
   fill = ((data[2] << 8) + data[3]) & 0x0FFF; 
   integ = ((data[4] << 8) + data[5]) & 0x0FFF;
   
   // calculate the integration time
   *integrationTime = ((ViReal64)(integ - fill + 8) * pow(2.0, (ViReal64)presc)) / 1000000.0;
   
   return err;  
}


/*===========================================================================


 Class: Action/Status Functions.


===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Start Scan
   Purpose:    This function triggers the the CCS to take one single scan.  

   Parameters:
   
   ViSession vi:              The actual session to opened device.
   
   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_startScan (ViSession vi)
{
   ViStatus err = VI_SUCCESS;
   
   err = tlccs_USB_out(vi, TLCCS_WCMD_MODUS, MODUS_INTERN_SINGLE_SHOT, 0, 0, VI_NULL);
   
   // error check and log
   err = tlccs_checkErrorLevel(vi, err);
   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Start Continuous Scan
   Purpose:    This function starts the CCS scanning continuously. 
               Any other function except 'Get Scan Data' and 'Get Device Status' 
               will stop the scanning.  

   Parameters:
   
   ViSession vi:              The actual session to opened device.
   
   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_startScanCont (ViSession vi)
{
   ViStatus err = VI_SUCCESS;
   
   err = tlccs_USB_out(vi, TLCCS_WCMD_MODUS, MODUS_INTERN_CONTINUOUS, 0, 0, VI_NULL);

   // error check and log
   err = tlccs_checkErrorLevel(vi, err);
   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Start Extern Scan
   Purpose:    This function arms the external trigger of the CCS. A
               following low to high transition at the trigger input of the 
               CCS then starts a scan.  

   Parameters:
   
   ViSession vi:              The actual session to opened device.
   
   Note:
   When you issue a read command 'Get Scan Data' before the CCS was 
   triggered you will get a timeout error. Use 'Get Device Status' to check 
   the scan status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_startScanExtTrg (ViSession vi)
{
   ViStatus err = VI_SUCCESS;
   
   err = tlccs_USB_out(vi, TLCCS_WCMD_MODUS, MODUS_EXTERN_SINGLE_SHOT, 0, 0, VI_NULL);

   // error check and log
   err = tlccs_checkErrorLevel(vi, err);
   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Start Continuous Extern Scan
   Purpose:    This function arms the CCS for scanning after an external 
               trigger. A following low to high transition at the trigger input 
               of the CCS then starts a scan. The CCS will rearm immediately 
               after the scan data is readout. Any other function except 
               'Get Scan Data' or 'Get Device Status' will stop the scanning.  

   Parameters:
   
   ViSession vi:              The actual session to opened device.
   
   Note:
   The scan data can be read out with the function 'Get Scan Data'

   Note:
   When you issue a read command 'Get Scan Data' before the CCS_SERIES was triggered 
   you will get a timeout error. Use 'Get Device Status' to check the scan status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_startScanContExtTrg (ViSession vi)
{
   ViStatus err = VI_SUCCESS;
   
   err = tlccs_USB_out(vi, TLCCS_WCMD_MODUS, MODUS_EXTERN_CONTINUOUS, 0, 0, VI_NULL);

   // error check and log
   err = tlccs_checkErrorLevel(vi, err);
   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Get Device Status
   Purpose:    This function returns the device status. Use macros to mask
               out single bits.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViPInt32 deviceStatus:     The device status returned from the instrument
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getDeviceStatus (ViSession vi, ViPInt32 deviceStatus)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViInt16 tmp = 0x00;
   
   if(!deviceStatus) return VI_ERROR_INV_PARAMETER;
   
   *deviceStatus = 0xFFFF;
   
   err = tlccs_USB_in(vi, TLCCS_RCMD_GET_STATUS, 0, 0, (ViUInt16)sizeof(ViInt16), (ViBuf)&tmp, &read_bytes);
   if((!err) && (read_bytes != sizeof(ViInt16))) err = VI_ERROR_TLCCS_READ_INCOMPLETE;
   
   *deviceStatus = (ViInt32)tmp;
   
   // error check and log
   err = tlccs_checkErrorLevel(vi, err);

   return (err);  
}


/*===========================================================================


 Class: Data Functions.


===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Get Scan Data
   Purpose:    This function reads out the processed scan data.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViReal64 data[]:           The measurement array (TLCCS_NUM_PIXELS elements).
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getScanData (ViSession vi, ViReal64 data[])
{
   ViStatus err         = VI_SUCCESS;     // error level
   ViUInt16 raw[TLCCS_NUM_RAW_PIXELS];    // array to copy raw data to
   
   // read raw scan data
   if((err = tlccs_getRawData(vi, raw))) return err;
   
   // process data
   err = tlccs_aquireRawScanData(vi, raw, data);
   
   // error check and log
   err = tlccs_checkErrorLevel(vi, err);
   
   return (err);
}


/*---------------------------------------------------------------------------
   Function:   Get Raw Scan Data
   Purpose:    This function reads out the raw scan data.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViInt32 scanDataArray[]:   The measurement array (TLCCS_NUM_RAW_PIXELS elements).
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getRawScanData (ViSession vi, ViInt32 scanDataArray[])
{
   ViStatus err         = VI_SUCCESS;
   ViUInt16 raw[TLCCS_NUM_RAW_PIXELS];  // array to copy raw data to
   ViInt32  i           = 0;
   
   err = tlccs_getRawData(vi, raw);
   
   for(i = 0; i < TLCCS_NUM_RAW_PIXELS; i++)
   {
      scanDataArray[i] = (ViInt32)raw[i];    
   }
   
   // check for errors 
   if((err = tlccs_checkErrorLevel(vi, err)))   return (err);  
   
   return (err);
}


/*---------------------------------------------------------------------------
   Function:   Set Wavelength Data
   Purpose:    This function stores data for user-defined pixel-wavelength
               correlation to the instrument's nonvolatile memory.

               The given data pair arrays are used to interpolate the
               pixel-wavelength correlation array returned by the
               TLCCS_getWavelengthData function.

   Note: In case the interpolated pixel-wavelength correlation
   contains negative values, or the values are not strictly
   increasing the function returns with error VI_ERROR_INV_USER_DATA.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViInt32 pixelDataArray[]:  The pixel data.
   ViReal64 wavelengthDataArray[]:The wavelength data.
   ViInt32 bufferLength:      The length of both arrays.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_setWavelengthData (ViSession vi, ViInt32 pixelDataArray[],
                                           ViReal64 wavelengthDataArray[], ViInt32 bufferLength)
{
#define FACTORY_ADJ_OFFSET       62749006
   
   ViStatus       err = VI_SUCCESS;
   tlccs_data_t   *data;
   tlccs_wl_cal_t cal;
   char           target = 0;
   
   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   if((!pixelDataArray) || (!wavelengthDataArray)) return VI_ERROR_INV_PARAMETER; 
   
   // check valid buffer length and determine target
   if((bufferLength >= TLCCS_MIN_NUM_USR_ADJ) && (bufferLength <= TLCCS_MAX_NUM_USR_ADJ))
   {
      target = 1; // target is user adjustment data
   }
   else if ((bufferLength >= (TLCCS_MIN_NUM_USR_ADJ + FACTORY_ADJ_OFFSET)) && (bufferLength <= (TLCCS_MAX_NUM_USR_ADJ  + FACTORY_ADJ_OFFSET)))
   {
      bufferLength -= FACTORY_ADJ_OFFSET;
      target = 0; // target is factory adjustment data
   }
   else return VI_ERROR_INV_PARAMETER;
   
   // check nodes array
   if((err = tlccs_checkNodes(pixelDataArray, wavelengthDataArray, bufferLength)) != VI_SUCCESS)  return err;
   
   // calculate new coefficients...
   if((err = tlccs_nodes2poly(pixelDataArray, wavelengthDataArray, bufferLength, cal.poly)) != VI_SUCCESS) return err;
   
   // use the coefficients to calculate the new wavelength array...
   if((err = tlccs_poly2wlArray(&cal)) != VI_SUCCESS)   return err;
   
   
   // write new data to EEPROM and data structure
   if(target)
   {
      ////  target is user adjustment data  ////
      
      // write polynomical coefficients to eeprom
      if((err = tlccs_writeEEUserPoly(vi, cal.poly)) != VI_SUCCESS) return err;
      
      // write user  calibration points
      if((err = tlccs_writeEEUserPoints(vi, pixelDataArray, wavelengthDataArray, bufferLength)) != VI_SUCCESS) return err;

      // write valid flags to eeprom
      cal.valid = 1;
      if((err = tlccs_writeEEUserFlag(vi, cal.valid)) != VI_SUCCESS) return err;
      
      // copy new values
      memcpy(&(data->user_cal), &cal, sizeof(tlccs_wl_cal_t));
       
      memcpy(data->user_points.user_cal_node_pixel, pixelDataArray, sizeof(ViUInt32) * bufferLength);
      memcpy(data->user_points.user_cal_node_wl, wavelengthDataArray, sizeof(ViReal64) * bufferLength);
      data->user_points.user_cal_node_cnt = bufferLength;
   }
   else
   {
      ////  target is factory adjustment data   ////
      
      // write polynomical coefficients to eeprom
      if((err = tlccs_writeEEFactoryPoly(vi, cal.poly)) != VI_SUCCESS) return err;
      
      // write valid flags to eeprom
      cal.valid = 1;
      if((err = tlccs_writeEEFactoryFlag(vi, cal.valid)) != VI_SUCCESS) return err;
      
      // copy new values to CCS data
      memcpy(&(data->factory_cal), &cal, sizeof(tlccs_wl_cal_t));
   }
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get Wavelength Data
   Purpose:    This function returns data for the pixel-wavelength correlation.
               The maximum and the minimum wavelength are additionally provided
               in two separate return values.

   Note:
   If you do not need either of these values you may pass NULL.

   The value returned in Wavelength_Data_Array[0] is the wavelength
   at pixel 0, this is also the minimum wavelength, the value
   returned in Wavelength_Data_Array[1] is the wavelength at pixel
   1 and so on until Wavelength_Data_Array[TLCCS_NUM_PIXELS-1]
   which provides the wavelength at pixel TLCCS_NUM_PIXELS-1
   (3647). This is the maximum wavelength.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViInt16 dataSet:           The pixel data selection according to macros below.

   Output Parameters:
   ViReal64 wavelengthDataArray[]:  The wavelength data.
   ViPReal64 minimumWavelength:     The minimum wavelength.
   ViPReal64 maximumWavelength:     The maximum wavelength.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getWavelengthData (ViSession vi, ViInt16 dataSet, ViReal64 wavelengthDataArray[], ViPReal64 minimumWavelength, ViPReal64 maximumWavelength)
{
   tlccs_data_t   *data;
   ViStatus       err = VI_SUCCESS;
   
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;    
   
   switch (dataSet)
   {
      case TLCCS_CAL_DATA_SET_FACTORY:
         // copy wavelength array form factory data
         
         if(wavelengthDataArray != NULL)  memcpy(wavelengthDataArray, data->factory_cal.wl, (TLCCS_NUM_PIXELS * sizeof(ViReal64)));
         if(minimumWavelength != NULL)    *minimumWavelength = data->factory_cal.min;
         if(maximumWavelength != NULL)    *maximumWavelength = data->factory_cal.max;
         break;
         
      case TLCCS_CAL_DATA_SET_USER:

         if(!data->user_cal.valid) return VI_ERROR_TLCCS_NO_USER_DATA;
    
         if(wavelengthDataArray != NULL)  memcpy(wavelengthDataArray, data->user_cal.wl, (TLCCS_NUM_PIXELS * sizeof(ViReal64)));
         if(minimumWavelength != NULL)    *minimumWavelength = data->user_cal.min;
         if(maximumWavelength != NULL)    *maximumWavelength = data->user_cal.max;
         break;
         
      default:
         return VI_ERROR_INV_PARAMETER;
   }

   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Get User Calibration Points
   Purpose:    This function returns the user-defined pixel-wavelength
               correlation supporting points. These given data pair arrays are
               used by the driver to interpolate the pixel-wavelength
               correlation array returned by the tlccs_getWavelengthData
               function.

   Note:
   If you do not need either of these values you may pass NULL.
   The function returns with error VI_ERROR_NO_USER_DATA if no user
   calibration data is present in the instruments nonvolatile
   memory.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViInt32 pixelDataArray[]:        The pixel data array.
   ViReal64 wavelengthDataArray[]:  The wavelength data array.
   ViPInt32 bufferLength:           The number of user calibration points.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getUserCalibrationPoints (ViSession vi, ViInt32 pixelDataArray[],
                                                  ViReal64 wavelengthDataArray[], ViPInt32 bufferLength)
{
   ViStatus err = VI_SUCCESS;
   tlccs_data_t    *data;
   
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err; 
   
   if(data->user_cal.valid)
   {
      if(pixelDataArray)      memcpy(pixelDataArray,      data->user_points.user_cal_node_pixel, data->user_points.user_cal_node_cnt * sizeof(ViUInt32));
      if(wavelengthDataArray) memcpy(wavelengthDataArray, data->user_points.user_cal_node_wl,    data->user_points.user_cal_node_cnt * sizeof(ViReal64));
      if(bufferLength)        *bufferLength = data->user_points.user_cal_node_cnt;
   }
   else
   {
      err = VI_ERROR_TLCCS_NO_USER_DATA;
   }
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Set Amplitude Data
   Purpose:    This function stores data for user-defined amplitude
               correction factors to nonvolatile memory of CCD device.

               The given data array can be used to correct the
               amplitude information returned by the
               tlccs_getScanDataData function.

   Note: In case the correction factors are out of the range
   TLCCS_AMP_CORR_FACT_MIN (0.001) ... TLCCS_AMP_CORR_FACT_MAX (1000.0)
   the function returns with error VI_ERROR_TLCCS_INV_USER_DATA. 

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViReal64 AmpCorrFact[]:    The array with amplitude correction factors
   ViInt32 bufferLength:      The length of the array.
   ViInt32 bufferStart:       The start index for the array.
   ViInt32 mode               With mode one can select if the new data will be applied to current measurements
                              only (ACOR_APPLY_TO_MEAS) or additionally goes to non volatile memory of the
                              device too (ACOR_APPLY_TO_MEAS_NVMEM).
                              If mode is not one of the two predefined macros the function returns VI_ERROR_INV_PARAMETER
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_setAmplitudeData (ViSession vi, ViReal64 AmpCorrFact[], ViInt32 bufferLength, ViInt32 bufferStart, ViInt32 mode)
{
#define FACTORY_SET_START           19901201
#define THORLABS_SET_START          91901201
#define TARGET_FACTROY_DATA         0
#define TARGET_USER_DATA            1
#define TARGET_THORLABS_DATA        2
   
   ViStatus             err      = VI_SUCCESS;
   tlccs_data_t    *data;
   ViUInt16             chksum = 0;
   char                 target = 0;
   int                  i;
   
   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // check valid buffer start and determine target
   if((bufferStart >= 0) && (bufferStart < TLCCS_NUM_PIXELS))
   {
      target = TARGET_USER_DATA; // target is user adjustment data
   }
   else if ((bufferStart >= FACTORY_SET_START) && (bufferStart < (FACTORY_SET_START + TLCCS_NUM_PIXELS)))
   {
      bufferStart -= FACTORY_SET_START;
      target = TARGET_FACTROY_DATA; // target is factory adjustment data
   }
   else if ((bufferStart >= THORLABS_SET_START) && (bufferStart < (THORLABS_SET_START + TLCCS_NUM_PIXELS)))
   {
      bufferStart -= THORLABS_SET_START;
      target = TARGET_THORLABS_DATA; // target is user adjustment data and will be interpreted as THORLABS data
   }
   else
   {
      return VI_ERROR_INV_PARAMETER;
   }

   // check valid buffer length
   if((bufferStart + bufferLength) < 1)                     return VI_ERROR_INV_PARAMETER;
   if((bufferStart + bufferLength) > TLCCS_NUM_PIXELS) return VI_ERROR_INV_PARAMETER;

   // check for data out of range
   for(i=0; i<bufferLength; i++)
   {
      if(AmpCorrFact[i] < TLCCS_AMP_CORR_FACT_MIN)  return VI_ERROR_TLCCS_INV_USER_DATA;
      if(AmpCorrFact[i] > TLCCS_AMP_CORR_FACT_MAX)  return VI_ERROR_TLCCS_INV_USER_DATA;
   }

   // check for correct mode
   switch(mode)
   {
      case ACOR_APPLY_TO_MEAS:
      case ACOR_APPLY_TO_MEAS_NVMEM:
            break;
      default:
            return VI_ERROR_INV_PARAMETER;
   }
      
   
   switch(target)
   {
      case TARGET_USER_DATA:
            // copy new values to data structure and ..
            for(i=0; i<bufferLength; i++)
            {
               data->user_acor_cal.acor[bufferStart + i] = (float)AmpCorrFact[i];
            }

            // .. eventually copy them to NVMEM (EEPROM) too
            if(mode == ACOR_APPLY_TO_MEAS_NVMEM)
            {
               err = tlccs_writeEEPROM(vi, (ViUInt16)EE_ACOR_USER, 0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor, &chksum);
            }
            break;

      case TARGET_FACTROY_DATA:
            // copy new values to data structure and ..
            for(i=0; i<bufferLength; i++)
            {
               data->factory_acor_cal.acor[bufferStart + i] = (float)AmpCorrFact[i];
            }

            // .. eventually copy them to NVMEM (EEPROM) too
            if(mode == ACOR_APPLY_TO_MEAS_NVMEM)
            {
               err = tlccs_writeEEPROM(vi, (ViUInt16)EE_ACOR_FACTORY, 0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)data->factory_acor_cal.acor, &chksum);
            }
            break;

      case TARGET_THORLABS_DATA:
            // copy new values to data structure and ..
            for(i=0; i<bufferLength; i++)
            {
               data->user_acor_cal.acor[bufferStart + i] = (float)AmpCorrFact[i];
            }

            // .. eventually copy them to NVMEM (EEPROM) too
            if(mode == ACOR_APPLY_TO_MEAS_NVMEM)
            {
               err = tlccs_writeEEPROM(vi, (ViUInt16)EE_ACOR_USER, 0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor, &chksum);
            }
            break;
   }

   return err;
}

/*---------------------------------------------------------------------------
   Function:   Get Amplitude Data
   Purpose:    This function retrieves data for user-defined amplitude
               correction factors from nonvolatile memory of CCD device.

               The given data array can be used to correct the
               amplitude information returned by the
               tlccs_getScanDataData function.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViInt32 bufferLength:      The length of the array.
   ViInt32 bufferStart:       The start index for the array.
   ViInt32 mode               With mode one can select if the data will be a copy of the currently used
                              amplitude correction factors (ACOR_FROM_CURRENT) or if the data is read out from the
                              device non volatile memory (ACOR_FROM_NVMEM)
                              If mode is not one of the two predefined macros the function returns VI_ERROR_INV_PARAMETER

   Output Parameters:
   ViReal64 _VI_FAR AmpCorrFact[]:  The array with amplitude correction factors
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getAmplitudeData (ViSession vi, ViReal64 AmpCorrFact[], ViInt32 bufferStart, ViInt32 bufferLength, ViInt32 mode)
{
   ViStatus             err      = VI_SUCCESS;
   tlccs_data_t    *data;
   char                 target = 0;
   int                  i;
   ViUInt16             read_bytes;
   ViUInt16             chksum = 0;
   
   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // check valid buffer start and determine target
   if((bufferStart >= 0) && (bufferStart < TLCCS_NUM_PIXELS))
   {
      target = 1; // target is user adjustment data
   }
   else if ((bufferStart >= FACTORY_SET_START) && (bufferStart < (FACTORY_SET_START + TLCCS_NUM_PIXELS)))
   {
      bufferStart -= FACTORY_SET_START;
      target = 0; // target is factory adjustment data
   }
   else if ((bufferStart >= THORLABS_SET_START) && (bufferStart < (THORLABS_SET_START + TLCCS_NUM_PIXELS)))
   {
      bufferStart -= THORLABS_SET_START;
      target = 1; // target is user adjustment data and will be interpreted as THORLABS data
   }
   else
   {
      return VI_ERROR_INV_PARAMETER;
   }

   // check valid buffer length
   if((bufferStart + bufferLength) < 1)                     return VI_ERROR_INV_PARAMETER;
   if((bufferStart + bufferLength) > TLCCS_NUM_PIXELS) return VI_ERROR_INV_PARAMETER;

   // check for correct mode
   switch(mode)
   {
      case ACOR_FROM_CURRENT:
      case ACOR_FROM_NVMEM:
            break;
      default:
            return VI_ERROR_INV_PARAMETER;
   }

   // return data from data structure
   if(target)
   {
      ////  target is user adjustment data  ////

      if(mode == ACOR_FROM_NVMEM)
      {

         // request the data for user amplitude factors
         err = tlccs_readEEPROM(vi, (ViUInt16)EE_ACOR_USER,    0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor, &read_bytes);
   
         // error mapping
         if((read_bytes != EE_LENGTH_ACOR) && (!err))   err = VI_ERROR_TLCCS_READ_INCOMPLETE; 

         // when checksum failed but everything else was ok -> assume a blank or new device, generate an array of 1.000 and write it to the EEPROM
         if(err == VI_ERROR_CYEEPROM_CHKSUM)
         {
            for(i=0; i<TLCCS_NUM_PIXELS; i++)
            {
               data->user_acor_cal.acor[i] = 1.0;
            }
            err = tlccs_writeEEPROM(vi, (ViUInt16)EE_ACOR_USER, 0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor, &chksum);
         }
      }
      
      // copy new values
      for(i=0; i<bufferLength; i++)
      {
         AmpCorrFact[i] = (ViReal64)data->user_acor_cal.acor[i + bufferStart];
      }
       
   }
   else
   {
      ////  target is factory adjustment data   ////
      
      if(mode == ACOR_FROM_NVMEM)
      {
         // request the data for factory amplitude factors
         err = tlccs_readEEPROM(vi, (ViUInt16)EE_ACOR_FACTORY, 0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)data->factory_acor_cal.acor, &read_bytes);
   
         // error mapping
         if((read_bytes != EE_LENGTH_ACOR) && (!err))   err = VI_ERROR_TLCCS_READ_INCOMPLETE; 

         // when checksum failed but everything else was ok -> assume a blank or new device, generate an array of 1.000 and write it to the EEPROM
         if(err == VI_ERROR_CYEEPROM_CHKSUM)
         {
            for(i=0; i<TLCCS_NUM_PIXELS; i++)
            {
               data->factory_acor_cal.acor[i] = 1.0;
            }
            err = tlccs_writeEEPROM(vi, (ViUInt16)EE_ACOR_FACTORY, 0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)data->factory_acor_cal.acor, &chksum);
         }
      }
      
      // copy new values
      for(i=0; i<bufferLength; i++)
      {
         AmpCorrFact[i] = (ViReal64)data->factory_acor_cal.acor[i + bufferStart];
      }
   }
    
   return VI_SUCCESS;
}



/*===========================================================================


 Class: Utility Functions.



===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Identification Query
   Purpose:    This function returns the device identification information.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViString manufacturerName: The manufacturers name.
   ViString deviceName:       The device name.
   ViString serialNumber:     The serial number.
   ViString firmwareRevision: The firmware revision.
   ViString instrumentDriverRevision:The instrument driver revision.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_identificationQuery (ViSession vi,
                                             ViString manufacturerName,
                                             ViString deviceName,
                                             ViString serialNumber,
                                             ViString firmwareRevision,
                                             ViString instrumentDriverRevision)
{
  
   tlccs_data_t    *data;
   ViStatus       err = VI_SUCCESS;
   
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;  
   
   if(manufacturerName != NULL)
   {
      strncpy(manufacturerName, data->manu, TLCCS_BUFFER_SIZE);
   }
    
   if(deviceName != NULL)
   {
      strncpy(deviceName, data->name, TLCCS_BUFFER_SIZE);
   }
    
   if(serialNumber != NULL)
   {
      strncpy(serialNumber, data->serNr, TLCCS_BUFFER_SIZE);
   }
    
   if(firmwareRevision != NULL)
   {
      sprintf(firmwareRevision, "%d.%d.%d/1.%d.0", data->firmware_version.major, data->firmware_version.minor, data->firmware_version.subminor, data->hardware_version.major);
   }
   
   if(instrumentDriverRevision != NULL)
   {
      sprintf(instrumentDriverRevision, "%d.%d.%d", TLCCS_VER_MAJOR, TLCCS_VER_MINOR, TLCCS_VER_SUBMINOR);
   }
   
   return (err);
}


/*---------------------------------------------------------------------------
   Function:   Revision Query
   Purpose:    This function returns the revision of the instrument driver
               and the firmware revision of the instrument being used.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViString driver_rev:       Instrument driver revision
   ViString instr_rev:        Instrument firmware revision
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_revision_query (ViSession vi, ViString driver_rev, ViString instr_rev)
{
   tlccs_data_t   *data;
   ViStatus       err = VI_SUCCESS;
   
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;  
   
   if(driver_rev != NULL)
   {
      sprintf(driver_rev, "%d.%d.%d", TLCCS_VER_MAJOR, TLCCS_VER_MINOR, TLCCS_VER_SUBMINOR);
   }
   
   if(instr_rev != NULL)
   {
      sprintf(instr_rev, "%d.%d.%d/1.%d.0", data->firmware_version.major, data->firmware_version.minor, data->firmware_version.subminor, data->hardware_version.major);
   }
   
   return (err);  
}


/*---------------------------------------------------------------------------
   Function:   Reset
   Purpose:    This function resets the device.

   Input Parameters:
   ViSession vi:              Instrument Handle
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_reset (ViSession vi)
{
   ViStatus err = VI_SUCCESS;

   // reset the device itself
   err = tlccs_USB_out(vi, TLCCS_WCMD_RESET, 0, 0, 0, VI_NULL);

   // check for errors 
   if((err = tlccs_checkErrorLevel(vi, err)))   return (err);  
   
   // do an one time readout to collect garbled buffers
   err = viFlush (vi, VI_READ_BUF);

   return err;
}

/*---------------------------------------------------------------------------
   Function:   Self Test
   Purpose:    This function resets the device.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViPInt16 test_result:      Numeric result from self-test operation
                              0 = no error (test passed)
   ViString test_message:     Self-test status message
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_self_test (ViSession vi, ViPInt16 test_result, ViString test_message)
{
   return VI_WARN_NSUP_SELF_TEST;
}


/*---------------------------------------------------------------------------
   Function:   Error Query
   Purpose:    This function queries the instrument and returns instrument-
               specific error information.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViPInt32 error_code:       Instrument error code
   ViString error_message:    Error message
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_error_query(ViSession vi, ViPInt32 error_code, ViString error_message)
{
   return VI_WARN_NSUP_ERROR_QUERY;
}


/*---------------------------------------------------------------------------
   Function:   Error Message
   Purpose:    This function translates the error return value from a
               VXIplug&play instrument driver function to a user-readable
               string.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViStatus status_code:      Instrument driver error code

   Output Parameters:
   ViString message:          VISA or instrument driver Error message
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_error_message(ViSession vi, ViStatus status_code, ViString message)
{
   const tlccs_errDescrStat_t *ptr;
   
   if(!message)  return VI_ERROR_INV_PARAMETER;       
   
   // VISA errors
   if(viStatusDesc(vi, status_code, message) != VI_WARN_UNKNOWN_STATUS) return VI_SUCCESS;

   // Static driver errors
   ptr = TLCCS_errDescrStat;
   while(ptr->descr != VI_NULL)
   {
      if(ptr->err == status_code)
      {
         strcpy(message, ptr->descr);
         return (VI_SUCCESS);
      }
      ptr++;
   }

   // Not found
   viStatusDesc(vi, VI_WARN_UNKNOWN_STATUS, message);
   return VI_WARN_UNKNOWN_STATUS;
}


/*---------------------------------------------------------------------------
   Function:   Set Attribute
   Purpose:    This function sets a specified attribute.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViAttr attribute:          The attribute to set. See "Driver attributes"
                              section for available attributes and further
                              information.
   ViAttrState value:         The attribute value.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_setAttribute (ViSession vi, ViAttr attribute, ViAttrState value)
{
   ViStatus     err;
   tlccs_data_t *data;

   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data))) return (err);
   switch(attribute)
   {
      case TLCCS_ATTR_USER_DATA:
         data->userData = value;
         break;

      default:
         return (VI_ERROR_PARAMETER2);
   }
   return (VI_SUCCESS);
}

/*---------------------------------------------------------------------------
   Function:   Get Attribute
   Purpose:    This function returns a specified attribute.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViAttr attribute:          The attribute to get. See "Driver attributes"
                              section for available attributes and further
                              information.

   Output Parameters:
   ViAttrState value:         The attribute value.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getAttribute (ViSession vi, ViAttr attribute, ViAttrState *value)
{
   ViStatus          err;
   tlccs_data_t *data;

   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data))) return (err);
   switch(attribute)
   {
      case TLCCS_ATTR_USER_DATA:
         *value = data->userData;
         break;

      case TLCCS_ATTR_CAL_MODE:
         *value = data->cal_mode;
         break;

      default:
         return (VI_ERROR_PARAMETER2);
   }
   return (VI_SUCCESS);
}



/*---------------------------------------------------------------------------
   Function:   Set User Text
   Purpose:    This function writes the given string to the novolatile memory of
               the spectrometer.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViString userText:         The new user text
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_setUserText (ViSession vi, ViString userText)
{
   ViStatus       err = VI_SUCCESS;    
   tlccs_data_t    *data;
   char           buf[TLCCS_MAX_USER_NAME_SIZE];

   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   if(!userText)  return VI_ERROR_INV_PARAMETER; 
   
   // copy the string
   strncpy(buf, userText, (TLCCS_MAX_USER_NAME_SIZE - 1));   // strncpy will fill with '\0' when userText is smaller than (USER_LABEL_LENGTH-1)
   
   // truncate 
   buf[TLCCS_MAX_USER_NAME_SIZE-1] = '\0';

   // write to eeprom
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_USER_LABEL, 0, TLCCS_MAX_USER_NAME_SIZE, (ViBuf)buf, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))   return (err);
   
   return err;
}

/*---------------------------------------------------------------------------
   Function:   Get User Text
   Purpose:    This function reads the user text from the novolatile memory of
               the spectrometer.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViString userText:         The user text read out from device
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC tlccs_getUserText (ViSession vi, ViString userText)
{
   ViStatus       err = VI_SUCCESS;    
   tlccs_data_t    *data = VI_NULL;
   ViUInt16       cnt = 0;

   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   if(!userText)  return VI_ERROR_INV_PARAMETER;
   
   // write to eeprom
   err = tlccs_readEEPROM(vi, (ViUInt16)EE_USER_LABEL, 0, TLCCS_MAX_USER_NAME_SIZE, (ViBuf)userText, &cnt);
   
   if(err)
   {
      memset(userText, 0, TLCCS_MAX_USER_NAME_SIZE);
      
      // in case there is no user text...
      strncpy(userText, DEFAULT_USER_TEXT, TLCCS_MAX_USER_NAME_SIZE);
      
      if(err == VI_ERROR_CYEEPROM_CHKSUM) err = VI_SUCCESS;
      
      // check for error
      tlccs_checkErrorLevel(vi, err);
   }
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   USB Out (USB control out transfer)
   Purpose:    This function encapsulates the VISA function viUsbControlOut.
               When CCS stalls the error VI_ERROR_IO will be returned by
               viUsbControlOut. Then USB Out will issue the VISA function
               viUsbControlIn and try to read one Byte from the CCS, if this
               succeeds the obtained Byte contains the error code from the
               CCS. This means we have NO communications error.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViInt16 bRequest:          The command sent to the CCS
   ViUInt16 wValue:           Arbitrary parameter, can be used for additional information 
   ViUInt16 wIndex:           Arbitrary parameter, can be used for additional information 
   ViUInt16 wLength:          Size of Buffer
   ViBuf Buffer:              Buffer of max. 64 Bytes
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_USB_out(ViSession vi, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer)
{
   ViStatus err;
   unsigned char TLCCS_Error;
   
   err = viUsbControlOut (vi, 0x40, bRequest, wValue, wIndex, wLength, Buffer);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (vi, 0xC0, TLCCS_RCMD_GET_ERROR, 0, 0, 1, &TLCCS_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' CCS error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)TLCCS_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   return err;
}

/*---------------------------------------------------------------------------
   Function:   USB In (USB control in transfer)
   Purpose:    This function encapsulates the VISA function viUsbControlIn.
               When CCS stalls the error VI_ERROR_IO will be returned by
               viUsbControlIn. Then USB In will issue the VISA function
               viUsbControlIn and try to read one Byte from the CCS, if this
               succeeds the obtained Byte contains the error code from the
               CCS. This means we have NO communications error.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViInt16 bRequest:          The command sent to the CCS
   ViUInt16 wValue:           Arbitrary parameter, can be used for additional information 
   ViUInt16 wIndex:           Arbitrary parameter, can be used for additional information 
   ViUInt16 wLength:          size of Buffer
   
   Output Parameters:
   ViBuf Buffer:              Buffer of max. 64 Bytes
   ViPUInt16 Read_Bytes:      Number of bytes actually read out
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_USB_in(ViSession vi, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer, ViPUInt16 Read_Bytes)
{
   ViStatus err;
   unsigned char TLCCS_Error;
   
   err = viUsbControlIn (vi, 0xC0, bRequest, wValue, wIndex, wLength, Buffer, Read_Bytes);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (vi, 0xC0, TLCCS_RCMD_GET_ERROR, 0, 0, 1, &TLCCS_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' CCS error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)TLCCS_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   return err;
}

/*---------------------------------------------------------------------------
   Function:   USB Write (USB bulk out transfer)
   Purpose:    This function encapsulates the VISA function viWrite. When CCS
               stalls the error VI_ERROR_IO will be returned by
               viUsbControlOut. Then USB Write will issue the VISA function
               viUsbControlIn and try to read one Byte from the CCS, if this
               succeeds the obtained Byte contains the error code from the
               CCS. This means we have NO communications error.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViBuf Buffer:              Buffer to send to device
   ViUInt32 Count:            Number of Bytes to send from buffer to device
   
   Output Parameters:
   ViPUInt32 ReturnCount:     Number of Bytes actually sent to device
                              You may pass NULL if you do not need the value
---------------------------------------------------------------------------*/
#ifdef NOT_NEEDED_UNTIL_NOW
static ViStatus tlccs_USB_write(ViSession vi, ViBuf Buffer, ViUInt32 Count, ViPUInt32 ReturnCount)
{
   ViStatus err;
   unsigned char TLCCS_Error;
   ViUInt32 retcount;
   
   err = viWrite (Instrument_Handle, Buffer, Count, &retcount);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (Instrument_Handle, 0xC0, TLCCS_RCMD_GET_ERROR, 0, 0, 1, &TLCCS_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' CCS error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)TLCCS_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   if((err == VI_SUCCESS) && (ReturnCount != NULL))
      *ReturnCount = retcount;
      
   return err;
}
#endif   // NOT_NEEDED_UNTIL_NOW


/*---------------------------------------------------------------------------
   Function:   USB Read (USB bulk in transfer)
   Purpose:    This function encapsulates the VISA function viRead. When CCS
               stalls the error VI_ERROR_IO will be returned by
               viUsbControlOut. Then USB Readt will issue the VISA function
               viUsbControlIn and try to read one Byte from the CCS, if this
               succeeds the obtained Byte contains the error code from the
               CCS. This means we have NO communications error.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViBuf ReceiveData:         Buffer where received data is put to
   ViUInt32 Count:            Number of Bytes to read from device to buffer
   
   Output Parameters:
   ViPUInt32 ReturnCount:     Number of Bytes actually read from device
                              You may pass NULL if you do not need the value
---------------------------------------------------------------------------*/
static ViStatus tlccs_USB_read(ViSession vi, ViBuf ReceiveData, ViUInt32 Count, ViPUInt32 ReturnCount)
{
   ViStatus err;
   unsigned char TLCCS_Error;
   ViUInt32 retcount;
   
   err = viRead (vi, ReceiveData, Count, &retcount);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (vi, 0xC0, TLCCS_RCMD_GET_ERROR, 0, 0, 1, &TLCCS_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' CCS error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)TLCCS_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   if((err == VI_SUCCESS) && (ReturnCount != NULL))
      *ReturnCount = retcount;
      
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Write EEPROM with checksum
   Purpose:    This helper function writes data to the nonvolatile memory
               of the CCS. It calculates a CRC16 checksum and writes the
               checksum to the EEPROM too.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViUInt16 addr:             Adress in the EEPROM where to write the data
   ViUInt16 idx:              Additonal parameter,here always 0
   ViUInt16 len:              Length of data to be written
   ViBuf buf:                 Buffer to write to EEPROM
   
   Output Parameters:
   ViPUInt16 chksum:          The CRC16 checksum
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_writeEEPROM(ViSession vi, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf, ViPUInt16 chksum)
{
   ViStatus err = VI_SUCCESS;
   uint16_t sum = 0;
   
   err = tlccs_writeEEPROM_wo_crc(vi, addr, idx, len, buf);

   // check if we have to use crc16 -> up from address EE_SERIAL_NO
   if((!err) && (addr >= EE_SW_VERSION))
   {
      sum = crc16_block((void*)buf, len);
      // when chksum != NULL provide caller with CRC16 checksum
      if(chksum) *chksum = sum;
      err = tlccs_writeEEPROM_wo_crc(vi, addr+len, idx, (ViUInt16)sizeof(uint16_t), (ViBuf)&sum);
   }
   
   return err;      
}


/*---------------------------------------------------------------------------
   Function:   Write EEPROM without checksum
   Purpose:    This helper function writes data to the nonvolatile memory
               of the CCS. No checksum is calculated the data goes straight
               forward to the EEPROM.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViUInt16 addr:             Adress in the EEPROM where to write the data
   ViUInt16 idx:              Additonal parameter,here always 0
   ViUInt16 len:              Length of data to be written
   ViBuf buf:                 Buffer to write to EEPROM
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_writeEEPROM_wo_crc(ViSession vi, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf)
{
   ViStatus err = VI_SUCCESS;
   
   ViInt32  iTransferSize = 0;
   ViUInt16 iAddress      = 0;
   ViUInt16 iLength       = 0;
   ViBuf    ibuf          = NULL;
   
   ibuf          = buf;
   iTransferSize = len;
   iAddress      = addr;
      
   // Loop until all data is gone
   while(iTransferSize > 0)
   {
      // decide how many bytes to transfer
      if(iTransferSize >= ENDPOINT_0_TRANSFERSIZE) 
      {
         iLength = ENDPOINT_0_TRANSFERSIZE;     
      }
      else
      {
         iLength = iTransferSize;   
      }
         
      // write data normal data, on error return
      err = tlccs_USB_out(vi, TLCCS_WCMD_WRITE_EEPROM, iAddress, idx, iLength, ibuf);
   
      // check for error
      if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
      // increase addresses of EEPROM and buffer for the amount of bytes sent
      ibuf          += iLength;
      iAddress      += iLength;
      // decrease the counter of bytes to be transmitted
      iTransferSize -= iLength;
   }

   return err;      
}




/*---------------------------------------------------------------------------
   Function:   Read EEPROM with checksum check
   Purpose:    This helper function reads data from the nonvolatile memory
               of the CCS. It checks a CRC16 checksum and returns a 
               checksum error when the checksum is not correct.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViUInt16 addr:             Adress in the EEPROM where to read the data
   ViUInt16 idx:              Additonal parameter,here always 0
   ViUInt16 len:              Length of data to be read
   
   Output Parameters:
   ViBuf buf:                 Buffer to put the data read from EEPROM
   ViPUInt16 cnt:             The number of Bytes actually read
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_readEEPROM(ViSession vi, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf, ViPUInt16 cnt)
{
   ViStatus err = VI_SUCCESS;   
   uint16_t sum = 0; 
   uint16_t ees = 0;
   ViUInt16 tmp = 0;
   
   if(cnt)  *cnt = 0;

   err = tlccs_readEEPROM_wo_crc(vi, addr, idx, len, buf, cnt);

   // check if we have to use crc16 -> up from address EE_SERIAL_NO
   if((!err) && (addr >= EE_SW_VERSION))
   {
      sum = crc16_block((void*)buf, len);
      if(err = tlccs_readEEPROM_wo_crc(vi, addr+len, idx, (ViUInt16)sizeof(uint16_t), (ViBuf)&ees, &tmp)) return err;
      if(sum != ees)    err = VI_ERROR_CYEEPROM_CHKSUM;
   } 
   
   return err; 
}


/*---------------------------------------------------------------------------
   Function:   Read EEPROM without checksum check
   Purpose:    This helper function reads data from the nonvolatile memory
               of the CCS. No checks with a checksum are performed.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViUInt16 addr:             Adress in the EEPROM where to read the data
   ViUInt16 idx:              Additonal parameter,here always 0
   ViUInt16 len:              Length of data to be read
   
   Output Parameters:
   ViBuf buf:                 Buffer to put the data read from EEPROM
   ViPUInt16 cnt:             The number of Bytes actually read
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_readEEPROM_wo_crc(ViSession vi, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf, ViPUInt16 cnt)
{
   ViStatus err = VI_SUCCESS;   
   
   ViInt32 iTransferSize = 0;
   ViUInt16 iAddress = 0;  
   ViUInt16 iLength = 0;
   ViUInt16 iCount = 0;
   ViBuf    ibuf = NULL;

   ibuf          = buf;
   iTransferSize = len;
   iAddress      = addr;

   if(cnt)  *cnt = 0;
   // Loop until all data is gone
   while(iTransferSize > 0)
   {
      // decide how many bytes to transfer
      if(iTransferSize >= ENDPOINT_0_TRANSFERSIZE) 
      {
         iLength = ENDPOINT_0_TRANSFERSIZE;     
      }
      else
      {
         iLength = iTransferSize;   
      }
      
      // read data
      if(err = tlccs_USB_in(vi, TLCCS_RCMD_READ_EEPROM, iAddress, idx, iLength, ibuf, &iCount)) return err;
      
      // check for error
      if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
      if(cnt) *cnt  += iCount;
      ibuf          += iCount;
      iAddress      += iCount;
      iTransferSize -= iCount;
   }

   return err; 
}


/*===========================================================================

 

 USER-CALLABLE FUNCTIONS (Exportable Functions)   ---   END
 


===========================================================================*/




/*===========================================================================

 --- NON EXPORTED ---
 
 Auxiliary Functions

===========================================================================*/

/*===========================================================================
 Init helpers
===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Init Close
   Purpose:    This helper function closes the connection to the instrument
               and frees allocated memory.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViStatus stat:             Error code from caller, will be used as return
                              value when no additional error occurred
---------------------------------------------------------------------------*/
static ViStatus tlccs_initClose (ViSession vi, ViStatus stat)
{
   ViStatus       err   = VI_SUCCESS;
   ViSession      rm    = VI_NULL;
   tlccs_data_t   *data = VI_NULL;

   // Get resource manager session and private data pointer
   viGetAttribute(vi, VI_ATTR_RM_SESSION, &rm);
   viGetAttribute(vi, VI_ATTR_USER_DATA,  &data);

   // Free private data
   if(data != NULL)
   {
      free(data);
   }

   // Close sessions
   if(vi) err = viClose(vi);
   if(rm) viClose(rm);

   return ((stat != VI_SUCCESS) ? stat : err);
}


#ifdef RESERVED_FOR_LATER_USE
//==============================================================================
// DLL main entry-point functions

int __stdcall DllMain (HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
    switch (fdwReason) {
        case DLL_PROCESS_ATTACH:
            if (InitCVIRTE (hinstDLL, 0, 0) == 0)
                return 0;     /* out of memory */
            break;
        case DLL_PROCESS_DETACH:
            CloseCVIRTE ();
            break;
    }

    return 1;
}

int __stdcall DllEntryPoint (HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
    /* Included for compatibility with Borland */

    return DllMain (hinstDLL, fdwReason, lpvReserved);
}
#endif   // RESERVED_FOR_LATER_USE



/*---------------------------------------------------------------------------
   Function:   Check Error Level
   Purpose:    This helper function is intended to serve as a logging or
               debugging function during development. Here it just returns
               the given error.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViStatus code:             The error code to be checked
---------------------------------------------------------------------------*/
static ViStatus tlccs_checkErrorLevel(ViSession vi, ViStatus code)
{
   return (code);
}

/*---------------------------------------------------------------------------
   Function:   Acquire Raw Scan Data
   Purpose:    This helper function calculates the spectrum data from the
               raw data from the sensor.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViUInt16 raw[]:            Raw data from ADC. This  buffer's size has
                              to be TLCCS_NUM_RAW_PIXELS (3694)
   
   Output Parameters:
   ViReal64 data[]:           Processed and normed spectrum data. This
                              buffer's size has to be TLCCS_NUM_PIXELS (3648)
---------------------------------------------------------------------------*/
static ViStatus tlccs_aquireRawScanData(ViSession vi, ViUInt16 raw[], ViReal64 data[])
{
#define NO_DARK_PIXELS                 12       // we got 12 dark pixels
#define DARK_PIXELS_OFFSET             16       // dark pixels start at positon 16 within raw data
#define SCAN_PIXELS_OFFSET             32       // real measurement start at position 32 within raw data
#define MAX_ADC_VALUE                  0xFFFF   // this is full scale of a 16bit Analog Digital Converter (ADC)
#define DARK_LEVEL_THRESHOLD           (0.99)   // when dark level is above 99% f.s. of ADC mark scan as invalid (overexposed)
#define DARK_LEVEL_THRESHOLD_ADC       (DARK_LEVEL_THRESHOLD * (ViReal64)MAX_ADC_VALUE)

   tlccs_data_t    *ccs_data;
   ViStatus err = VI_SUCCESS;
   ViReal64 norm_com = 0.0;
   ViReal64 dark_com = 0.0;
   int i = 0;

   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &ccs_data)) != VI_SUCCESS) return err;

   // sum the dark Pixels
   for(i = 0; i < NO_DARK_PIXELS; i++)
   {
      dark_com += (double)raw[(DARK_PIXELS_OFFSET + i)];
   }

   // calculate dark current average
   dark_com /= (double)(NO_DARK_PIXELS);
 
   // when dark level is too high we assume an overexposure, set all data to 0.0 and return an error
   if(dark_com > DARK_LEVEL_THRESHOLD_ADC)
   {
      for(i = 0; i < TLCCS_NUM_PIXELS; i++)
      {
         data[i] = 0.0;
      }
      return VI_ERROR_SCAN_DATA_INVALID;
   }

   // calculate normalizing factor
   norm_com = 1.0 / ((ViReal64)MAX_ADC_VALUE - dark_com);

   for(i = 0; i < TLCCS_NUM_PIXELS; i++)
   {
      data[i] = (((ViReal64)raw[SCAN_PIXELS_OFFSET + i]) - dark_com) * norm_com;
   }

   for(i = 0; i < TLCCS_NUM_PIXELS; i++)
   {
      // only correct datat that is within ADC range
      if(data[i] < 1.0)
      {
         data[i] *= ccs_data->factory_acor_cal.acor[i];
      }
   }

   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Get Wavelength Parameters
   Purpose:    This function reads the parameters necessary to calculate
               from pixels to wavelength and vice versa stored in EEPROM of
               the spectrometer and stores the values in the tlccs_data_t
               structure.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
---------------------------------------------------------------------------*/
static ViStatus tlccs_getWavelengthParameters (ViSession vi)
{
   tlccs_data_t   *data;
   ViStatus       err = VI_SUCCESS;

   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // set the factory calibration valid flag to false
   data->factory_cal.valid = 0;
   
   // read factory adjustment coefficients from EEPROM
   tlccs_readEEFactoryPoly(vi, data->factory_cal.poly);
   
   tlccs_poly2wlArray(&(data->factory_cal));
   
   // read user adjustment nodes from EEPROM and calculate coefficients and wavelength array
   data->user_cal.valid = 0;
   err = tlccs_readEEUserPoints(vi, data->user_points.user_cal_node_pixel, data->user_points.user_cal_node_wl, &(data->user_points.user_cal_node_cnt));
   if(err == VI_SUCCESS) err = tlccs_nodes2poly(data->user_points.user_cal_node_pixel, data->user_points.user_cal_node_wl, data->user_points.user_cal_node_cnt, data->user_cal.poly);
   if(err == VI_SUCCESS) err = tlccs_poly2wlArray(&(data->user_cal));
   if(err == VI_SUCCESS) data->user_cal.valid = 1;
                                        
   return VI_SUCCESS;   // errors ignored by intention
}




/*---------------------------------------------------------------------------
   Function:   Check Nodes
   Purpose:    This helper function checks the given pixel-wavelength pairs
               for monotonic increase or decrease and checks also that the
               number off given pairs is in correct range (4 ... 10)
  
   Input Parameters:
   ViInt32 pixel[]:           The array of pixels
   ViReal64 wl[]:             The array of wavelengths
   ViInt32 cnt:               The number of pixel-wavelength pairs to check
---------------------------------------------------------------------------*/
static ViStatus tlccs_checkNodes(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt)
{
   int      i;
   ViInt32  p;
   ViReal64 d;
   int      iDirectionFlagWl = 0;   // 1 means increasing, -1 means decreasing, 0 is an error
   int      iDirectionFlagPx = 0;   // 1 means increasing, -1 means decreasing, 0 is an error

   // check valid buffer length and determine target
   if((cnt < TLCCS_MIN_NUM_USR_ADJ) || (cnt > TLCCS_MAX_NUM_USR_ADJ)) return VI_ERROR_TLCCS_INV_USER_DATA;

   // check if values are decreasing
   if(wl[0] < wl[1])
   {
      iDirectionFlagWl = 1;   
   }
   else if(wl[0] > wl[1])  
   {
      iDirectionFlagWl = -1;  
   }
   else
      return VI_ERROR_TLCCS_INV_USER_DATA;
   
   
   if(pixel[0] < pixel[1])
   {
      iDirectionFlagPx = 1;   
   }
   else if(pixel[0] > pixel[1])  
   {
      iDirectionFlagPx = -1;  
   }
   else
      return VI_ERROR_TLCCS_INV_USER_DATA;
   
   
   // check pixel range
   if((pixel[0] < 0) || (pixel[cnt - 1] > (TLCCS_NUM_PIXELS - 1))) return VI_ERROR_TLCCS_INV_USER_DATA;
   
   // check wavelength range
   if(wl[0] <= 0.0)  return VI_ERROR_TLCCS_INV_USER_DATA; 
   
   // check monoton ascending wavelength and pixel values    
   p = pixel[0];
   d = wl[0];
   
   for(i = 1; i < cnt; i++)
   {
      // check increasing pixels...
      if(iDirectionFlagPx == 1)
      {
         if(pixel[i] <= p) return VI_ERROR_TLCCS_INV_USER_DATA;
      }
      else
      {
         if(pixel[i] >= p) return VI_ERROR_TLCCS_INV_USER_DATA;
      }
         
         
      if(iDirectionFlagWl == 1)  // increasing
      {
         if(wl[i] <= d) return VI_ERROR_TLCCS_INV_USER_DATA; 
      }
      else
      {
         if(wl[i] >= d) return VI_ERROR_TLCCS_INV_USER_DATA;
      }

      p = pixel[i];
      d = wl[i];
   } 
   
   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Nodes to Polynome
   Purpose:    Calculates polynome coefficients from user defined supporting
               points.
  
   Input Parameters:
   ViInt32 pixel[]:           The pixels of the pixel-wavelength pairs
   ViReal64 wl[]:             The wavelengths of the pixel-wavelength pairs
   ViInt32 cnt:               The number of pixel-wavelength pairs
   
   Output Parameters:
   ViReal64 poly[]:           The 4 polynomial coefficients
---------------------------------------------------------------------------*/
static ViStatus tlccs_nodes2poly(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt, ViReal64 poly[])
{
   if(LeastSquareInterpolation ((int *)pixel, (double *)wl, (int)cnt, (double *)poly)) return VI_ERROR_TLCCS_INV_USER_DATA;
   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Polynom to Wavelength Array
   Purpose:    Calculates a wavelength array from polynom coefficients.
               The poly array must contain 4 elements.
  
   Input Parameters:
   tlccs_wl_cal_t *wl         Pointer to a tlccs specific struct containing
                              the input polynomial coefficients as well as
                              the output wavelength array
---------------------------------------------------------------------------*/
static ViStatus tlccs_poly2wlArray(tlccs_wl_cal_t *wl)
{
   int      i;
   ViReal64 d = 0.0;
   int iDirectionFlag = 0; // 1 means increasing, -1 means decreasing, 0 is an error
   
   for (i = 0; i < TLCCS_NUM_PIXELS; i++)
      wl->wl[i] = wl->poly[0] + (double)i * (wl->poly[1] + (double)i * (wl->poly[2] + (double)i * wl->poly[3]));

   
   // check if values are decreasing
   if(wl->wl[0] < wl->wl[1])
   {
      iDirectionFlag = 1;  
   }
   else if(wl->wl[0] > wl->wl[1])  
   {
      iDirectionFlag = -1; 
   }
   else
      return VI_ERROR_TLCCS_INV_USER_DATA;
   
   
   d = wl->wl[0];
   for(i = 1; i < TLCCS_NUM_PIXELS; i++)
   {
      if(iDirectionFlag == 1) // increasing
      {
         if(wl->wl[i] <= d)   return VI_ERROR_TLCCS_INV_USER_DATA; 
      }
      else
      {
         if(wl->wl[i] >= d)   return VI_ERROR_TLCCS_INV_USER_DATA;
      }

      d = wl->wl[i];
   }
   
   if(iDirectionFlag == 1)
   {
      wl->min     = wl->poly[0];
      wl->max     = wl->wl[TLCCS_NUM_PIXELS - 1];
   }
   else
   {
      wl->min     = wl->wl[TLCCS_NUM_PIXELS - 1]; 
      wl->max     = wl->poly[0];
   }
   
   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Read EEPROM factory calibration data polynom coefficients
   Purpose:    This function reads the polynome coefficients necessary to 
               calculate from pixels to wavelength and vice versa stored in 
               the EEPROM of the CCS and stores the values in the 
               poly array. The poly array must contain 4 elements.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   
   Output Parameters:
   ViReal64 poly[]:           The 4 polynomial coefficients
---------------------------------------------------------------------------*/
static ViStatus tlccs_readEEFactoryPoly(ViSession vi, ViReal64 poly[])
{
   ViStatus       err = VI_SUCCESS;    
// tlccs_data_t    *data;
   ViUInt16       cnt = 0;

   // get private data
// if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = tlccs_readEEPROM(vi, (ViUInt16)EE_FACT_CAL_COEF_DATA, 0, (ViUInt16)EE_LENGTH_FACT_CAL_COEF_DATA, (ViBuf)poly, &cnt);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Write EEPROM factory calibration coefficients to EEPROM
   Purpose:    This function writes the factory calibration coefficients to the
               EEPROM of the CCS. The coefficients array must contain
               four elements.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViReal64 poly[]:           The 4 polynomial coefficients
---------------------------------------------------------------------------*/
static ViStatus tlccs_writeEEFactoryPoly(ViSession vi, ViReal64 poly[])
{
   ViStatus       err = VI_SUCCESS;    
// tlccs_data_t   *data;

   // get private data
// if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_FACT_CAL_COEF_DATA, 0, (ViUInt16)EE_LENGTH_FACT_CAL_COEF_DATA, (ViBuf)poly, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Reads EEPROM user calibration coefficients
   Purpose:    This function reads the user calibration coefficients to the
               EEPROM.
               The coefficients array must contain four elements.

   Input Parameters:
   ViSession vi:              Instrument Handle
   
   Output Parameters:
   ViReal64 poly[]:           The 4 polynomial coefficients
---------------------------------------------------------------------------*/
#ifdef NOT_NEEDED_UNTIL_NOW
static ViStatus tlccs_readEEUserPoly(ViSession vi, ViReal64 poly[])
{
   ViStatus       err = VI_SUCCESS;    
   ViUInt16       cnt = 0;

   // read from eeprom
   err = tlccs_readEEPROM(vi, EE_USER_CAL_COEF_DATA, 0, EE_LENGTH_USER_CAL_COEF_DATA, (ViBuf)poly, &cnt);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;
}
#endif   // NOT_NEEDED_UNTIL_NOW


/*---------------------------------------------------------------------------
   Function:   Write EEPROM user calibration coefficients to EEPROM
   Purpose:    This function writes the user calibration coefficients to the
               EEPROM of the CCS. The coefficients array must contain
               four elements.
  
   Input Parameters:
   ViSession vi:              Instrument Handle
   ViReal64 poly[]:           The 4 polynomial coefficients
---------------------------------------------------------------------------*/
static ViStatus tlccs_writeEEUserPoly(ViSession vi, ViReal64 poly[])
{
   ViStatus       err = VI_SUCCESS;    
   tlccs_data_t   *data;

   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_USER_CAL_COEF_DATA, 0, (ViUInt16)EE_LENGTH_USER_CAL_COEF_DATA, (ViBuf)poly, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Read EEPROM factory calibration flags
   Purpose:    This function reads the factory calibration flags from the
               EEPROM.

   Input Parameters:
   ViSession vi:              Instrument Handle
   
   Output Parameters:
   ViPUInt16 flag:            Flag indicating the validity of the factory
                              wavelength calibration
---------------------------------------------------------------------------*/
#ifdef NOT_NEEDED_UNTIL_NOW
static ViStatus tlccs_readEEFactoryFlag(ViSession vi, ViPUInt16 flag)
{
   ViStatus       err = VI_SUCCESS;    
   ViUInt16       cnt = 0;

   // read from eeprom
   err = tlccs_readEEPROM(vi, EE_FACT_CAL_COEF_FLAG, 0, EE_LENGTH_FACT_CAL_COEF_FLAG, (ViBuf)flag, &cnt);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;

}
#endif   // NOT_NEEDED_UNTIL_NOW


/*---------------------------------------------------------------------------
   Function:   Write EEPROM factory calibration flags
   Purpose:    This function writes the factory calibration flags to the
               EEPROM.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViUInt16 flag:             Flag indicating the validity of the user
                              wavelength calibration
---------------------------------------------------------------------------*/
static ViStatus tlccs_writeEEFactoryFlag(ViSession vi, ViUInt16 flag)
{
   ViStatus       err = VI_SUCCESS;    
// tlccs_data_t   *data;

   // get private data
// if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_FACT_CAL_COEF_FLAG, 0, (ViUInt16)EE_LENGTH_FACT_CAL_COEF_FLAG, (ViBuf)&flag, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Read EEPROM user calibration flags
   Purpose:    This function reads the factory calibration flags from the
               EEPROM.

   Input Parameters:
   ViSession vi:              Instrument Handle
   
   Output Parameters:
   ViPUInt16 flag:            Flag indicating the validity of the user
                              wavelength calibration
---------------------------------------------------------------------------*/
#ifdef NOT_NEEDED_UNTIL_NOW
static ViStatus tlccs_readEEUserFlag(ViSession vi, ViPUInt16 flag)
{
   ViStatus       err = VI_SUCCESS;    
   ViUInt16       cnt = 0;

   // write to eeprom
   err = tlccs_readEEPROM(vi, EE_USER_CAL_COEF_FLAG, 0, EE_LENGTH_USER_CAL_COEF_FLAG, (ViBuf)flag, &cnt);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;
}
#endif   // NOT_NEEDED_UNTIL_NOW


/*---------------------------------------------------------------------------
   Function:   Write EEPROM factory calibration flags
   Purpose:    This function writes the user calibration flags to the
               EEPROM.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViUInt16 flag:             Flag indicating the validity of the user
                              wavelength calibration
---------------------------------------------------------------------------*/
static ViStatus tlccs_writeEEUserFlag(ViSession vi, ViUInt16 flag)
{
   ViStatus       err = VI_SUCCESS;    
// tlccs_data_t   *data;

   // get private data
// if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // write to eeprom
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_USER_CAL_COEF_FLAG, 0, (ViUInt16)EE_LENGTH_USER_CAL_COEF_FLAG, (ViBuf)&flag, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Reads EEPROM user calibration nodes
   Purpose:    This function reads the user-defined pixel-wavelength pairs
               from the instrument's EEPROM.

   Input Parameters:
   ViSession vi:              Instrument Handle
   
   Output Parameters:
   ViInt32 pixel[]:           The pixels of the pixel-wavelength pairs
   ViReal64 wl[]:             The wavelengths of the pixel-wavelength pairs
   ViPUInt16 flag:            The number of pixel-wavelength pairs
---------------------------------------------------------------------------*/
static ViStatus tlccs_readEEUserPoints(ViSession vi, ViUInt32 pixel[], ViReal64 wl[], ViPUInt16 cnt)
{
   ViStatus       err = VI_SUCCESS;    
// tlccs_data_t    *data;
   ViUInt16       tmp = 0;
   ViUInt8        buf[EE_LENGTH_USER_CAL_POINTS_DATA];

   // get private data
// if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // reads user calibration points count from eeprom
   err = tlccs_readEEPROM(vi, (ViUInt16)EE_USER_CAL_POINTS_CNT, 0, (ViUInt16)EE_LENGTH_USER_CAL_POINTS_CNT, (ViBuf)cnt, &tmp);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   // reads to eeprom
   err = tlccs_readEEPROM(vi, (ViUInt16)EE_USER_CAL_POINTS_DATA, 0, (ViUInt16)EE_LENGTH_USER_CAL_POINTS_DATA, (ViBuf)buf, &tmp);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   // fill data into buffer
   memcpy(pixel, buf, (*cnt) * sizeof(ViUInt32));
   memcpy(wl, &buf[TLCCS_MAX_NUM_USR_ADJ * sizeof(ViUInt32)], (*cnt) * sizeof(ViReal64));
   
   return err;
}



/*---------------------------------------------------------------------------
   Function:   Write EEPROM user calibration nodes
   Purpose:    This function writes the user-defined pixel-wavelength pairs
               to the instruments EEPROM.
               The nodes array must contain at least TLCCS_MIN_NUM_USR_ADJ
               elements and a maximum of TLCCS_MAX_NUM_USR_ADJ elements.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViInt32 pixel[]:           The pixels of the pixel-wavelength pairs
   ViReal64 wl[]:             The wavelengths of the pixel-wavelength pairs
   ViUInt16 cnt:              The number of pixel-wavelength pairs
---------------------------------------------------------------------------*/
static ViStatus tlccs_writeEEUserPoints(ViSession vi, ViUInt32 pixel[], ViReal64 wl[], ViUInt16 cnt)
{
   ViStatus       err = VI_SUCCESS;    
// tlccs_data_t    *data;
   ViUInt8        buf[EE_LENGTH_USER_CAL_POINTS_DATA];

   // get private data
// if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // write user calibration points count to eeprom
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_USER_CAL_POINTS_CNT, 0, (ViUInt16)EE_LENGTH_USER_CAL_POINTS_CNT, (ViBuf)&cnt, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   // fill data into buffer
   memcpy(buf, pixel, cnt * sizeof(ViUInt32));
   memcpy(&buf[TLCCS_MAX_NUM_USR_ADJ * sizeof(ViUInt32)], wl, cnt * sizeof(ViReal64));

   // write to eeprom
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_USER_CAL_POINTS_DATA, 0, (ViUInt16)EE_LENGTH_USER_CAL_POINTS_DATA, (ViBuf)&buf, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Gets the firmware revision
   Purpose:    This function requests the firmware version and stores the
               information into device structure.

   Input Parameters:
   ViSession vi:              Instrument Handle
---------------------------------------------------------------------------*/
static ViStatus tlccs_getFirmwareRevision(ViSession vi)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViUInt8 buf[TLCCS_NUM_VERSION_BYTES];
   tlccs_data_t    *data; 
   
   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // request the data
   err = tlccs_USB_in(vi, TLCCS_RCMD_PRODUCT_INFO, TLCCS_FIRMWARE_VERSION, 0, (ViUInt16)(TLCCS_NUM_VERSION_BYTES * sizeof(ViUInt8)), (ViBuf)buf, &read_bytes);
   
   // error mapping
   if((read_bytes != (TLCCS_NUM_VERSION_BYTES * sizeof(ViUInt8))) && (!err))   err = VI_ERROR_TLCCS_READ_INCOMPLETE; 
   
   // check for errors
   if(err = tlccs_checkErrorLevel(vi, err)) return (err);
   
   // decode the response
   data->firmware_version.major = buf[0];
   data->firmware_version.minor = buf[1]; 
   data->firmware_version.subminor = buf[2]; 
   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Gets the hardware revision
   Purpose:    This function requests the hardware version and stores the
               information into device structure.

   Input Parameters:
   ViSession vi:              Instrument Handle
---------------------------------------------------------------------------*/
static ViStatus tlccs_getHardwareRevision(ViSession vi)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViUInt8 buf[TLCCS_NUM_VERSION_BYTES];
   tlccs_data_t    *data; 
   
   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // request the data
   err = tlccs_USB_in(vi, TLCCS_RCMD_PRODUCT_INFO, TLCCS_HARDWARE_VERSION, 0, (ViUInt16)(TLCCS_NUM_VERSION_BYTES * sizeof(ViUInt8)), (ViBuf)buf, &read_bytes);
   
   // error mapping
   if((read_bytes != (TLCCS_NUM_VERSION_BYTES * sizeof(ViUInt8))) && (!err))   err = VI_ERROR_TLCCS_READ_INCOMPLETE; 
   
   // check for errors
   if(err = tlccs_checkErrorLevel(vi, err)) return (err);
   
   // decode the response
   data->hardware_version.major = buf[0];
   data->hardware_version.minor = buf[1]; 
   data->hardware_version.subminor = buf[2]; 
   
   return err;  
}

/*---------------------------------------------------------------------------
   Function:   Gets the amplitude correction factor
   Purpose:    This function requests an array of amplitude correction
               factors from the device, each pixel has its own factor
               when the factors cannot be read out the array will be set 
               to all 1es, 

   Input Parameters:
   ViSession vi:              Instrument Handle
---------------------------------------------------------------------------*/
static ViStatus tlccs_getAmplitudeCorrection(ViSession vi)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViUInt16 checksums[2];
   tlccs_data_t    *data; 
   int chkchk_corrupted_flag = 0;

   
   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // get checksums for amplitude factors
   err = tlccs_readEEPROM(vi, (ViUInt16)EE_CHECKSUMS, 0, (ViUInt16)EE_LENGTH_CHECKSUMS, (ViBuf)checksums, &read_bytes);
   if(err == VI_ERROR_CYEEPROM_CHKSUM)
   {
      // when we have a checksum error here assume that the CCS is a pre ver. 1.9.0 version
      // --> we need to update it
      chkchk_corrupted_flag = 1;
      checksums[TLCCS_ACOR_USER]    = 0;
      checksums[TLCCS_ACOR_FACTORY] = 0;
      err = VI_SUCCESS;
   }

   // get the amplitude factors itself, they me be already written anew when the following functions are finished
   if(!err) err = tlccs_getAmplitudeCorrectionArray(vi, data, TLCCS_ACOR_FACTORY);
   if(!err) err = tlccs_getAmplitudeCorrectionArray(vi, data, TLCCS_ACOR_USER);

   if(!err)
   {
      // we have no serious error so now look at the data read out
      if(chkchk_corrupted_flag)
      {
         // we assume an old CCS and we set the chk-chksum to be 1st != 2nd checksum
         // to indicate that there is no THORLABS amplitude corraction available
         
         data->cal_mode = TLCCS_CAL_MODE_USER;
         // when the chksums of factroy or user acor_cal are zero by incident make 2nd checksums to 0xFFFF
         if(data->factory_acor_cal.chksum == 0) checksums[TLCCS_ACOR_FACTORY] = 0xFFFF;
         if(data->user_acor_cal.chksum    == 0) checksums[TLCCS_ACOR_USER]    = 0xFFFF;
         
         // now write second checksums to EEPROM
         err = tlccs_writeEEPROM(vi, (ViUInt16)EE_CHECKSUMS, 0, (ViUInt16)EE_LENGTH_CHECKSUMS, (ViBuf)checksums, NULL);
      }
      else
      {
          // now we assume a new CCS, let's check wheter it has USER or THORLABS amplitude correction
      
         if(checksums[TLCCS_ACOR_USER] == data->user_acor_cal.chksum)
         {
            // this is the case when everything is OK and the data is from THORLABS
            data->cal_mode = TLCCS_CAL_MODE_THORLABS;
         }
         else
         {
            data->cal_mode = TLCCS_CAL_MODE_USER;
         }
      }
      
      // check if factory data is OK is not necessary here, it FACTORY data by definition
   }

   

   // check for errors
   err = tlccs_checkErrorLevel(vi, err);
   return err;  
}



/*---------------------------------------------------------------------------
   Function:   Gets one array of amplitude correction factors
   Purpose:    This function requests an array of amplitude correction
               factors from the device, each pixel has its own factor
               when the factors cannot be read out the array will be set 
               to all 1es, 

   Input Parameters:
   ViSession vi:              Instrument Handle
   tlccs_data_t *data:        Pointer to a tlccs specific struct containing
                              among others the amplitude correction factors
   int what:                  Selection for user or factory with macros
                              TLCCS_ACOR_FACTORY, TLCCS_ACOR_USER
---------------------------------------------------------------------------*/
static ViStatus tlccs_getAmplitudeCorrectionArray(ViSession vi, tlccs_data_t *data, int what)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViUInt16 addr;
   ViUInt16 chksum;
   tlccs_acor_t *acor_data;
   int adj_corrupted_flag = 0;
   int i;

   switch(what)
   {
      case TLCCS_ACOR_FACTORY  :
            addr = (ViUInt16)EE_ACOR_FACTORY;
            acor_data = &(data->factory_acor_cal);
            break;

//    case TLCCS_ACOR_USER     :
      default                       :
            addr = (ViUInt16)EE_ACOR_USER;
            acor_data = &(data->user_acor_cal);
            break;
   }

   // request the data for factory amplitude factors
   err = tlccs_readEEPROM_wo_crc(vi, addr, 0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)acor_data->acor, &read_bytes);

   // error mapping
   if((read_bytes != EE_LENGTH_ACOR) && (!err))   err = VI_ERROR_TLCCS_READ_INCOMPLETE; 

   if(!err)
   {
      chksum = crc16_block((void*)acor_data->acor, EE_LENGTH_ACOR);
      // read in the checksum for the data   
      if(err = tlccs_readEEPROM_wo_crc(vi, (ViUInt16)(addr+EE_LENGTH_ACOR), 0, (ViUInt16)sizeof(uint16_t), (ViBuf)&(acor_data->chksum), &read_bytes)) return err;
      if(chksum != acor_data->chksum)    err = VI_ERROR_CYEEPROM_CHKSUM;
   } 

   
   switch(err)
   {
      case VI_ERROR_TLCCS_READ_INCOMPLETE:
      case VI_ERROR_CYEEPROM_CHKSUM:
            // on expected or recoverable errors, generate a default array and write it to device
            adj_corrupted_flag = 1;
            break;

      case VI_SUCCESS:
            // on succes check if the data is consistent
            for(i=0; i<TLCCS_NUM_PIXELS; i++)
            {
               if(acor_data->acor[i] < TLCCS_AMP_CORR_FACT_MIN)
               {
                  acor_data->acor[i] = TLCCS_AMP_CORR_FACT_MIN;
                  adj_corrupted_flag = 1;
               }
               if(acor_data->acor[i] > TLCCS_AMP_CORR_FACT_MAX)
               {
                  acor_data->acor[i] = TLCCS_AMP_CORR_FACT_MAX;
                  adj_corrupted_flag = 1;
               }
            }
            break;

      default:
            break;
   }

   // try to recover in writing a correct array to EEPROM of CCS
   if(adj_corrupted_flag)
   {
      for(i=0; i<TLCCS_NUM_PIXELS; i++)
      {
         acor_data->acor[i] = 1.0;
      }
      // this writes new data WITH new checksum to EEPROM
      err = tlccs_writeEEPROM(vi, addr, 0, (ViUInt16)EE_LENGTH_ACOR, (ViBuf)acor_data->acor, &(acor_data->chksum));
   }

   return err;  
}



/*---------------------------------------------------------------------------
   Function:   Set Dark Current Offset
   Purpose:    This function writes the dark current values for even and
               odd pixels to EEPROM of the spectrometer and stores
               the values in the tlccs_data_t structure.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViUInt16 evenOffset:       The offset in ADC bits for even pixels
   ViUInt16 oddOffset:        The offset in ADC bits for odd pixels
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_setDarkCurrentOffset(ViSession vi, ViUInt16 evenOffset, ViUInt16 oddOffset)
{
   ViStatus err = VI_SUCCESS;
   tlccs_data_t    *data;
   
   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // writes user even offset
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_EVEN_OFFSET_MAX, 0, EE_LENGTH_OFFSET_MAX, (ViBuf)&evenOffset, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   // store even offset
   data->evenOffsetMax = evenOffset;
   
   // writes user even offset
   err = tlccs_writeEEPROM(vi, (ViUInt16)EE_ODD_OFFSET_MAX, 0, EE_LENGTH_OFFSET_MAX, (ViBuf)&oddOffset, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   // store even offset
   data->oddOffsetMax = oddOffset;
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get Dark Current Offset
   Purpose:    This function reads the dark current values for even and
               odd pixels stored in EEPROM of the spectrometer and stores
               the values in the tlccs_data_t structure.

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViPUInt16 evenOffset:      The offset in ADC bits for even pixels
   ViPUInt16 oddOffset:       The offset in ADC bits for odd pixels
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_getDarkCurrentOffset(ViSession vi, ViPUInt16 even, ViPUInt16 odd)
{
   ViStatus       err = VI_SUCCESS;    
   tlccs_data_t    *data;
   ViUInt16       cnt = 0;
   ViUInt16       tmp;

   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // reads user even offset
   err = tlccs_readEEPROM(vi, (ViUInt16)EE_EVEN_OFFSET_MAX, 0, EE_LENGTH_OFFSET_MAX, (ViBuf)&tmp, &cnt);
   
   // in case a checksum error occured we use default values
   if(err == VI_ERROR_CYEEPROM_CHKSUM)
   {
      tmp = 0xFFFF;
      err = VI_SUCCESS;
   }
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   // store even offset
   data->evenOffsetMax = tmp;
   
   // reads user odd offset
   err = tlccs_readEEPROM(vi, (ViUInt16)EE_ODD_OFFSET_MAX, 0, EE_LENGTH_OFFSET_MAX, (ViBuf)&tmp, &cnt);
   
   // in case a checksum error occured we use default values
   if(err == VI_ERROR_CYEEPROM_CHKSUM)
   {
      tmp = 0xFFFF;
      err = VI_SUCCESS;
   }
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   // store even offset
   data->oddOffsetMax = tmp;
   
   if(even) *even = data->evenOffsetMax;
   if(odd)  *odd = data->oddOffsetMax; 
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Set Serial Number
   Purpose:    This function stores the serial number to structure and eeprom.

   Input Parameters:
   ViSession vi:              Instrument Handle
   ViPChar serial:            The serial number (as a string)
---------------------------------------------------------------------------*/
__attribute__((visibility("default"))) ViStatus tlccs_setSerialNumber(ViSession vi, ViPChar serial)
{
   ViStatus       err = VI_SUCCESS;    
   tlccs_data_t    *data;
   char           buf[TLCCS_SERIAL_NO_LENGTH];

   // get private data
   if((err = viGetAttribute(vi, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // copy the string
   strncpy(buf, serial, (TLCCS_SERIAL_NO_LENGTH - 1)); // strncpy will fill with '\0' when userText is smaller than (USER_LABEL_LENGTH-1)
   
   // truncate 
   buf[TLCCS_SERIAL_NO_LENGTH-1] = '\0';

   // write to eeprom
   err = tlccs_writeEEPROM(vi, EE_SERIAL_NO, 0, TLCCS_SERIAL_NO_LENGTH, (ViBuf)buf, NULL);
   
   // check for error
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);
   
   strncpy(data->serNr, buf, sizeof(char) * TLCCS_SERIAL_NO_LENGTH);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get Raw Data
   Purpose:    This function reads out the raw scan data
               as 16Bit unsigned integers (as read from ADC).

   Input Parameters:
   ViSession vi:              Instrument Handle

   Output Parameters:
   ViUInt16 data[]:           The measurement array (TLCCS_NUM_RAW_PIXELS elements).
---------------------------------------------------------------------------*/
static ViStatus tlccs_getRawData(ViSession vi, ViUInt16 data[])
{
   ViStatus err         = VI_SUCCESS;
   ViUInt32 read_bytes  = 0;

   // read the raw scan data
   err = tlccs_USB_read(vi, (ViBuf)data, (ViUInt32)(TLCCS_NUM_RAW_PIXELS * sizeof(ViUInt16)), &read_bytes);
   
   // error mapping
   if((read_bytes != TLCCS_NUM_RAW_PIXELS * sizeof(ViUInt16)) & (!err)) err = VI_ERROR_TLCCS_READ_INCOMPLETE;
   
   // check for errors 
   if((err = tlccs_checkErrorLevel(vi, err)))  return (err);  
   
   return (err);
}



/*---------------------------------------------------------------------------
   Function:   CRC16 Update
   Purpose:    This function is an optimized CRC-16 calculation.
               (Derived from AVR-libc). This CRC is normally used in
               disk-drive controllers. 
               Polynomial: x^16 + x^15 + x^2 + 1 (0xa001)
               Initial value: 0xffff
   

   Input Parameters:
   uint16_t crc:              the old CRC16
   uint8_t a:                 The Byte to calculate the new CRC16

   Return Value:
   uint16_t:                  The new CRC16
---------------------------------------------------------------------------*/
static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
   int i;

   crc ^= a;
   for (i = 0; i < 8; ++i)
   {
      if (crc & 1)   crc = (crc >> 1) ^ 0xA001;
      else           crc = (crc >> 1);
   }
   return (crc);
}


/*---------------------------------------------------------------------------
   Function:   CRC16 Block
   Purpose:    This function is an optimized CRC-16 calculation.
               It calculates the CRC16 from a given block of data
               (See crc16_update for details)
   

   Input Parameters:
   const void *dat:           The data block start address
   size_t len:                The length of the data block

   Return Value:
   uint16_t:                  The CRC16 of the data block
---------------------------------------------------------------------------*/
static uint16_t crc16_block(const void *dat, size_t len)
{
   uint16_t crc = 0xFFFF;
   uint8_t  *ptr = (uint8_t*)dat;

   while(len)
   {
      crc = crc16_update(crc, *ptr);
      ptr ++;
      len --;
   }
   return (crc);
}


/*---------------------------------------------------------------------------
   Function:   Least Square Algorithm
   Purpose:    This function calculates the coeficients for a polynom of
               third grade to represent the given array as good as possible
               

   Input Parameters:
   int *PixelArray:           The pixels of the pixel-wavelength pairs
   double * WaveLengthArray:  The wavelengths of the pixel-wavelength pairs
   int iLength:               The number of pixel-wavelength pairs
   
   Output Parameters:
   double Coefficients[]      The four coefficients for the polynom

   Return Value:
   int:                       0 on success, negative value on error
---------------------------------------------------------------------------*/
static int LeastSquareInterpolation (int * PixelArray, double * WaveLengthArray, int iLength, double Coefficients[])
{
   double B[MATRIX_ROWS];
   double A[MATRIX_ROWS][MATRIX_COLS],C[MATRIX_ROWS][MATRIX_COLS];
   double D[MATRIX_ROWS][MATRIX_COLS],E[MATRIX_ROWS][MATRIX_COLS],F[MATRIX_ROWS][MATRIX_COLS];
   double detA;
   double CompleteWavelengthArray[TLCCS_NUM_PIXELS];
   int i;
   
   if(iLength < MATRIX_ROWS)
      return -1;
   
   // clear CompleteWavelengthArray
   memset(CompleteWavelengthArray, 0, sizeof(double) * TLCCS_NUM_PIXELS);
   
   // fill CompleteWavelengthArray with the supporting points
   for(i = 0; i < iLength; i++)
   {
      if((PixelArray[i] < 0) || (PixelArray[i] > TLCCS_NUM_PIXELS))   break;
      CompleteWavelengthArray[PixelArray[i]] = WaveLengthArray[i];   
   }

   
   A[0][0] = iLength;
   A[0][1] = SpecSummation(CompleteWavelengthArray,1,TLCCS_NUM_PIXELS);
   A[0][2] = SpecSummation(CompleteWavelengthArray,2,TLCCS_NUM_PIXELS);
   A[0][3] = SpecSummation(CompleteWavelengthArray,3,TLCCS_NUM_PIXELS);

   A[1][0] = SpecSummation(CompleteWavelengthArray,1,TLCCS_NUM_PIXELS);
   A[1][1] = SpecSummation(CompleteWavelengthArray,2,TLCCS_NUM_PIXELS);
   A[1][2] = SpecSummation(CompleteWavelengthArray,3,TLCCS_NUM_PIXELS);
   A[1][3] = SpecSummation(CompleteWavelengthArray,4,TLCCS_NUM_PIXELS);

   A[2][0] = SpecSummation(CompleteWavelengthArray,2,TLCCS_NUM_PIXELS);
   A[2][1] = SpecSummation(CompleteWavelengthArray,3,TLCCS_NUM_PIXELS);
   A[2][2] = SpecSummation(CompleteWavelengthArray,4,TLCCS_NUM_PIXELS);
   A[2][3] = SpecSummation(CompleteWavelengthArray,5,TLCCS_NUM_PIXELS);

   A[3][0] = SpecSummation(CompleteWavelengthArray,3,TLCCS_NUM_PIXELS);
   A[3][1] = SpecSummation(CompleteWavelengthArray,4,TLCCS_NUM_PIXELS);
   A[3][2] = SpecSummation(CompleteWavelengthArray,5,TLCCS_NUM_PIXELS);
   A[3][3] = SpecSummation(CompleteWavelengthArray,6,TLCCS_NUM_PIXELS);
   
   
   B[0] = Summation(CompleteWavelengthArray,TLCCS_NUM_PIXELS);
   B[1] = Spec2Summation(CompleteWavelengthArray,1,TLCCS_NUM_PIXELS);
   B[2] = Spec2Summation(CompleteWavelengthArray,2,TLCCS_NUM_PIXELS);
   B[3] = Spec2Summation(CompleteWavelengthArray,3,TLCCS_NUM_PIXELS);
   
   MergeMC(A,B,C,0);
   MergeMC(A,B,D,1);
   MergeMC(A,B,E,2);
   MergeMC(A,B,F,3);
   
   detA = Determinant(A);
   
   if (detA == 0.0){
      return -1;
   }

   Coefficients[0] = Determinant(C) / detA;
   Coefficients[1] = Determinant(D) / detA;
   Coefficients[2] = Determinant(E) / detA;
   Coefficients[3] = Determinant(F) / detA;
   
   return 0;
}

/*---------------------------------------------------------------------------
   Function:   Summation
   Purpose:    This function calculates a sum

   Input Parameters:
   double *pdata:             Pointer to array that shall be added up
   int values:                Number of values to be added up

   Return Value:
   double:                    The sum
---------------------------------------------------------------------------*/
static double Summation(double *pdata, int values)
{
   double tmp=0.0;

   do {
      tmp += *pdata;
      ++pdata;
   } while (--values);

   return(tmp);
}


/*---------------------------------------------------------------------------
   Function:   Special Summation
   Purpose:    This function calculates a 'special' sum of values

   Input Parameters:
   double *pcorrel:           Parameter needed for calculation
   int pwr:                   A internal Value will be raised to power of this parameter
   int values:                Number of values to be added up

   Return Value:
   double:                    The sum
---------------------------------------------------------------------------*/
static double SpecSummation(double *pcorrel,int pwr,int values)
{
   double tmp=0.0;
   int pixel=0;

   do {
      if (*pcorrel > 0.0) tmp += pow((double)pixel,(double)pwr);
      ++pcorrel;
      ++pixel;
   } while (--values);

   return(tmp);
}

/*---------------------------------------------------------------------------
   Function:   Special Summation II
   Purpose:    This function calculates an other 'special' sum of values

   Input Parameters:
   double *pcorrel:           Parameter needed for calculation
   int pwr:                   A internal Value will be raised to power of this parameter
   int values:                Number of values to be added up

   Return Value:
   double:                    The sum
---------------------------------------------------------------------------*/
static double Spec2Summation(double *pcorrel,int pwr,int values)
{
   double tmp=0.0;
   int pixel=0;

   do {
      if (*pcorrel > 0.0) tmp += *pcorrel * pow((double)pixel,(double)pwr);
      ++pcorrel;
      ++pixel;
   } while (--values);

   return(tmp);
}


/*---------------------------------------------------------------------------
   Function:   Merge Matrices
   Purpose:    This function replaces a column in matrice

   Input Parameters:
   double s[MATRIX_ROWS][MATRIX_COLS]: Input matrice
   double i[MATRIX_ROWS]:              Input vector that shall replace a
                                       column of the input matrice
   int column:                         The column of the input matrice that
                                       will be replaced
                                       
   Ouput Parameters:
   double d[MATRIX_ROWS][MATRIX_COLS]: Output matrice
---------------------------------------------------------------------------*/
static void MergeMC(double s[MATRIX_ROWS][MATRIX_COLS], double i[MATRIX_ROWS], double d[MATRIX_ROWS][MATRIX_COLS], int column)
{
   int x,y;

   for (x = 0; x < MATRIX_COLS; x++)
   {
      if (x == column) 
      {
         for (y = 0; y < MATRIX_ROWS; y++) 
            d[y][x] = i[y];
      }
      else
      {
         for (y = 0; y < MATRIX_ROWS; y++)
            d[y][x] = s[y][x];
      }
   }
}



/*---------------------------------------------------------------------------
   Function:   Determinant
   Purpose:    This function calculates the determinant of a 4x4 matrice

   Input Parameters:
   double mt[MATRIX_ROWS][MATRIX_COLS]:   Input matrice

   Return Value:
   double:                                The determinant
---------------------------------------------------------------------------*/
static double Determinant(double mt[MATRIX_ROWS][MATRIX_COLS])
{
   double a=mt[0][0],b=mt[0][1],c=mt[0][2],d=mt[0][3];
   double e=mt[1][0],f=mt[1][1],g=mt[1][2],h=mt[1][3];
   double i=mt[2][0],j=mt[2][1],k=mt[2][2],l=mt[2][3];
   double m=mt[3][0],n=mt[3][1],o=mt[3][2],p=mt[3][3];
   double tmp=0.0;

   tmp =  a * f * k * p;
   tmp -= a * f * l * o;
   tmp -= a * j * g * p;
   tmp += a * j * h * o;
   tmp += a * n * g * l;
   tmp -= a * n * h * k;
   tmp -= e * b * k * p;
   tmp += e * b * l * o;
   tmp += e * j * c * p;
   tmp -= e * j * d * o;
   tmp -= e * n * c * l;
   tmp += e * n * d * k;
   tmp += i * b * g * p;
   tmp -= i * b * h * o;
   tmp -= i * f * c * p;
   tmp += i * f * d * o;
   tmp += i * n * c * h;
   tmp -= i * n * d * g;
   tmp -= m * b * g * l;
   tmp += m * b * h * k;
   tmp += m * f * c * l;
   tmp -= m * f * d * k;
   tmp -= m * j * c * h;
   tmp += m * j * d * g;

   return(tmp);
}

/****************************************************************************

  End of Source file

****************************************************************************/
#endif   // DO_NOT_COMPILE_DRIVER_HERE

