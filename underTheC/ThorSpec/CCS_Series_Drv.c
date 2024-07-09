/****************************************************************************

   Thorlabs CCS Series Spectrometer - VISA instrument driver

   FOR DETAILED DESCRIPTION OF THE DRIVER FUNCTIONS SEE THE ONLINE HELP FILE
   AND THE PROGRAMMERS REFERENCE MANUAL.

   Copyright:  Copyright(c) 2008-2011 Thorlabs (www.thorlabs.com)
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

   Date:          May-05-2011
   Built with:    NI LabWindows/CVI 2010
   Version:       1.7.0

   Changelog:     see 'readme.rtf'

****************************************************************************/
/****   MSC  minimal changes to compile in Linux without NI-VISA   ****/

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
#include "vitypes.h"
#include "CCS_Series_Drv.h"
#include "spxusb.h"

/*===========================================================================

   VERSION INFORMATION

===========================================================================*/

#define CCS_SERIES_VER_MAJOR      1
#define CCS_SERIES_VER_MINOR      7
#define CCS_SERIES_VER_SUBMINOR   0

#define CCS_SERIES_MAKE_REVISION(major, minor, subminor)  (((major & 0x00000FFF) << 20) | ((minor & 0x00000FFF) << 8) | (subminor & 0x000000FF))

#define CCS_SERIES_VERSION                               CCS_SERIES_MAKE_REVISION(CCS_SERIES_VER_MAJOR, CCS_SERIES_VER_MINOR, CCS_SERIES_VER_SUBMINOR)


/*===========================================================================
 
   Macros
 
===========================================================================*/
// Resource locking
#ifdef _CVI_DEBUG_
   // We are in a debugging session - do not lock resource
   #define CCS_SERIES_LOCK_STATE            VI_NULL
#else
   #define CCS_SERIES_LOCK_STATE          VI_EXCLUSIVE_LOCK
   //#define CCS_SERIES_LOCK_STATE           VI_NULL
#endif

// Buffers
#define CCS_SERIES_SERIAL_NO_LENGTH          24
#define CCS_SERIES_NUM_POLY_POINTS           4
#define CCS_SERIES_NUM_INTEG_CTRL_BYTES      6
#define CCS_SERIES_NUM_VERSION_BYTES         3

// Version
//#define CCS_SERIES_DRIVER_REVISION_TXT   "0.1"    // Instrument driver revision
#define CCS_SERIES_UNKNOWN_REVISION_TXT    "unknown"

#define CCS_SERIES_NOT_AVAILABLE          "n/a"

#define CCS_SERIES_FIRMWARE_VERSION       0
#define CCS_SERIES_HARDWARE_VERSION       1

// Error query mode
#define CCS_SERIES_DEFAULT_ERR_QUERY_MODE  VI_ON

// Range checking
#define INVAL_RANGE(val, min, max)  ( ((val) < (min)) || ((val) > (max)) )

// Logging
#define DEBUG_BUF_SIZE              512

// Macros for Cypress USB chip
#define ENDPOINT_0_TRANSFERSIZE     64          // this is the max. size of bytes that can be transferred at once for Endpoint 0
//#define MAX_USB_CTRL_TRANSFER_SIZE  4096        // this is the absolute maximum size for a USB control transfer size

// Analysis 
#define MATRIX_ROWS  4
#define MATRIX_COLS  4
   
/*===========================================================================
   EEPROM mapping
===========================================================================*/
// a 256kBit = 32kB EEPROM is used here   
#define EE_LENGTH_SERIAL_NO               ( sizeof(ViChar)                       * CCS_SERIES_SERIAL_NO_LENGTH    )
#define EE_LENGTH_SW_VERSION              (4                                                                      )
#define EE_LENGTH_USER_LABEL              ( sizeof(ViChar)                       * CCS_SERIES_MAX_USER_NAME_SIZE  )  
#define EE_LENGTH_FACT_CAL_COEF_FLAG      (2                                                                      )
#define EE_LENGTH_FACT_CAL_COEF_DATA      ( sizeof(ViReal64)                     * CCS_SERIES_NUM_POLY_POINTS     )
#define EE_LENGTH_USER_CAL_COEF_FLAG      (2                                                                      )
#define EE_LENGTH_USER_CAL_COEF_DATA      ( sizeof(ViReal64)                     * CCS_SERIES_NUM_POLY_POINTS     )
#define EE_LENGTH_USER_CAL_POINTS_CNT     (2                                                                      )
#define EE_LENGTH_USER_CAL_POINTS_DATA    ((sizeof(ViInt32) + sizeof(ViReal64))  * CCS_SERIES_MAX_NUM_USR_ADJ     )
#define EE_LENGTH_OFFSET_MAX              (2                                                                      ) 
#define EE_LENGTH_ACOR                    ( sizeof(ViReal32)                     * CCS_SERIES_NUM_PIXELS          )
   
// eeprom sizes
#define EE_SIZE_CHECKSUM               (2)
#define EE_SIZE_BOOT_CODE              (1)
#define EE_SIZE_SERIAL_NO              (EE_LENGTH_SERIAL_NO)         
#define EE_SIZE_SW_VERSION             (EE_LENGTH_SW_VERSION            + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_LABEL             (EE_LENGTH_USER_LABEL            + EE_SIZE_CHECKSUM)
#define EE_SIZE_FACT_CAL_COEF_FLAG     (EE_LENGTH_FACT_CAL_COEF_FLAG    + EE_SIZE_CHECKSUM)
#define EE_SIZE_FACT_CAL_COEF_DATA     (EE_LENGTH_FACT_CAL_COEF_DATA    + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_CAL_COEF_FLAG     (EE_LENGTH_USER_CAL_COEF_FLAG    + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_CAL_COEF_DATA     (EE_LENGTH_USER_CAL_COEF_DATA    + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_CAL_POINTS_CNT    (EE_LENGTH_USER_CAL_POINTS_CNT   + EE_SIZE_CHECKSUM)
#define EE_SIZE_USER_CAL_POINTS_DATA   (EE_LENGTH_USER_CAL_POINTS_DATA  + EE_SIZE_CHECKSUM)
#define EE_SIZE_OFFSET_MAX             (EE_LENGTH_OFFSET_MAX            + EE_SIZE_CHECKSUM)
#define EE_SIZE_ACOR                   (EE_LENGTH_ACOR                  + EE_SIZE_CHECKSUM)

// eeprom adresses
#define EE_BOOT_CODE                   0
#define EE_VENDOR_ID                   1
#define EE_PRODUCT_ID                  3
#define EE_DEVICE_ID                   5
#define EE_SERIAL_NO                   8
#define EE_SW_VERSION                  (EE_SERIAL_NO              + EE_SIZE_SERIAL_NO              )  // software version
#define EE_USER_LABEL                  (EE_SW_VERSION             + EE_SIZE_SW_VERSION             )  // user label
#define EE_FACT_CAL_COEF_FLAG          (EE_USER_LABEL             + EE_SIZE_USER_LABEL             )  // factory calibration flags
#define EE_FACT_CAL_COEF_DATA          (EE_FACT_CAL_COEF_FLAG     + EE_SIZE_FACT_CAL_COEF_FLAG     )  // factory calibration coefficients
#define EE_USER_CAL_COEF_FLAG          (EE_FACT_CAL_COEF_DATA     + EE_SIZE_FACT_CAL_COEF_DATA     )  // user calibration flags
#define EE_USER_CAL_COEF_DATA          (EE_USER_CAL_COEF_FLAG     + EE_SIZE_USER_CAL_COEF_FLAG     )  // user calibration coefficients
#define EE_USER_CAL_POINTS_CNT         (EE_USER_CAL_COEF_DATA     + EE_SIZE_USER_CAL_COEF_DATA     )  // user calibration points count
#define EE_USER_CAL_POINTS_DATA        (EE_USER_CAL_POINTS_CNT    + EE_SIZE_USER_CAL_POINTS_CNT    )  // user calibration points
#define EE_EVEN_OFFSET_MAX             (EE_USER_CAL_POINTS_DATA   + EE_SIZE_USER_CAL_POINTS_DATA   )  // even offset max
#define EE_ODD_OFFSET_MAX              (EE_EVEN_OFFSET_MAX        + EE_SIZE_OFFSET_MAX             )  // odd offset max
#define EE_ACOR_FACTORY                (EE_ODD_OFFSET_MAX         + EE_SIZE_OFFSET_MAX             )  // amplitude correction, factory setting
#define EE_ACOR_USER                   (EE_ACOR_FACTORY           + EE_SIZE_ACOR                    )  // amplitude correction, factory setting
#define EE_FREE                        (EE_ACOR_USER              + EE_SIZE_ACOR                   )  // free memory 

// Macros for error sources
#define CCS_SERIES_ERR_SRC_EEPROM          1
#define CCS_SERIES_ERR_SRC_FPGA            2
#define CCS_SERIES_ERR_SRC_FIRMWARE        3

// Macros for CCS_SERIES commands
// reflect commands found in CCS_SERIES_cmds.h of 8051 firmware project 08010
//    WRITE commands (OUT)
//#define CCS_SERIES_WCMD_LOAD_FPGA            0x10
//#define CCS_SERIES_WCMD_SET_VID_PID          0x11
//#define CCS_SERIES_WCMD_SET_CONFIG           0x12
//#define CCS_SERIES_WCMD_WRITE_I2C            0x20
#define CCS_SERIES_WCMD_WRITE_EEPROM         0x21
#define CCS_SERIES_WCMD_INTEGRATION_TIME     0x23
#define CCS_SERIES_WCMD_MODUS                0x24
#define CCS_SERIES_WCMD_RESET                0x26
//#define CCS_SERIES_WCMD_ALWAYS_STALL         0x2F

//    READ commands (IN)
//#define CCS_SERIES_RCMD_READ_I2C             0x20
#define CCS_SERIES_RCMD_READ_EEPROM          0x21
#define CCS_SERIES_RCMD_INTEGRATION_TIME     0x23
//#define CCS_SERIES_RCMD_MODUS                0x24
#define CCS_SERIES_RCMD_PRODUCT_INFO         0x25
//#define CCS_SERIES_RCMD_ALWAYS_STALL         0x2F
#define CCS_SERIES_RCMD_GET_STATUS           0x30
#define CCS_SERIES_RCMD_GET_ERROR            0xFF

// Operation Modes
#define MODUS_INTERN_SINGLE_SHOT             0
#define MODUS_INTERN_CONTINUOUS              1
#define MODUS_EXTERN_SINGLE_SHOT             2
#define MODUS_EXTERN_CONTINUOUS              3

#define CCS_SERIES_CALIB_VALID_FLAG          0x5A     // this is the value for check bytes
#define CCS_SERIES_USERCAL_VALID_FLAG        0x5A     // user wavelenght adjustment data is valid

// strings
#define DEFAULT_USER_TEXT                    "My CCS Spectrometer"
   
/*===========================================================================
 Structures
===========================================================================*/
// static error list
typedef struct
{
   ViStatus err;
   ViString descr;
} CCS_SERIES_errDescrStat_t;


typedef struct
{
   ViUInt16       user_cal_node_cnt;                        // number of user-defined supporting points
   ViUInt32       user_cal_node_pixel[CCS_SERIES_MAX_NUM_USR_ADJ]; //   pixel array of supporting points
   ViReal64       user_cal_node_wl[CCS_SERIES_MAX_NUM_USR_ADJ];      // wavelength array of supporting points
   
} CCS_SERIES_usr_cal_pts_t; 


typedef struct
{
   ViReal64       poly[CCS_SERIES_NUM_POLY_POINTS];   // polynomial coefficients for pixel - wavelength computation
   ViReal64       min;                                // lower wavelength limit
   ViReal64       max;
   ViReal64       wl[CCS_SERIES_NUM_PIXELS];          // array of wavelengths according to pixel number
   ViUInt16       valid;                              // valid flag
} CCS_SERIES_wl_cal_t;

typedef struct
{
   ViReal32       acor[CCS_SERIES_NUM_PIXELS];        // array of wavelengths according to pixel number
   ViUInt16       valid;                              // valid flag
} CCS_SERIES_acor_t;


typedef struct
{
   ViUInt8        major;
   ViUInt8        minor;                        
   ViUInt8        subminor;
} CCS_SERIES_version_t; 


// driver private data
typedef struct
{
   // common data
   ViSession                  instr;      // instrument handle
   ViBoolean                  reset;      // reset instrument
   ViBoolean                  errQuery;   // TRUE - query instruments error queue on every access
   ViUInt32                   timeout;    // given communication timeout in ms
   ViUInt16                   pid;        // USB PID value. used to determine the instrument type
   ViUInt16                   vid;        // USB VID value, used to determine the manufacturer
   
   // device information
   ViChar                     name[CCS_SERIES_BUFFER_SIZE];
   ViChar                     serNr[CCS_SERIES_BUFFER_SIZE];
   ViChar                     manu[CCS_SERIES_BUFFER_SIZE];
   ViChar                     firm[CCS_SERIES_BUFFER_SIZE];
   ViChar                     instrDrv[CCS_SERIES_BUFFER_SIZE];
   
   // device settings
   ViReal64                   intTime;
   ViUInt16                   evenOffsetMax;
   ViUInt16                   oddOffsetMax;
   
   // device calibration
   CCS_SERIES_wl_cal_t        factory_cal;
   CCS_SERIES_wl_cal_t        user_cal;
   CCS_SERIES_usr_cal_pts_t   user_points;
   
   CCS_SERIES_acor_t          factory_acor_cal;
   CCS_SERIES_acor_t          user_acor_cal;
   
   // version
   CCS_SERIES_version_t       firmware_version;
   CCS_SERIES_version_t       hardware_version;


} CCS_SERIES_data_t;


/*===========================================================================
 Constants
===========================================================================*/
/*---------------------------------------------------------------------------
 Static error descriptions
---------------------------------------------------------------------------*/
static const CCS_SERIES_errDescrStat_t CCS_SERIES_errDescrStat[] =
{
   {VI_ERROR_PARAMETER1,                     "Parameter 1 out of range"                         },
   {VI_ERROR_PARAMETER2,                     "Parameter 2 out of range"                         },
   {VI_ERROR_PARAMETER3,                     "Parameter 3 out of range"                         },
   {VI_ERROR_PARAMETER4,                     "Parameter 4 out of range"                         },
   {VI_ERROR_PARAMETER5,                     "Parameter 5 out of range"                         },
   {VI_ERROR_PARAMETER6,                     "Parameter 6 out of range"                         },
   {VI_ERROR_PARAMETER7,                     "Parameter 7 out of range"                         },
   {VI_ERROR_PARAMETER8,                     "Parameter 8 out of range"                         },
   {VI_ERROR_INV_RESPONSE,                   "Errors occured interpreting instrument's response"},
   
   {VI_ERROR_NSUP_COMMAND,                   "Command not supported by instrument"              },
   {VI_ERROR_CCS_SERIES_UNKNOWN,                 "Unknown CCS_SERIES error, please report this"         },
   {VI_ERROR_XSVF_SIZE,                      "XSVF stream size must be greater 0"               },
   {VI_ERROR_XSVF_MEMORY,                    "Memory allocation for XSVF stream failed"         },
   {VI_ERROR_XSVF_FILE,                      "Access to XSVF file failed"                       },

   {VI_ERROR_FIRMWARE_SIZE,                  "Firmware size must be greater than 0"             },
   {VI_ERROR_FIRMWARE_MEMORY,                "Memory allocation for firmware data failed"       },
   {VI_ERROR_FIRMWARE_FILE,                  "Access to firmware file failed"                   },
   {VI_ERROR_FIRMWARE_CHKSUM,                "Checksum error in firmware HEX-File"              },
   {VI_ERROR_FIRMWARE_BUFOFL,                "Given buffer is to small for firmware HEX-File"   },

   {VI_ERROR_CYEEPROM_SIZE,                  "EEPROM size mismatch"                             },
   {VI_ERROR_CYEEPROM_MEMORY,                "Memory allocation for EEPROM data failed"         },
   {VI_ERROR_CYEEPROM_FILE,                  "Access to EEPROM file failed"                     },
   {VI_ERROR_CYEEPROM_CHKSUM,                "Checksum error in EEPROM"                         },
   {VI_ERROR_CYEEPROM_BUFOVL,                "Given buffer is to small for EEPROM HEX-File"     },


   {VI_ERROR_CCS_SERIES_ENDP0_SIZE,          "Attempt to send or receive more than MAX_EP0_TRANSACTION_SIZE (64) bytes at once over endpoint 0"},
   {VI_ERROR_CCS_SERIES_EEPROM_ADR_TO_BIG,   "Given EEPROM address is to big"                   },

   // CCS_SERIES XSVF errors copied from 'xsvf.c'
   {VI_ERROR_CCS_SERIES_XSVF_UNKNOWN,        "XSVF Error 1: unknown XSVF error"                 },
   {VI_ERROR_CCS_SERIES_XSVF_TDOMISMATCH,    "XSVF Error 2: TDO mismatch"                       },
   {VI_ERROR_CCS_SERIES_XSVF_MAXRETRIES,     "XSVF Error 3: TDO mismatch after max. retries"    },
   {VI_ERROR_CCS_SERIES_XSVF_ILLEGALCMD,     "XSVF Error 4: illegal XSVF command"               },
   {VI_ERROR_CCS_SERIES_XSVF_ILLEGALSTATE,   "XSVF Error 5: illegal TAP state"                  },
   {VI_ERROR_CCS_SERIES_XSVF_DATAOVERFLOW,   "XSVF Error 6: XSVF record length too big"         },
   {VI_ERROR_CCS_SERIES_I2C_NACK,            "CCS_SERIES I2C bus did not acknowledge"           },
   {VI_ERROR_CCS_SERIES_I2C_ERR,             "CCS_SERIES I2C bus error"                         },

   {VI_ERROR_CCS_SERIES_READ_INCOMPLETE,     "Data readout from spectrometer was incomplete"    },
   {VI_ERROR_CCS_SERIES_NO_USER_DATA,        "No user wavelength adjustment data available"     },
   {VI_ERROR_CCS_SERIES_INV_USER_DATA,       "Invalid user wavelength adjustment data"          },
   {VI_ERROR_CCS_SERIES_INV_ADJ_DATA,        "Adjustment data invalid/corrupt"                  },
   
   {0 , VI_NULL}  // termination

};

/*===========================================================================
 Global Variables
===========================================================================*/

/*===========================================================================



 USER-CALLABLE FUNCTIONS (Exportable Functions)   ---   START



===========================================================================*/

/*===========================================================================

 
 Init/close


===========================================================================*/

/*---------------------------------------------------------------------------
   Function:   Initialize
   Purpose:    This function initializes the instrument driver session and
               returns an instrument handle which is used in subsequent calls. 

   Parameters:
   
   ViRsrc resourceName:    The visa resource string.  
   ViBoolean IDQuery:      Boolean to query the ID or not.
   ViBoolean resetDevice:  Boolean to reset the device or not.
   ViPSession pInstr:      Pointer to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_init (ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice, ViSession *pInstr)
{
   ViStatus       err;
   ViSession      rm = VI_NULL;
   ViUInt16       vid, pid;
   CCS_SERIES_data_t     *data;
   
   //Open instrument session and set 'user data' to 'VI_NULL'
   *pInstr = VI_NULL;
   if((err = viOpenDefaultRM(&rm))) return (err);
   if((err = viOpen(rm, resourceName, CCS_SERIES_LOCK_STATE, VI_NULL, pInstr)))
   {
      viClose(rm);
      return (err);
   }
   if((err = viSetAttribute(*pInstr, VI_ATTR_USER_DATA, (ViAttrState)VI_NULL)))
   {
      viClose(*pInstr);
      viClose(rm);
      return (err);
   }
   
   // Is it a Thorlabs CCS_SERIES
   if((err = viGetAttribute(*pInstr, VI_ATTR_MANF_ID,    &vid)))                    return (CCSseries_initClose(*pInstr, err));
   if((err = viGetAttribute(*pInstr, VI_ATTR_MODEL_CODE, &pid)))                    return (CCSseries_initClose(*pInstr, err));
   if(vid != CCS_SERIES_VID)                                                        return (CCSseries_initClose(*pInstr, VI_ERROR_FAIL_ID_QUERY));
   if((pid != CCS100_PID) && (pid != CCS125_PID) && 
      (pid != CCS150_PID) && (pid != CCS175_PID) &&
      (pid != CCS200_PID))                                                          return (CCSseries_initClose(*pInstr, VI_ERROR_FAIL_ID_QUERY));

   // Communication buffers
   if((err = viFlush (*pInstr, VI_WRITE_BUF_DISCARD | VI_READ_BUF_DISCARD)))        return (CCSseries_initClose(*pInstr, err));

   // Configure Session
   if ((err = viSetAttribute(*pInstr, VI_ATTR_TMO_VALUE, CCS_SERIES_TIMEOUT_DEF)))  return (CCSseries_initClose(*pInstr, err));


   // Private driver data
   if((data = (CCS_SERIES_data_t*)malloc(sizeof(CCS_SERIES_data_t))) == NULL)       return (CCSseries_initClose(*pInstr, VI_ERROR_SYSTEM_ERROR));

   if((err = viSetAttribute(*pInstr, VI_ATTR_USER_DATA, (ViAttrState)data)))        return (CCSseries_initClose(*pInstr, err));
   data->instr    = *pInstr;
   data->reset    = resetDevice;
   data->errQuery = VI_OFF;   // turn off auto-error-query
   data->pid      = pid;
   data->vid      = vid;
   data->timeout  = CCS_SERIES_TIMEOUT_DEF;

   viGetAttribute(*pInstr, VI_ATTR_MODEL_NAME,     data->name);
   viGetAttribute(*pInstr, VI_ATTR_MANF_NAME,      data->manu);
   viGetAttribute(*pInstr, VI_ATTR_USB_SERIAL_NUM, data->serNr);
   
   // spx driver does this, and spxusb needs it
   viSetAttribute(*pInstr, VI_ATTR_USB_BULK_IN_PIPE,  0x86);  // verified, only othe possibility is 88
   viSetAttribute(*pInstr, VI_ATTR_USB_BULK_OUT_PIPE,  0x02); // not tested yet, could be 4
   
   // Reset device
   if((err = viClear(*pInstr))) return (CCSseries_initClose(*pInstr, err));

   // Error query
   data->errQuery = CCS_SERIES_DEFAULT_ERR_QUERY_MODE;
   
   // Configure device
   // set the default integration time
   if((err = CCSseries_setIntegrationTime (*pInstr, CCS_SERIES_DEF_INT_TIME))) return CCSseries_initClose(*pInstr, err);

   // get wavelength to pixel calculation parameters
   if((err = CCSseries_getWavelengthParameters (*pInstr))) return CCSseries_initClose(*pInstr, err);

   // get dark current offset values
   if((err = CCSseries_getDarkCurrentOffset (*pInstr, VI_NULL, VI_NULL))) return CCSseries_initClose(*pInstr, err);

   // get firmware revision
   if((err = CCSseries_getFirmwareRevision (*pInstr))) return CCSseries_initClose(*pInstr, err);

   // get hardware revision
   if((err = CCSseries_getHardwareRevision (*pInstr))) return CCSseries_initClose(*pInstr, err);
   
   // get amplitude correction
   if((err = CCSseries_getAmplitudeCorrection (*pInstr))) return CCSseries_initClose(*pInstr, err);
   
   //Ready
   return (VI_SUCCESS);
}


/*---------------------------------------------------------------------------
   Function:   Close
   Purpose:    This function close an instrument driver session. 

   Parameters:
   
   ViSession instr:     The session to close.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_close (ViSession instr)
{
   return (CCSseries_initClose(instr, VI_SUCCESS));
}


/*===========================================================================


 Class: Configuration Functions.


===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Set Integration Time
   Purpose:    This function set the optical integration time in seconds. 

   Parameters:
   
   ViSession instr:           The actual session to opened device.
   ViReal64 integrationTime:  The optical integration time. 
                              Min: CCS_SERIES_MIN_INT_TIME (1.0E-5)
                              Max: CCS_SERIES_MAX_INT_TIME (6.0E+1)
                              Default value: 1.0E-3. 
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_setIntegrationTime (ViSession instrumentHandle, ViReal64 integrationTime) 
{
   ViStatus err = VI_SUCCESS;
   ViUInt8  data[CCS_SERIES_NUM_INTEG_CTRL_BYTES];
   ViInt32 integ = 0;
   ViInt32 presc = 0;
   ViInt32 fill = 0;

   // check for time range
   if((integrationTime < CCS_SERIES_MIN_INT_TIME) || (integrationTime > CCS_SERIES_MAX_INT_TIME)) return VI_ERROR_PARAMETER2;
   
   // convert the integration time from seconds to micro seconds
   integ = integrationTime * 1000000;
   
   // calculate the prescaler value
   presc = ((log10(integ))/(log10(2))) - 11;
   if(presc < 0)  presc = 0;
   
   // calculate the filling value
   if(integ <= 3800) 
      fill = (3800 - integ + 1 + (integ % 2));
   else
      fill = 0;
   
   // calculate the integration time
   integ = (int) ((integ / pow(2.0, (double)presc) ) - 8 + fill);
   
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
   err = CCSseries_USB_out(instrumentHandle, CCS_SERIES_WCMD_INTEGRATION_TIME, 0, 0, CCS_SERIES_NUM_INTEG_CTRL_BYTES, (ViBuf)data);

   // error check and log
   err = CCSseries_checkErrorLevel(instrumentHandle, err);
   
   return err; 
}


/*---------------------------------------------------------------------------
   Function:   Get Integration Time
   Purpose:    This function returns the optical integration time in seconds. 

   Parameters:
   
   ViSession instr:           The actual session to opened device.
   ViPReal64 integrationTime: The optical integration time. 
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_getIntegrationTime (ViSession instrumentHandle, ViPReal64 integrationTime)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViUInt8 data[CCS_SERIES_NUM_INTEG_CTRL_BYTES];
   ViInt32 integ = 0;
   ViInt32 presc = 0;
   ViInt32 fill = 0;
   
   // request the data
   err = CCSseries_USB_in(instrumentHandle, CCS_SERIES_RCMD_INTEGRATION_TIME, 0, 0, CCS_SERIES_NUM_INTEG_CTRL_BYTES * sizeof(ViUInt8), (ViBuf)data, &read_bytes);
   
   // error mapping
   if((read_bytes != (CCS_SERIES_NUM_INTEG_CTRL_BYTES * sizeof(ViUInt8))) && (!err))   err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE; 
   
   // check for errors
   if((err = CCSseries_checkErrorLevel(instrumentHandle, err)))  return (err);
   
   // decode the response
   presc = ((data[0] << 8) + data[1]) & 0x0FFF;
   fill = ((data[2] << 8) + data[3]) & 0x0FFF; 
   integ = ((data[4] << 8) + data[5]) & 0x0FFF;
   
   // calculate the integration time
   *integrationTime = ((ViReal64)(integ - fill + 8) * pow(2.0, (ViReal64)presc)) / 1000000.0;
   
   return err;  
}

status
/*===========================================================================


 Class: Action/Status Functions.


===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Start Scan
   Purpose:    This function triggers the the CCS to take one single scan.  

   Parameters:
   
   ViSession instr:           The actual session to opened device.
   
   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_startScan (ViSession instrumentHandle)
{
   ViStatus err = VI_SUCCESS;
   
   err = CCSseries_USB_out(instrumentHandle, CCS_SERIES_WCMD_MODUS, MODUS_INTERN_SINGLE_SHOT, 0, 0, VI_NULL);
   
   // error check and log
   err = CCSseries_checkErrorLevel(instrumentHandle, err);
   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Start Continuous Scan
   Purpose:    This function starts the CCS scanning continuously. 
               Any other function except 'Get Scan Data' and 'Get Device Status' 
               will stop the scanning.  

   Parameters:
   
   ViSession instr:           The actual session to opened device.
   
   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_startScanCont (ViSession instrumentHandle)
{
   ViStatus err = VI_SUCCESS;
   
   err = CCSseries_USB_out(instrumentHandle, CCS_SERIES_WCMD_MODUS, MODUS_INTERN_CONTINUOUS, 0, 0, VI_NULL);

   // error check and log
   err = CCSseries_checkErrorLevel(instrumentHandle, err);
   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Start Extern Scan
   Purpose:    This function arms the external trigger of the CCS. A
               following low to high transition at the trigger input of the 
               CCS then starts a scan.  

   Parameters:
   
   ViSession instr:           The actual session to opened device.
   
   Note:
   When you issue a read command 'Get Scan Data' before the CCS was 
   triggered you will get a timeout error. Use 'Get Device Status' to check 
   the scan status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_startScanExtTrg (ViSession instrumentHandle)
{
   ViStatus err = VI_SUCCESS;
   
   err = CCSseries_USB_out(instrumentHandle, CCS_SERIES_WCMD_MODUS, MODUS_EXTERN_SINGLE_SHOT, 0, 0, VI_NULL);

   // error check and log
   err = CCSseries_checkErrorLevel(instrumentHandle, err);
   
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
   
   ViSession instr:           The actual session to opened device.
   
   Note:
   The scan data can be read out with the function 'Get Scan Data'

   Note:
   When you issue a read command 'Get Scan Data' before the CCS_SERIES was triggered 
   you will get a timeout error. Use 'Get Device Status' to check the scan status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_startScanContExtTrg (ViSession instrumentHandle)
{
   ViStatus err = VI_SUCCESS;
   
   err = CCSseries_USB_out(instrumentHandle, CCS_SERIES_WCMD_MODUS, MODUS_EXTERN_CONTINUOUS, 0, 0, VI_NULL);

   // error check and log
   err = CCSseries_checkErrorLevel(instrumentHandle, err);
   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Get Device Status
   Purpose:    This function returns a 16bit status value (by ref).
               Use macros to mask out single bits. 
               
               #define CCS_SERIES_STATUS_SCAN_IDLE          0x0002 
               #define CCS_SERIES_STATUS_SCAN_TRIGGERED     0x0004
               #define CCS_SERIES_STATUS_SCAN_TRANSFER         0x0010
               #define CCS_SERIES_STATUS_WAIT_FOR_EXT_TRIG  0x0080  

   Parameters:
   
   ViSession instr:           The actual session to opened device.
   ViPUInt16 deviceStatus:    The device status.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_getDeviceStatus (ViSession instrumentHandle, ViPInt32 deviceStatus)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViInt16 tmp = 0x00;
   
   if(!deviceStatus) return VI_ERROR_INV_PARAMETER;
   
   *deviceStatus = 0xFFFF;
   
   err = CCSseries_USB_in(instrumentHandle, CCS_SERIES_RCMD_GET_STATUS, 0, 0, sizeof(ViInt16), (ViBuf)&tmp, &read_bytes);
   if((!err) && (read_bytes != sizeof(ViInt16))) err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE;
   
   *deviceStatus = tmp;
   
   // error check and log
   err = CCSseries_checkErrorLevel(instrumentHandle, err);

   return (err);  
}


/*===========================================================================


 Class: Data Functions.


===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Get Scan Data
   Purpose:    This function reads out the processed scan data.

   Parameters:
   
   ViSession instr:           The actual session to opened device.
   ViReal64 _VI_FAR data[]:   The measurement array (CCS_SERIES_NUM_PIXELS elements).
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_getScanData (ViSession instrumentHandle, ViReal64 _VI_FAR data[])
{
   ViStatus err         = VI_SUCCESS;     // error level
   ViUInt16 raw[CCS_SERIES_NUM_RAW_PIXELS];  // array to copy raw data to
   
   // read raw scan data
   if((err = CCSseries_getRawData(instrumentHandle, raw))) return err;
   
   // process data
   err = CCSseries_aquireRawScanData(instrumentHandle, raw, data);
   
   // error check and log
   err = CCSseries_checkErrorLevel(instrumentHandle, err);
   
   return (err);
}


/*---------------------------------------------------------------------------
   Function:   Get Raw Scan Data
   Purpose:    This function reads out the raw scan data. 

   Parameters:
   
   ViSession instr:                 The actual session to opened device.
   ViReal64 _VI_FAR scanDataArray[]:The measurement array 
                                    (CCS_SERIES_NUM_RAW_PIXELS elements).
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_getRawScanData (ViSession instrumentHandle, ViInt32 _VI_FAR scanDataArray[])
{
   ViStatus err         = VI_SUCCESS;
   ViUInt16 raw[CCS_SERIES_NUM_RAW_PIXELS];  // array to copy raw data to
   ViInt32  i           = 0;
   
   err = CCSseries_getRawData(instrumentHandle, raw);
   
   for(i = 0; i < CCS_SERIES_NUM_RAW_PIXELS; i++)
   {
      scanDataArray[i] = (ViInt32)raw[i];    
   }
   
   // check for errors 
   if((err = CCSseries_checkErrorLevel(instrumentHandle, err)))   return (err);  
   
   return (err);
}



/*---------------------------------------------------------------------------
   Function:   Set Wavelength Data
   Purpose:    This function stores data for user-defined pixel-wavelength
               correlation to the instruments nonvolatile memory.

               The given data pair arrays are used to interpolate the
               pixel-wavelength correlation array returned by the
               CCSseries_getWavelengthData function.

   Note: In case the interpolated pixel-wavelength correlation
   contains negative values, or the values are not strictly
   increasing the function returns with error VI_ERROR_INV_USER_DATA. 

   Parameters:
   
   ViSession instr:                       The actual session to opened device.
   ViInt32 _VI_FAR pixelDataArray[]:      The pixel data. 
   ViReal64 _VI_FAR wavelengthDataArray[]:The wavelength data.
   ViInt32 bufferLength:                  The length of both arrays.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_setWavelengthData (ViSession instrumentHandle, ViInt32 _VI_FAR pixelDataArray[], ViReal64 _VI_FAR wavelengthDataArray[], ViInt32 bufferLength)
{
#define FACTORY_ADJ_OFFSET       62749006
   
   ViStatus err      = VI_SUCCESS;
   CCS_SERIES_data_t       *data;
   CCS_SERIES_wl_cal_t  cal;
   char              target = 0;
   
   // get private data
   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   if((!pixelDataArray) || (!wavelengthDataArray)) return VI_ERROR_INV_PARAMETER; 
   
   // check valid buffer length and determine target
   if((bufferLength >= CCS_SERIES_MIN_NUM_USR_ADJ) && (bufferLength <= CCS_SERIES_MAX_NUM_USR_ADJ))
   {
      target = 1; // target is user adjustment data
   }
   else if ((bufferLength >= (CCS_SERIES_MIN_NUM_USR_ADJ + FACTORY_ADJ_OFFSET)) && (bufferLength <= (CCS_SERIES_MAX_NUM_USR_ADJ  + FACTORY_ADJ_OFFSET)))
   {
      bufferLength -= FACTORY_ADJ_OFFSET;
      target = 0; // target is factory adjustment data
   }
   else return VI_ERROR_INV_PARAMETER;
   
   // check nodes array
   if((err = CCSseries_checkNodes(pixelDataArray, wavelengthDataArray, bufferLength)) != VI_SUCCESS)  return err;
   
   // calculate new coefficients...
   if((err = CCSseries_nodes2poly(pixelDataArray, wavelengthDataArray, bufferLength, cal.poly)) != VI_SUCCESS) return err;
   
   // use the coefficients to calculate the new wavelength array...
   if((err = CCSseries_poly2wlArray(&cal)) != VI_SUCCESS)   return err;
   
   
   // write new data to EEPROM and data structure
   if(target)
   {
      ////  target is user adjustment data  ////
      
      // write polynomical coefficients to eeprom
      if((err = CCSseries_writeEEUserPoly(instrumentHandle, cal.poly)) != VI_SUCCESS) return err;
      
      // write user  calibration points
      if((err = CCSseries_writeEEUserPoints(instrumentHandle, pixelDataArray, wavelengthDataArray, bufferLength)) != VI_SUCCESS) return err;

      // write valid flags to eeprom
      cal.valid = 1;
      if((err = CCSseries_writeEEUserFlag(instrumentHandle, cal.valid)) != VI_SUCCESS) return err;
      
      // copy new values
      memcpy(&(data->user_cal), &cal, sizeof(CCS_SERIES_wl_cal_t));
       
      memcpy(data->user_points.user_cal_node_pixel, pixelDataArray, sizeof(ViUInt32) * bufferLength);
      memcpy(data->user_points.user_cal_node_wl, wavelengthDataArray, sizeof(ViReal64) * bufferLength);
      data->user_points.user_cal_node_cnt = bufferLength;
   }
   else
   {
      ////  target is factory adjustment data   ////
      
      // write polynomical coefficients to eeprom
      if((err = CCSseries_writeEEFactoryPoly(instrumentHandle, cal.poly)) != VI_SUCCESS) return err;
      
      // write valid flags to eeprom
      cal.valid = 1;
      if((err = CCSseries_writeEEFactoryFlag(instrumentHandle, cal.valid)) != VI_SUCCESS) return err;
      
      // copy new values to CCS data
      memcpy(&(data->factory_cal), &cal, sizeof(CCS_SERIES_wl_cal_t));
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
   1 and so on until Wavelength_Data_Array[CCS_SERIES_NUM_PIXELS-1]
   which provides the wavelength at pixel CCS_SERIES_NUM_PIXELS-1
   (3647). This is the maximum wavelength.

   Parameters:
   
   ViSession instr:                       The actual session to opened device.
   ViUInt16 dataSet:                      The pixel data. 
   ViReal64 _VI_FAR wavelengthDataArray[]:The wavelength data.
   ViPReal64 minimumWavelength:           The minimum wavelength.
   ViPReal64 maximumWavelength:           The maximum wavelength. 
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_getWavelengthData (ViSession instrumentHandle, ViInt16 dataSet, ViReal64 _VI_FAR wavelengthDataArray[], ViPReal64 minimumWavelength, ViPReal64 maximumWavelength)
{
   CCS_SERIES_data_t    *data;
   ViStatus       err = VI_SUCCESS;
   
   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;    
   
   switch (dataSet)
   {
      case CCS_SERIES_CAL_DATA_SET_FACTORY:
         // copy wavelength array form factory data
         
         if(wavelengthDataArray != NULL)  memcpy(wavelengthDataArray, data->factory_cal.wl, (CCS_SERIES_NUM_PIXELS * sizeof(ViReal64)));
         if(minimumWavelength != NULL)    *minimumWavelength = data->factory_cal.min;
         if(maximumWavelength != NULL)    *maximumWavelength = data->factory_cal.max;
         break;
         
      case CCS_SERIES_CAL_DATA_SET_USER:

         if(!data->user_cal.valid) return VI_ERROR_CCS_SERIES_NO_USER_DATA;
    
         if(wavelengthDataArray != NULL)  memcpy(wavelengthDataArray, data->user_cal.wl, (CCS_SERIES_NUM_PIXELS * sizeof(ViReal64)));
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
               correlation array returned by the CCSseries_getWavelengthData
               function.

   Note:
   If you do not need either of these values you may pass NULL.
   The function returns with error VI_ERROR_NO_USER_DATA if no user
   calibration data is present in the instruments nonvolatile
   memory.

   Parameters:
   
   ViSession instrumentHandle:            The actual session to opened device.
   ViInt32 _VI_FAR pixelDataArray[]:      The pixel data array.
   ViReal64 _VI_FAR wavelengthDataArray[]:The wavelength data array. 
   ViInt32 _VI_FAR bufferLength[]:        The number of user calibration points. 
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_getUserCalibrationPoints (ViSession instrumentHandle, ViInt32 _VI_FAR pixelDataArray[],
                                                   ViReal64 _VI_FAR wavelengthDataArray[], ViPInt32 bufferLength)
{
   ViStatus err = VI_SUCCESS;
   CCS_SERIES_data_t    *data;
   
   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err; 
   
   if(data->user_cal.valid)
   {
      if(pixelDataArray)      memcpy(pixelDataArray,      data->user_points.user_cal_node_pixel, data->user_points.user_cal_node_cnt * sizeof(ViUInt32));
      if(wavelengthDataArray) memcpy(wavelengthDataArray, data->user_points.user_cal_node_wl,    data->user_points.user_cal_node_cnt * sizeof(ViReal64));
      if(bufferLength)        *bufferLength = data->user_points.user_cal_node_cnt;
   }
   else
   {
      err = VI_ERROR_CCS_SERIES_NO_USER_DATA;
   }
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Set Amplitude Data
   Purpose:    This function stores data for user-defined amplitude
               correction factors to nonvolatile memory of CCD device.

               The given data array can be used to correct the
               amplitude information returned by the
               CCSseries_getScanDataData function.

   Note: In case the correction factors are out of the range
   CCS_SERIES_AMP_CORR_FACT_MIN (0.001) ... CCS_SERIES_AMP_CORR_FACT_MAX (1000.0)
   the function returns with error VI_ERROR_CCS_SERIES_INV_USER_DATA. 

   Parameters:
   
   ViSession instr:                 The actual session to opened device.
   ViReal64 _VI_FAR AmpCorrFact[]:  The array with amplitude correction factors
   ViInt32 bufferLength:            The length of the array.
   ViInt32 bufferStart:             The start index for the array.
   ViInt32 mode                     With mode one can select if the new data will be applied to current measurements
                                    only (ACOR_APPLY_TO_MEAS) or additionally goes to non volatile memory of the
                                    device too (ACOR_APPLY_TO_MEAS_NVMEM).
                                    If mode is not one of the two predefined macros the function returns VI_ERROR_INV_PARAMETER
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_setAmplitudeData (ViSession instr, ViReal64 AmpCorrFact[], ViInt32 bufferLength, ViInt32 bufferStart, ViInt32 mode)
{
#define FACTORY_SET_START           19901201
   
   ViStatus             err      = VI_SUCCESS;
   CCS_SERIES_data_t    *data;
   char                 target = 0;
   int                  i;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // check valid buffer start and determine target
   if((bufferStart >= 0) && (bufferStart < CCS_SERIES_NUM_PIXELS))
   {
      target = 1; // target is user adjustment data
   }
   else if ((bufferStart >= FACTORY_SET_START) && (bufferStart < (FACTORY_SET_START + CCS_SERIES_NUM_PIXELS)))
   {
      bufferStart -= FACTORY_SET_START;
      target = 0; // target is factory adjustment data
   }
   else
   {
      return VI_ERROR_INV_PARAMETER;
   }

   // check valid buffer length
   if((bufferStart + bufferLength) < 1)                     return VI_ERROR_INV_PARAMETER;
   if((bufferStart + bufferLength) > CCS_SERIES_NUM_PIXELS) return VI_ERROR_INV_PARAMETER;

   // check for data out of range
   for(i=0; i<bufferLength; i++)
   {
      if(AmpCorrFact[i] < CCS_SERIES_AMP_CORR_FACT_MIN)  return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      if(AmpCorrFact[i] > CCS_SERIES_AMP_CORR_FACT_MAX)  return VI_ERROR_CCS_SERIES_INV_USER_DATA;
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
      
   
   if(target)     ////  target is user adjustment data  ////
   {
      // copy new values to data structure and ..
      for(i=0; i<bufferLength; i++)
      {
         data->user_acor_cal.acor[bufferStart + i] = (float)AmpCorrFact[i];
      }

      // .. eventually copy them to NVMEM (EEPROM) too
      if(mode == ACOR_APPLY_TO_MEAS_NVMEM)
      {
         err = CCSseries_writeEEPROM(instr, EE_ACOR_USER, 0, EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor);
      }
   }
   else        ////  target is factory adjustment data   ////
   {
      // copy new values to data structure and ..
      for(i=0; i<bufferLength; i++)
      {
         data->factory_acor_cal.acor[bufferStart + i] = (float)AmpCorrFact[i];
      }

      // .. eventually copy them to NVMEM (EEPROM) too
      if(mode == ACOR_APPLY_TO_MEAS_NVMEM)
      {
         err = CCSseries_writeEEPROM(instr, EE_ACOR_FACTORY, 0, EE_LENGTH_ACOR, (ViBuf)data->factory_acor_cal.acor);
      }
   }
   
   return err;
}

/*---------------------------------------------------------------------------
   Function:   Get Amplitude Data
   Purpose:    This function retrieves data for user-defined amplitude
               correction factors from nonvolatile memory of CCD device.

               The given data array can be used to correct the
               amplitude information returned by the
               CCSseries_getScanDataData function.

   Parameters:
   
   ViSession instr:                 The actual session to opened device.
   ViReal64 _VI_FAR AmpCorrFact[]:  The array with amplitude correction factors
   ViInt32 bufferLength:            The length of the array.
   ViInt32 bufferStart:             The start index for the array.
   ViInt32 mode                     With mode one can select if the data will be a copy of the currently used
                                    amplitude correction factors (ACOR_FROM_CURRENT) or if the data is read out from the
                                    device non volatile memory (ACOR_FROM_NVMEM)
                                    If mode is not one of the two predefined macros the function returns VI_ERROR_INV_PARAMETER
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_getAmplitudeData (ViSession instr, ViReal64 AmpCorrFact[], ViInt32 bufferStart, ViInt32 bufferLength, ViInt32 mode)
{
   ViStatus             err      = VI_SUCCESS;
   CCS_SERIES_data_t    *data;
   char                 target = 0;
   int                  i;
   ViUInt16             read_bytes;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // check valid buffer start and determine target
   if((bufferStart >= 0) && (bufferStart < CCS_SERIES_NUM_PIXELS))
   {
      target = 1; // target is user adjustment data
   }
   else if ((bufferStart >= FACTORY_SET_START) && (bufferStart < (FACTORY_SET_START + CCS_SERIES_NUM_PIXELS)))
   {
      bufferStart -= FACTORY_SET_START;
      target = 0; // target is factory adjustment data
   }
   else
   {
      return VI_ERROR_INV_PARAMETER;
   }

   // check valid buffer length
   if((bufferStart + bufferLength) < 1)                     return VI_ERROR_INV_PARAMETER;
   if((bufferStart + bufferLength) > CCS_SERIES_NUM_PIXELS) return VI_ERROR_INV_PARAMETER;

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
         err = CCSseries_readEEPROM(instr, EE_ACOR_USER,    0, EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor, &read_bytes);
   
         // error mapping
         if((read_bytes != EE_LENGTH_ACOR) && (!err))   err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE; 

         // when checksum failed but everything else was ok -> assume a blank or new device, generate an array of 1.000 and write it to the EEPROM
         if(err == VI_ERROR_CYEEPROM_CHKSUM)
         {
            for(i=0; i<CCS_SERIES_NUM_PIXELS; i++)
            {
               data->user_acor_cal.acor[i] = 1.0;
            }
            err = CCSseries_writeEEPROM(instr, EE_ACOR_USER, 0, EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor);
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
         err = CCSseries_readEEPROM(instr, EE_ACOR_FACTORY, 0, EE_LENGTH_ACOR, (ViBuf)data->factory_acor_cal.acor, &read_bytes);
   
         // error mapping
         if((read_bytes != EE_LENGTH_ACOR) && (!err))   err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE; 

         // when checksum failed but everything else was ok -> assume a blank or new device, generate an array of 1.000 and write it to the EEPROM
         if(err == VI_ERROR_CYEEPROM_CHKSUM)
         {
            for(i=0; i<CCS_SERIES_NUM_PIXELS; i++)
            {
               data->factory_acor_cal.acor[i] = 1.0;
            }
            err = CCSseries_writeEEPROM(instr, EE_ACOR_FACTORY, 0, EE_LENGTH_ACOR, (ViBuf)data->factory_acor_cal.acor);
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

   Parameters:
   
   ViSession instr:                 The actual session to opened device.
   ViChar manufacturerName[]:       The manufacturers name. 
   ViChar deviceName[]:             The device name.
   ViChar serialNumber[]:           The serial number.
   ViChar firmwareRevision[]:       The firmware revision.
   ViChar instrumentDriverRevision[]:The instrument driver revision.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_identificationQuery (ViSession instr,
                                           ViChar manufacturerName[],
                                           ViChar deviceName[],
                                           ViChar serialNumber[],
                                           ViChar firmwareRevision[],
                                           ViChar instrumentDriverRevision[])
{
  
   CCS_SERIES_data_t    *data;
   ViStatus       err = VI_SUCCESS;
   
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;  
   
   if(manufacturerName != NULL)
   {
      strncpy(manufacturerName, data->manu, CCS_SERIES_BUFFER_SIZE);
   }
    
   if(deviceName != NULL)
   {
      strncpy(deviceName, data->name, CCS_SERIES_BUFFER_SIZE);
   }
    
   if(serialNumber != NULL)
   {
      strncpy(serialNumber, data->serNr, CCS_SERIES_BUFFER_SIZE);
   }
    
   if(firmwareRevision != NULL)
   {
      sprintf(firmwareRevision, "%d.%d.%d", data->firmware_version.major, data->firmware_version.minor, data->firmware_version.subminor);
   }
   
   if(instrumentDriverRevision != NULL)
   {
      sprintf(instrumentDriverRevision, "%d.%d.%d", CCS_SERIES_VER_MAJOR, CCS_SERIES_VER_MINOR, CCS_SERIES_VER_SUBMINOR);
   }
   
   return (err);
}


/*---------------------------------------------------------------------------
   Function:   Revision Query
   Purpose:    This function returns the revision numbers of the instrument
               driver and the device firmware.

   Parameters:
   
   ViSession instr:                    The actual session to opened device.
   ViChar instrumentDriverRevision[]:  The instrument driver revision. 
   ViChar firmwareRevision[]:          The firmware revision.  
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_revisionQuery (ViSession instr,
                                     ViChar instrumentDriverRevision[],
                                     ViChar firmwareRevision[])
{
   CCS_SERIES_data_t    *data;
   ViStatus       err = VI_SUCCESS;
   
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;  
   
   if(instrumentDriverRevision != NULL)
   {
      sprintf(instrumentDriverRevision, "%d.%d.%d", CCS_SERIES_VER_MAJOR, CCS_SERIES_VER_MINOR, CCS_SERIES_VER_SUBMINOR);
   }
   
   if(firmwareRevision != NULL)
   {
      sprintf(firmwareRevision, "%d.%d.%d", data->firmware_version.major, data->firmware_version.minor, data->firmware_version.subminor);
   }
   
   return (err);  
}


/*---------------------------------------------------------------------------
   Function:   Reset
   Purpose:    This function resets the device.

   Parameters:
   
   ViSession instr:  The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_reset (ViSession instrumentHandle)
{
   CCS_SERIES_data_t    *data;
   ViStatus       err = VI_SUCCESS;

   // get handle
   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // reset the device itself
   err = CCSseries_USB_out(instrumentHandle, CCS_SERIES_WCMD_RESET, 0, 0, 0, VI_NULL);

   // check for errors 
   if((err = CCSseries_checkErrorLevel(instrumentHandle, err)))   return (err);  
   
   // do an one time readout to collect garbled buffers
   err = viFlush (instrumentHandle, VI_READ_BUF);

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get error message
   Purpose:    This function translates the error return value from the
               instrument driver into a user-readable string.

   Parameters
   ViSession instr        :  the handle obtained by 'CCSseries_init()'
   ViStatus stat          :  the error code for which a description is looked for
   ViChar description[]   :  the buffer to which the error message is written to
                              size must be at least CCS_SERIES_ERR_DESCR_BUFFER_SIZE = 512 bytes
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_errorMessage(ViSession instr, ViStatus statusCode, ViChar description[])
{
   const CCS_SERIES_errDescrStat_t *ptr;
   
   if(!description)  return VI_ERROR_INV_PARAMETER;       
   
   // VISA errors
   if(viStatusDesc(instr, statusCode, description) != VI_WARN_UNKNOWN_STATUS) return VI_SUCCESS;

   // Static driver errors
   ptr = CCS_SERIES_errDescrStat;
   while(ptr->descr != VI_NULL)
   {
      if(ptr->err == statusCode)
      {
         strcpy(description, ptr->descr);
         return (VI_SUCCESS);
      }
      ptr ++;
   }

   // Not found
   viStatusDesc(instr, VI_WARN_UNKNOWN_STATUS, description);
   return VI_WARN_UNKNOWN_STATUS;
}


/*---------------------------------------------------------------------------
   Function:   Set User Text
   Purpose:    This function writes the given string to the novolatile memory of
               the spectrometer.
   Parameters:
   
   ViSession instrumentHandle :  the handle obtained by 'CCSseries_init()'
   ViChar userText            :  the new user text
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_setUserText (ViSession instrumentHandle, ViChar _VI_FAR userText[]) 
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   char           buf[CCS_SERIES_MAX_USER_NAME_SIZE];

   // get private data
   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   if(!userText)  return VI_ERROR_INV_PARAMETER; 
   
   // copy the string
   strncpy(buf, userText, (CCS_SERIES_MAX_USER_NAME_SIZE - 1));   // strncpy will fill with '\0' when userText is smaller than (USER_LABEL_LENGTH-1)
   
   // truncate 
   buf[CCS_SERIES_MAX_USER_NAME_SIZE-1] = '\0';

   // write to eeprom
   err = CCSseries_writeEEPROM(instrumentHandle, EE_USER_LABEL, 0, CCS_SERIES_MAX_USER_NAME_SIZE, (ViBuf)buf);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instrumentHandle, err)))   return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get User Text
   Purpose:    This function reads the user text from the novolatile memory of
               the spectrometer.
   Parameters:
   
   ViSession instrumentHandle :  the handle obtained by 'CCSseries_init()'
   ViChar userText            :  the user text
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC CCSseries_getUserText (ViSession instrumentHandle, ViChar _VI_FAR userText[])
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data = VI_NULL;
   ViUInt16       cnt = 0;

   // get private data
   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   if(!userText)  return VI_ERROR_INV_PARAMETER;
   
   // write to eeprom
   err = CCSseries_readEEPROM(instrumentHandle, EE_USER_LABEL, 0, CCS_SERIES_MAX_USER_NAME_SIZE, (ViBuf)userText, &cnt);
   
   if(err)
   {
      memset(userText, 0, CCS_SERIES_MAX_USER_NAME_SIZE);
      
      // in case there is no user text...
      strncpy(userText, DEFAULT_USER_TEXT, CCS_SERIES_MAX_USER_NAME_SIZE);
      
      if(err == VI_ERROR_CYEEPROM_CHKSUM) err = VI_SUCCESS;
      
      // check for error
      CCSseries_checkErrorLevel(instrumentHandle, err);
   }
   
   return err;
}


/*---------------------------------------------------------------------------
  USB Out - encapsulates the VISA function 'viUsbControlOut()'. When CCS
  stalls the error VI_ERROR_IO will be returned by 'viUsbControlOut()'.
  Then USB Out will issue the VISA function 'viUsbControlIn' and tries to
  read one Byte from the CCS, if this succeeds the obtained Byte contains
  the error code from the CCS. This means we have NO communications error.
  
  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'CCSseries_init()'
  ViInt16 bRequest            :  the command sent to the CCS
  ViUInt16 wValue             :  arbitrary parameter, can be used for additional information 
  ViUInt16 wIndex             :  arbitrary parameter, can be used for additional information 
  ViUInt16 wLength            :  size of Buffer
  ViBuf Buffer                :  buffer of max. 64 Bytes

  Result                      :  Error
---------------------------------------------------------------------------*/
static ViStatus CCSseries_USB_out(ViSession Instrument_Handle, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer)
{
   ViStatus err;
   unsigned char CCS_SERIES_Error;
   
   err = viUsbControlOut (Instrument_Handle, 0x40, bRequest, wValue, wIndex, wLength, Buffer);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (Instrument_Handle, 0xC0, CCS_SERIES_RCMD_GET_ERROR, 0, 0, 1, &CCS_SERIES_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' CCS error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)CCS_SERIES_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   return err;
}

/*---------------------------------------------------------------------------
  USB In - encapsulates the VISA function 'viUsbControlIn()'. When CCS stalls
  the error VI_ERROR_IO will be returned by 'viUsbControlIn()'. Then USB Out
  will issue the VISA function 'viUsbControlIn' and tries to read one Byte
  from the CCS, if this succeeds the obtained Byte contains the error code
  from the CCS. This means we have NO communications error.
  
  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'CCSseries_init()'
  ViInt16 bRequest            :  the command sent to the CCS
  ViUInt16 wValue             :  arbitrary parameter, can be used for additional information 
  ViUInt16 wIndex             :  arbitrary parameter, can be used for additional information 
  ViUInt16 wLength            :  size of Buffer
  ViBuf Buffer                :  buffer of max. 64 Bytes
  ViPUInt16 Read_Bytes        :  number of bytes actually read out

  Result                      :  Error
---------------------------------------------------------------------------*/
static ViStatus CCSseries_USB_in(ViSession Instrument_Handle, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer, ViPUInt16 Read_Bytes)
{
   ViStatus err;
   unsigned char CCS_SERIES_Error;
   
   err = viUsbControlIn (Instrument_Handle, 0xC0, bRequest, wValue, wIndex, wLength, Buffer, Read_Bytes);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (Instrument_Handle, 0xC0, CCS_SERIES_RCMD_GET_ERROR, 0, 0, 1, &CCS_SERIES_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' CCS error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)CCS_SERIES_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   return err;
}

/*---------------------------------------------------------------------------
  USB Write - encapsulates the VISA function 'viWrite'. When CCS
  stalls the error VI_ERROR_IO will be returned by 'viUsbControlOut()'.
  Then USB Out will issue the VISA function 'viUsbControlIn' and tries to
  read one Byte from the CCS, if this succeeds the obtained Byte contains
  the error code from the CCS. This means we have NO communications error.
  
  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'CCSseries_init()'
  ViBuf Buffer                :  buffer to send to device
  ViUInt32 Count              :  number of Bytes to send from buffer to device

  Return Value
  ViUInt32 *ReturnCount       :  number of Bytes actually sent to device
                                 You may pass NULL if you do not need the value

  Result                      :  Error
---------------------------------------------------------------------------*/
#ifdef NOT_NEEDE_UNTIL_NOW
static ViStatus CCSseries_USB_write(ViSession Instrument_Handle, ViBuf Buffer, ViUInt32 Count, ViUInt32 *ReturnCount)
{
   ViStatus err;
   unsigned char CCS_SERIES_Error;
   ViUInt32 retcount;
   
   err = viWrite (Instrument_Handle, Buffer, Count, &retcount);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (Instrument_Handle, 0xC0, CCS_SERIES_RCMD_GET_ERROR, 0, 0, 1, &CCS_SERIES_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' CCS error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)CCS_SERIES_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   if((err == VI_SUCCESS) && (ReturnCount != NULL))
      *ReturnCount = retcount;
      
   return err;
}
#endif   // NOT_NEEDE_UNTIL_NOW


/*---------------------------------------------------------------------------
  USB Read - encapsulates the VISA function 'viRead'. When CCS
  stalls the error VI_ERROR_IO will be returned by 'viUsbControlOut()'.
  Then USB Out will issue the VISA function 'viUsbControlIn' and tries to
  read one Byte from the CCS, if this succeeds the obtained Byte contains
  the error code from the CCS. This means we have NO communications error.
  
  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'CCSseries_init()'
  unsigned char *ReceiveData  :  pointer to a buffer where received data is put to
  ViUInt32 Count              :  number of Bytes to read from device to buffer

  Return Value
  ViUInt32 *ReturnCount       :  number of Bytes actually read from device
                                 You may pass NULL if you do not need the value

  Result                      :  Error
---------------------------------------------------------------------------*/
static ViStatus CCSseries_USB_read(ViSession Instrument_Handle, unsigned char *ReceiveData, ViUInt32 Count, ViUInt32 *ReturnCount)
{
   ViStatus err;
   unsigned char CCS_SERIES_Error;
   ViUInt32 retcount;
   
   err = viRead (Instrument_Handle, ReceiveData, Count, &retcount);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (Instrument_Handle, 0xC0, CCS_SERIES_RCMD_GET_ERROR, 0, 0, 1, &CCS_SERIES_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' CCS error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)CCS_SERIES_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   if((err == VI_SUCCESS) && (ReturnCount != NULL))
      *ReturnCount = retcount;
      
   return err;
}


/*---------------------------------------------------------------------------
 Function: Writes data stored in buffer to EEPROM.
---------------------------------------------------------------------------*/
__declspec(dllexport) ViStatus CCSseries_writeEEPROM(ViSession instr, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf)
{
   ViStatus err = VI_SUCCESS;
   uint16_t sum = 0;
   
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
      err = CCSseries_USB_out(instr, CCS_SERIES_WCMD_WRITE_EEPROM, iAddress, idx, iLength, ibuf);
   
      // check for error
      if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
      // increase addresses of EEPROM and buffer for the amount of bytes sent
      ibuf          += iLength;
      iAddress      += iLength;
      // decrease the counter of bytes to be transmitted
      iTransferSize -= iLength;
   }

   // check if we have to use crc16 -> up from address EE_SERIAL_NO
   if(addr >= EE_SW_VERSION)
   {
      sum = crc16_block((void*)buf, len);
      if((err = CCSseries_USB_out(instr, CCS_SERIES_WCMD_WRITE_EEPROM, addr + len, 0, sizeof(uint16_t), (ViBuf)&sum))) return err; 
   }
   
   return err;      
}


/*---------------------------------------------------------------------------
 Function: Reads data stored in EEPROM to buffer.
---------------------------------------------------------------------------*/
__declspec(dllexport) ViStatus CCSseries_readEEPROM(ViSession instr, ViUInt16 addr, ViUInt16 idx, ViUInt16 len, ViBuf buf, ViUInt16* cnt)
{
   ViStatus err = VI_SUCCESS;   
   uint16_t sum = 0; 
   uint16_t ees = 0;
   ViUInt16 tmp = 0;
   
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
      if((err = CCSseries_USB_in(instr, CCS_SERIES_RCMD_READ_EEPROM, iAddress, idx, iLength, ibuf, &iCount))) return err;
      
      // check for error
      if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
      if(cnt) *cnt  += iCount;
      ibuf          += iCount;
      iAddress      += iCount;
      iTransferSize -= iCount;
   }

   // check if we have to use crc16 -> up from address EE_SERIAL_NO
   if(addr >= EE_SW_VERSION)
   {
      sum = crc16_block((void*)buf, len);
      if((err = CCSseries_USB_in(instr, CCS_SERIES_RCMD_READ_EEPROM, addr + len, idx, sizeof(uint16_t), (ViBuf)&ees, &tmp))) return err;  
      if(sum != ees)    err = VI_ERROR_CYEEPROM_CHKSUM;
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
 Close everything
---------------------------------------------------------------------------*/
static ViStatus CCSseries_initClose (ViSession instr, ViStatus stat)
{
   ViStatus       err   = VI_SUCCESS;
   ViSession      rm    = VI_NULL;
   CCS_SERIES_data_t     *data = VI_NULL;

   // Get resource manager session and private data pointer
   viGetAttribute(instr, VI_ATTR_RM_SESSION, &rm);
   viGetAttribute(instr, VI_ATTR_USER_DATA,  &data);

   // Free private data
   if(data != NULL)
   {
      free(data);
   }

   // Close sessions
   if(instr)   err = viClose(instr);
   if(rm)      viClose(rm);

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
 Check for error - interpretes code as status and pops up an error screen 
 if necessary, returns the code itself
---------------------------------------------------------------------------*/
static ViStatus CCSseries_checkErrorLevel(ViSession instr, ViStatus code)
{
   return (code);
}

#define CCS_DARK_PIXELS_COMMON
#ifdef CCS_DARK_PIXELS_COMMON
/*---------------------------------------------------------------------------
 Aquire Raw Scan Data - aquires the raw scan data to inverted values normed
 to one.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_aquireRawScanData(ViSession instrumentHandle, ViUInt16 raw[], ViReal64 data[])
{
#define NO_DARK_PIXELS                 12       // we got 12 dark pixels
#define DARK_PIXELS_OFFSET             16       // dark pixels start at positon 16 within raw data
#define SCAN_PIXELS_OFFSET             32       // real measurement start at position 32 within raw data
#define MAX_ADC_VALUE                  0xFFFF

   CCS_SERIES_data_t    *ccs_data;
   ViStatus err = VI_SUCCESS;
   ViReal64 norm_com = 0.0;
   ViReal64 dark_com = 0.0;

   int i = 0;

   // get private data
   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &ccs_data)) != VI_SUCCESS) return err;

   // sum the dark Pixels
   for(i = 0; i < NO_DARK_PIXELS; i++)
   {
      dark_com += raw[(DARK_PIXELS_OFFSET + i)];
   }

   // calculate dark current average
   dark_com /= (double)(NO_DARK_PIXELS);

   // calculate normalizing factor
   norm_com = 1.0 / ((ViReal64)MAX_ADC_VALUE - dark_com);


   for(i = 0; i < CCS_SERIES_NUM_PIXELS; i++)
   {
      data[i] = (((ViReal64)raw[SCAN_PIXELS_OFFSET + i]) - dark_com) * norm_com;
   }

   for(i = 0; i < CCS_SERIES_NUM_PIXELS; i++)
   {
      // only correct datat that is within ADC range
      if(data[i] < 1.0)
      {
         data[i] *= ccs_data->factory_acor_cal.acor[i];
      }
   }

   
   return VI_SUCCESS;
}

#else
/*---------------------------------------------------------------------------
 Aquire Raw Scan Data - aquires the raw scan data to inverted values normed
 to one.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_aquireRawScanData(ViSession instrumentHandle, ViUInt16 raw[], ViReal64 data[])
{
#define NO_DARK_PIXELS                 12       // we got 12 dark pixels
#define DARK_PIXELS_OFFSET             16       // dark pixels start at positon 16 within raw data
#define SCAN_PIXELS_OFFSET             32       // real measurement start at position 32 within raw data
#define MAX_ADC_VALUE                  0xFFFF

   CCS_SERIES_data_t    *ccs_data;
   ViStatus err = VI_SUCCESS;
   ViReal64 norm_com = 0.0;
   ViReal64 dark_even = 0.0;
   ViReal64 dark_odd  = 0.0;

   int i = 0;

   // get private data
   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &ccs_data)) != VI_SUCCESS) return err;

   // sum the dark Pixels
   for(i = 0; i < NO_DARK_PIXELS; i+= 2)
   {
      dark_even += raw[(DARK_PIXELS_OFFSET + i + 0)];
      dark_odd  += raw[(DARK_PIXELS_OFFSET + i + 1)];
   }

   // calculate dark current average
   dark_even /= (double)(NO_DARK_PIXELS / 2);
   dark_odd  /= (double)(NO_DARK_PIXELS / 2);

   // calculate normalizing factor
   if(dark_even > dark_odd)
   {
      norm_com = 1.0 / ((ViReal64)MAX_ADC_VALUE - dark_even);
   }
   else
   {
      norm_com = 1.0 / ((ViReal64)MAX_ADC_VALUE - dark_odd);
   }


   for(i = 0; i < CCS_SERIES_NUM_PIXELS; i+= 2)
   {
      data[i+0] = (((ViReal64)raw[SCAN_PIXELS_OFFSET + i + 0]) - dark_even) * norm_com;
      data[i+1] = (((ViReal64)raw[SCAN_PIXELS_OFFSET + i + 1]) - dark_odd ) * norm_com;
   }

   for(i = 0; i < CCS_SERIES_NUM_PIXELS; i++)
   {
      data[i] *= ccs_data->factory_acor_cal.acor[i];
   }

   
   return VI_SUCCESS;
}


#endif


/*---------------------------------------------------------------------------
   Function:   Get Wavelength Parameters
   Purpose:    This function reads the parameters necessary to calculate from
               pixels to wavelength and vice versa stored in EEPROM of the
               spectrometer and stores the values in the CCS_SERIES_data_t structure.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_getWavelengthParameters (ViSession instr)
{
   CCS_SERIES_data_t *data;
   ViStatus       err = VI_SUCCESS;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // set the factory calibration valid flag to false
   data->factory_cal.valid = 0;
   
   // read factory adjustment coefficients from EEPROM
   CCSseries_readEEFactoryPoly(instr, data->factory_cal.poly);
   
   CCSseries_poly2wlArray(&(data->factory_cal));
   
   // read user adjustment nodes from EEPROM and calculate coefficients and wavelength array
   data->user_cal.valid = 0;
   err = CCSseries_readEEUserPoints(instr, data->user_points.user_cal_node_pixel, data->user_points.user_cal_node_wl, &(data->user_points.user_cal_node_cnt));
   if(err == VI_SUCCESS) err = CCSseries_nodes2poly(data->user_points.user_cal_node_pixel, data->user_points.user_cal_node_wl, data->user_points.user_cal_node_cnt, data->user_cal.poly);
   if(err == VI_SUCCESS) err = CCSseries_poly2wlArray(&(data->user_cal));
   if(err == VI_SUCCESS) data->user_cal.valid = 1;
                                        
   return VI_SUCCESS;   // errors ignored by intention
}





/*---------------------------------------------------------------------------
   Function:   Check Nodes
   Purpose:    Checks the calibration nodes for ascending.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_checkNodes(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt)
{
   int      i;
   ViInt32  p;
   ViReal64 d;
   int      iDirectionFlagWl = 0;   // 1 means increasing, -1 means decreasing, 0 is an error
   int      iDirectionFlagPx = 0;   // 1 means increasing, -1 means decreasing, 0 is an error

   // check valid buffer length and determine target
   if((cnt < CCS_SERIES_MIN_NUM_USR_ADJ) || (cnt > CCS_SERIES_MAX_NUM_USR_ADJ)) return VI_ERROR_CCS_SERIES_INV_USER_DATA;

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
      return VI_ERROR_CCS_SERIES_INV_USER_DATA;
   
   
   if(pixel[0] < pixel[1])
   {
      iDirectionFlagPx = 1;   
   }
   else if(pixel[0] > pixel[1])  
   {
      iDirectionFlagPx = -1;  
   }
   else
      return VI_ERROR_CCS_SERIES_INV_USER_DATA;
   
   
   // check pixel range
   if((pixel[0] < 0) || (pixel[cnt - 1] > (CCS_SERIES_NUM_PIXELS - 1))) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
   
   // check wavelength range
   if(wl[0] <= 0.0)  return VI_ERROR_CCS_SERIES_INV_USER_DATA; 
   
   // check monoton ascending wavelength and pixel values    
   p = pixel[0];
   d = wl[0];
   
   for(i = 1; i < cnt; i++)
   {
      // check increasing pixels...
      if(iDirectionFlagPx == 1)
      {
         if(pixel[i] <= p) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      }
      else
      {
         if(pixel[i] >= p) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      }
         
         
      if(iDirectionFlagWl == 1)  // increasing
      {
         if(wl[i] <= d) return VI_ERROR_CCS_SERIES_INV_USER_DATA; 
      }
      else
      {
         if(wl[i] >= d) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      }

      p = pixel[i];
      d = wl[i];
   } 
   
   /*
   for(i = 1; i < cnt; i++)
   {
      // check increasing pixels...
      if(pixel[i] <= p) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      
      if(iDirectionFlag == 1) // increasing
      {
         if(wl[i] <= d) return VI_ERROR_CCS_SERIES_INV_USER_DATA; 
      }
      else
      {
         if(wl[i] >= d) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      }

      p = pixel[i];
      d = wl[i];
   } 
   */
   
   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Nodes to Polynome
   Purpose:    Calculates polynome coefficients from user defined supporting
               points.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_nodes2poly(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt, ViReal64 poly[])
{
   if(LeastSquareInterpolation ((int *)pixel, (double *)wl, (int)cnt, (double *)poly)) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Polynom to Wavelength Array
   Purpose:    Calculates wavelenth array from polynom coefficients.
               The poly array must contain 4 elements.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_poly2wlArray(CCS_SERIES_wl_cal_t *wl)
{
   int      i;
   ViReal64 d = 0.0;
   int iDirectionFlag = 0; // 1 means increasing, -1 means decreasing, 0 is an error
   
   for (i = 0; i < CCS_SERIES_NUM_PIXELS; i++)
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
      return VI_ERROR_CCS_SERIES_INV_USER_DATA;
   
   
   d = wl->wl[0];
   for(i = 1; i < CCS_SERIES_NUM_PIXELS; i++)
   {
      if(iDirectionFlag == 1) // increasing
      {
         if(wl->wl[i] <= d)   return VI_ERROR_CCS_SERIES_INV_USER_DATA; 
      }
      else
      {
         if(wl->wl[i] >= d)   return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      }

      d = wl->wl[i];
   }
   
   if(iDirectionFlag == 1)
   {
      wl->min     = wl->poly[0];
      wl->max     = wl->wl[CCS_SERIES_NUM_PIXELS - 1];
   }
   else
   {
      wl->min     = wl->wl[CCS_SERIES_NUM_PIXELS - 1]; 
      wl->max     = wl->poly[0];
   }
   
   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Read EEPROM factory calibration data polynom coefficients
   Purpose:    This function reads the polynome coefficients necessary to 
               calculate from pixels to wavelength and vice versa stored in 
               the EEPROM of the spectrometer and stores the values in the 
               poly array. The poly array must contain 4 elements.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_readEEFactoryPoly(ViSession instr, ViReal64 poly[])
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   ViUInt16       cnt = 0;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = CCSseries_readEEPROM(instr, EE_FACT_CAL_COEF_DATA, 0, EE_LENGTH_FACT_CAL_COEF_DATA, (ViBuf)poly, &cnt);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Write EEPROM factory calibration coefficients
   Purpose:    This function writes the factory calibration coefficients to the
               EEPROM.
               The coefficients array must contain four elements.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_writeEEFactoryPoly(ViSession instr, ViReal64 poly[])
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = CCSseries_writeEEPROM(instr, EE_FACT_CAL_COEF_DATA, 0, EE_LENGTH_FACT_CAL_COEF_DATA, (ViBuf)poly);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Reads EEPROM user calibration coefficients
   Purpose:    This function reads the user calibration coefficients to the
               EEPROM.
               The coefficients array must contain four elements.
---------------------------------------------------------------------------*/
#ifdef NOT_NEEDE_UNTIL_NOW
static ViStatus CCSseries_readEEUserPoly(ViSession instr, ViReal64 poly[])
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   ViUInt16       cnt = 0;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = CCSseries_readEEPROM(instr, EE_USER_CAL_COEF_DATA, 0, EE_LENGTH_USER_CAL_COEF_DATA, (ViBuf)poly, &cnt);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;
}
#endif   // NOT_NEEDE_UNTIL_NOW


/*---------------------------------------------------------------------------
   Function:   Write EEPROM user calibration coefficients
   Purpose:    This function writes the user calibration coefficients to the
               EEPROM.
               The coefficients array must contain four elements.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_writeEEUserPoly(ViSession instr, ViReal64 poly[])
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = CCSseries_writeEEPROM(instr, EE_USER_CAL_COEF_DATA, 0, EE_LENGTH_USER_CAL_COEF_DATA, (ViBuf)poly);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Reads EEPROM factory calibration flags
   Purpose:    This function reads the factory calibration flags from the
               EEPROM.
---------------------------------------------------------------------------*/
#ifdef NOT_NEEDE_UNTIL_NOW
static ViStatus CCSseries_readEEFactoryFlag(ViSession instr, ViPUInt16 flag)
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   ViUInt16       cnt = 0;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = CCSseries_readEEPROM(instr, EE_FACT_CAL_COEF_FLAG, 0, EE_LENGTH_FACT_CAL_COEF_FLAG, (ViBuf)flag, &cnt);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;

}
#endif   // NOT_NEEDE_UNTIL_NOW


/*---------------------------------------------------------------------------
   Function:   Write EEPROM factory calibration flags
   Purpose:    This function writes the factory calibration flags to the
               EEPROM.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_writeEEFactoryFlag(ViSession instr, ViUInt16 flag)
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // write to eeprom
   err = CCSseries_writeEEPROM(instr, EE_FACT_CAL_COEF_FLAG, 0, EE_LENGTH_FACT_CAL_COEF_FLAG, (ViBuf)&flag);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Reads EEPROM user calibration flags
   Purpose:    This function reads the user calibration flags from the
               EEPROM.
---------------------------------------------------------------------------*/
#ifdef NOT_NEEDE_UNTIL_NOW
static ViStatus CCSseries_readEEUserFlag(ViSession instr, ViPUInt16 flag)
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   ViUInt16       cnt = 0;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // write to eeprom
   err = CCSseries_readEEPROM(instr, EE_USER_CAL_COEF_FLAG, 0, EE_LENGTH_USER_CAL_COEF_FLAG, (ViBuf)flag, &cnt);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;
}
#endif   // NOT_NEEDE_UNTIL_NOW


/*---------------------------------------------------------------------------
   Function:   Write EEPROM user calibration flags
   Purpose:    This function writes the user calibration flags to the
               EEPROM.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_writeEEUserFlag(ViSession instr, ViUInt16 flag)
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // write to eeprom
   err = CCSseries_writeEEPROM(instr, EE_USER_CAL_COEF_FLAG, 0, EE_LENGTH_USER_CAL_COEF_FLAG, (ViBuf)&flag);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Reads EEPROM user calibration nodes
   Purpose:    This function reads the user-defined supporting points from the
               instruments EEPROM.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_readEEUserPoints(ViSession instr, ViUInt32 pixel[], ViReal64 wl[], ViPUInt16 cnt)
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   ViUInt16       tmp = 0;
   ViUInt8        buf[EE_LENGTH_USER_CAL_POINTS_DATA];

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // reads user calibration points count from eeprom
   err = CCSseries_readEEPROM(instr, EE_USER_CAL_POINTS_CNT, 0, EE_LENGTH_USER_CAL_POINTS_CNT, (ViBuf)cnt, &tmp);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   // reads to eeprom
   err = CCSseries_readEEPROM(instr, EE_USER_CAL_POINTS_DATA, 0, EE_LENGTH_USER_CAL_POINTS_DATA, (ViBuf)buf, &tmp);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   // fill data into buffer
   memcpy(pixel, buf, (*cnt) * sizeof(ViUInt32));
   memcpy(wl, &buf[CCS_SERIES_MAX_NUM_USR_ADJ * sizeof(ViUInt32)], (*cnt) * sizeof(ViReal64));
   
   return err;
}



/*---------------------------------------------------------------------------
   Function:   Write EEPROM user calibration nodes
   Purpose:    This function writes the user-defined supporting points to the
               instruments EEPROM.
               The nodes array must contain at least CCS_SERIES_MIN_NUM_USR_ADJ
               elements and a maximum of CCS_SERIES_MAX_NUM_USR_ADJ elements.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_writeEEUserPoints(ViSession instr, ViUInt32 pixel[], ViReal64 wl[], ViUInt16 cnt)
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   ViUInt8        buf[EE_LENGTH_USER_CAL_POINTS_DATA];

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // write user calibration points count to eeprom
   err = CCSseries_writeEEPROM(instr, EE_USER_CAL_POINTS_CNT, 0, EE_LENGTH_USER_CAL_POINTS_CNT, (ViBuf)&cnt);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   // fill data into buffer
   memcpy(buf, pixel, cnt * sizeof(ViUInt32));
   memcpy(&buf[CCS_SERIES_MAX_NUM_USR_ADJ * sizeof(ViUInt32)], wl, cnt * sizeof(ViReal64));

   // write to eeprom
   err = CCSseries_writeEEPROM(instr, EE_USER_CAL_POINTS_DATA, 0, EE_LENGTH_USER_CAL_POINTS_DATA, (ViBuf)&buf);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Gets the firmware revision
   Purpose:    This function requests the firmware version and stores the
               information into device structure.
---------------------------------------------------------------------------*/
static ViStatus CCSseries_getFirmwareRevision(ViSession instr)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViUInt8 buf[CCS_SERIES_NUM_VERSION_BYTES];
   CCS_SERIES_data_t    *data; 
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // request the data
   err = CCSseries_USB_in(instr, CCS_SERIES_RCMD_PRODUCT_INFO, CCS_SERIES_FIRMWARE_VERSION, 0, CCS_SERIES_NUM_VERSION_BYTES * sizeof(ViUInt8), (ViBuf)buf, &read_bytes);
   
   // error mapping
   if((read_bytes != (CCS_SERIES_NUM_VERSION_BYTES * sizeof(ViUInt8))) && (!err))   err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE; 
   
   // check for errors
   if((err = CCSseries_checkErrorLevel(instr, err))) return (err);
   
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
---------------------------------------------------------------------------*/
static ViStatus CCSseries_getHardwareRevision(ViSession instr)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   ViUInt8 buf[CCS_SERIES_NUM_VERSION_BYTES];
   CCS_SERIES_data_t    *data; 
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // request the data
   err = CCSseries_USB_in(instr, CCS_SERIES_RCMD_PRODUCT_INFO, CCS_SERIES_HARDWARE_VERSION, 0, CCS_SERIES_NUM_VERSION_BYTES * sizeof(ViUInt8), (ViBuf)buf, &read_bytes);
   
   // error mapping
   if((read_bytes != (CCS_SERIES_NUM_VERSION_BYTES * sizeof(ViUInt8))) && (!err))   err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE; 
   
   // check for errors
   if((err = CCSseries_checkErrorLevel(instr, err))) return (err);
   
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
---------------------------------------------------------------------------*/
static ViStatus CCSseries_getAmplitudeCorrection(ViSession instr)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 read_bytes = 0;
   CCS_SERIES_data_t    *data; 
   int adj_corrupted_flag;
   int i;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // request the data for factory amplitude factors
   err = CCSseries_readEEPROM(instr, EE_ACOR_FACTORY, 0, EE_LENGTH_ACOR, (ViBuf)data->factory_acor_cal.acor, &read_bytes);
   
   // error mapping
   if((read_bytes != EE_LENGTH_ACOR) && (!err))   err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE; 

   // when checksum failed but everything else was ok -> assume a blank or new device, generate an array of 1.000 and write it to the EEPROM
   if(err == VI_ERROR_CYEEPROM_CHKSUM)
   {
      for(i=0; i<CCS_SERIES_NUM_PIXELS; i++)
      {
         data->factory_acor_cal.acor[i] = 1.0;
      }
      err = CCSseries_writeEEPROM(instr, EE_ACOR_FACTORY, 0, EE_LENGTH_ACOR, (ViBuf)data->factory_acor_cal.acor);
   }
   else
   {
      adj_corrupted_flag = 0;
      for(i=0; i<CCS_SERIES_NUM_PIXELS; i++)
      {
         if(data->factory_acor_cal.acor[i] < CCS_SERIES_AMP_CORR_FACT_MIN)
         {
            data->factory_acor_cal.acor[i] = CCS_SERIES_AMP_CORR_FACT_MIN;
            adj_corrupted_flag = 1;
         }
         if(data->factory_acor_cal.acor[i] > CCS_SERIES_AMP_CORR_FACT_MAX)
         {
            data->factory_acor_cal.acor[i] = CCS_SERIES_AMP_CORR_FACT_MAX;
            adj_corrupted_flag = 1;
         }
      }
      if(adj_corrupted_flag)  err = VI_ERROR_CCS_SERIES_INV_ADJ_DATA;
   }

   // check for errors
   if((err = CCSseries_checkErrorLevel(instr, err))) return (err);

   
   // now request the data for user amplitude factors
   err = CCSseries_readEEPROM(instr, EE_ACOR_USER, 0, EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor, &read_bytes);
   
   // error mapping
   if((read_bytes != EE_LENGTH_ACOR) && (!err))   err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE; 

   // when checksum failed but everything else was ok -> assume a blank or new device, generate an array of 1.000 and write it to the EEPROM
   if(err == VI_ERROR_CYEEPROM_CHKSUM)
   {
      for(i=0; i<CCS_SERIES_NUM_PIXELS; i++)
      {
         data->user_acor_cal.acor[i] = 1.0;
      }
      err = CCSseries_writeEEPROM(instr, EE_ACOR_USER, 0, EE_LENGTH_ACOR, (ViBuf)data->user_acor_cal.acor);
   }
   else
   {
      adj_corrupted_flag = 0;
      for(i=0; i<CCS_SERIES_NUM_PIXELS; i++)
      {
         if(data->user_acor_cal.acor[i] < CCS_SERIES_AMP_CORR_FACT_MIN)
         {
            data->user_acor_cal.acor[i] = CCS_SERIES_AMP_CORR_FACT_MIN;
            adj_corrupted_flag = 1;
         }
         if(data->user_acor_cal.acor[i] > CCS_SERIES_AMP_CORR_FACT_MAX)
         {
            data->user_acor_cal.acor[i] = CCS_SERIES_AMP_CORR_FACT_MAX;
            adj_corrupted_flag = 1;
         }
      }
      if(adj_corrupted_flag)  err = VI_ERROR_CCS_SERIES_INV_ADJ_DATA;
   }
      

   // check for errors
   if((err = CCSseries_checkErrorLevel(instr, err))) return (err);

   
   return err;  
}


/*---------------------------------------------------------------------------
   Function:   Set Dark Current Offset
   Purpose:    This function writes the dark current values for even and
               odd pixels to EEPROM of the spectrometer and stores
               the values in the CCS_SERIES_data_t structure.
---------------------------------------------------------------------------*/
__declspec(dllexport) ViStatus CCSseries_setDarkCurrentOffset(ViSession instr, ViUInt16 evenOffset, ViUInt16 oddOffset)
{
   ViStatus err = VI_SUCCESS;
   CCS_SERIES_data_t    *data;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // writes user even offset
   err = CCSseries_writeEEPROM(instr, EE_EVEN_OFFSET_MAX, 0, EE_LENGTH_OFFSET_MAX, (ViBuf)&evenOffset);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   // store even offset
   data->evenOffsetMax = evenOffset;
   
   // writes user even offset
   err = CCSseries_writeEEPROM(instr, EE_ODD_OFFSET_MAX, 0, EE_LENGTH_OFFSET_MAX, (ViBuf)&oddOffset);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   // store even offset
   data->oddOffsetMax = oddOffset;
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get Dark Current Offset
   Purpose:    This function reads the dark current values for even and
               odd pixels stored in EEPROM of the spectrometer and stores
               the values in the CCS_SERIES_data_t structure.
---------------------------------------------------------------------------*/
__declspec(dllexport) ViStatus CCSseries_getDarkCurrentOffset(ViSession instr, ViPUInt16 even, ViPUInt16 odd)
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   ViUInt16       cnt = 0;
   ViUInt16       tmp;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // reads user even offset
   err = CCSseries_readEEPROM(instr, EE_EVEN_OFFSET_MAX, 0, EE_LENGTH_OFFSET_MAX, (ViBuf)&tmp, &cnt);
   
   // in case a checksum error occured we use default values
   if(err == VI_ERROR_CYEEPROM_CHKSUM)
   {
      tmp = 0xFFFF;
      err = VI_SUCCESS;
   }
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   // store even offset
   data->evenOffsetMax = tmp;
   
   // reads user odd offset
   err = CCSseries_readEEPROM(instr, EE_ODD_OFFSET_MAX, 0, EE_LENGTH_OFFSET_MAX, (ViBuf)&tmp, &cnt);
   
   // in case a checksum error occured we use default values
   if(err == VI_ERROR_CYEEPROM_CHKSUM)
   {
      tmp = 0xFFFF;
      err = VI_SUCCESS;
   }
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   // store even offset
   data->oddOffsetMax = tmp;
   
   if(even) *even = data->evenOffsetMax;
   if(odd)  *odd = data->oddOffsetMax; 
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Set Serial Number
   Purpose:    This function stores the serial number to structure and eeprom.
---------------------------------------------------------------------------*/
__declspec(dllexport) ViStatus CCSseries_setSerialNumber(ViSession instr, ViPChar serial)
{
   ViStatus       err = VI_SUCCESS;    
   CCS_SERIES_data_t    *data;
   char           buf[CCS_SERIES_SERIAL_NO_LENGTH];

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // copy the string
   strncpy(buf, serial, (CCS_SERIES_SERIAL_NO_LENGTH - 1)); // strncpy will fill with '\0' when userText is smaller than (USER_LABEL_LENGTH-1)
   
   // truncate 
   buf[CCS_SERIES_SERIAL_NO_LENGTH-1] = '\0';

   // write to eeprom
   err = CCSseries_writeEEPROM(instr, EE_SERIAL_NO, 0, CCS_SERIES_SERIAL_NO_LENGTH, (ViBuf)buf);
   
   // check for error
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);
   
   strncpy(data->serNr, buf, sizeof(char) * CCS_SERIES_SERIAL_NO_LENGTH);
   
   return err;
}


/*-----------------------------------------------------------------------------
  Get the raw data as VIUInt16
-----------------------------------------------------------------------------*/
static ViStatus CCSseries_getRawData(ViSession instr, ViUInt16 data[])
{
   ViStatus err         = VI_SUCCESS;
   ViUInt32 read_bytes  = 0;

   // read the raw scan data
   err = CCSseries_USB_read(instr, (ViBuf)data, CCS_SERIES_NUM_RAW_PIXELS * sizeof(ViUInt16), &read_bytes);
   
   // error mapping
   if((read_bytes != CCS_SERIES_NUM_RAW_PIXELS * sizeof(ViUInt16)) & (!err)) err = VI_ERROR_CCS_SERIES_READ_INCOMPLETE;
   
   // check for errors 
   if((err = CCSseries_checkErrorLevel(instr, err)))  return (err);  
   
   return (err);
}



/*-----------------------------------------------------------------------------
  Optimized CRC-16 calculation. (Derived from AVR-libc).
  This CRC is normally used in disk-drive controllers.
   Polynomial: x^16 + x^15 + x^2 + 1 (0xa001)
   Initial value: 0xffff
  Return value: crc
-----------------------------------------------------------------------------*/
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


/*-----------------------------------------------------------------------------
  Calculate CRC-16 of given data.
  Return value: crc
-----------------------------------------------------------------------------*/
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


/*-----------------------------------------------------------------------------
  Least Square logarithm.
-----------------------------------------------------------------------------*/
static int LeastSquareInterpolation (int * PixelArray, double * WaveLengthArray, int iLength, double Coefficients[])
{
   double B[MATRIX_ROWS];
   double A[MATRIX_ROWS][MATRIX_COLS],C[MATRIX_ROWS][MATRIX_COLS];
   double D[MATRIX_ROWS][MATRIX_COLS],E[MATRIX_ROWS][MATRIX_COLS],F[MATRIX_ROWS][MATRIX_COLS];
   double detA;
   double CompleteWavelengthArray[CCS_SERIES_NUM_PIXELS];
   int i;
   
   if(iLength < MATRIX_ROWS)
      return -1;
   
   // clear CompleteWavelengthArray
   memset(CompleteWavelengthArray, 0, sizeof(double) * CCS_SERIES_NUM_PIXELS);
   
   // fill CompleteWavelengthArray with the supporting points
   for(i = 0; i < iLength; i++)
   {
      if((PixelArray[i] < 0) || (PixelArray[i] > CCS_SERIES_NUM_PIXELS))   break;
      CompleteWavelengthArray[PixelArray[i]] = WaveLengthArray[i];   
   }

   
   A[0][0] = iLength;
   A[0][1] = SpecSummation(CompleteWavelengthArray,1,CCS_SERIES_NUM_PIXELS);
   A[0][2] = SpecSummation(CompleteWavelengthArray,2,CCS_SERIES_NUM_PIXELS);
   A[0][3] = SpecSummation(CompleteWavelengthArray,3,CCS_SERIES_NUM_PIXELS);

   A[1][0] = SpecSummation(CompleteWavelengthArray,1,CCS_SERIES_NUM_PIXELS);
   A[1][1] = SpecSummation(CompleteWavelengthArray,2,CCS_SERIES_NUM_PIXELS);
   A[1][2] = SpecSummation(CompleteWavelengthArray,3,CCS_SERIES_NUM_PIXELS);
   A[1][3] = SpecSummation(CompleteWavelengthArray,4,CCS_SERIES_NUM_PIXELS);

   A[2][0] = SpecSummation(CompleteWavelengthArray,2,CCS_SERIES_NUM_PIXELS);
   A[2][1] = SpecSummation(CompleteWavelengthArray,3,CCS_SERIES_NUM_PIXELS);
   A[2][2] = SpecSummation(CompleteWavelengthArray,4,CCS_SERIES_NUM_PIXELS);
   A[2][3] = SpecSummation(CompleteWavelengthArray,5,CCS_SERIES_NUM_PIXELS);

   A[3][0] = SpecSummation(CompleteWavelengthArray,3,CCS_SERIES_NUM_PIXELS);
   A[3][1] = SpecSummation(CompleteWavelengthArray,4,CCS_SERIES_NUM_PIXELS);
   A[3][2] = SpecSummation(CompleteWavelengthArray,5,CCS_SERIES_NUM_PIXELS);
   A[3][3] = SpecSummation(CompleteWavelengthArray,6,CCS_SERIES_NUM_PIXELS);
   
   
   B[0] = Summation(CompleteWavelengthArray,CCS_SERIES_NUM_PIXELS);
   B[1] = Spec2Summation(CompleteWavelengthArray,1,CCS_SERIES_NUM_PIXELS);
   B[2] = Spec2Summation(CompleteWavelengthArray,2,CCS_SERIES_NUM_PIXELS);
   B[3] = Spec2Summation(CompleteWavelengthArray,3,CCS_SERIES_NUM_PIXELS);
   
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

static double Summation(double *pdata, int values)
{
   double tmp=0.0;

   do {
      tmp += *pdata;
      ++pdata;
   } while (--values);

   return(tmp);
}


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
