/* low level communications with SPx device */

#include <stdlib.h>
#include <usb.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "vitypes.h"
#include "spxusb.h"

// save spxdrv.h
#define SPX_BUFFER_SIZE            256
#define SPX_ERR_DESCR_BUFFER_SIZE  512

/* There's no point in passing session data around. I only care about one
 * device, opened once */
struct session {
        int timeout;
        struct usb_dev_handle *usbhandle;
        int bulk_in_pipe;
        int bulk_out_pipe;
        void* user_data;  //SPX_data_t* or CCS_SERIES_data_t*
        int usb_end_in;
        unsigned short vid;
        unsigned short pid;
        int usbtimeout;
};

static struct session sess;


// called from SPX_init
ViStatus viOpenDefaultRM(ViPSession vi){
        // No need for this, so give it a dummy handle
        *vi = (ViSession)2;
        return VI_SUCCESS;
}

// called from SPX_init -- we find the USB device and stash its parameters
// in the ViSession
// Our syntax for the name is just hex:hex vid:pid[:optionalserial]  -- not the same as VISA
ViStatus viOpen(ViSession sesn, ViRsrc name, ViAccessMode mode,
                                    ViUInt32 timeout, ViPSession vi){
    unsigned short vid, pid;
    vid = 0;
    pid = 0;
    // Ignore the serial for now. sscanf is happy if not there.   Maybe match it one day.
    sscanf(name, "%hx:%hx", &vid, &pid);
    sess.usbhandle = find_usb_device(vid, pid);
    if (! sess.usbhandle) {
        return VI_ERROR_RSRC_NFOUND;
    }
    if(usb_claim_interface(sess.usbhandle, 0)) {
        return VI_ERROR_RSRC_BUSY;
    } 
    sess.timeout = timeout;  /// TODO this may be only for this function; may be set to null so use something else for usb
    sess.usbtimeout = 3000;
    
    // Stash for viGetAttribute calls
    sess.vid = vid;
    sess.pid = pid;
    
    *vi = (ViSession)1;
    return VI_SUCCESS;
}


// called from  SPX_init (cleanup)  SPX_initCleanUp  SPX_initClose
ViStatus viClose (ViObject vi){
        if (vi == 1) {  // the main session
            usb_release_interface(sess.usbhandle, 0);
            usb_reset(sess.usbhandle);
            usb_close(sess.usbhandle);
        }
    return VI_SUCCESS;
}

// called from CCSseries_init
ViStatus viClear(ViSession vi) {
///        TODO implement me  - I have no idea whether this null implementation works
        return VI_SUCCESS;
        
        /* The visa manual says VISA must send the INITIATE_CLEAR and CHECK_CLEAR_STATUS commands on the control pipe.
         * http://svn.openmoko.org/developers/werner/ahrt/host/tmc/usbtmc.c dose something like this
         * Unfortunately, what I have here gives a -32 broken pipe error at the usb_control_msg.
        
        nbytes = usb_control_msg(sess.usbhandle, 
        0xA1,         //USB_ENDPOINT_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE  // req type
        0x5,          //INITIATE_CLEAR,           //bRequest 
        0,            //wValue, 
        0,            //wIndex,   (interface number)
        buf, 
        1,            //wLength,
        sess.usbtimeout);
        
        if (nbytes < 0) {
            printf("INITIATE_CLEAR failed %d\n", nbytes);    // -32 broken pipe
            printf("INITIATE_CLEAR status? 0x%02x\n", buf[0]);
            printf("strerror says %s\n", usb_strerror());
            return VI_ERROR_IO;
        }
        printf("INITIATE_CLEAR status 0x%02x\n", buf[0]);
        /// TODO loop on CHECK_CLEAR_STATUS  then flush bulk
        return VI_SUCCESS;
        */
}

// called from   SPX_init(setting usb params, data locn)
ViStatus viSetAttribute(ViObject vi, ViAttr attrName, ViAttrState attrValue){
        switch (attrName) {
        case VI_ATTR_TMO_VALUE:
            sess.timeout = attrValue;
            break; 
        case VI_ATTR_USB_BULK_IN_PIPE:
            sess.bulk_in_pipe = attrValue;
            break;
        case VI_ATTR_USB_BULK_OUT_PIPE:
            sess.bulk_out_pipe = attrValue;
            break;
        case VI_ATTR_USB_END_IN:
            sess.usb_end_in = attrValue;  // what is this? (4) not used anywhere - maybe usb wants it?
            break; 
        case VI_ATTR_USER_DATA:
            sess.user_data = (void*)attrValue;  // a ptr that we are saving  
            break;
        default:
            return VI_ERROR_INV_PARAMETER;
        }
        return VI_SUCCESS;
}

// called from   SPX_init, a lot of places to get userdata, 
ViStatus viGetAttribute(ViObject vi, ViAttr attrName, void *attrValue){
int ret;
    char statusdata[2];
    struct usb_device *dev;
    dev = usb_device(sess.usbhandle);
        
    switch (attrName) {
        case VI_ATTR_USER_DATA:
            *(void**)attrValue = sess.user_data;
            break;
            
        case VI_ATTR_MANF_ID:
                *(short*)attrValue = sess.vid;
            break;
            
        case VI_ATTR_MODEL_CODE:
                *(short*)attrValue = sess.pid;
            break;
            
        case VI_ATTR_MANF_NAME: 
            usb_get_string_simple(sess.usbhandle, dev->descriptor.iManufacturer,
                                             (char*)attrValue, SPX_BUFFER_SIZE);
            break;
        case VI_ATTR_MODEL_NAME:
            usb_get_string_simple(sess.usbhandle, dev->descriptor.iProduct,
                                             (char*)attrValue, SPX_BUFFER_SIZE);
            break;
        case VI_ATTR_USB_SERIAL_NUM:
            usb_get_string_simple(sess.usbhandle, dev->descriptor.iSerialNumber,
                                             (char*)attrValue, SPX_BUFFER_SIZE);        
            break;

        case VI_ATTR_USB_BULK_IN_STATUS:
            // getstatus request
            ret = usb_control_msg(sess.usbhandle, 
                USB_ENDPOINT_IN|USB_TYPE_STANDARD|USB_RECIP_ENDPOINT, // bmRequesttype
                USB_REQ_GET_STATUS, // bRequest
                0,  // wValue 
                sess.bulk_in_pipe, // wIndex 
                statusdata, // bytes returned
                2,          // size of return data
                sess.timeout);
            if (ret < 0) {
                *(ViInt16*)attrValue = VI_USB_PIPE_STATE_UNKNOWN;
                return VI_ERROR_IO;
            }
            if (statusdata[0] & 1) {  // halt bit
                *(ViInt16*)attrValue = VI_USB_PIPE_STALLED;
            } else {
                *(ViInt16*)attrValue = VI_USB_PIPE_READY;
            }
            return VI_SUCCESS;
            break;
           
        case VI_ATTR_RM_SESSION:  // return the dummy rmsession variable
            *(ViSession*)attrValue = (ViSession)2; 
            break;
            
        default:
            return VI_ERROR_INV_PARAMETER;
    } // end of switch(attrName)
    return VI_SUCCESS;
}

// called from    SPX_reset  to clean out buffers
ViStatus viFlush(ViSession vi, ViUInt16 mask){
        char buf[3068 * 2];
    usb_bulk_read(sess.usbhandle,
                      sess.bulk_in_pipe,
                      buf,
                      3068 * 2,
                      sess.timeout);  // throw away any result
        return VI_SUCCESS;
}

// called from   SPX_errorMessage
ViStatus viStatusDesc(ViObject vi, ViStatus status, ViChar desc[]){
        // Translate any messages we set
        char *msgtext;
        
        switch(status) {
        case VI_ERROR_IO:
            msgtext = "lowlevel io error";
            break;
        case VI_ERROR_INV_PARAMETER:
            msgtext = "lowlevel invalid param";
            break;
        case VI_WARN_UNKNOWN_STATUS:
            msgtext = "unknown status";
            break;
        case VI_ERROR_RSRC_NFOUND:
            msgtext = "device not found";
            break;    
        case VI_ERROR_RSRC_BUSY:
            msgtext = "device busy (cannot claim interface)";
            break;
        case VI_ERROR_TMO:
            msgtext = "device timed out";
            break;
        case VI_SUCCESS:
            msgtext = "lowlevel no error";
            break;
        default:
            msgtext = NULL;
        }
        
        if (msgtext) {
            strncpy(desc, msgtext, SPX_ERR_DESCR_BUFFER_SIZE);
            return VI_SUCCESS;
        } else {
                strncpy(desc, "lowlevel no idea", SPX_ERR_DESCR_BUFFER_SIZE);
                return VI_WARN_UNKNOWN_STATUS;
        }
}


// called from   SPX_acquireScanDataRaw  to read scan data
ViStatus viRead(ViSession vi, ViPBuf buf, ViUInt32 cnt, ViPUInt32 retCnt){
        // ViPbuf is unsigned char*, so almost ready for usb_bulk_read
        int nread;
        
        // some googling suggests viRead is supposed to send a bulk write
        // to specify max size of data first.  Does not seem to be needed.
        
        nread = usb_bulk_read(sess.usbhandle,
                      sess.bulk_in_pipe,
                      (char*)buf,    // cast to signed
                      cnt,    // will be 3068 * 2
                      sess.usbtimeout);    ///TODO watch out may not be correct
        
        if (nread == -ETIMEDOUT) {
                return VI_ERROR_TMO;
        }
        if (nread < 0) {
                return  VI_ERROR_IO;
        }

    *retCnt = nread;
        return VI_SUCCESS;
}


// called from   SPX_writeEEPROM, CCSseries_USB_out
ViStatus viUsbControlOut(ViSession vi, ViInt16 bmRequestType, ViInt16 bRequest,
                                    ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength,
                                    char* buf){
    int nbytes;
    nbytes = usb_control_msg(sess.usbhandle, bmRequestType, bRequest, wValue, wIndex, buf, wLength, sess.usbtimeout);
    if (nbytes < 0) {
        return VI_ERROR_IO;
    }
    return VI_SUCCESS;
}

// called from   SPX_readEEPROM, CCSseries_USB_in/out, CCSseries_USB_read/write
ViStatus viUsbControlIn(ViSession vi, ViInt16 bmRequestType, ViInt16 bRequest,
                        ViUInt16 wValue, ViUInt16 wIndex,
                        ViUInt16 wLength, char* buf, ViPUInt16 retCnt){
    int nread;
    nread = usb_control_msg(sess.usbhandle, bmRequestType, bRequest, wValue,
                                            wIndex, buf, wLength, sess.usbtimeout);
    if (nread < 0){
        return VI_ERROR_IO;
    }
    if (retCnt) {
        *retCnt = nread;
    }
    return VI_SUCCESS;
}





/* Find a device on the USB, given vendor and product ID.
 * Returns a handle for the opened device, or NULL if problems */
struct usb_dev_handle *find_usb_device(int vid, int pid) {
    struct usb_bus *busses;
    struct usb_bus *bus;
    struct usb_device *dev;
    int ret;

    usb_init();
    ret = usb_find_busses();
    if (ret < 0) {
        fprintf(stderr, "Failed to access /[dev|proc]/bus/usb\n");
        return NULL;
    }
    usb_find_devices();
    busses = usb_get_busses();
    for (bus = busses; bus; bus = bus->next) {
        for (dev = bus->devices; dev; dev = dev->next) {
            if (dev->descriptor.idVendor == vid && dev->descriptor.idProduct == pid) {
                return usb_open(dev);
            }
        }
    }
    fprintf(stderr, "Device not found on USB\n");
    return NULL;
}
