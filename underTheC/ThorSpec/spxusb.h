/* low level communications with SPx device *, replacing visa functions,
 * also other unavailable lib functions */
#ifndef __spxusb_h__
#define __spxusb_h__

#include "vitypes.h"

ViStatus viOpenDefaultRM(ViPSession vi);

ViStatus viOpen(ViSession sesn, ViRsrc name, ViAccessMode mode,
                                    ViUInt32 timeout, ViPSession vi);

ViStatus viClose(ViObject vi);

ViStatus viClear(ViSession vi);

ViStatus viSetAttribute(ViObject vi, ViAttr attrName, ViAttrState attrValue);

ViStatus viGetAttribute(ViObject vi, ViAttr attrName, void *attrValue);

ViStatus viFlush(ViSession vi, ViUInt16 mask);

ViStatus viStatusDesc(ViObject vi, ViStatus status, ViChar desc[]);

ViStatus viRead(ViSession vi, ViPBuf buf, ViUInt32 cnt, ViPUInt32 retCnt);

ViStatus viUsbControlOut(ViSession vi, ViInt16 bmRequestType, ViInt16 bRequest,
                                    ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength,
                                    char* buf);
                                    
ViStatus viUsbControlIn(ViSession vi, ViInt16 bmRequestType, ViInt16 bRequest,
                                    ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength,
                                    char* buf, ViPUInt16 retCnt);
                                                            
                                                            

/* Find a device on the USB, given vendor and product ID.
 * Returns a handle for the opened device, or NULL if problems */
struct usb_dev_handle *find_usb_device(int vid, int pid);

#endif
