import usb.core
import usb.util
import const
from array import array

class LLIO: 
    """This class handles low level usb communication with a Thorlabs ccs-device.
    Communication is done with pyusb. Includes control IN/OUT transfers and raw (bulk) read.
    """
    def __init__(self, vid=0x1313, pid=0x8087):
        self.vid = vid
        self.pid = pid
        self.dev = None
        self.bulk_in_pipe = None
        self.timeout = None
        self._connect()


    def _connect(self):
        try:
            self.dev = usb.core.find(idVendor=self.vid, idProduct=self.pid, find_all=False)
            if self.dev is None:
                raise ValueError('Device not found')
        except usb.core.USBError as e:
            raise ConnectionError(f"Failed to connect to device: {e}")


    def __del__(self):
        if self.dev is not None:
            self.close()
        print("Device disconnected.")   # FIXME: LLIO shouldn't print.

    
    def open(self):
        self.dev.set_configuration()
        usb.util.claim_interface(self.dev, 0)      
        self.bulk_in_pipe = const.LL_DEFAULT_BULK_IN_PIPE    
        self.timeout = const.LL_DEFAULT_TIMEOUT
        self.flush()


    def close(self):
        usb.util.release_interface(self.dev, 0)
        usb.util.dispose_resources(self.dev)
        self.dev = None


    def get_bulk_in_status(self):
        # Prepare the request
        bmRequestType = usb.util.build_request_type(usb.util.ENDPOINT_IN, usb.util.CTRL_TYPE_STANDARD, usb.util.CTRL_RECIPIENT_ENDPOINT)
        bRequest = 0
        wValue = 0
        wIndex = self.bulk_in_pipe
        length = 2
        attrValue = "DEFAULT"
        # Perform the control transfer
        statusdata = self.dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, length, timeout=self.timeout)
        
        # Check the returned status
        if len(statusdata) < 2:
            attrValue = "USB_PIPE_STATE_UNKNOWN"
        if statusdata[0] & 1:  # Halt bit
            attrValue = "USB_PIPE_STALLED"
        else:
            attrValue = "USB_PIPE_READY"

        return attrValue


    def flush(self):
        """Reads the bulk_in_pipe until timeout and throws out the result.
        
        """
        try:
            full_flush_size = const.CCS_SERIES_NUM_RAW_PIXELS * 2
            self.dev.read(self.bulk_in_pipe, full_flush_size, timeout = self.timeout) 
            return True
        except:
            return True


    def read_raw(self, readTo: array):
        """Bulk read from default bulk_in_pipe. Note: Reading is done in bytes. Catches errors with try-except

        Args:
            readTo (array): data is read into this. The size of the array specifies the size of the read.
        """
        try:
            self.dev.read(self.bulk_in_pipe, readTo, timeout=self.timeout)
        except usb.core.USBError as e:
            print(f"USB error in read_raw: {e}")
        

    def control_out(self, bRequest, payload, bmRequestType = 0x40, wValue = 0, wIndex= 0):
        """Sends a control OUT transfer to the device. (usually) For setting data.

        Args:
            bRequest (hexadecimal): type of request, provided in const.py
            payload (array): data payload to be transfered with the command, if needed.
            bmRequestType (hexadecimal, optional): specifies type of transfer. Defaults to 0x40.
            wValue (int, optional): Defaults to 0.
            wIndex (int, optional): Defaults to 0.
        """
        try:
            self.dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, payload, timeout=self.timeout)
        except usb.core.USBError as e:
            print(f"USB error in control_out: {e}")

        
    def control_in(self, bRequest, readTo: array, bmRequestType = 0xC0, wValue = 0, wIndex= 0):
        """Sends a control IN transfer and reads data. (usually) For reading data. 

        Args:
            bRequest (hexadecimal): type of request, provided in const.py
            readTo (array): data is read into this. The size of the array specifies the size of the read.
            bmRequestType (hexadecimal, optional): specifies type of transfer.. Defaults to 0xC0.
            wValue (int, optional): Defaults to 0.
            wIndex (int, optional): Defaults to 0.

        """
        try:
            self.dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, readTo, timeout=self.timeout)
        except usb.core.USBError as e:
            print(f"USB error in control_in: {e}")

    

