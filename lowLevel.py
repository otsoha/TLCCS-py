import usb
import const
from array import array

class LLIO: 
    """This class handles low level usb communication with a Thorlabs ccs-device.
    Communication is done with pyusb. Includes control IN/OUT transfers and raw (bulk) read.
    """
    def __init__(self, vid=0x1313, pid=0x8087):
        # Try to connect and set default values for vars.
        try:
            self.dev = usb.core.find(idVendor=vid, idProduct=pid)
            if self.dev is None:
                raise ValueError('Device not found')
            self.dev.set_configuration()
            self.bulk_in_pipe = const.LL_DEFAULT_BULK_PIPE    
            self.timeout = const.LL_DEFAULT_TIMEOUT         

        # Didn't connect, lose all hope
        except usb.core.USBError as e:
            self.dev = None
            self.bulk_in_pipe = 0x00
            self.timeout = 0
            raise usb.core.USBError(e)


    def __del__(self):
        if self.dev != None:
            usb.util.dispose_resources(self.dev)
            print("Device disconnected.")   # FIXME: LLIO shouldn't print.

    # FIXME: does not work yet
    def flush(self):
        """Does not work right now, but should just read the messy buffers in case of an error.
        FIXME
        """
        full_flush_size = 3694 * 2
        self.dev.read(self.bulk_in_pipe, full_flush_size, timeout = self.timeout) 


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

    

