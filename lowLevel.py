import usb
import numpy as np
from array import array



# Function to print details of the device. Thanks chatGPT.
def print_device_details(device):
    # Print device info
    print("Device:")
    print(f"  idVendor: {hex(device.idVendor)}")
    print(f"  idProduct: {hex(device.idProduct)}")
    print(f"  Manufacturer: {usb.util.get_string(device, device.iManufacturer)}")
    print(f"  Product: {usb.util.get_string(device, device.iProduct)}")
    print(f"  Serial Number: {usb.util.get_string(device, device.iSerialNumber)}")
    print(f"  Device Class: {device.bDeviceClass}")
    print(f"  Device Subclass: {device.bDeviceSubClass}")
    print(f"  Device Protocol: {device.bDeviceProtocol}")
    print(f"  Max Packet Size: {device.bMaxPacketSize0}")
    print(f"  Number of Configurations: {device.bNumConfigurations}")

    # Print configuration info
    for config in device:
        print(f"\nConfiguration {config.bConfigurationValue}:")
        print(f"  Total Length: {config.wTotalLength}")
        print(f"  Number of Interfaces: {config.bNumInterfaces}")
        print(f"  Configuration Value: {config.bConfigurationValue}")
        print(f"  Configuration: {usb.util.get_string(device, config.iConfiguration)}")
        print(f"  Attributes: {config.bmAttributes}")
        print(f"  Max Power: {config.bMaxPower}mA")

        # Print interface info
        for interface in config:
            print(f"\n  Interface {interface.bInterfaceNumber}:")
            print(f"    Number of Endpoints: {interface.bNumEndpoints}")
            print(f"    Interface Class: {interface.bInterfaceClass}")
            print(f"    Interface Subclass: {interface.bInterfaceSubClass}")
            print(f"    Interface Protocol: {interface.bInterfaceProtocol}")
            print(f"    Interface: {usb.util.get_string(device, interface.iInterface)}")

            # Print endpoint info
            for endpoint in interface:
                print(f"\n    Endpoint {endpoint.bEndpointAddress}:")
                print(f"      Attributes: {endpoint.bmAttributes}")
                print(f"      Endpoint Address: {endpoint.bEndpointAddress}")
                print(f"      Max Packet Size: {endpoint.wMaxPacketSize}")
                print(f"      Interval: {endpoint.bInterval}")


# Constants
LL_SUCCESS = 0
LL_ERROR = -1

#FIXME: Create better error detection and reporting.
class LLIO: 
    """Umm so this is just a wrapper for like 4 lines of code. I like overhead.
    """
    def __init__(self, vid=0x1313, pid=0x8087):
        try:
            self.dev = usb.core.find(idVendor=vid, idProduct=pid)
            if self.dev is None:
                raise ValueError('Device not found')
            # Might not be necessary but doesn't hurt.
            self.dev.set_configuration()
            self.bulk_in_pipe = 0x86    #FIXME: remove magic number 
            self.timeout = 2000         #FIXME: remove magic number
            print("Device connected")
        except usb.core.USBError as e:
            print(f"Error in initialization: {e}")
            self.dev = None
        
    def __del__(self):
        # Prolly wont be necessary to free manually, but it doesn't hurt.
        if self.dev != None:
            usb.util.dispose_resources(self.dev)
        print("Device disconnected.")

    def flush(self):
        full_flush_size = 3694 * 2
        self.dev.read(self.bulk_in_pipe, full_flush_size, timeout = self.timeout) 

    def read_raw(self, readTo: array):
        try:
            self.dev.read(self.bulk_in_pipe, readTo, timeout=self.timeout)
        except usb.core.USBError as e:
            print(f"USB Error: {e}")
            raise
        

    def control_out(self, bRequest, payload, bmRequestType = 0x40, wValue = 0, wIndex= 0):
        """Sends a command to CCS with an attached payload. For example for setting data

        Args:
            bRequest (hexadecimal): _description_
            payload (_type_): _description_
            bmRequestType (hexadecimal, optional): _description_. Defaults to 0x40.
            wValue (int, optional): _description_. Defaults to 0.
            wIndex (int, optional): _description_. Defaults to 0.
        """  
        try:
            self.dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, payload, timeout=self.timeout)
        except usb.core.USBError as e:
            print(f"USB Error: {e}")
            raise
        
    def control_in(self, bRequest, readTo: array, bmRequestType = 0xC0, wValue = 0, wIndex= 0):
        """Sends a command to the CCS and watches for returned data. For example for reading data.

        Args:
            bRequest (_type_): _description_
            readTo (array): the array in which the output is read
            bmRequestType (_type_, optional): _description_. Defaults to 0xC0.
            wValue (int, optional): _description_. Defaults to 0.
            wIndex (int, optional): _description_. Defaults to 0.

        """
        try:
            self.dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, readTo, timeout=self.timeout)
        except usb.core.USBError as e:
            print(f"USB Error: {e}")
            raise
    

