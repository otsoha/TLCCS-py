from lowLevel import LLIO
from array import array
import const
import math
import usb
import numpy as np
import time
import struct
import matplotlib.pyplot as plt

class CCSDRV:

    def __init__(self):
        self.io = None
        self.dev = None

    def open(self, vid=0x1313, pid=0x8087):
        self.io = LLIO(vid, pid)
        self.dev = self.io.dev
        # Set default integration time
        assert self.set_integration_time(const.CCS_SERIES_DEF_INT_TIME)

    def get_integration_time(self):
        # Create a buffer of the required size
        readTo = usb.util.create_buffer(const.CCS_SERIES_NUM_INTEG_CTRL_BYTES)
        
        # Read the data
        self.io.control_in(const.CCS_SERIES_RCMD_INTEGRATION_TIME, readTo)
        
        # Unpack the response using struct
        raw_presc, raw_fill, raw_integ = struct.unpack('>HHH', readTo)

        # Extract the 12-bit values by masking the lower 12 bits
        presc = raw_presc & 0x0FFF
        fill = raw_fill & 0x0FFF
        integ = raw_integ & 0x0FFF

        # Calculate the integration time in microseconds
        integration_time_microseconds = (integ - fill + 8) * (2 ** presc)

        # Convert microseconds to seconds
        integration_time_seconds = integration_time_microseconds / 1000000.0

        return integration_time_seconds

    # FIXME: Add better error checking.
    def set_integration_time(self, intg_time: np.float64)-> bool:
        """
        Checked with the original code, and intg_time of 1 s should result in:
        00 08 10 00 2F 3A
        Checked against this, it seems to work. for some reason the default does not seem to be the actual default.
        """
    # Check for valid integration time range
        if intg_time < const.CCS_SERIES_MIN_INT_TIME or intg_time > const.CCS_SERIES_MAX_INT_TIME:
            raise ValueError("Integration time out of valid range")
        
        # Convert integration time from seconds to microseconds
        integ = intg_time * 1000000
        
        # Calculate prescaler value
        presc = int(np.log10(integ) / np.log10(2)) - 11
        if presc < 0:
            presc = 0
        
        # Calculate filling value
        if integ <= 3800:
            fill = (3800 - integ + 1 + (integ % 2))
        else:
            fill = 0
        
        # Recalculate integration time
        integ = int((integ / (2 ** presc)) - 8 + fill)
        
        # Construct the data packet
        data = bytearray(const.CCS_SERIES_NUM_INTEG_CTRL_BYTES)
        data[0] = (presc >> 8) & 0xFF
        data[1] = presc & 0xFF
        data[2] = (fill >> 8) & 0xFF
        data[3] = fill & 0xFF
        data[4] = (integ >> 8) & 0xFF
        data[5] = integ & 0xFF
        
        # Set address masking bits
        data[0] |= 0x00  # Prescaler address
        data[2] |= 0x10  # Filling timer address
        data[4] |= 0x20  # Integration timer address

        # Transfer to device
        self.io.control_out(const.CCS_SERIES_WCMD_INTEGRATION_TIME, data)
        return True


    def get_device_status(self):
        readTo = array('h', [0])    # signed short of 16 bits.
        self.io.control_in(const.CCS_SERIES_RCMD_GET_STATUS, readTo)    #Ahh yes my favourite, a magic constant.
        readTo = readTo[0]
        statuses = []

        if readTo & const.CCS_SERIES_STATUS_SCAN_IDLE:
            statuses.append("SCAN_IDLE")
        if readTo & const.CCS_SERIES_STATUS_SCAN_TRIGGERED:
            statuses.append("SCAN_TRIGGERED")
        if readTo & const.CCS_SERIES_STATUS_SCAN_TRANSFER:
            statuses.append("SCAN_TRANSFER")
        if readTo & const.CCS_SERIES_STATUS_WAIT_FOR_EXT_TRIG:
            statuses.append("WAIT_FOR_EXT_TRIG")
        if readTo & const.CCS_SERIES_STATUS_SCAN_START_TRANS:
            statuses.append("SCAN_START_TRANS")
        
        if not statuses:
            statuses.append("Unknown or no specific status bits are set")
        
        return statuses

    def start_scan(self):
        self.io.control_out(const.CCS_SERIES_WCMD_MODUS, None, wValue=const.MODUS_INTERN_SINGLE_SHOT)

    #FIXME: does not get properly parsed at the moment.
    def get_error(self):
        buffer = usb.util.create_buffer(1)
        self.io.control_in(const.CCS_SERIES_RCMD_GET_ERROR,buffer)
        return buffer

    def get_raw_scan_data(self):
        # Calculate size of read and create a buffer to read into
        buffer_size = const.CCS_SERIES_NUM_RAW_PIXELS * 2  # since uint16 is 2 bytes
        buffer = usb.util.create_buffer(buffer_size)

        # Read to buffer and convert to uint16
        self.io.read_raw(buffer)
        readTo = np.frombuffer(buffer, dtype=np.uint16)

        return readTo


def plot(data):
    # Ensure the number of points matches your data length
    num_points = const.CCS_SERIES_NUM_RAW_PIXELS
    if len(data) != num_points:
        raise ValueError("Data length does not match the number of raw pixels")

    # Generate wavelengths from 500nm to 1000nm
    wavelengths = np.linspace(500, 1000, num_points)
    intensities = data

    # Plot the spectrometer data
    plt.figure(figsize=(10, 6))
    plt.plot(wavelengths, intensities, label='Spectrometer Data', color='blue')
    plt.xlabel('Wavelength (nm)')
    plt.ylabel('Intensity')
    plt.title('Spectrometer Results')
    plt.legend()
    plt.grid(True)

    # Save the plot as a PNG file
    plt.savefig('spectrometer_results.png')
    print("Plot saved as 'spectrometer_results.png'")

def head(data):
    print("First 10 bytes of raw scan data:")
    for i in range(min(10, len(data))):
        print(f"Byte {i}: {data[i]:02X}")


device = CCSDRV()
device.open()
print(device.get_integration_time())
print(device.get_device_status())
device.start_scan()
print(device.get_device_status())
device.get_raw_scan_data()
print(device.get_device_status())





