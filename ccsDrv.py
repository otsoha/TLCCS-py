from lowLevel import LLIO
from array import array
import const
import usb
import numpy as np
import struct

class CCSDRV:

    def __init__(self):
        self.io = None
        self.dev = None

    def open(self, vid=0x1313, pid=0x8087):
        """Opens a connection through LLIO.

        Args:
            vid (hexadecimal, optional): vendor ID. Defaults to 0x1313.
            pid (hexadecimal, optional): product ID. Defaults to 0x8087.
        """
        # Set class vars
        self.io = LLIO(vid, pid)
        self.dev = self.io.dev

        # Set default integration time
        assert self.set_integration_time(const.CCS_SERIES_DEF_INT_TIME)
        self.get_integration_time()



    def get_integration_time(self):
        """Returns current integration time in seconds

        Returns:
            _type_: integration time in seconds
        """
        # Create a buffer of the required size and read
        readTo = usb.util.create_buffer(const.CCS_SERIES_NUM_INTEG_CTRL_BYTES)
        self.io.control_in(const.CCS_SERIES_RCMD_INTEGRATION_TIME, readTo)
        
        # Unpack the response using struct
        raw_presc, raw_fill, raw_integ = struct.unpack('>HHH', readTo)

        # Extract 12-bit values
        presc = raw_presc & 0x0FFF
        fill = raw_fill & 0x0FFF
        integ = raw_integ & 0x0FFF

        # Calculate the integration time in microseconds
        integration_time_microseconds = (integ - fill + 8) * (2 ** presc)

        # Convert microseconds to seconds
        integration_time_seconds = integration_time_microseconds / 1000000.0

        return integration_time_seconds


    """
    Checked with the original code, and intg_time of 1 s should result in:
    00 08 10 00 2F 3A
    Checked against this, it seems to work. for some reason the default does not seem to be the actual default.
    """
    # FIXME: Add better error checking.
    def set_integration_time(self, intg_time: np.float64)-> bool:
        """Sets the integration time. 

        Args:
            intg_time (np.float64): integration time in seconds. Should be between CCS_SERIES_MIN_INT_TIME and
            CCS_SERIES_MAX_INT_TIME

        Raises:
            ValueError: desired intg_time out of range

        Returns:
            bool: pass or fail for setting the value.
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
        """Gets device status and parses the status bytes

        Returns:
            array: A list of set status bits in a readable form.
        """
        readTo = array('h', [0])    # signed short of 16 bits to get status.
        self.io.control_in(const.CCS_SERIES_RCMD_GET_STATUS, readTo)
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
        
        return statuses

    def start_scan(self):
        """Starts a single scan
        """
        self.io.control_out(const.CCS_SERIES_WCMD_MODUS, None, wValue=const.MODUS_INTERN_SINGLE_SHOT)



    def get_raw_scan_data(self):
        """Get raw scan data for a single scan from the device buffer
        The scan is sent from the device in uint16 with size CCS_SERIES_NUM_RAW_PIXELS

        Returns:
            np.array(np.uint16): raw scan data
        """
        # Calculate size of read and create a buffer to read into
        buffer_size = const.CCS_SERIES_NUM_RAW_PIXELS * 2  # since uint16 is 2 bytes
        buffer = usb.util.create_buffer(buffer_size)

        # Read to buffer and convert to uint16
        self.io.read_raw(buffer)
        readTo = np.frombuffer(buffer, dtype=np.uint16)

        return readTo

    # FIXME: remove self and create an abstraction layer to get scan data?
    def acquire_raw_scan_data(self, raw):
        # Initialize array for modified data
        data = np.zeros(const.CCS_SERIES_NUM_PIXELS, dtype=np.float64)    

        # Sum the dark pixels
        dark_com = np.sum(raw[const.DARK_PIXELS_OFFSET:const.DARK_PIXELS_OFFSET + const.NO_DARK_PIXELS])

        # Calculate dark current average
        dark_com /= const.NO_DARK_PIXELS

        # Calculate normalizing factor
        norm_com = 1.0 / (const.MAX_ADC_VALUE - dark_com)

        # Process raw data
        for i in range(const.CCS_SERIES_NUM_PIXELS):
            data[i] = (raw[const.SCAN_PIXELS_OFFSET + i] - dark_com) * norm_com

        return data


    def read_eeprom(self, addr, idx, length):

        # Buffers
        data = bytearray()
        remaining = length
        address = addr

        while remaining > 0:
            # Determine how many bytes to transfer
            transfer_length = min(remaining, const.ENDPOINT_0_TRANSFERSIZE)
            buffer = usb.util.create_buffer(transfer_length)
            
            # Read from EEPROM
            self.io.control_in(const.CCS_SERIES_RCMD_READ_EEPROM, readTo=buffer, wValue=address, wIndex=idx)
   

            # Append read data to buffer
            data.extend(buffer)
            
            # Update counters
            address += transfer_length
            remaining -= transfer_length

        return data


    def get_firmware_revision(self): 
        buffer = usb.util.create_buffer(const.CCS_SERIES_NUM_VERSION_BYTES)
        self.io.control_in(const.CCS_SERIES_RCMD_PRODUCT_INFO, buffer, wValue=const.CCS_SERIES_FIRMWARE_VERSION)
        return (buffer[0], buffer[1], buffer[2])
    
    def get_hardware_revision(self):
        buffer = usb.util.create_buffer(const.CCS_SERIES_NUM_VERSION_BYTES)
        self.io.control_in(const.CCS_SERIES_RCMD_PRODUCT_INFO, buffer, wValue=const.CCS_SERIES_HARDWARE_VERSION)
        return (buffer[0], buffer[1], buffer[2])