from lowLevel import LLIO
import const
import usb
import numpy as np
import struct
import time

class CCSDRV:

    def open(self, vid=0x1313, pid=0x8087):
        """Opens a connection through LLIO.

        Args:
            vid (hexadecimal, optional): vendor ID. Defaults to 0x1313 (ThorLabs).
            pid (hexadecimal, optional): product ID. Defaults to 0x8087 (CCS175).
        """
        # Set class vars
        self.io = LLIO(vid, pid)
        self.dev = self.io.dev
        self.io.open()

        # Set default integration time
        assert self.set_integration_time(const.CCS_SERIES_DEF_INT_TIME)
        state = self.get_device_status()
        self.integration_time = const.CCS_SERIES_DEF_INT_TIME
        
        if "SCAN_IDLE" not in state:
            # FIXME: This is a workaround for the device not being in idle state 
            # after opening connection for first time. Look into reset.
            print("Device not in idle state after opening connection. Running a scan and flushing to reset")
            time.sleep(3) 
            self.start_scan()
            self.get_scan_data()   
        
        
    def close(self):
        """Closes the connection through LLIO.
        """
        self.io.close()
    
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

    def pipe_status(self):
        return self.io.get_bulk_in_status()

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
        self.integration_time = intg_time
        return True


    def get_device_status(self, debug=False):
        """Gets device status and parses the status bytes

        Returns:
            array: A list of set status bits in a readable form.
        """

        status = np.int32(0xFFFF)
        readTo = usb.util.create_buffer(2)    # 16 bits to get status.
        self.io.control_in(const.CCS_SERIES_RCMD_GET_STATUS, readTo)
        status = np.frombuffer(readTo, dtype=np.int16)[0]
        if debug: print(f"status(binary): {format(status, '016b')}")


        statuses = []
        if debug: print(f"comparing: {format(status, '016b')} and {format(const.CCS_SERIES_STATUS_SCAN_IDLE, '016b')}")
        if status & const.CCS_SERIES_STATUS_SCAN_IDLE: 
            statuses.append("SCAN_IDLE")

        if debug: print(f"comparing: {format(status, '016b')} and {format(const.CCS_SERIES_STATUS_SCAN_TRIGGERED, '016b')}")
        if status & const.CCS_SERIES_STATUS_SCAN_TRIGGERED:
            statuses.append("SCAN_TRIGGERED")

        if debug: print(f"comparing: {format(status, '016b')} and {format(const.CCS_SERIES_STATUS_SCAN_START_TRANS, '016b')}")    
        if status & const.CCS_SERIES_STATUS_SCAN_START_TRANS:
            statuses.append("SCAN_START_TRANS")

        if debug: print(f"comparing: {format(status, '016b')} and {format(const.CCS_SERIES_STATUS_SCAN_TRANSFER, '016b')}")    
        if status & const.CCS_SERIES_STATUS_SCAN_TRANSFER:
            statuses.append("SCAN_TRANSFER")
        
        if debug: print(f"comparing: {format(status, '016b')} and {format(const.CCS_SERIES_STATUS_WAIT_FOR_EXT_TRIG, '016b')}")   
        if status & const.CCS_SERIES_STATUS_WAIT_FOR_EXT_TRIG:
            statuses.append("WAIT_FOR_EXT_TRIG")
        
        return statuses


    def start_scan(self):
        """Starts a single scan
        """
        self.io.control_out(const.CCS_SERIES_WCMD_MODUS, None, wValue=const.MODUS_INTERN_SINGLE_SHOT)
        time.sleep(self.integration_time) # Wait for scan to complete

    def start_scan_continuous(self):
        """Starts continuous scanning. Any function except get_scan_data() and get_device_status() will stop the scan.
        """
        self.io.control_out(const.CCS_SERIES_WCMD_MODUS, None, wValue=const.MODUS_INTERN_CONTINUOUS)

    def start_scan_ext_trigger(self):
        """Starts a single scan with external trigger
        """
        self.io.control_out(const.CCS_SERIES_WCMD_MODUS, None, wValue=const.MODUS_EXTERN_SINGLE_SHOT)


    def get_scan_data(self):
        """Get scan data from the device buffer

        Returns:
            np.array(np.float64): scan data
        """
        # Get raw data
        raw = self._get_raw_data()

        # Process raw data
        data = self._acquire_raw_scan_data(raw)

        return data

    # FIXME: There seems to be a a period of zeroes at the beginning and end of the data.
    def _get_raw_data(self):
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
    def _acquire_raw_scan_data(self, raw: np.ndarray):
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
            data[i] = (raw[const.SCAN_PIXELS_OFFSET + i]- dark_com) * norm_com

        return data

    # FIXME: untested
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