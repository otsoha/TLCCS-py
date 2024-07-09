from ccsDrv import CCSDRV
from array import array
import const
import numpy as np
import matplotlib.pyplot as plt

def plot(data: array):
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