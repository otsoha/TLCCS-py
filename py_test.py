from ccsDrv import CCSDRV
from array import array
import const
import numpy as np
import matplotlib.pyplot as plt
import time

def plotRaw(data: np.ndarray, name: str = 'spectrometer_results_raw.png'):
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
    
    # Set the y-axis limits to start at 0
    plt.ylim(bottom=0)
 
    # Save the plot as a PNG file
    plt.savefig(name)
    print(f"Plot saved as '{name}'")

def plot(data: np.ndarray, name: str = 'ressA.png'):
    # Ensure the number of points matches your data length
    num_points = const.CCS_SERIES_NUM_PIXELS
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

    # Set the y-axis limits to start at 0
    plt.ylim(bottom=0)

    # Save the plot as a PNG file
    plt.savefig(name)
    print(f"Plot saved as '{name}'")

def head(data):
    print("First 10 bytes of raw scan data:")
    for i in range(min(10, len(data))):
        print(f"Byte {i}: {data[i]:02f}")
    



def open_test():
    ccs = CCSDRV()
    ccs.open()
    print(f"Dev status: {ccs.get_device_status()}")
    print(f"Integration time: {ccs.get_integration_time()}")
    print(ccs.pipe_status())

    time.sleep(5)
    print(f"Dev status: {ccs.get_device_status()}")

    ccs.get_firmware_revision()
    time.sleep(5)
    print(f"Dev status: {ccs.get_device_status()}")
    
    ccs.get_hardware_revision()
    time.sleep(5)
    print(f"Dev status: {ccs.get_device_status()}")

    ccs.get_integration_time()
    time.sleep(5)
    print(f"Dev status: {ccs.get_device_status()}")

    ccs.close()

def scan_test():
    ccs = CCSDRV()
    ccs.open()
    ccs.start_scan()
    print(f"Dev status: {ccs.get_device_status()}")
    data = ccs.get_scan_data()
    print(f"Dev status: {ccs.get_device_status()}")
    ccs.close()


ccs = CCSDRV()
ccs.open()
print(f"Dev status: {ccs.get_device_status()}")
ccs.close()