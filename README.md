### Simplified python drivers for Thorlabs CCS175(CCSXXX)
NOTE: the device should first be connected properly to a windows device, with the thorlabs driver software. This makes sure that the correct values are loaded in the EEPROM upon first connection.


## Setup
Setup explained in setup dir.


## Features
- Runs on linux
- Get/set integration time
- Scans: single, continuous, external trigger
- Get scan data
- Get device status, firm/hardware revisions

## Limitations
- Only tested on CCS175, but this should work on the full CCS series. Just make sure that the firmware upload is setup properly (see SETUP.md)
- Error checking is way less informative than on the original windows drivers.

