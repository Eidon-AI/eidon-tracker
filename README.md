# Eidon Tracker

A Bluetooth IMU tracker using the Seeed XIAO nRF52840 Sense board and BNO085 sensor. This project creates a custom HID device that transmits quaternion orientation data over Bluetooth.

## Hardware Requirements

- [Seeed XIAO nRF52840 Sense](https://www.seeedstudio.com/XIAO-nRF52840-Sense-p-5256.html)
- BNO085 IMU sensor (integrated on the XIAO nRF52840 Sense)

## Software Setup

### Arduino IDE Setup

1. Download and install [Arduino IDE](https://www.arduino.cc/en/software) (version 2.0 or later recommended)

2. Add the Seeed nRF52 board support:
   - Open Arduino IDE
   - Go to `Tools > Board > Boards Manager`
   - Search for "Seeed nRF52"
   - Install "Seeed nRF52 mbed-enabled Boards"

3. Select the correct board:
   - Go to `Tools > Board > Seeed nRF52 mbed-enabled Boards`
   - Select "XIAO nRF52840 Sense"

4. Configure DFU mode:
   - Go to `Tools > Programmer`
   - Select "Boot Loader DFU for Bluefruit NRF52"
   - This will enable automatic DFU mode entry when uploading
   - No more need to double-tap the reset button!

### Required Libraries

Install the following libraries through the Arduino Library Manager (`Tools > Manage Libraries`):

1. [Adafruit BluefruitLE nRF52](https://github.com/adafruit/Adafruit_BluefruitLE_nRF52)
   - Search for "Adafruit BluefruitLE nRF52"
   - Install the library

2. [Adafruit BNO08x](https://github.com/adafruit/Adafruit_BNO08x)
   - Search for "Adafruit BNO08x"
   - Install the library

3. [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Unified_Sensor)
   - Search for "Adafruit Unified Sensor"
   - Install the library

## Project Features

- Custom HID device that transmits quaternion orientation data
- Bluetooth connectivity with battery level reporting
- Real-time IMU data processing
- LED status indicators
- DFU (Device Firmware Update) support

## Usage

1. Upload the sketch to your XIAO nRF52840 Sense
2. The device will appear as "Eidon Tracker" in Bluetooth settings
3. Connect to the device using a compatible Bluetooth host
4. The device will transmit quaternion orientation data (w, x, y, z) as HID reports

## Calibration

The BNO085 sensor requires calibration for optimal performance:

1. After powering on, wave the device in a figure-8 pattern
2. Rotate slowly through all orientations
3. Keep away from magnetic interference
4. Wait for the "Magnetometer Calibrated!" message in the Serial Monitor

## Debug Information

Connect to the Serial Monitor (115200 baud) to view:
- Connection status
- Quaternion values
- Battery level
- Calibration status

## DFU Mode

To enter DFU mode:
1. Open Serial Monitor
2. Send 'D' character
3. The device will reset into DFU mode

## License

This project is open source and available under the MIT License.
