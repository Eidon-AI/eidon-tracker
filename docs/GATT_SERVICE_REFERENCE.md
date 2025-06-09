EIDON TRACKER - BLE GATT SERVICE REFERENCE
==========================================

FIRMWARE VERSION: 1.2
DEVICE: nRF52840 with BNO085 IMU

SERVICE AND CHARACTERISTIC UUIDS
================================

Service UUID:
E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE

Characteristic UUIDs:
- Quaternion Data:  E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE  (Notify/Read)
- Calibration:      E1D00003-8B5A-3E5B-9E23-4F9B5C91BBDE  (Write/Read)
- Color:            E1D00004-8B5A-3E5B-9E23-4F9B5C91BBDE  (Write/Read)
- Device Info:      E1D00005-8B5A-3E5B-9E23-4F9B5C91BBDE  (Read)


DEVICE DISCOVERY
================

Device Name Format: "Eidon Tracker-XXXX" (where XXXX is unique per device)
Service Filter: E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE


DATA FORMATS
============

1. QUATERNION DATA (20 bytes total)
-----------------------------------
Bytes 0-3:   float w (little-endian)
Bytes 4-7:   float x (little-endian)
Bytes 8-11:  float y (little-endian)
Bytes 12-15: float z (little-endian)
Byte 16:     switches (bit 0: isLeft, bit 1: isUpper)
Bytes 17-19: reserved (padding)

2. CALIBRATION COMMANDS (1 byte)
--------------------------------
0x01: Trigger IMU calibration/reset
0x02: Request device info update

3. COLOR DATA (3 bytes)
-----------------------
Byte 0: Red (0-255)
Byte 1: Green (0-255)
Byte 2: Blue (0-255)

4. DEVICE INFO (8 bytes)
------------------------
Bytes 0-1: Device ID (uint16, little-endian)
Bytes 2-3: Firmware version (major.minor)
Byte 4:    Battery level (0-100%)
Bytes 5-7: Reserved


FLUTTER/DART CODE EXAMPLES
==========================

// Service and Characteristic UUIDs
const String EIDON_SERVICE_UUID = "E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE";
const String QUATERNION_CHAR_UUID = "E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE";
const String CALIBRATION_CHAR_UUID = "E1D00003-8B5A-3E5B-9E23-4F9B5C91BBDE";
const String COLOR_CHAR_UUID = "E1D00004-8B5A-3E5B-9E23-4F9B5C91BBDE";
const String DEVICE_INFO_CHAR_UUID = "E1D00005-8B5A-3E5B-9E23-4F9B5C91BBDE";

// Quaternion Data Parser
class QuaternionData {
  double w, x, y, z;
  bool isLeft;
  bool isUpper;
  
  QuaternionData.fromBytes(Uint8List bytes) {
    // Parse 4 floats (little-endian)
    var buffer = bytes.buffer;
    var data = ByteData.view(buffer);
    
    w = data.getFloat32(0, Endian.little);
    x = data.getFloat32(4, Endian.little);
    y = data.getFloat32(8, Endian.little);
    z = data.getFloat32(12, Endian.little);
    
    // Parse switch states
    int switches = bytes[16];
    isLeft = (switches & 0x01) != 0;
    isUpper = (switches & 0x02) != 0;
  }
}

// Device Info Parser
class DeviceInfo {
  int deviceId;
  String firmwareVersion;
  int batteryLevel;
  
  DeviceInfo.fromBytes(Uint8List bytes) {
    var data = ByteData.view(bytes.buffer);
    
    deviceId = data.getUint16(0, Endian.little);
    int major = bytes[2];
    int minor = bytes[3];
    firmwareVersion = "$major.$minor";
    batteryLevel = bytes[4];
  }
}

// Example: Connect and Subscribe to Quaternion Data
Future<void> connectToDevice(BluetoothDevice device) async {
  await device.connect();
  
  List<BluetoothService> services = await device.discoverServices();
  
  for (BluetoothService service in services) {
    if (service.uuid.toString().toUpperCase() == EIDON_SERVICE_UUID) {
      for (BluetoothCharacteristic char in service.characteristics) {
        switch (char.uuid.toString().toUpperCase()) {
          case QUATERNION_CHAR_UUID:
            // Enable notifications
            await char.setNotifyValue(true);
            
            // Listen to quaternion updates
            char.value.listen((value) {
              if (value.length >= 20) {
                var quaternion = QuaternionData.fromBytes(value);
                // Process quaternion data
                print('W: ${quaternion.w}, X: ${quaternion.x}, Y: ${quaternion.y}, Z: ${quaternion.z}');
                print('Left: ${quaternion.isLeft}, Upper: ${quaternion.isUpper}');
              }
            });
            break;
            
          case CALIBRATION_CHAR_UUID:
            // Store for later use
            calibrationChar = char;
            break;
            
          case COLOR_CHAR_UUID:
            // Store for later use
            colorChar = char;
            break;
            
          case DEVICE_INFO_CHAR_UUID:
            // Read device info
            var infoBytes = await char.read();
            var deviceInfo = DeviceInfo.fromBytes(infoBytes);
            print('Device ID: ${deviceInfo.deviceId}');
            print('Firmware: ${deviceInfo.firmwareVersion}');
            print('Battery: ${deviceInfo.batteryLevel}%');
            break;
        }
      }
    }
  }
}

// Example: Trigger Calibration
Future<void> calibrateIMU() async {
  if (calibrationChar != null) {
    await calibrationChar.write([0x01]);
  }
}

// Example: Set Color
Future<void> setDeviceColor(int r, int g, int b) async {
  if (colorChar != null) {
    await colorChar.write([r, g, b]);
  }
}


CONNECTION PARAMETERS
=====================

Recommended for optimal performance:
- Connection Interval: 7.5-15ms
- Enable notifications immediately after connection
- The device streams quaternion data continuously when notifications are enabled


MULTIPLE DEVICE SUPPORT
=======================

- Each device has a unique suffix in its name (last 4 hex digits of BLE address)
- Devices can be differentiated by their switch states (isLeft/isUpper)
- iOS supports up to 6 simultaneous BLE connections
- Android typically supports 7-10 simultaneous connections


IMPORTANT NOTES
===============

1. The firmware supports both BLE HID and custom GATT services
2. iOS apps should use the custom GATT service (HID is restricted on iOS)
3. Android/Windows/macOS can use either HID or GATT
4. Quaternion data is pre-corrected for IMU mounting (180° Z-axis rotation applied)
5. Battery level updates every 5 minutes to minimize power consumption
6. Color settings are persisted in flash memory
7. Double-tap on device triggers calibration (hardware feature)


CALIBRATION PROCESS
===================

When calibration is triggered (via GATT command 0x01 or double-tap):
1. Blue LED turns on
2. IMU streaming stops momentarily
3. IMU is reset and recalibrated
4. Streaming resumes at 200Hz
5. Blue LED turns off after 2 seconds
6. Acknowledgment byte (0x01) is written to calibration characteristic


ERROR HANDLING
==============

- If IMU initialization fails, device LED flashes rapidly
- Failed HID reports are logged but don't affect GATT notifications
- Color values outside RGB range are clamped to 0-255
- Invalid calibration commands are ignored


PERFORMANCE SPECIFICATIONS
==========================

- IMU sampling rate: Up to 1000Hz (currently set to 200Hz after calibration)
- BLE connection interval: 7.5-15ms
- Quaternion precision: 32-bit float
- Battery monitoring: 12-bit ADC resolution
- Flash storage: 2MB QSPI flash for persistent settings 