# Mobile App Integration Guide

## Overview

This guide provides step-by-step instructions for integrating Eidon Tracker devices with mobile applications using Flutter and native iOS/Android development.

## Prerequisites

### Flutter Development
- Flutter SDK 3.0+
- Dart 2.17+
- flutter_blue_plus package (or similar BLE library)

### iOS Development
- iOS 13.0+
- Info.plist permissions:
  ```xml
  <key>NSBluetoothAlwaysUsageDescription</key>
  <string>This app needs Bluetooth to connect to Eidon Trackers</string>
  <key>NSBluetoothPeripheralUsageDescription</key>
  <string>This app needs Bluetooth to receive motion data</string>
  ```

### Android Development
- Android 5.0+ (API 21+)
- Manifest permissions:
  ```xml
  <uses-permission android:name="android.permission.BLUETOOTH"/>
  <uses-permission android:name="android.permission.BLUETOOTH_ADMIN"/>
  <uses-permission android:name="android.permission.BLUETOOTH_SCAN"/>
  <uses-permission android:name="android.permission.BLUETOOTH_CONNECT"/>
  <uses-permission android:name="android.permission.ACCESS_FINE_LOCATION"/>
  ```

## Integration Steps

### 1. Device Discovery

```dart
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

// Start scanning for devices
FlutterBluePlus.startScan(
  timeout: Duration(seconds: 10),
  withServices: [Guid("E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE")]
);

// Listen for scan results
FlutterBluePlus.scanResults.listen((results) {
  for (ScanResult r in results) {
    if (r.device.name.startsWith("Eidon Tracker-")) {
      print('Found ${r.device.name} with RSSI ${r.rssi}');
      // Add to device list
    }
  }
});
```

### 2. Device Connection

```dart
// Connect to device
await device.connect(
  timeout: Duration(seconds: 5),
  autoConnect: false,
);

// Set connection parameters for optimal performance
await device.requestConnectionPriority(ConnectionPriority.high);

// Discover services
List<BluetoothService> services = await device.discoverServices();
```

### 3. Service and Characteristic Setup

```dart
BluetoothService? eidonService;
BluetoothCharacteristic? quaternionChar;
BluetoothCharacteristic? calibrationChar;
BluetoothCharacteristic? colorChar;
BluetoothCharacteristic? deviceInfoChar;

// Find Eidon service and characteristics
for (BluetoothService service in services) {
  if (service.uuid.toString().toUpperCase() == "E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE") {
    eidonService = service;
    
    for (BluetoothCharacteristic char in service.characteristics) {
      String charUuid = char.uuid.toString().toUpperCase();
      
      switch (charUuid) {
        case "E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE":
          quaternionChar = char;
          break;
        case "E1D00003-8B5A-3E5B-9E23-4F9B5C91BBDE":
          calibrationChar = char;
          break;
        case "E1D00004-8B5A-3E5B-9E23-4F9B5C91BBDE":
          colorChar = char;
          break;
        case "E1D00005-8B5A-3E5B-9E23-4F9B5C91BBDE":
          deviceInfoChar = char;
          break;
      }
    }
  }
}
```

### 4. Enable Quaternion Notifications

```dart
if (quaternionChar != null) {
  // Enable notifications
  await quaternionChar.setNotifyValue(true);
  
  // Subscribe to quaternion updates
  quaternionChar.value.listen((value) {
    if (value.length >= 20) {
      // Parse quaternion data
      var buffer = value.buffer;
      var data = ByteData.view(buffer);
      
      double w = data.getFloat32(0, Endian.little);
      double x = data.getFloat32(4, Endian.little);
      double y = data.getFloat32(8, Endian.little);
      double z = data.getFloat32(12, Endian.little);
      
      int switches = value[16];
      bool isLeft = (switches & 0x01) != 0;
      bool isUpper = (switches & 0x02) != 0;
      
      // Process quaternion data
      onQuaternionUpdate(w, x, y, z, isLeft, isUpper);
    }
  });
}
```

### 5. Device Calibration

```dart
Future<void> calibrateDevice() async {
  if (calibrationChar != null) {
    // Send calibration command
    await calibrationChar.write([0x01]);
    
    // Wait for acknowledgment
    var response = await calibrationChar.read();
    if (response.isNotEmpty && response[0] == 0x01) {
      print("Calibration successful");
    }
  }
}
```

### 6. Set Device Color

```dart
Future<void> setDeviceColor(Color color) async {
  if (colorChar != null) {
    await colorChar.write([
      color.red,
      color.green,
      color.blue
    ]);
  }
}
```

### 7. Read Device Information

```dart
Future<void> readDeviceInfo() async {
  if (deviceInfoChar != null) {
    var infoBytes = await deviceInfoChar.read();
    if (infoBytes.length >= 8) {
      var data = ByteData.view(infoBytes.buffer);
      
      int deviceId = data.getUint16(0, Endian.little);
      int fwMajor = infoBytes[2];
      int fwMinor = infoBytes[3];
      int batteryLevel = infoBytes[4];
      
      print("Device ID: $deviceId");
      print("Firmware: $fwMajor.$fwMinor");
      print("Battery: $batteryLevel%");
    }
  }
}
```

## Best Practices

### Connection Management
1. Implement automatic reconnection logic
2. Handle connection state changes gracefully
3. Disconnect properly when app goes to background

### Data Processing
1. Process quaternion data on a separate thread/isolate
2. Implement data buffering for recording
3. Apply smoothing filters if needed

### Multi-Device Support
```dart
class TrackerDevice {
  final BluetoothDevice device;
  final String position; // "left_upper", "right_lower", etc.
  StreamSubscription? quaternionSubscription;
  
  // Device-specific data and methods
}

// Manage multiple devices
Map<String, TrackerDevice> connectedTrackers = {};
```

### Error Handling
```dart
try {
  await device.connect();
} on PlatformException catch (e) {
  if (e.code == 'already_connected') {
    // Handle already connected
  } else if (e.code == 'connect_timeout') {
    // Handle timeout
  }
}
```

## Recording and Upload

### Data Structure for Recording
```dart
class MotionFrame {
  final DateTime timestamp;
  final String deviceId;
  final Quaternion quaternion;
  final bool isLeft;
  final bool isUpper;
  
  Map<String, dynamic> toJson() => {
    'timestamp': timestamp.toIso8601String(),
    'device_id': deviceId,
    'quaternion': {
      'w': quaternion.w,
      'x': quaternion.x,
      'y': quaternion.y,
      'z': quaternion.z,
    },
    'position': {
      'is_left': isLeft,
      'is_upper': isUpper,
    }
  };
}
```

### Synchronized Recording
```dart
class RecordingSession {
  final List<TrackerDevice> devices;
  final List<MotionFrame> frames = [];
  Timer? recordingTimer;
  
  void startRecording() {
    recordingTimer = Timer.periodic(Duration(milliseconds: 5), (_) {
      // Capture current state from all devices
      for (var device in devices) {
        frames.add(device.getCurrentFrame());
      }
    });
  }
  
  Future<void> stopAndUpload() async {
    recordingTimer?.cancel();
    
    // Package data for upload
    var sessionData = {
      'session_id': Uuid().v4(),
      'start_time': frames.first.timestamp.toIso8601String(),
      'end_time': frames.last.timestamp.toIso8601String(),
      'devices': devices.map((d) => d.deviceId).toList(),
      'frames': frames.map((f) => f.toJson()).toList(),
    };
    
    // Upload to API
    await uploadToAPI(sessionData);
  }
}
```

## Troubleshooting

### Common Issues

1. **Device not found**
   - Ensure Bluetooth is enabled
   - Check location permissions (Android)
   - Verify device is advertising

2. **Connection failures**
   - Reset Bluetooth adapter
   - Ensure device isn't already connected
   - Check battery level

3. **Data not updating**
   - Verify notifications are enabled
   - Check characteristic properties
   - Monitor connection state

4. **Performance issues**
   - Use high connection priority
   - Process data asynchronously
   - Implement data throttling if needed

### Debug Tools
- Use nRF Connect app for BLE debugging
- Monitor device logs via serial console
- Implement comprehensive logging in app 