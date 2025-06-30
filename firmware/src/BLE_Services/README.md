# BLE Services

This folder contains all BLE-related service definitions and callbacks for the Eidon Tracker.

## Files

### BLE Callbacks
- **BLE_Callbacks.h/cpp** - Callback classes for BLE events
  - `ServerCallbacks` - Connection/disconnection events
  - `OutputReportCallbacks` - HID output report handling
  - `CalibrationCallbacks` - IMU calibration commands
  - `QuaternionCharCallbacks` - Quaternion characteristic events

### HID Service
- **HID_Descriptor.h/cpp** - HID report descriptor for quaternion data transmission
  - Defines the HID report format (8 bytes: 4 × 16-bit quaternion values)
  - Used for HID device communication with hosts

### GATT Service  
- **GATT_Service.h/cpp** - Custom GATT service for quaternion data and device control
  - `QuaternionData` struct (16 bytes: 4 × float quaternion values)
  - `DeviceInfo` struct for device information
  - Service UUIDs and characteristic definitions
  - Service setup function

## Usage

All BLE components are included in `main.cpp`:
```cpp
#include "BLE_Services/BLE_Callbacks.h"
#include "BLE_Services/HID_Descriptor.h"
#include "BLE_Services/GATT_Service.h"
```

The services provide two different ways to transmit quaternion data:
1. **HID**: Standard HID device protocol for compatibility
2. **GATT**: Custom service for direct quaternion access and device control

The callbacks handle all BLE events and user interactions. 