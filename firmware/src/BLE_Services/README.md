# BLE Services

This folder contains all BLE-related service definitions, callbacks, and polling systems for the Eidon Tracker.

## Files

### BLE Callbacks
- **BLE_Callbacks.h/cpp** - Callback classes for BLE events
  - `ServerCallbacks` - Connection/disconnection events
  - `OutputReportCallbacks` - HID output report handling
  - `QuaternionCharCallbacks` - Quaternion characteristic events
  - `CalibrationCallbacks` - IMU calibration commands (legacy - being replaced by polling)

### BLE Polling Service
- **BLE_Polling_Service.h/cpp** - Centralized polling system for BLE characteristics
  - `BLEPollingManager` - Main polling manager class
  - Handles Role changes, Calibration requests, and Color changes
  - Provides statistics, error handling, and dynamic configuration
  - Supports battery and performance modes

### HID Service
- **HID_Descriptor.h/cpp** - HID report descriptor for quaternion data transmission
  - Defines the HID report format (8 bytes: 4 × 16-bit quaternion values)
  - Used for HID device communication with hosts

### Role Configuration Service
- **RoleConfig_Service.h/cpp** - Device role configuration service
  - `RoleConfigData` struct for role information
  - Handles device role assignment and management
  - Integrated with polling system for reliable role changes

## Polling System

The BLE Polling Service replaces unreliable callbacks with a robust polling system for critical device functions:

### Supported Polling Targets
1. **Role Changes** - 200ms interval (5Hz)
   - Detects single-byte role values (0x00-0x06)
   - Updates device configuration and LED feedback
   
2. **Calibration Requests** - 100ms interval (10Hz)
   - Detects calibration commands
   - Triggers IMU reset and LED feedback
   
3. **Color Changes** - 150ms interval (~7Hz)
   - Detects 3-byte RGB color values
   - Updates color manager and LED feedback

### Features
- **Dynamic Configuration**: Adjust polling intervals at runtime
- **Battery Mode**: Slower polling to conserve power
- **Performance Mode**: Faster polling for responsiveness
- **Error Handling**: Automatic failure detection and recovery
- **Statistics**: Comprehensive performance monitoring
- **Debug Mode**: Detailed logging for troubleshooting

### Usage
```cpp
#include "BLE_Services/BLE_Polling_Service.h"

// Initialize polling system
pollingManager.begin();

// Add polling targets
pollingManager.addTarget(roleConfigChar, 200, handleRoleChange, "Role");
pollingManager.addTarget(calibrationChar, 100, handleCalibration, "Calibration");
pollingManager.addTarget(colorChar, 150, handleColorChange, "Color");

// In main loop
pollingManager.update();

// Configuration
pollingManager.setDebugMode(true);
pollingManager.setBatteryMode(lowBattery);
pollingManager.printStats();
```

## Usage

All BLE components are included in `main.cpp`:
```cpp
#include "BLE_Services/BLE_Callbacks.h"
#include "BLE_Services/BLE_Polling_Service.h"
#include "BLE_Services/HID_Descriptor.h"
#include "BLE_Services/RoleConfig_Service.h"
```

The services provide two different ways to transmit quaternion data:
1. **HID**: Standard HID device protocol for compatibility
2. **GATT**: Custom service for direct quaternion access and device control

The polling system handles critical device interactions reliably, while callbacks handle connection events and quaternion subscriptions. 