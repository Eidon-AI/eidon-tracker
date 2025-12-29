# Raw Sensor Data Implementation Plan

## Overview
This document outlines the plan to extract and transmit raw sensor data (Accelerometer, Gyroscope, Magnetometer) from all connected devices (Hub, Hand, Forearm) to the mobile application. This data pipeline runs in parallel with the existing Quaternion tracking pipeline.

# Done Sections
## 1. BLE Interface (Phone Communication) - COMPLETED
*   **Implemented 3 new characteristics** for raw data:
    *   **Hub Raw Data**: `E1D0000B-...`
    *   **Hand Raw Data**: `E1D0000C-...`
    *   **Forearm Raw Data**: `E1D0000D-...`
*   **Defined `RawMotionData` struct** in `Hub_Structures.h`:
    *   36-byte payload: 9 floats (Accel X/Y/Z, Gyro X/Y/Z, Mag X/Y/Z).
*   **Updated Main Logic**:
    *   Initialized `RawMotionData` instance (`gattRawData`).
    *   Configured characteristics with `NOTIFY` property in `main.cpp`.

# Sections to complete

## 2. ESP-NOW Interface (Child -> Hub Communication)
Child devices (Hand, Forearm) need to send their raw data to the Hub.

### Protocol Updates
*   **New Message Type**: `MESSAGE_TYPE_RAW` (0x03)

### Packet Structure
```cpp
struct ESPNowRawPacket {
    ESPNowPacketHeader header; // messageType = 0x03
    RawMotionData data;
} __attribute__((packed));
```

## 3. Firmware Changes

### A. BNO085 Library (`firmware/lib/BNO085`)
1.  **Enable Reports**: Update `enableReports()` to enable:
    *   `SH2_ACCELEROMETER`
    *   `SH2_GYROSCOPE_CALIBRATED`
    *   `SH2_MAGNETIC_FIELD_CALIBRATED`
2.  **Capture Data**: Update `update()` to handle these new reports and store the values.
3.  **Public Interface**: Add `getRawData(RawMotionData& data)` to expose the latest readings.

### B. Hub Structures (`firmware/src/Role_Services/Hub_Structures.h`)
1.  ~~Define `RawMotionData` struct.~~ (Done in Step 1)
2.  Define `ESPNowRawPacket` struct.
3.  Add `RawMotionData` storage to `ESPNowChildDevice` and `AggregatedQuaternionData` (or a separate aggregation struct).

### C. Hub Client Service (`firmware/src/Role_Services/HubClient_Service.cpp`)
1.  Update `onESPNowDataRecv` to handle `MESSAGE_TYPE_RAW`.
2.  Store received raw data in the corresponding `childDevices` slot.

### D. Main Logic (`firmware/src/main.cpp`)
1.  ~~**BLE Setup**: Create and start the 3 new characteristics.~~ (Done in Step 1)
2.  **Child Mode**: In the IMU update loop, read raw data and send via ESP-NOW immediately after sending quaternions.
3.  **Hub Mode**: In the main loop (or BLE transmission loop), retrieve local raw data + stored child raw data and notify the new characteristics.
