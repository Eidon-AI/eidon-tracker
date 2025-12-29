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

## 2. ESP-NOW Interface (Child -> Hub Communication) - COMPLETED
*   **Defined `ESPNowRawPacket` struct** in `Hub_Structures.h`:
    *   Includes `ESPNowPacketHeader` (messageType = 0x03) and `RawMotionData`.
*   **Updated `ESPNowChildDevice` struct**:
    *   Added `RawMotionData lastRawData` to store the latest raw data from each child.

## 3. Firmware Changes

### A. BNO085 Library (`firmware/lib/BNO085`) - COMPLETED
1.  **Enable Reports**: Updated `enableReports()` to enable:
    *   `SH2_ACCELEROMETER`
    *   `SH2_GYROSCOPE_CALIBRATED`
    *   `SH2_MAGNETIC_FIELD_CALIBRATED`
2.  **Capture Data**: Updated `update()` to handle these new reports and store the values in a `rawData` variable.
3.  **Public Interface**: Added `getRawData(RawMotionData& data)` to expose the latest readings.
4.  **Header Updates**: Defined `RawMotionData` in `BNO085.h` to avoid circular dependencies.

### B. Hub Structures (`firmware/src/Role_Services/Hub_Structures.h`) - COMPLETED
1.  ~~Define `RawMotionData` struct.~~ (Moved to BNO085.h in Step 3A to avoid circular dependency)
2.  ~~Define `ESPNowRawPacket` struct.~~ (Done in Step 2)
3.  ~~Add `RawMotionData` storage to `ESPNowChildDevice`~~ (Done in Step 2) and `AggregatedQuaternionData` (or a separate aggregation struct).
4.  **Updated Includes**: Included `BNO085.h` to access `RawMotionData`.

### C. Hub Client Service (`firmware/src/Role_Services/HubClient_Service.cpp`) - COMPLETED
1.  Updated `onESPNowDataRecv` (via `processESPNowPacket`) to handle `MESSAGE_TYPE_RAW`.
2.  Cast received data to `ESPNowRawPacket`.
3.  Stored received raw data in the corresponding `childDevices` slot using `lastRawData`.

### D. Main Logic (`firmware/src/main.cpp`) - COMPLETED
1.  ~~**BLE Setup**: Create and start the 3 new characteristics.~~ (Done in Step 1)
2.  **Child Mode**: Updated main loop to send raw data via ESP-NOW immediately after sending quaternions.
3.  **Hub Mode**: Updated BLE transmission loop to:
    *   Read local raw data from BNO085 and notify `hubRawDataChar`.
    *   Retrieve stored child raw data from `hubClientService` and notify `handRawDataChar`/`forearmRawDataChar`.

# Implementation Complete
All steps for the Raw Sensor Data Implementation Plan have been executed.
1.  **BLE Interface**: 3 new characteristics created and UUIDs assigned.
2.  **ESP-NOW Interface**: `MESSAGE_TYPE_RAW` and `ESPNowRawPacket` defined.
3.  **Firmware Changes**:
    *   **BNO085 Lib**: Updated to capture Accel/Gyro/Mag and provide `getRawData()`.
    *   **Hub Structures**: Structures updated and circular dependencies resolved.
    *   **Hub Client**: Logic added to receive and store raw packets.
    *   **Main Logic**: Child transmission and Hub BLE notification loops updated.
