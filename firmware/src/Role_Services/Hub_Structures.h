#ifndef HUB_STRUCTURES_H
#define HUB_STRUCTURES_H

#include <Arduino.h>
#include "DeviceConfig.h"
#include "BNO085.h"

// Message type constants for ESP-NOW packets
#define MESSAGE_TYPE_QUAT 0x01    // Quaternion data
#define MESSAGE_TYPE_CMD  0x02    // Command
#define MESSAGE_TYPE_RAW  0x03    // Raw sensor data

// Raw motion data structure (36 bytes)
struct RawMotionData {
    float accel_x, accel_y, accel_z; // Accelerometer (m/s^2)
    float gyro_x,  gyro_y,  gyro_z;  // Gyroscope (rad/s)
    float mag_x,   mag_y,   mag_z;   // Magnetometer (uT)
} __attribute__((packed));

// Quaternion data structure for GATT (16 bytes) - unchanged
struct QuaternionData {
    float w;
    float x;
    float y;
    float z;
} __attribute__((packed));

// Common header for all ESP-NOW packets (simplified)
struct ESPNowPacketHeader {
    uint8_t messageType;     // 0x01=QUAT
    uint8_t senderRole;      // Sender's role (0-6) - needed for data categorization
    uint8_t reserved[2];     // Future use, alignment
} __attribute__((packed));

// ESP-NOW packet structure for child to hub communication (simplified)
struct ESPNowQuaternionPacket {
    ESPNowPacketHeader header;  // messageType = 0x01
    QuaternionData quaternion;  // Existing structure (unchanged)
} __attribute__((packed));

// ESP-NOW command packet for hub to child communication
struct ESPNowCommandPacket {
    ESPNowPacketHeader header;  // messageType = 0x02
    uint8_t commandType;        // 0x01 = IMU reset
    uint8_t reserved[3];        // Future use, alignment
} __attribute__((packed));

// ESP-NOW child device structure (replaces BLE ChildConnection)
struct ESPNowChildDevice {
    uint8_t macAddress[6];     // Child device MAC address
    DeviceRole role;           // Child device role
    bool dataAvailable;        // Whether we've received data recently
    QuaternionData lastData;   // Last received quaternion data
} __attribute__((packed));

// Aggregated quaternion data for hub (unchanged structure)
struct AggregatedQuaternionData {
    QuaternionData hubData;
    QuaternionData handData;
    QuaternionData forearmData;
    bool handConnected;
    bool forearmConnected;
    unsigned long timestamp;
} __attribute__((packed));

// Aggregated raw data for hub
struct AggregatedRawData {
    RawMotionData hubData;
    RawMotionData handData;
    RawMotionData forearmData;
    bool handConnected;
    bool forearmConnected;
    unsigned long timestamp;
} __attribute__((packed));

#endif // HUB_STRUCTURES_H 