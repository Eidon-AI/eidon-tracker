#ifndef HUB_STRUCTURES_H
#define HUB_STRUCTURES_H

#include <Arduino.h>
#include "DeviceConfig.h"
#include "BNO085.h" // Includes RawMotionData definition

// Message type constants for ESP-NOW packets
#define MESSAGE_TYPE_QUAT 0x01    // Quaternion data
#define MESSAGE_TYPE_CMD  0x02    // Command
#define MESSAGE_TYPE_RAW  0x03    // Raw sensor data

// RawMotionData is defined in BNO085.h

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

// ESP-NOW raw data packet for child to hub communication
struct ESPNowRawPacket {
    ESPNowPacketHeader header;  // messageType = 0x03
    RawMotionData data;         // Raw sensor data
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
    uint8_t batteryLevel;      // Child battery percentage (0 = no data)
    QuaternionData lastData;   // Last received quaternion data
    RawMotionData lastRawData; // Last received raw data
} __attribute__((packed));

// Aggregated quaternion data for hub
// Note: Each right hub has exactly one left child (hand, forearm, or shoulder)
// The hub aggregates its own data + its left child's data
struct AggregatedQuaternionData {
    QuaternionData hubData;        // Hub's own quaternion data
    QuaternionData handData;        // Left hand child data (for right hand hub)
    QuaternionData forearmData;    // Left forearm child data (for right forearm hub)
    QuaternionData shoulderData;   // Left shoulder child data (for right shoulder hub)
    bool handConnected;             // Left hand child connection status
    bool forearmConnected;          // Left forearm child connection status
    bool shoulderConnected;         // Left shoulder child connection status
    unsigned long timestamp;
} __attribute__((packed));

// Aggregated raw data for hub
// Note: Each right hub has exactly one left child (hand, forearm, or shoulder)
struct AggregatedRawData {
    RawMotionData hubData;          // Hub's own raw data
    RawMotionData handData;         // Left hand child data (for right hand hub)
    RawMotionData forearmData;      // Left forearm child data (for right forearm hub)
    RawMotionData shoulderData;     // Left shoulder child data (for right shoulder hub)
    bool handConnected;             // Left hand child connection status
    bool forearmConnected;          // Left forearm child connection status
    bool shoulderConnected;         // Left shoulder child connection status
    unsigned long timestamp;
} __attribute__((packed));

#endif // HUB_STRUCTURES_H 