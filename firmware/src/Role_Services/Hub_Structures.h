#ifndef HUB_STRUCTURES_H
#define HUB_STRUCTURES_H

#include <Arduino.h>
#include "DeviceConfig.h"

// Quaternion data structure for GATT (16 bytes) - unchanged
struct QuaternionData {
    float w;
    float x;
    float y;
    float z;
} __attribute__((packed));

// ESP-NOW packet structure for child to hub communication
struct ESPNowQuaternionPacket {
    uint8_t senderRole;        // Sender's role (0-6)
    QuaternionData quaternion; // Existing structure (unchanged)
} __attribute__((packed));

// ESP-NOW child device structure (replaces BLE ChildConnection)
struct ESPNowChildDevice {
    uint8_t macAddress[6];     // Child device MAC address
    DeviceRole role;           // Child device role
    bool dataAvailable;        // Whether we've received data recently
    QuaternionData lastData;   // Last received quaternion data
    unsigned long lastDataTime; // Timestamp of last data reception
    unsigned long consecutiveFailures; // Track reception failures
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

#endif // HUB_STRUCTURES_H 