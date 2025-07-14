#ifndef HUB_STRUCTURES_H
#define HUB_STRUCTURES_H

#include <NimBLEDevice.h>
#include <NimBLEClient.h>
#include "DeviceConfig.h"

// Quaternion data structure for GATT (16 bytes)
struct QuaternionData {
    float w;
    float x;
    float y;
    float z;
} __attribute__((packed));

// Hub client connection structure
struct ChildConnection {
    NimBLEClient* client;
    NimBLEAddress address;
    DeviceRole role;
    bool connected;
    bool dataAvailable;
    QuaternionData lastData;
    unsigned long lastDataTime;
    unsigned long connectionAttempts;
    unsigned long lastConnectionAttempt;
};

// Aggregated quaternion data for hub
struct AggregatedQuaternionData {
    QuaternionData hubData;
    QuaternionData handData;
    QuaternionData forearmData;
    bool handConnected;
    bool forearmConnected;
    unsigned long timestamp;
} __attribute__((packed));

#endif // HUB_STRUCTURES_H 