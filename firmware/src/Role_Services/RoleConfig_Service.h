#ifndef ROLE_CONFIG_SERVICE_H
#define ROLE_CONFIG_SERVICE_H

#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>
#include "DeviceConfig.h"

// Role Configuration Service UUID
#define ROLE_CONFIG_SERVICE_UUID  "E1D00006-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define ROLE_CONFIG_CHAR_UUID     "E1D00007-8B5A-3E5B-9E23-4F9B5C91BBDE"

// Role configuration data structure - Enhanced for ESP-NOW support
struct RoleConfigData {
    uint8_t role;                    // Current role (0-6), 255 = ROLE_UNKNOWN (unassigned)
    char roleName[16];               // Role name string
    uint8_t hubMacAddress[6];        // Hub MAC address (all zeros = unassigned)
} __attribute__((packed));

// Role configuration characteristic callback (defined in BLE_Callbacks.h)

// Function declarations
void createRoleConfigService(NimBLEServer* pServer);
void updateRoleConfigCharacteristic();
void startRoleChangeLEDPattern();

// External variables
extern NimBLEService* roleConfigService;
extern NimBLECharacteristic* roleConfigChar;
extern RoleConfigData roleConfigData;

#endif // ROLE_CONFIG_SERVICE_H 