#include "RoleConfig_Service.h"
#include "../BLE_Services/BLE_Callbacks.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>

// External variables
extern DeviceConfig deviceConfig;
extern bool deviceConnected;

// Service and characteristic instances
NimBLEService* roleConfigService = nullptr;
NimBLECharacteristic* roleConfigChar = nullptr;
RoleConfigData roleConfigData;

// LED feedback function declaration
void startRoleChangeLEDPattern();

// Create the role configuration service
void createRoleConfigService(NimBLEServer* pServer) {
    // Create service
    roleConfigService = pServer->createService(ROLE_CONFIG_SERVICE_UUID);
    
    // Create characteristic with read/write permissions
    roleConfigChar = roleConfigService->createCharacteristic(
        ROLE_CONFIG_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    
    if (roleConfigChar == nullptr) {
        Serial.println("Role Config: ERROR - Characteristic creation failed!");
        return;
    }
    
    // Callbacks will be registered in main.cpp for consistency
    
    // Initialize characteristic with current role data (AFTER setting callbacks)
    updateRoleConfigCharacteristic();
    
    // Start service
    roleConfigService->start();
}

// Update characteristic with current role data
void updateRoleConfigCharacteristic() {
    DeviceRole currentRole = deviceConfig.getRole();
    bool isAssigned = deviceConfig.isRoleAssigned();
    
    // Fill the data structure
    roleConfigData.role = (uint8_t)currentRole;
    
    // Copy hub MAC address from device config
    if (deviceConfig.isHubMacAssigned()) {
        deviceConfig.getHubMacAddress(roleConfigData.hubMacAddress);
    } else {
        memset(roleConfigData.hubMacAddress, 0, sizeof(roleConfigData.hubMacAddress));
    }
    
    // Update characteristic value
    if (roleConfigChar != nullptr) {
        roleConfigChar->setValue((uint8_t*)&roleConfigData, sizeof(roleConfigData));
    } else {
        Serial.println("Role Config: ERROR - Characteristic pointer is null!");
    }
}

// Note: RoleConfigCallbacks implementation moved to BLE_Callbacks.cpp

// LED feedback pattern for role changes
void startRoleChangeLEDPattern() {
    // Quick double-blink pattern: on-off-on-off
    digitalWrite(2, HIGH);  // LED on
    delay(100);
    digitalWrite(2, LOW);   // LED off
    delay(100);
    digitalWrite(2, HIGH);  // LED on
    delay(100);
    digitalWrite(2, LOW);   // LED off
} 