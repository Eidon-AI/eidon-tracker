#include "DeviceConfig.h"
#include <WiFi.h>

// Static member initialization
Preferences DeviceConfig::prefs;
DeviceConfigData DeviceConfig::config;
bool DeviceConfig::initialized = false;

// Configuration keys
const char* DeviceConfig::CONFIG_NAMESPACE = "eidon_config";
const char* DeviceConfig::ROLE_KEY = "role";
const char* DeviceConfig::HUB_MAC_KEY = "hub_mac";

// Global instance
DeviceConfig deviceConfig;

bool DeviceConfig::begin() {
    if (initialized) {
        return true;
    }
    
    // Initialize preferences
    if (!prefs.begin(CONFIG_NAMESPACE, false)) {
        Serial.println("DeviceConfig: Failed to initialize preferences");
        return false;
    }
    
    // Set initialized flag BEFORE calling loadConfig/saveConfig
    initialized = true;
    
    // Load configuration
    if (!loadConfig()) {
        Serial.println("DeviceConfig: Failed to load configuration, using defaults");
        // Initialize with defaults
        config.role = ROLE_UNKNOWN;
        memset(config.hubMacAddress, 0, sizeof(config.hubMacAddress));
        
        // Save default configuration
        saveConfig();
    }
    
    Serial.printf("DeviceConfig: Initialized - Role: %s\n", getRoleName(config.role));
    return true;
}

DeviceRole DeviceConfig::getRole() {
    return config.role;
}

bool DeviceConfig::setRole(DeviceRole role) {
    if (!isValidRole(role)) {
        Serial.printf("DeviceConfig: Invalid role %d\n", (int)role);
        return false;
    }
    
    config.role = role;
    

    return saveConfig();
}

bool DeviceConfig::isRoleAssigned() {
    return (config.role != ROLE_UNKNOWN);
}

const char* DeviceConfig::getRoleName(DeviceRole role) {
    switch (role) {
        case ROLE_LEFT_HAND: return "Left Hand";
        case ROLE_RIGHT_HAND: return "Right Hand";
        case ROLE_LEFT_FOREARM: return "Left Forearm";
        case ROLE_RIGHT_FOREARM: return "Right Forearm";
        case ROLE_LEFT_HUB: return "Left Hub";
        case ROLE_RIGHT_HUB: return "Right Hub";
        case ROLE_CHEST: return "Chest";
        case ROLE_UNKNOWN: return "Unknown";
        default: return "Invalid";
    }
}

bool DeviceConfig::isHubMode() {
    return (config.role == ROLE_LEFT_HUB || config.role == ROLE_RIGHT_HUB);
}

bool DeviceConfig::isNodeMode() {
    return (config.role == ROLE_LEFT_HAND || config.role == ROLE_RIGHT_HAND ||
            config.role == ROLE_LEFT_FOREARM || config.role == ROLE_RIGHT_FOREARM);
}

// Hub MAC address management functions
bool DeviceConfig::setHubMacAddress(const uint8_t* macAddress) {
    if (macAddress == nullptr) {
        Serial.println("DeviceConfig: Invalid MAC address pointer");
        return false;
    }
    
    // Copy MAC address
    memcpy(config.hubMacAddress, macAddress, sizeof(config.hubMacAddress));
    

    
    return saveConfig();
}

bool DeviceConfig::getHubMacAddress(uint8_t* macAddress) {
    if (macAddress == nullptr) {
        Serial.println("DeviceConfig: Invalid MAC address pointer");
        return false;
    }
    
    if (isAllZerosMacAddress(config.hubMacAddress)) {
        Serial.println("DeviceConfig: No hub MAC address assigned");
        return false;
    }
    
    memcpy(macAddress, config.hubMacAddress, sizeof(config.hubMacAddress));
    return true;
}

bool DeviceConfig::isHubMacAssigned() {
    return !isAllZerosMacAddress(config.hubMacAddress);
}

bool DeviceConfig::clearHubMacAddress() {
    memset(config.hubMacAddress, 0, sizeof(config.hubMacAddress));
    
    Serial.println("DeviceConfig: Hub MAC address cleared");
    return saveConfig();
}

bool DeviceConfig::isAllZerosMacAddress(const uint8_t* macAddress) {
    if (macAddress == nullptr) {
        return true;
    }
    
    for (int i = 0; i < 6; i++) {
        if (macAddress[i] != 0x00) {
            return false;
        }
    }
    return true;
}

bool DeviceConfig::isValidMacAddress(const uint8_t* macAddress) {
    if (macAddress == nullptr) {
        return false;
    }
    
    // Check if MAC address is all zeros (unassigned) - this is valid
    if (isAllZerosMacAddress(macAddress)) {
        return true;
    }
    
    // Check for broadcast MAC (FF:FF:FF:FF:FF:FF) - not valid for hub
    bool allFF = true;
    for (int i = 0; i < 6; i++) {
        if (macAddress[i] != 0xFF) {
            allFF = false;
            break;
        }
    }
    
    if (allFF) {
        Serial.println("DeviceConfig: Broadcast MAC address not valid for hub");
        return false;
    }
    
    // Check for locally administered MAC (bit 1 of first byte set)
    if ((macAddress[0] & 0x02) != 0) {
        Serial.println("DeviceConfig: Locally administered MAC address not valid for hub");
        return false;
    }
    
    return true;
}

bool DeviceConfig::saveConfig() {
    if (!initialized) {
        Serial.println("DeviceConfig: Not initialized, cannot save");
        return false;
    }
    
    // Save role configuration
    prefs.putUChar(ROLE_KEY, (uint8_t)config.role);
    
    // Save hub MAC address configuration
    prefs.putBytes(HUB_MAC_KEY, config.hubMacAddress, sizeof(config.hubMacAddress));
    

    return true;
}

bool DeviceConfig::loadConfig() {
    if (!initialized) {
        Serial.println("DeviceConfig: Not initialized, cannot load");
        return false;
    }
    
    // Load role configuration
    config.role = (DeviceRole)prefs.getUChar(ROLE_KEY, ROLE_UNKNOWN);
    
    // Load hub MAC address configuration
    size_t macSize = prefs.getBytes(HUB_MAC_KEY, config.hubMacAddress, sizeof(config.hubMacAddress));
    if (macSize != sizeof(config.hubMacAddress)) {
        // Initialize with zeros if not found
        memset(config.hubMacAddress, 0, sizeof(config.hubMacAddress));
    }
    
    Serial.printf("DeviceConfig: Configuration loaded - Role: %s, Hub MAC: %s\n",
                 getRoleName(config.role),
                 isHubMacAssigned() ? "ASSIGNED" : "NOT ASSIGNED");
    
    return true;
}

bool DeviceConfig::resetConfig() {
    if (!initialized) {
        Serial.println("DeviceConfig: Not initialized, cannot reset");
        return false;
    }
    
    // Clear all preferences
    prefs.clear();
    
    // Reset to defaults
    config.role = ROLE_UNKNOWN;
    memset(config.hubMacAddress, 0, sizeof(config.hubMacAddress));
    
    Serial.println("DeviceConfig: Configuration reset to defaults");
    return true;
}

String DeviceConfig::generateDeviceName() {
    // Get device's WiFi MAC address for unique identification
    uint8_t deviceMac[6];
    WiFi.macAddress(deviceMac);
    
    // Debug: Print the MAC address we're using
    
    
    // Try to get BLE MAC (WiFi MAC + 2)
    uint8_t bleMac[6];
    memcpy(bleMac, deviceMac, 6);
    bleMac[5] += 2;
    
    
    
    // Create device name: Eidon-Tracker-<last 4 digits of BLE MAC>
    char macSuffix[5];
    snprintf(macSuffix, sizeof(macSuffix), "%02X%02X", bleMac[4], bleMac[5]);
    
    String deviceName = String("Eidon-Tracker-") + String(macSuffix);

    
    return deviceName;
}

bool DeviceConfig::isValidRole(DeviceRole role) {
    return (role >= ROLE_LEFT_HAND && role <= ROLE_RIGHT_HUB) || role == ROLE_CHEST || role == ROLE_UNKNOWN;
} 