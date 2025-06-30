#include "DeviceConfig.h"
#include <NimBLEDevice.h>

// Static member initialization
Preferences DeviceConfig::prefs;
DeviceConfigData DeviceConfig::config;
bool DeviceConfig::initialized = false;

// Configuration keys
const char* DeviceConfig::CONFIG_NAMESPACE = "device_config";
const char* DeviceConfig::ROLE_KEY = "role";
const char* DeviceConfig::ROLE_ASSIGNED_KEY = "role_assigned";

// Role name mapping
static const char* ROLE_NAMES[] = {
    "L_HAND",
    "R_HAND", 
    "L_FOREARM",
    "R_FOREARM",
    "L_HUB",
    "R_HUB",
    "CHEST",
    "UNKNOWN"
};

bool DeviceConfig::begin() {
    if (initialized) {
        return true;
    }
    
    Serial.println("Initializing Device Configuration Manager...");
    
    // Initialize preferences
    if (!prefs.begin(CONFIG_NAMESPACE, false)) {
        Serial.println("Failed to initialize preferences for device config");
        return false;
    }
    
    // Load configuration
    if (!loadConfig()) {
        Serial.println("No saved configuration found, using defaults");
        // Set default configuration
        config.role = ROLE_UNKNOWN;
        config.role_assigned = false;
        
        // Save default configuration
        saveConfig();
    }
    
    initialized = true;
    
    Serial.print("Device Configuration loaded - Role: ");
    Serial.print(getRoleName(config.role));
    Serial.print(", Assigned: ");
    Serial.print(config.role_assigned ? "YES" : "NO");
    Serial.print(", Mode: ");
    Serial.println(isHubMode() ? "HUB" : "NODE");
    
    return true;
}

DeviceRole DeviceConfig::getRole() {
    if (!initialized) {
        begin();
    }
    return config.role;
}

bool DeviceConfig::setRole(DeviceRole role) {
    if (!initialized) {
        begin();
    }
    
    if (!isValidRole(role)) {
        Serial.println("Invalid device role");
        return false;
    }
    
    DeviceRole oldRole = config.role;
    config.role = role;
    config.role_assigned = true;
    
    // Save to preferences
    if (saveConfig()) {
        Serial.print("Device role changed from ");
        Serial.print(getRoleName(oldRole));
        Serial.print(" to ");
        Serial.println(getRoleName(role));
        return true;
    } else {
        Serial.println("Failed to save device role");
        return false;
    }
}

bool DeviceConfig::isRoleAssigned() {
    if (!initialized) {
        begin();
    }
    return config.role_assigned;
}

const char* DeviceConfig::getRoleName(DeviceRole role) {
    if (role == ROLE_UNKNOWN) {
        return ROLE_NAMES[7]; // "UNKNOWN"
    }
    if (role >= 0 && role <= 6) {
        return ROLE_NAMES[role];
    }
    return ROLE_NAMES[7]; // "UNKNOWN"
}

bool DeviceConfig::isHubMode() {
    if (!initialized) {
        begin();
    }
    // Hub mode is LEFT_HUB or RIGHT_HUB
    return (config.role == ROLE_LEFT_HUB || config.role == ROLE_RIGHT_HUB);
}

bool DeviceConfig::isNodeMode() {
    if (!initialized) {
        begin();
    }
    // Node mode is any assigned role that's not a hub
    return (config.role != ROLE_UNKNOWN && !isHubMode());
}

bool DeviceConfig::isValidRole(DeviceRole role) {
    return (role >= ROLE_LEFT_HAND && role <= ROLE_CHEST) || (role == ROLE_UNKNOWN);
}

bool DeviceConfig::saveConfig() {
    if (!prefs.putUChar(ROLE_KEY, (uint8_t)config.role)) {
        Serial.println("Failed to save device role");
        return false;
    }
    
    if (!prefs.putBool(ROLE_ASSIGNED_KEY, config.role_assigned)) {
        Serial.println("Failed to save role assigned flag");
        return false;
    }
    
    Serial.println("Device configuration saved successfully");
    return true;
}

bool DeviceConfig::loadConfig() {
    // Load role
    uint8_t roleValue = prefs.getUChar(ROLE_KEY, 255);
    config.role = (DeviceRole)roleValue;
    
    // Load role assigned flag
    config.role_assigned = prefs.getBool(ROLE_ASSIGNED_KEY, false);
    
    Serial.println("Device configuration loaded successfully");
    return true;
}

bool DeviceConfig::resetConfig() {
    if (!prefs.clear()) {
        Serial.println("Failed to clear device configuration");
        return false;
    }
    
    initialized = false;
    Serial.println("Device configuration reset successfully");
    return true;
}

String DeviceConfig::generateDeviceName() {
    if (!initialized) {
        begin();
    }
    
    // Get MAC address for unique suffix
    NimBLEAddress addr = NimBLEDevice::getAddress();
    String mac = addr.toString().c_str();
    String suffix;
    
    // Extract last 4 characters from MAC (excluding colons)
    for (int i = mac.length() - 1; i >= 0 && suffix.length() < 4; --i) {
        if (mac[i] != ':') {
            suffix = String((char)toupper(mac[i])) + suffix;
        }
    }
    
    // Generate role-specific name
    if (config.role == ROLE_UNKNOWN || !config.role_assigned) {
        char name[32];
        snprintf(name, sizeof(name), "Eidon-Tracker-%s", suffix.c_str());
        return String(name);
    } else {
        String rolePrefix = getRoleName(config.role);
        
        char name[32];
        snprintf(name, sizeof(name), "%s-Eidon-%s", rolePrefix.c_str(), suffix.c_str());
        return String(name);
    }
}

// Global instance
DeviceConfig deviceConfig; 