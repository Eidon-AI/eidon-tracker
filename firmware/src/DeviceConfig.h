#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>

// Device role enumeration with clear documentation
enum DeviceRole {
    ROLE_LEFT_HAND = 0,      // Left hand/arm tracker
    ROLE_RIGHT_HAND = 1,     // Right hand/arm tracker
    ROLE_LEFT_FOREARM = 2,   // Left forearm tracker
    ROLE_RIGHT_FOREARM = 3,  // Right forearm tracker
    ROLE_LEFT_HUB = 4,       // Left hub/upper arm
    ROLE_RIGHT_HUB = 5,      // Right hub/upper arm
    ROLE_CHEST = 6,          // Chest tracker
    ROLE_LEFT_GLOVE = 8,     // Left glove (direct to phone, with finger sensors)
    ROLE_RIGHT_GLOVE = 9,    // Right glove (direct to phone, with finger sensors)
    ROLE_UNKNOWN = 255       // No role assigned (default state)
};

// Device configuration structure (enhanced for ESP-NOW support)
struct DeviceConfigData {
    DeviceRole role;           // ROLE_UNKNOWN = unassigned
    uint8_t hubMacAddress[6];  // All zeros = unassigned
} __attribute__((packed));

// Device configuration manager class
class DeviceConfig {
public:
    // Initialize configuration
    static bool begin();
    
    // Role management
    static DeviceRole getRole();
    static bool setRole(DeviceRole role);
    static bool isRoleAssigned();
    static const char* getRoleName(DeviceRole role);
    static bool isHubMode();
    static bool isNodeMode();
    static bool isGloveMode();
    static bool isStandaloneMode();
    
    // Hub MAC address management (for ESP-NOW)
    static bool setHubMacAddress(const uint8_t* macAddress);
    static bool getHubMacAddress(uint8_t* macAddress);
    static bool isHubMacAssigned();
    static bool clearHubMacAddress();
    static bool isValidMacAddress(const uint8_t* macAddress);
    static bool isAllZerosMacAddress(const uint8_t* macAddress);
    
    // Configuration persistence
    static bool saveConfig();
    static bool loadConfig();
    static bool resetConfig();
    
    // Device name generation
    static String generateDeviceName();
    
    // Validation
    static bool isValidRole(DeviceRole role);
    
private:
    static Preferences prefs;
    static DeviceConfigData config;
    static bool initialized;
    
    // Configuration keys
    static const char* CONFIG_NAMESPACE;
    static const char* ROLE_KEY;
    static const char* HUB_MAC_KEY;
};

// Global instance
extern DeviceConfig deviceConfig;

#endif // DEVICE_CONFIG_H 