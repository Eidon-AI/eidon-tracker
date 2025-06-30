#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>

// Device role enumeration with clear documentation
enum DeviceRole {
    ROLE_LEFT_HAND = 0,      // Left hand/arm
    ROLE_RIGHT_HAND = 1,     // Right hand/arm
    ROLE_LEFT_FOREARM = 2,   // Left forearm
    ROLE_RIGHT_FOREARM = 3,  // Right forearm
    ROLE_LEFT_HUB = 4,       // Left hub/upper arm
    ROLE_RIGHT_HUB = 5,      // Right hub/upper arm
    ROLE_CHEST = 6,          // Chest
    ROLE_UNKNOWN = 255       // No role assigned (default state)
};

// Device configuration structure (simplified)
struct DeviceConfigData {
    DeviceRole role;
    bool role_assigned;
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
    static const char* ROLE_ASSIGNED_KEY;
};

// Global instance
extern DeviceConfig deviceConfig;

#endif // DEVICE_CONFIG_H 