#ifndef BLE_POLLING_SERVICE_H
#define BLE_POLLING_SERVICE_H

#include <Arduino.h>
#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>
#include <functional>
#include <string>

// Forward declarations
class BNO085;
class DeviceConfig;
class ColorManager;

// Polling statistics structure
struct PollingStats {
    unsigned long totalPolls = 0;
    unsigned long successfulChanges = 0;
    unsigned long failedReads = 0;
    unsigned long lastChangeTime = 0;
    unsigned long averageResponseTime = 0;
    unsigned long totalResponseTime = 0;
};

// Main polling manager class
class BLEPollingManager {
private:
    struct PollingTarget {
        NimBLECharacteristic* characteristic;
        std::string lastValue;
        unsigned long lastCheck;
        unsigned long interval;
        std::function<void(const std::string&, bool)> handler;
        const char* name;
        bool enabled;
        unsigned long consecutiveFailures;
        unsigned long maxFailures;
        PollingStats stats;
    };
    
    static const int MAX_TARGETS = 10;
    PollingTarget targets[MAX_TARGETS];
    int targetCount = 0;
    bool initialized = false;
    bool debugMode = false;
    bool batteryMode = false;
    bool performanceMode = false;
    
public:
    // Core functionality
    bool begin();
    void update();
    bool addTarget(NimBLECharacteristic* characteristic, unsigned long interval, 
                   std::function<void(const std::string&, bool)> handler, const char* name);
    bool removeTarget(const char* name);
    void enableTarget(const char* name);
    void disableTarget(const char* name);
    void setInterval(const char* name, unsigned long interval);
    
    // Configuration
    void setDebugMode(bool enabled) { debugMode = enabled; }
    void setBatteryMode(bool lowBattery);
    void setPerformanceMode(bool highPerformance);
    
    // Statistics and monitoring
    void printStats();
    void printDetailedStats();
    void resetStats(const char* name);
    void resetAllStats();
    
    // Utility
    bool isTargetEnabled(const char* name) const;
    unsigned long getTargetInterval(const char* name) const;
    int getTargetCount() const { return targetCount; }
    
private:
    // Internal helper methods
    PollingTarget* findTarget(const char* name);
    const PollingTarget* findTarget(const char* name) const;
    void adjustIntervalsForBatteryMode();
    void adjustIntervalsForPerformanceMode();
    void updateStats(PollingTarget& target, bool success, unsigned long responseTime);
};

// Global instance
extern BLEPollingManager pollingManager;

// Handler function declarations
void handleRoleChange(const std::string& value, bool success);
void handleCalibration(const std::string& value, bool success);
void handleColorChange(const std::string& value, bool success);
void validateRoleConfigState();

// Advertising update function
void updateAdvertisingData();

// Hub client functions
void startChildDiscovery();

// Setup function
void setupPollingSystem();

#endif // BLE_POLLING_SERVICE_H 