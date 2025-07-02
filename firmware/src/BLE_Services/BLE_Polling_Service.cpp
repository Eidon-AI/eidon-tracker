#include "BLE_Polling_Service.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "DeviceConfig.h"
#include "BNO085.h"
#include "BLE_Services/RoleConfig_Service.h"

// External variables that handlers need access to
extern BNO085 imu;
extern DeviceConfig deviceConfig;
extern NimBLECharacteristic* roleConfigChar;
extern bool deviceConnected;

// Global instance
BLEPollingManager pollingManager;

// Default polling intervals
const unsigned long DEFAULT_ROLE_INTERVAL = 200;      // 5Hz
const unsigned long DEFAULT_CALIBRATION_INTERVAL = 100; // 10Hz
const unsigned long DEFAULT_COLOR_INTERVAL = 150;     // ~7Hz

// Battery mode intervals (slower to save power)
const unsigned long BATTERY_ROLE_INTERVAL = 500;      // 2Hz
const unsigned long BATTERY_CALIBRATION_INTERVAL = 250; // 4Hz
const unsigned long BATTERY_COLOR_INTERVAL = 300;     // ~3Hz

// Performance mode intervals (faster for responsiveness)
const unsigned long PERFORMANCE_ROLE_INTERVAL = 100;  // 10Hz
const unsigned long PERFORMANCE_CALIBRATION_INTERVAL = 50; // 20Hz
const unsigned long PERFORMANCE_COLOR_INTERVAL = 75;  // ~13Hz

bool BLEPollingManager::begin() {
    if (initialized) return true;
    
    // Initialize all targets
    for (int i = 0; i < MAX_TARGETS; i++) {
        targets[i].characteristic = nullptr;
        targets[i].lastValue = "";
        targets[i].lastCheck = 0;
        targets[i].interval = 0;
        targets[i].handler = nullptr;
        targets[i].name = nullptr;
        targets[i].enabled = false;
        targets[i].consecutiveFailures = 0;
        targets[i].maxFailures = 5; // Default max failures
        targets[i].stats = PollingStats();
    }
    
    targetCount = 0;
    initialized = true;
    
    Serial.println("BLE Polling Manager initialized");
    return true;
}

void BLEPollingManager::update() {
    if (!initialized) return;
    
    unsigned long currentTime = millis();
    
    for (int i = 0; i < targetCount; i++) {
        PollingTarget& target = targets[i];
        
        // Skip disabled targets
        if (!target.enabled) continue;
        
        // Check if it's time to poll this target
        if (currentTime - target.lastCheck >= target.interval) {
            unsigned long pollStartTime = currentTime;
                       
            // Attempt to read characteristic value
            std::string currentValue;
            bool success = false;
            
            if (target.characteristic != nullptr) {
                currentValue = target.characteristic->getValue();
                success = true; // Assume success if we got a value
            } else {
                success = false;
                target.consecutiveFailures++;
            }
            
            // Calculate response time
            unsigned long responseTime = millis() - pollStartTime;
            
            // Update statistics
            updateStats(target, success, responseTime);
            
            // Check if value changed
            if (success && currentValue != target.lastValue) {
                if (debugMode) {
                    Serial.printf("Polling %s: value changed, length=%d\n", 
                                 target.name, currentValue.length());
                }
                
                // Update statistics for successful change
                target.stats.successfulChanges++;
                target.stats.lastChangeTime = millis();
                
                // Call handler with new value
                if (target.handler) {
                    target.handler(currentValue, true);
                }
                
                target.lastValue = currentValue;
                target.consecutiveFailures = 0; // Reset failure counter on success
            } else if (!success) {
                // Handle failure
                if (target.consecutiveFailures >= target.maxFailures) {
                    Serial.printf("Polling failed for %s after %lu attempts\n", 
                                target.name, target.maxFailures);
                    
                    // Call handler with failure
                    if (target.handler) {
                        target.handler("", false);
                    }
                }
            }
            
            // Update last check time
            target.lastCheck = currentTime;
        }
    }
}

bool BLEPollingManager::addTarget(NimBLECharacteristic* characteristic, unsigned long interval, 
                                  std::function<void(const std::string&, bool)> handler, const char* name) {
    if (!initialized) {
        Serial.println("Polling Manager not initialized!");
        return false;
    }
    
    if (targetCount >= MAX_TARGETS) {
        Serial.printf("Cannot add target %s: maximum targets reached (%d)\n", name, MAX_TARGETS);
        return false;
    }
    
    // Check if target with this name already exists
    if (findTarget(name) != nullptr) {
        Serial.printf("Target %s already exists!\n", name);
        return false;
    }
    
    // Add new target
    PollingTarget& target = targets[targetCount];
    target.characteristic = characteristic;
    target.lastValue = "";
    target.lastCheck = 0;
    target.interval = interval;
    target.handler = handler;
    target.name = name;
    target.enabled = true;
    target.consecutiveFailures = 0;
    target.maxFailures = 5;
    target.stats = PollingStats();
    
    targetCount++;
    
    Serial.printf("Added polling target: %s (interval: %lu ms)\n", name, interval);
    return true;
}

bool BLEPollingManager::removeTarget(const char* name) {
    PollingTarget* target = findTarget(name);
    if (!target) {
        Serial.printf("Target %s not found for removal\n", name);
        return false;
    }
    
    // Shift remaining targets down
    int targetIndex = target - targets;
    for (int i = targetIndex; i < targetCount - 1; i++) {
        targets[i] = targets[i + 1];
    }
    
    // Clear the last target
    targets[targetCount - 1] = PollingTarget();
    targetCount--;
    
    Serial.printf("Removed polling target: %s\n", name);
    return true;
}

void BLEPollingManager::enableTarget(const char* name) {
    PollingTarget* target = findTarget(name);
    if (target) {
        target->enabled = true;
        Serial.printf("Enabled polling target: %s\n", name);
    } else {
        Serial.printf("Target %s not found for enable\n", name);
    }
}

void BLEPollingManager::disableTarget(const char* name) {
    PollingTarget* target = findTarget(name);
    if (target) {
        target->enabled = false;
        Serial.printf("Disabled polling target: %s\n", name);
    } else {
        Serial.printf("Target %s not found for disable\n", name);
    }
}

void BLEPollingManager::setInterval(const char* name, unsigned long interval) {
    PollingTarget* target = findTarget(name);
    if (target) {
        target->interval = interval;
        Serial.printf("Set polling interval for %s: %lu ms\n", name, interval);
    } else {
        Serial.printf("Target %s not found for interval change\n", name);
    }
}

void BLEPollingManager::setBatteryMode(bool lowBattery) {
    batteryMode = lowBattery;
    if (lowBattery) {
        Serial.println("Switching to battery mode (slower polling)");
        adjustIntervalsForBatteryMode();
    } else {
        Serial.println("Switching to normal mode");
        adjustIntervalsForPerformanceMode();
    }
}

void BLEPollingManager::setPerformanceMode(bool highPerformance) {
    performanceMode = highPerformance;
    if (highPerformance) {
        Serial.println("Switching to performance mode (faster polling)");
        adjustIntervalsForPerformanceMode();
    } else {
        Serial.println("Switching to normal mode");
        adjustIntervalsForPerformanceMode();
    }
}

void BLEPollingManager::printStats() {
    Serial.println("=== BLE Polling Statistics ===");
    for (int i = 0; i < targetCount; i++) {
        const PollingTarget& target = targets[i];
        const PollingStats& stats = target.stats;
        
        Serial.printf("%s: %lu polls, %lu changes, %lu fails, enabled=%d\n",
                     target.name, stats.totalPolls, stats.successfulChanges, 
                     stats.failedReads, target.enabled);
    }
}

void BLEPollingManager::printDetailedStats() {
    Serial.println("=== Detailed BLE Polling Statistics ===");
    for (int i = 0; i < targetCount; i++) {
        const PollingTarget& target = targets[i];
        const PollingStats& stats = target.stats;
        
        Serial.printf("%s:\n", target.name);
        Serial.printf("  Total polls: %lu\n", stats.totalPolls);
        Serial.printf("  Successful changes: %lu\n", stats.successfulChanges);
        Serial.printf("  Failed reads: %lu\n", stats.failedReads);
        Serial.printf("  Last change: %lu ms ago\n", millis() - stats.lastChangeTime);
        Serial.printf("  Average response time: %lu ms\n", stats.averageResponseTime);
        Serial.printf("  Current interval: %lu ms\n", target.interval);
        Serial.printf("  Consecutive failures: %lu\n", target.consecutiveFailures);
        Serial.printf("  Enabled: %s\n", target.enabled ? "YES" : "NO");
        Serial.println();
    }
}

void BLEPollingManager::resetStats(const char* name) {
    PollingTarget* target = findTarget(name);
    if (target) {
        target->stats = PollingStats();
        Serial.printf("Reset statistics for %s\n", name);
    } else {
        Serial.printf("Target %s not found for stats reset\n", name);
    }
}

void BLEPollingManager::resetAllStats() {
    for (int i = 0; i < targetCount; i++) {
        targets[i].stats = PollingStats();
    }
    Serial.println("Reset all polling statistics");
}

bool BLEPollingManager::isTargetEnabled(const char* name) const {
    const PollingTarget* target = findTarget(name);
    return target ? target->enabled : false;
}

unsigned long BLEPollingManager::getTargetInterval(const char* name) const {
    const PollingTarget* target = findTarget(name);
    return target ? target->interval : 0;
}

// Private helper methods
BLEPollingManager::PollingTarget* BLEPollingManager::findTarget(const char* name) {
    for (int i = 0; i < targetCount; i++) {
        if (targets[i].name && strcmp(targets[i].name, name) == 0) {
            return &targets[i];
        }
    }
    return nullptr;
}

const BLEPollingManager::PollingTarget* BLEPollingManager::findTarget(const char* name) const {
    for (int i = 0; i < targetCount; i++) {
        if (targets[i].name && strcmp(targets[i].name, name) == 0) {
            return &targets[i];
        }
    }
    return nullptr;
}

void BLEPollingManager::adjustIntervalsForBatteryMode() {
    for (int i = 0; i < targetCount; i++) {
        PollingTarget& target = targets[i];
        
        if (strcmp(target.name, "Role") == 0) {
            target.interval = BATTERY_ROLE_INTERVAL;
        } else if (strcmp(target.name, "Calibration") == 0) {
            target.interval = BATTERY_CALIBRATION_INTERVAL;
        } else if (strcmp(target.name, "Color") == 0) {
            target.interval = BATTERY_COLOR_INTERVAL;
        }
    }
}

void BLEPollingManager::adjustIntervalsForPerformanceMode() {
    for (int i = 0; i < targetCount; i++) {
        PollingTarget& target = targets[i];
        
        if (strcmp(target.name, "Role") == 0) {
            target.interval = PERFORMANCE_ROLE_INTERVAL;
        } else if (strcmp(target.name, "Calibration") == 0) {
            target.interval = PERFORMANCE_CALIBRATION_INTERVAL;
        } else if (strcmp(target.name, "Color") == 0) {
            target.interval = PERFORMANCE_COLOR_INTERVAL;
        }
    }
}

void BLEPollingManager::updateStats(PollingTarget& target, bool success, unsigned long responseTime) {
    PollingStats& stats = target.stats;
    stats.totalPolls++;
    
    if (success) {
        // Note: We don't track changes here since that's handled in the main polling loop
        // where we have proper type conversion
    } else {
        stats.failedReads++;
    }
    
    // Update average response time
    stats.totalResponseTime += responseTime;
    stats.averageResponseTime = stats.totalResponseTime / stats.totalPolls;
}

// ============================================================================
// Handler Function Implementations
// ============================================================================

// State validation function to ensure characteristic matches device state
void validateRoleConfigState() {
    if (roleConfigChar == nullptr) return;
    
    std::string currentValue = roleConfigChar->getValue();
    DeviceRole actualRole = deviceConfig.getRole();
    bool isAssigned = deviceConfig.isRoleAssigned();
    
    // Check if characteristic matches device state
    bool needsUpdate = false;
    
    if (currentValue.length() == 18) {
        // Full struct - check if it matches current state
        RoleConfigData* data = (RoleConfigData*)currentValue.data();
        if (data->role != (uint8_t)actualRole || data->assigned != (isAssigned ? 1 : 0)) {
            needsUpdate = true;
        }
    } else if (currentValue.length() != 1) {
        // Unexpected format - reset to current state
        needsUpdate = true;
    }
    
    if (needsUpdate) {
        Serial.println("Role Config: State mismatch detected, updating characteristic");
        updateRoleConfigCharacteristic();
    }
}

void handleRoleChange(const std::string& value, bool success) {
    if (!success) {
        Serial.println("Role Change: Failed to read characteristic value");
        return;
    }
    
    // Handle single-byte role assignment (client writes role value)
    if (value.length() == 1) {
        uint8_t newRole = (uint8_t)value[0];
        
        // Validate role value
        if (newRole <= 6) {
            Serial.println("=== ROLE CHANGE DETECTED ===");
            Serial.print("Role changed to: ");
            Serial.print(newRole);
            Serial.print(" (");
            Serial.print(deviceConfig.getRoleName((DeviceRole)newRole));
            Serial.println(")");
            
            // Apply the role change
            if (deviceConfig.setRole((DeviceRole)newRole)) {
                Serial.println("Role change applied successfully!");
                
                // 1. Update characteristic with full struct (state management)
                updateRoleConfigCharacteristic();
                
                // 2. Provide LED feedback
                startRoleChangeLEDPattern();
                
                // 3. Log final status
                Serial.print("Final status - Role: '");
                Serial.print(deviceConfig.getRoleName(deviceConfig.getRole()));
                Serial.print("', Assigned: ");
                Serial.print(deviceConfig.isRoleAssigned() ? "YES" : "NO");
                Serial.print(", Mode: ");
                Serial.println(deviceConfig.isHubMode() ? "HUB" : "NODE");
                
            } else {
                Serial.println("Failed to apply role change!");
            }
        } else {
            Serial.printf("Role Change: Invalid role value: %d\n", newRole);
        }
    } else {
        Serial.printf("Role Change: Unexpected value length: %d (expected 1)\n", value.length());
    }
    
    // Always validate state consistency after any role change attempt
    validateRoleConfigState();
}

void handleCalibration(const std::string& value, bool success) {
    if (!success) {
        Serial.println("Calibration: Failed to read characteristic value");
        return;
    }
    
    // Check for calibration command
    if (!value.empty()) {
        uint8_t cmd = static_cast<uint8_t>(value[0]);
        
        if (cmd == 0x01) { // Calibration command
            Serial.println("=== CALIBRATION REQUEST DETECTED ===");
            
            if (imu.isAvailable()) {
                Serial.println("Resetting IMU...");
                imu.reset();
                
                // LED feedback (if available)
                // Note: startIMUResetPattern() would need to be accessible
                // or we can implement LED feedback here
                
                Serial.println("IMU reset completed");
            } else {
                Serial.println("Calibration: IMU not available for reset");
            }
        } else {
            Serial.printf("Calibration: Unknown command: 0x%02X\n", cmd);
        }
    } else {
        Serial.println("Calibration: Empty value received");
    }
}

void handleColorChange(const std::string& value, bool success) {
    if (!success) {
        Serial.println("Color Change: Failed to read characteristic value");
        return;
    }
    
    // Expect 3-byte RGB values
    if (value.length() == 3) {
        uint8_t r = value[0];
        uint8_t g = value[1];
        uint8_t b = value[2];
        
        Serial.println("=== COLOR CHANGE DETECTED ===");
        Serial.printf("New color: R=%02X G=%02X B=%02X\n", r, g, b);
        
        // Update color manager (if available)
        // Note: This would need ColorManager to be accessible
        // For now, we'll just log the color change
        
        // Example of how this would work with ColorManager:
        // if (colorManager) {
        //     colorManager.setColor(r, g, b);
        // }
        
        Serial.println("Color change processed");
        
    } else {
        Serial.printf("Color Change: Unexpected value length: %d (expected 3)\n", value.length());
    }
}

void setupPollingSystem() {
    Serial.println("Setting up BLE Polling System...");
    
    // Initialize the polling manager
    if (!pollingManager.begin()) {
        Serial.println("ERROR: Failed to initialize polling manager!");
        return;
    }
    
    // Enable debug mode for initial setup
    pollingManager.setDebugMode(true);
    
    Serial.println("BLE Polling System setup complete");
} 