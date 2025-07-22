#include "HubClient_Service.h"
#include <Arduino.h>
#include <WiFi.h>

// External dependencies
extern DeviceConfig deviceConfig;

// Global instance
HubClientService hubClientService;

// ESP-NOW callback function
void onESPNowDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int dataLen) {
    // Debug: Log every callback to see if it's being called
    static unsigned long lastCallbackLog = 0;
    if (millis() - lastCallbackLog >= 5000) { // Log every 5 seconds
        Serial.printf("HubClient: ESP-NOW callback triggered at %lu ms\n", millis());
        lastCallbackLog = millis();
    }
    
    // Rate limit reception logs to avoid flooding
    static unsigned long lastReceptionLog = 0;
    static int receptionCount = 0;
    
    receptionCount++;
    if (millis() - lastReceptionLog >= 5000) { // Log every 5 seconds
        Serial.printf("HubClient: Received %d ESP-NOW packets in last 5s from %02X:%02X:%02X:%02X:%02X:%02X, size: %d\n",
                     receptionCount, esp_now_info->src_addr[0], esp_now_info->src_addr[1], esp_now_info->src_addr[2],
                     esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5], dataLen);
        lastReceptionLog = millis();
        receptionCount = 0;
    }
    
    // Debug: Log every 100th packet to track if ESP-NOW is still working
    static int packetCounter = 0;
    static unsigned long lastESPNowPacketTime = 0;
    packetCounter++;
    lastESPNowPacketTime = millis();
    
    if (packetCounter % 100 == 0) {
        Serial.printf("HubClient: ESP-NOW packet #%d received at %lu ms\n", packetCounter, millis());
    }
    
    // Debug: Log WiFi and ESP-NOW status
    static unsigned long lastStatusLog = 0;
    if (millis() - lastStatusLog >= 10000) {
        lastStatusLog = millis();
        Serial.printf("HubClient: WiFi status - Mode: %d, Connected: %s\n", 
                     WiFi.getMode(), WiFi.status() == WL_CONNECTED ? "YES" : "NO");
        Serial.printf("HubClient: ESP-NOW callback active at %lu ms\n", millis());
    }
    
    hubClientService.processESPNowPacket(esp_now_info->src_addr, data, dataLen);
}

// Constructor
HubClientService::HubClientService() 
    : childDeviceCount(0), espNowInitialized(false) {
    // Initialize child devices
    for (int i = 0; i < MAX_CHILDREN; i++) {
        memset(childDevices[i].macAddress, 0, sizeof(childDevices[i].macAddress));
        childDevices[i].role = ROLE_UNKNOWN;
        childDevices[i].dataAvailable = false;
        childDevices[i].consecutiveFailures = 0;
    }
    
    // Initialize aggregated data
    memset(&aggregatedData, 0, sizeof(aggregatedData));
    aggregatedData.handConnected = false;
    aggregatedData.forearmConnected = false;
}

// Destructor
HubClientService::~HubClientService() {
    // Clean up ESP-NOW if initialized
    if (espNowInitialized) {
        esp_now_del_peer(0); // Remove all peers
    }
}

// Initialize ESP-NOW receiver
bool HubClientService::initializeESPNow() {
    if (espNowInitialized) {
        return true; // Already initialized
    }
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("HubClient: Failed to initialize ESP-NOW");
        return false;
    }
    
    // Set ESP-NOW role to receiver
    if (esp_now_set_pmk((uint8_t*)"pmk1234567890123") != ESP_OK) {
        Serial.println("HubClient: Failed to set ESP-NOW PMK");
        return false;
    }
    
    // Set WiFi channel to match child devices (channel 1)
    WiFi.setChannel(1);
    Serial.println("HubClient: WiFi channel set to 1 for ESP-NOW");
    
    // Configure WiFi for BLE coexistence
    WiFi.setSleep(false); // Disable WiFi sleep to prevent conflicts
    Serial.println("HubClient: WiFi sleep disabled for BLE coexistence");
    
    // Register callback function
    esp_now_register_recv_cb(onESPNowDataRecv);
    
    espNowInitialized = true;
    Serial.println("HubClient: ESP-NOW receiver initialized successfully");
    return true;
}

// Initialize the hub client service
void HubClientService::begin() {
    if (!deviceConfig.isHubMode()) {
        Serial.println("HubClient: Not a hub, ESP-NOW receiver service disabled");
        return;
    }
    
    Serial.println("HubClient: Initializing ESP-NOW receiver service...");
    
    // Initialize ESP-NOW
    if (!initializeESPNow()) {
        Serial.println("HubClient: Failed to initialize ESP-NOW");
        return;
    }
    
    // Reset child devices
    childDeviceCount = 0;
    for (int i = 0; i < MAX_CHILDREN; i++) {
        memset(childDevices[i].macAddress, 0, sizeof(childDevices[i].macAddress));
        childDevices[i].role = ROLE_UNKNOWN;
        childDevices[i].dataAvailable = false;
        childDevices[i].consecutiveFailures = 0;
    }
    
    // Initialize aggregated data
    memset(&aggregatedData, 0, sizeof(aggregatedData));
    aggregatedData.handConnected = false;
    aggregatedData.forearmConnected = false;
    
    Serial.println("HubClient: ESP-NOW receiver service initialized successfully");
}

// Main update function
void HubClientService::update() {
    if (!deviceConfig.isHubMode() || !espNowInitialized) {
        return; // Not a hub or ESP-NOW not initialized
    }
    
    // Check for stale child data (timeout handling)
    unsigned long currentTime = millis();
    for (int i = 0; i < childDeviceCount; i++) {
        if (childDevices[i].dataAvailable) {
            unsigned long timeSinceLastData = currentTime - childDevices[i].lastDataTime;
            
            // Debug: Log timeout check (every 5 seconds to avoid spam)
            static unsigned long lastTimeoutDebug = 0;
            if (millis() - lastTimeoutDebug >= 5000) {
                Serial.printf("HubClient: Timeout check - %s: %lu ms since last data (timeout: %d ms)\n", 
                             deviceConfig.getRoleName(childDevices[i].role), timeSinceLastData, CHILD_DATA_TIMEOUT_MS);
                lastTimeoutDebug = millis();
            }
            
            if (timeSinceLastData > CHILD_DATA_TIMEOUT_MS) {
                Serial.printf("HubClient: Child %s data timeout, marking as disconnected\n", 
                             deviceConfig.getRoleName(childDevices[i].role));
                childDevices[i].dataAvailable = false;
                childDevices[i].consecutiveFailures++;
            }
        }
    }
    
    // Log connection status (every 10 seconds)
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime > 10000) {
        lastDebugTime = currentTime;
        
        // Count connected children and data availability
        int connectedCount = 0;
        int dataAvailableCount = 0;
        for (int i = 0; i < childDeviceCount; i++) {
            if (childDevices[i].dataAvailable) {
                connectedCount++;
                dataAvailableCount++;
            }
        }
        
        Serial.printf("HubClient: %d/%d children connected via ESP-NOW, %d sending data\n", 
                     connectedCount, MAX_CHILDREN, dataAvailableCount);
        
        // Show detailed status for each child
        for (int i = 0; i < childDeviceCount; i++) {
            if (childDevices[i].role != ROLE_UNKNOWN) {
                unsigned long timeSinceLastData = childDevices[i].dataAvailable ? 
                    (currentTime - childDevices[i].lastDataTime) : 0;
                Serial.printf("  - %s: %s (last data: %lu ms ago)\n", 
                             deviceConfig.getRoleName(childDevices[i].role),
                             childDevices[i].dataAvailable ? "CONNECTED" : "DISCONNECTED",
                             timeSinceLastData);
            }
        }
        
        // ESP-NOW health check
        static unsigned long lastESPNowPacketTime = 0;
        if (lastESPNowPacketTime > 0 && (currentTime - lastESPNowPacketTime) > 15000) {
            Serial.println("HubClient: WARNING - No ESP-NOW packets received in 15+ seconds!");
            Serial.println("HubClient: ESP-NOW may have been disrupted by BLE connection");
        }
    }
}

// Process incoming ESP-NOW packet
void HubClientService::processESPNowPacket(const uint8_t* macAddr, const uint8_t* data, int dataLen) {
    if (dataLen != sizeof(ESPNowQuaternionPacket)) {
        Serial.printf("HubClient: Invalid ESP-NOW packet size: %d (expected %d)\n", 
                     dataLen, sizeof(ESPNowQuaternionPacket));
        return;
    }
    
    ESPNowQuaternionPacket* packet = (ESPNowQuaternionPacket*)data;
    DeviceRole senderRole = (DeviceRole)packet->senderRole;
    
    // Validate sender role
    if (senderRole != ROLE_LEFT_HAND && senderRole != ROLE_RIGHT_HAND &&
        senderRole != ROLE_LEFT_FOREARM && senderRole != ROLE_RIGHT_FOREARM) {
        Serial.printf("HubClient: Invalid sender role in ESP-NOW packet: %d\n", senderRole);
        return;
    }
    
    // Find or create child device slot
    int slotIndex = findChildSlot(senderRole);
    if (slotIndex == -1) {
        slotIndex = createChildSlot();
    }
    
    if (slotIndex == -1) {
        Serial.println("HubClient: No available slots for child device");
        return;
    }
    
    // Update child device data
    ESPNowChildDevice& child = childDevices[slotIndex];
    memcpy(child.macAddress, macAddr, sizeof(child.macAddress));
    child.role = senderRole;
    child.lastData = packet->quaternion;
    child.dataAvailable = true;
    child.lastDataTime = millis();
    child.consecutiveFailures = 0;
    
    // Debug: Log when we update the timestamp (every 10 seconds to avoid spam)
    static unsigned long lastTimestampLog = 0;
    if (millis() - lastTimestampLog >= 10000) {
        Serial.printf("HubClient: Updated timestamp for %s to %lu\n", 
                     deviceConfig.getRoleName(senderRole), child.lastDataTime);
        lastTimestampLog = millis();
    }
    
    // Log received data (every 5 seconds to avoid spam)
    static unsigned long lastDataLog = 0;
    if (millis() - lastDataLog >= 5000) {
        Serial.printf("HubClient: ESP-NOW data from %s - W=%.4f X=%.4f Y=%.4f Z=%.4f\n", 
                     deviceConfig.getRoleName(senderRole),
                     packet->quaternion.w, packet->quaternion.x, 
                     packet->quaternion.y, packet->quaternion.z);
        lastDataLog = millis();
    }
}

// Find existing child slot by role
int HubClientService::findChildSlot(DeviceRole childRole) {
    for (int i = 0; i < childDeviceCount; i++) {
        if (childDevices[i].role == childRole) {
            return i;
        }
    }
    return -1; // Not found
}

// Create new child slot
int HubClientService::createChildSlot() {
    if (childDeviceCount < MAX_CHILDREN) {
        return childDeviceCount++;
    }
    return -1; // No available slots
}

// Register a child device (for future reference)
void HubClientService::registerChildDevice(const uint8_t* macAddress, DeviceRole childRole) {
    if (!deviceConfig.isHubMode()) {
        return;
    }
    
    Serial.printf("HubClient: Registering child %s with MAC %02X:%02X:%02X:%02X:%02X:%02X\n", 
                 deviceConfig.getRoleName(childRole),
                 macAddress[0], macAddress[1], macAddress[2], 
                 macAddress[3], macAddress[4], macAddress[5]);
    
    // Find or create child device slot
    int slotIndex = findChildSlot(childRole);
    if (slotIndex == -1) {
        slotIndex = createChildSlot();
    }
    
    if (slotIndex == -1) {
        Serial.println("HubClient: No available slots for child registration");
        return;
    }
    
    // Update child device info
    ESPNowChildDevice& child = childDevices[slotIndex];
    memcpy(child.macAddress, macAddress, sizeof(child.macAddress));
    child.role = childRole;
    child.dataAvailable = false;
    child.consecutiveFailures = 0;
}

// Unregister a child device
void HubClientService::unregisterChildDevice(DeviceRole childRole) {
    for (int i = 0; i < childDeviceCount; i++) {
        if (childDevices[i].role == childRole) {
            Serial.printf("HubClient: Unregistering child %s\n", deviceConfig.getRoleName(childRole));
            
            // Shift remaining devices to fill the gap
            for (int j = i; j < childDeviceCount - 1; j++) {
                childDevices[j] = childDevices[j + 1];
            }
            childDeviceCount--;
            break;
        }
    }
}

// Check if a child is connected (has recent data)
bool HubClientService::isChildConnected(DeviceRole childRole) {
    for (int i = 0; i < childDeviceCount; i++) {
        if (childDevices[i].role == childRole && childDevices[i].dataAvailable) {
            return true;
        }
    }
    return false;
}

// Update child data (called from main loop)
void HubClientService::updateChildData() {
    if (!deviceConfig.isHubMode()) {
        return;
    }
    
    syncConnectionStatus();
}

// Update hub's own quaternion data with current IMU readings
void HubClientService::updateHubQuaternionData(float w, float x, float y, float z) {
    if (!deviceConfig.isHubMode()) {
        return;
    }
    
    aggregatedData.hubData.w = w;
    aggregatedData.hubData.x = x;
    aggregatedData.hubData.y = y;
    aggregatedData.hubData.z = z;
}

// Synchronize connection status with actual child data state
void HubClientService::syncConnectionStatus() {
    // Update timestamp for data freshness tracking
    aggregatedData.timestamp = millis();
    
    // Reset connection flags - we'll set them based on actual child status
    aggregatedData.handConnected = false;
    aggregatedData.forearmConnected = false;
    
    // Iterate through all child devices to determine actual status
    for (int i = 0; i < childDeviceCount; i++) {
        ESPNowChildDevice& child = childDevices[i];
        
        if (child.dataAvailable) {
            if (child.role == ROLE_LEFT_HAND || child.role == ROLE_RIGHT_HAND) {
                aggregatedData.handData = child.lastData;
                aggregatedData.handConnected = true;
            } else if (child.role == ROLE_LEFT_FOREARM || child.role == ROLE_RIGHT_FOREARM) {
                aggregatedData.forearmData = child.lastData;
                aggregatedData.forearmConnected = true;
            }
        }
    }
}

// Setup function for integration with main.cpp
void setupHubClientService() {
    hubClientService.begin();
}

// Update function for integration with main.cpp
void updateHubClientService() {
    hubClientService.update();
}

// Wrapper functions for main.cpp compatibility
void registerChildDevice(const uint8_t* macAddress, DeviceRole childRole) {
    hubClientService.registerChildDevice(macAddress, childRole);
}

void unregisterChildDevice(DeviceRole childRole) {
    hubClientService.unregisterChildDevice(childRole);
}

void updateChildData() {
    hubClientService.updateChildData();
}

void updateHubQuaternionData(float w, float x, float y, float z) {
    hubClientService.updateHubQuaternionData(w, x, y, z);
}

bool isChildConnected(DeviceRole childRole) {
    return hubClientService.isChildConnected(childRole);
}

AggregatedQuaternionData* getAggregatedData() {
    return hubClientService.getAggregatedData();
} 