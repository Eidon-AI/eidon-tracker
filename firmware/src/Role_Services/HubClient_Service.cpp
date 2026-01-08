#include "HubClient_Service.h"
#include <Arduino.h>
#include <WiFi.h>
#include "Hub_Structures.h"

// External dependencies
extern DeviceConfig deviceConfig;

// Global instance
HubClientService hubClientService;



// Constructor
HubClientService::HubClientService() 
    : childDeviceCount(0), espNowInitialized(false), packetCounter(0), lastLogTime(0),
      handMissedPolls(0), forearmMissedPolls(0) {
    // Initialize child devices
    for (int i = 0; i < MAX_CHILDREN; i++) {
        memset(childDevices[i].macAddress, 0, sizeof(childDevices[i].macAddress));
        childDevices[i].role = ROLE_UNKNOWN;
        childDevices[i].dataAvailable = false;
    }
    
    // Initialize aggregated data
    memset(&aggregatedData, 0, sizeof(aggregatedData));
    aggregatedData.handConnected = false;
    aggregatedData.forearmConnected = false;
    
    // Reset disconnection tracking
    handMissedPolls = 0;
    forearmMissedPolls = 0;
}

// Send calibration command to all connected children
void HubClientService::sendCalibrationCommand() {
    if (!espNowInitialized) {
        Serial.println("HubClient: ESP-NOW not initialized, cannot send command");
        return;
    }
    
    ESPNowCommandPacket cmd;
    cmd.header.messageType = MESSAGE_TYPE_CMD;
    cmd.header.senderRole = (uint8_t)deviceConfig.getRole();
    cmd.header.reserved[0] = 0;
    cmd.header.reserved[1] = 0;
    cmd.commandType = 0x01;  // IMU reset
    cmd.reserved[0] = 0;
    cmd.reserved[1] = 0;
    cmd.reserved[2] = 0;
    
    // Send to all registered children (ignore dataAvailable status for calibration)
    int sentCount = 0;
    Serial.printf("HUB: Attempting to send calibration to %d children\n", childDeviceCount);
    
    for (int i = 0; i < childDeviceCount; i++) {
        // Debug: Log each child's details
        Serial.printf("HUB: Child %d - MAC: %02X:%02X:%02X:%02X:%02X:%02X, Role: %s\n", 
                     i,
                     childDevices[i].macAddress[0], childDevices[i].macAddress[1], childDevices[i].macAddress[2],
                     childDevices[i].macAddress[3], childDevices[i].macAddress[4], childDevices[i].macAddress[5],
                     deviceConfig.getRoleName(childDevices[i].role));
        
        // Send calibration command to all registered children, regardless of recent data status
        esp_err_t result = esp_now_send(childDevices[i].macAddress, (const uint8_t*)&cmd, sizeof(cmd));
        if (result == ESP_OK) {
            sentCount++;
            Serial.printf("HUB: Calibration sent successfully to child %d\n", i);
        } else {
            Serial.printf("HUB: Failed to send calibration to child %d, error: %d\n", i, result);
        }
    }
    
    // Debug: Log final result
    Serial.printf("HUB: Calibration sent to %d/%d children\n", sentCount, childDeviceCount);
}

// Register a child device as an ESP-NOW peer so hub can send commands to it
bool HubClientService::registerChildAsESPNowPeer(const uint8_t* macAddress) {
    if (!espNowInitialized) {
        Serial.println("HubClient: ESP-NOW not initialized, cannot register peer");
        return false;
    }
    
    // Create ESP-NOW peer info
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, macAddress, 6);
    peerInfo.channel = 1;  // Use same channel as quaternion communication
    peerInfo.encrypt = false;  // No encryption to match existing setup
    
    // Add the peer
    esp_err_t result = esp_now_add_peer(&peerInfo);
    if (result == ESP_OK) {
        return true;
    } else if (result == ESP_ERR_ESPNOW_EXIST) {
        return true;  // Already exists, consider it successful
    } else {
        Serial.printf("HubClient: Failed to register child as ESP-NOW peer, error: %d\n", result);
        return false;
    }
}

// Destructor
HubClientService::~HubClientService() {
    // Clean up ESP-NOW if initialized
    if (espNowInitialized) {
        esp_now_del_peer(0); // Remove all peers
    }
}

// Restore ESP-NOW peer registrations after reinitialization
void HubClientService::restoreESPNowPeers() {
    if (!espNowInitialized) {
        Serial.println("HubClient: Cannot restore peers - ESP-NOW not initialized");
        return;
    }
    
    Serial.println("HubClient: Restoring ESP-NOW peer registrations...");
    
    // Re-register all known children as ESP-NOW peers
    for (int i = 0; i < childDeviceCount; i++) {
        registerChildAsESPNowPeer(childDevices[i].macAddress);
    }
    
    Serial.printf("HubClient: ESP-NOW peer restoration complete - %d children\n", childDeviceCount);
}

// Initialize ESP-NOW receiver
bool HubClientService::initializeESPNow() {
    if (espNowInitialized) {
        return true; // Already initialized
    }
    
    // Force WiFi channel to ESP-NOW channel first
    WiFi.setChannel(1);
    delay(50); // Give WiFi time to settle (shorter than re-initialization)
    Serial.println("HubClient: WiFi channel set to 1 for ESP-NOW");
    
    // Configure WiFi for BLE coexistence
    WiFi.setSleep(false); // Disable WiFi sleep to prevent conflicts
    Serial.println("HubClient: WiFi sleep disabled for BLE coexistence");
    
    // Initialize ESP-NOW (same sequence as working re-initialization)
    if (esp_now_init() != ESP_OK) {
        Serial.println("HubClient: Failed to initialize ESP-NOW");
        return false;
    }
    
    // Set ESP-NOW PMK (same as re-initialization)
    if (esp_now_set_pmk((uint8_t*)"pmk1234567890123") != ESP_OK) {
        Serial.println("HubClient: Failed to set ESP-NOW PMK");
        return false;
    }
    
    // Register callback function (same as re-initialization)
    esp_now_register_recv_cb(onESPNowDataRecv);
    
    espNowInitialized = true;
    return true;
}

// Initialize the hub client service
void HubClientService::begin() {
    if (!deviceConfig.isHubMode()) {
        return;
    }
    
    // Initialize ESP-NOW
    if (!initializeESPNow()) {
        return;
    }
    
    // Reset child devices
    childDeviceCount = 0;
    for (int i = 0; i < MAX_CHILDREN; i++) {
        memset(childDevices[i].macAddress, 0, sizeof(childDevices[i].macAddress));
        childDevices[i].role = ROLE_UNKNOWN;
        childDevices[i].dataAvailable = false;
    }
    
    // Initialize aggregated data
    memset(&aggregatedData, 0, sizeof(aggregatedData));
    aggregatedData.handConnected = false;
    aggregatedData.forearmConnected = false;
    
    // Reset disconnection tracking
    handMissedPolls = 0;
    forearmMissedPolls = 0;
    

}

// Main update function
void HubClientService::update(bool bleConnected) {
    if (!deviceConfig.isHubMode() || !espNowInitialized) {
        return; // Not a hub or ESP-NOW not initialized
    }
    
    // Child timeout handling removed - just burning CPU cycles and giving useless data
    
    // Periodic logging (every 30 seconds to match child devices)
    unsigned long currentTime = millis();
    if (currentTime - lastLogTime >= 30000) {
        // Count connected children
        int connectedCount = 0;
        for (int i = 0; i < childDeviceCount; i++) {
            if (childDevices[i].dataAvailable) {
                connectedCount++;
            }
        }
        
        // Calculate ESP-NOW receive rate
        float espNowRate = (float)packetCounter / 30.0; // packets per second over 30 seconds
        
        Serial.printf("HUB: ESP-NOW received: %.1f Hz, Children: %d/%d, BLE: %s\n", 
                     espNowRate, connectedCount, 2, bleConnected ? "Connected" : "Disconnected");

        // Raw Data Debug (same frequency as HUB log)
        // Print HUB's own raw data (need access to IMU, but it's global in main.cpp)
        // Since we can't easily access the global IMU here without circular deps or externs, 
        // we'll rely on the fact that this service is for managing children.
        // However, we CAN print the raw data of the connected children if available.
        
        for (int i = 0; i < childDeviceCount; i++) {
            if (childDevices[i].dataAvailable) {
                RawMotionData& raw = childDevices[i].lastRawData;
                Serial.printf("RAW DEBUG (Child %d %s): Accel(%.2f, %.2f, %.2f) Gyro(%.2f, %.2f, %.2f)\n", 
                             i, childDevices[i].role == ROLE_LEFT_HAND ? "L_HAND" : 
                                (childDevices[i].role == ROLE_RIGHT_HAND ? "R_HAND" : 
                                (childDevices[i].role == ROLE_LEFT_FOREARM ? "L_FORE" : 
                                (childDevices[i].role == ROLE_RIGHT_FOREARM ? "R_FORE" : "UNK"))),
                             raw.accel_x, raw.accel_y, raw.accel_z,
                             raw.gyro_x, raw.gyro_y, raw.gyro_z);
            }
        }
                
        // Reset counters
        packetCounter = 0;
        lastLogTime = currentTime;
    }
}

// Process incoming ESP-NOW packet (simplified - only handles quaternions)
void HubClientService::processESPNowPacket(const uint8_t* macAddr, const uint8_t* data, int dataLen) {
    // Check minimum packet size (header size)
    if (dataLen < sizeof(ESPNowPacketHeader)) {
        return;
    }
    
    // Extract header to determine packet type
    ESPNowPacketHeader* header = (ESPNowPacketHeader*)data;
    
    // Only handle quaternion and raw data packets
    if (header->messageType == MESSAGE_TYPE_QUAT) {
        // Increment packet counter for periodic logging
        packetCounter++;
        processQuaternionPacket(macAddr, data, dataLen);
    } else if (header->messageType == MESSAGE_TYPE_RAW) {
        // Increment packet counter
        packetCounter++;
        // Process raw data packet (inline implementation for now as it's simple)
        if (dataLen != sizeof(ESPNowRawPacket)) {
            return;
        }
        
        ESPNowRawPacket* packet = (ESPNowRawPacket*)data;
        int slotIndex = findChildByMac(macAddr);
        
        if (slotIndex != -1) {
            ESPNowChildDevice& child = childDevices[slotIndex];
            child.lastRawData = packet->data;
            // Note: We don't update dataAvailable flag here as it's driven by quaternion updates
            // which are the primary heartbeat
        }
    }
    // Silently ignore other packets to reduce log spam
}

// Process quaternion packet (simplified - uses MAC address instead of role)
void HubClientService::processQuaternionPacket(const uint8_t* macAddr, const uint8_t* data, int dataLen) {
    if (dataLen != sizeof(ESPNowQuaternionPacket)) {
        Serial.printf("HubClient: Invalid quaternion packet size: %d (expected %d)\n", 
                     dataLen, sizeof(ESPNowQuaternionPacket));
        return;
    }
    
    ESPNowQuaternionPacket* packet = (ESPNowQuaternionPacket*)data;
    
    // Find or create child device slot by MAC address
    int slotIndex = findChildByMac(macAddr);
    bool isNewChild = false;
    
    if (slotIndex == -1) {
        slotIndex = createChildSlot();
        isNewChild = true;
    }
    
    if (slotIndex == -1) {
        Serial.println("HubClient: No available slots for child device");
        return;
    }
    
    // Update child device data
    ESPNowChildDevice& child = childDevices[slotIndex];
    
    memcpy(child.macAddress, macAddr, sizeof(child.macAddress));
    child.role = (DeviceRole)packet->header.senderRole;  // Get role from packet
    child.lastData = packet->quaternion;
    child.dataAvailable = true;
    
    // If this is a new child, register it as an ESP-NOW peer so we can send commands to it
    if (isNewChild) {
        if (registerChildAsESPNowPeer(macAddr)) {
            Serial.printf("HubClient: New child registered and ready for bidirectional communication\n");
        } else {
            Serial.printf("HubClient: Warning: New child registered but ESP-NOW peer setup failed\n");
        }
    }
    
    // Reset the appropriate missed polls counter based on role
    if (child.role == ROLE_LEFT_HAND || child.role == ROLE_RIGHT_HAND) {
        handMissedPolls = 0;
    } else if (child.role == ROLE_LEFT_FOREARM || child.role == ROLE_RIGHT_FOREARM) {
        forearmMissedPolls = 0;
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

// Find existing child slot by MAC address
int HubClientService::findChildByMac(const uint8_t* macAddress) {
    for (int i = 0; i < childDeviceCount; i++) {
        if (memcmp(childDevices[i].macAddress, macAddress, 6) == 0) {
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

// Check for child disconnections (called every second)
void HubClientService::checkChildDisconnections() {
    if (!deviceConfig.isHubMode()) {
        return;
    }
    
    // Increment hand missed polls
    handMissedPolls++;
    if (handMissedPolls >= CHILD_DISCONNECT_TIMEOUT_SECONDS) {
        // Mark all hand children as disconnected
        for (int i = 0; i < childDeviceCount; i++) {
            ESPNowChildDevice& child = childDevices[i];
            if (child.dataAvailable && (child.role == ROLE_LEFT_HAND || child.role == ROLE_RIGHT_HAND)) {
                child.dataAvailable = false;
            }
        }
    }
    
    // Increment forearm missed polls
    forearmMissedPolls++;
    if (forearmMissedPolls >= CHILD_DISCONNECT_TIMEOUT_SECONDS) {
        // Mark all forearm children as disconnected
        for (int i = 0; i < childDeviceCount; i++) {
            ESPNowChildDevice& child = childDevices[i];
            if (child.dataAvailable && (child.role == ROLE_LEFT_FOREARM || child.role == ROLE_RIGHT_FOREARM)) {
                child.dataAvailable = false;
            }
        }
    }
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
void updateHubClientService(bool bleConnected) {
    hubClientService.update(bleConnected);
}

// Check for child disconnections (global wrapper)
void checkChildDisconnections() {
    hubClientService.checkChildDisconnections();
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

void sendCalibrationCommand() {
    hubClientService.sendCalibrationCommand();
}

bool registerChildAsESPNowPeer(const uint8_t* macAddress) {
    return hubClientService.registerChildAsESPNowPeer(macAddress);
}

void restoreESPNowPeers() {
    hubClientService.restoreESPNowPeers();
}

bool isChildConnected(DeviceRole childRole) {
    return hubClientService.isChildConnected(childRole);
}

AggregatedQuaternionData* getAggregatedData() {
    return hubClientService.getAggregatedData();
}

ESPNowChildDevice* getChildDevices() {
    return hubClientService.getChildDevices();
} 