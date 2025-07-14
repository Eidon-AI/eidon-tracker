#include "HubClient_Service.h"
#include "BLE_Services/BLE_Callbacks.h"
#include <Arduino.h>

// External dependencies
extern DeviceConfig deviceConfig;

// Static callback instance for hub client
static HubClientCallbacks hubClientCallbacksInstance;

// Global instance
HubClientService hubClientService;

// Constructor
HubClientService::HubClientService() 
    : childConnectionCount(0) {
    // Initialize child connections
    for (int i = 0; i < MAX_CHILDREN; i++) {
        childConnections[i].client = nullptr;
        childConnections[i].connected = false;
        childConnections[i].dataAvailable = false;
        childConnections[i].connectionAttempts = 0;
        childConnections[i].lastConnectionAttempt = 0;
    }
    
    // Initialize aggregated data
    memset(&aggregatedData, 0, sizeof(aggregatedData));
    aggregatedData.handConnected = false;
    aggregatedData.forearmConnected = false;
}

// Destructor
HubClientService::~HubClientService() {
    // Disconnect all children
    for (int i = 0; i < childConnectionCount; i++) {
        if (childConnections[i].client != nullptr) {
            childConnections[i].client->disconnect();
        }
    }
}

// Initialize the hub client service
void HubClientService::begin() {
    if (!deviceConfig.isHubMode()) {
        Serial.println("HubClient: Not a hub, client service disabled");
        return;
    }
    
    Serial.println("HubClient: Initializing hub client service...");
    
    // Reset child connections
    childConnectionCount = 0;
    for (int i = 0; i < MAX_CHILDREN; i++) {
        childConnections[i].client = nullptr;
        childConnections[i].connected = false;
        childConnections[i].dataAvailable = false;
        childConnections[i].connectionAttempts = 0;
        childConnections[i].lastConnectionAttempt = 0;
    }
    
    // Initialize aggregated data
    memset(&aggregatedData, 0, sizeof(aggregatedData));
    aggregatedData.handConnected = false;
    aggregatedData.forearmConnected = false;
    
    Serial.println("HubClient: Service initialized successfully");
}

// Main update function
void HubClientService::update() {
    if (!deviceConfig.isHubMode()) {
        return; // Not a hub, no client functionality needed
    }
    
    // Update aggregated data
    updateAggregatedData();
    
    // Log connection status and data flow (every 5 seconds)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 5000) {
        lastDebugTime = millis();
        
        // Count connected children and data availability
        int connectedCount = 0;
        int dataAvailableCount = 0;
        for (int i = 0; i < childConnectionCount; i++) {
            if (childConnections[i].connected) {
                connectedCount++;
                if (childConnections[i].dataAvailable) {
                    dataAvailableCount++;
                }
            }
        }
        
        Serial.printf("HubClient: %d/%d children connected, %d sending data\n", 
                     connectedCount, MAX_CHILDREN, dataAvailableCount);
    }
}

// Find existing child slot by role
int HubClientService::findChildSlot(DeviceRole childRole) {
    for (int i = 0; i < childConnectionCount; i++) {
        if (childConnections[i].role == childRole) {
            return i;
        }
    }
    return -1; // Not found
}

// Create new child slot
int HubClientService::createChildSlot() {
    if (childConnectionCount < MAX_CHILDREN) {
        return childConnectionCount++;
    }
    return -1; // No available slots
}

// Connect to a child device
void HubClientService::connectToChild(const NimBLEAddress& address, DeviceRole childRole) {
    if (!deviceConfig.isHubMode()) {
        return;
    }
    
    Serial.printf("HubClient: Connecting to child %s at %s...\n", 
                 deviceConfig.getRoleName(childRole), address.toString().c_str());
    
    // Find or create child connection slot
    int slotIndex = findChildSlot(childRole);
    if (slotIndex == -1) {
        slotIndex = createChildSlot();
    }
    
    if (slotIndex == -1) {
        Serial.println("HubClient: No available slots for child connection");
        return;
    }
    
    ChildConnection& child = childConnections[slotIndex];
    
    // Create client if needed
    if (child.client == nullptr) {
        child.client = NimBLEDevice::createClient();
        if (child.client == nullptr) {
            Serial.println("HubClient: Failed to create client");
            return;
        }
        // Set callbacks for connect/disconnect events
        child.client->setClientCallbacks(&hubClientCallbacksInstance);
    }
    
    // Connect to child
    if (child.client->connect(address)) {
        Serial.printf("HubClient: Connected to child %s\n", deviceConfig.getRoleName(childRole));
        child.connected = true;
        child.address = address;
        child.role = childRole;
        child.connectionAttempts = 0;
        
        // Discover services and characteristics
        if (child.client->discoverAttributes()) {
            // Find quaternion characteristic
            NimBLERemoteService* service = child.client->getService(EIDON_SERVICE_UUID);
            if (service != nullptr) {
                NimBLERemoteCharacteristic* quatChar = service->getCharacteristic(QUATERNION_CHAR_UUID);
                
                if (quatChar != nullptr) {
                    // Subscribe to notifications
                    if (quatChar->subscribe(true, [childRole](NimBLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
                        // Handle child quaternion data
                        if (length == sizeof(QuaternionData)) {
                            QuaternionData* quatData = (QuaternionData*)data;
                            
                            // Update child data
                            for (int i = 0; i < hubClientService.getChildConnectionCount(); i++) {
                                ChildConnection* connections = hubClientService.getChildConnections();
                                if (connections[i].role == childRole) {
                                    connections[i].lastData = *quatData;
                                    connections[i].dataAvailable = true;
                                    connections[i].lastDataTime = millis();
                                    break;
                                }
                            }
                        }
                    })) {
                        Serial.printf("HubClient: Subscribed to quaternion data from %s\n", deviceConfig.getRoleName(childRole));
                    } else {
                        Serial.printf("HubClient: Failed to subscribe to quaternion data from %s\n", deviceConfig.getRoleName(childRole));
                    }
                } else {
                    Serial.printf("HubClient: Quaternion characteristic not found on %s\n", deviceConfig.getRoleName(childRole));
                }
            } else {
                Serial.printf("HubClient: Eidon service not found on %s\n", deviceConfig.getRoleName(childRole));
            }
        } else {
            Serial.printf("HubClient: Failed to discover services on %s\n", deviceConfig.getRoleName(childRole));
        }
    } else {
        Serial.printf("HubClient: Failed to connect to child %s\n", deviceConfig.getRoleName(childRole));
        child.connected = false;
    }
}

// Disconnect from a child device
void HubClientService::disconnectFromChild(DeviceRole childRole) {
    for (int i = 0; i < childConnectionCount; i++) {
        if (childConnections[i].role == childRole) {
            ChildConnection& child = childConnections[i];
            if (child.client != nullptr && child.connected) {
                child.client->disconnect();
                child.connected = false;
                child.dataAvailable = false;
                Serial.printf("HubClient: Disconnected from child %s\n", deviceConfig.getRoleName(childRole));
            }
            break;
        }
    }
}

// Check if a child is connected
bool HubClientService::isChildConnected(DeviceRole childRole) {
    for (int i = 0; i < childConnectionCount; i++) {
        if (childConnections[i].role == childRole && childConnections[i].connected) {
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
    
    updateAggregatedData();
}

// Update aggregated data with current child information
void HubClientService::updateAggregatedData() {
    aggregatedData.timestamp = millis();
    
    // Reset connection flags
    aggregatedData.handConnected = false;
    aggregatedData.forearmConnected = false;
    
    // Update child data
    for (int i = 0; i < childConnectionCount; i++) {
        ChildConnection& child = childConnections[i];
        
        if (child.connected && child.dataAvailable) {
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
void connectToChild(const NimBLEAddress& address, DeviceRole childRole) {
    hubClientService.connectToChild(address, childRole);
}

void disconnectFromChild(DeviceRole childRole) {
    hubClientService.disconnectFromChild(childRole);
}

void updateChildData() {
    hubClientService.updateChildData();
}

bool isChildConnected(DeviceRole childRole) {
    return hubClientService.isChildConnected(childRole);
}

AggregatedQuaternionData* getAggregatedData() {
    return hubClientService.getAggregatedData();
} 