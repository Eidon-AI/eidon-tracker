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
    
    // Note: updateAggregatedData() is called from updateChildData() in main loop
    // to avoid redundant calls and ensure proper timing
    
    // Log connection status and data flow (every 10 seconds)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 10000) {
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
    
    // Add delay before connection attempt to avoid timing issues
    delay(100);
    
    // Connect to child
    Serial.printf("HubClient: Attempting BLE connection to %s...\n", address.toString().c_str());
    
    // Add some connection parameters for debugging
    child.client->setConnectionParams(12, 24, 0, 400); // min interval, max interval, latency, timeout
    
    bool connectResult = child.client->connect(address);
    Serial.printf("HubClient: BLE connect() returned: %s\n", connectResult ? "SUCCESS" : "FAILED");
    
    if (connectResult) {
        Serial.printf("HubClient: BLE connection successful to child %s\n", deviceConfig.getRoleName(childRole));
        child.connected = true;
        child.address = address;
        child.role = childRole;
        child.connectionAttempts = 0;
        
        // Discover services and characteristics
        Serial.printf("HubClient: Discovering services on child %s...\n", deviceConfig.getRoleName(childRole));
        if (child.client->discoverAttributes()) {
            Serial.printf("HubClient: Service discovery successful for child %s\n", deviceConfig.getRoleName(childRole));
            // Find quaternion characteristic
            NimBLERemoteService* service = child.client->getService(EIDON_SERVICE_UUID);
            if (service != nullptr) {
                Serial.printf("HubClient: Found Eidon service on child %s\n", deviceConfig.getRoleName(childRole));
                NimBLERemoteCharacteristic* quatChar = service->getCharacteristic(QUATERNION_CHAR_UUID);
                
                if (quatChar != nullptr) {
                    Serial.printf("HubClient: Found quaternion characteristic on child %s\n", deviceConfig.getRoleName(childRole));
                    
                    // Check if characteristic supports notifications
                    if (quatChar->canNotify()) {
                        Serial.printf("HubClient: Characteristic supports notifications for %s\n", deviceConfig.getRoleName(childRole));
                    } else {
                        Serial.printf("HubClient: WARNING - Characteristic does NOT support notifications for %s\n", deviceConfig.getRoleName(childRole));
                    }
                    
                    // Subscribe to notifications
                    if (quatChar->subscribe(true, [childRole](NimBLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
                        // Handle child quaternion data
                        if (length == sizeof(QuaternionData)) {
                            QuaternionData* quatData = (QuaternionData*)data;
                            
                            // Log received child data (every 5 seconds to avoid spam)
                            static unsigned long lastChildDataLog = 0;
                            if (millis() - lastChildDataLog >= 5000) {
                                Serial.printf("HubClient: Received from %s - W=%.4f X=%.4f Y=%.4f Z=%.4f\n", 
                                             deviceConfig.getRoleName(childRole),
                                             quatData->w, quatData->x, quatData->y, quatData->z);
                                lastChildDataLog = millis();
                            }
                            
                            // Update child data immediately
                            for (int i = 0; i < hubClientService.getChildConnectionCount(); i++) {
                                ChildConnection* connections = hubClientService.getChildConnections();
                                if (connections[i].role == childRole) {
                                    // First update the child connection data
                                    connections[i].lastData = *quatData;
                                    connections[i].dataAvailable = true;
                                    connections[i].lastDataTime = millis();
                                    
                                    // Then update aggregated data structure
                                    AggregatedQuaternionData* agg = hubClientService.getAggregatedData();
                                    if (childRole == ROLE_LEFT_HAND || childRole == ROLE_RIGHT_HAND) {
                                        agg->handData = *quatData;
                                        agg->handConnected = true;
                                    } else if (childRole == ROLE_LEFT_FOREARM || childRole == ROLE_RIGHT_FOREARM) {
                                        agg->forearmData = *quatData;
                                        agg->forearmConnected = true;
                                    }
                                    agg->timestamp = millis();
                                    
                                    break;
                                }
                            }
                        }
                    })) {
                        Serial.printf("HubClient: Subscribed to quaternion data from %s\n", deviceConfig.getRoleName(childRole));
                    } else {
                        Serial.printf("HubClient: FAILED to subscribe to quaternion data from %s\n", deviceConfig.getRoleName(childRole));
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
        Serial.printf("HubClient: BLE connection failed to child %s\n", deviceConfig.getRoleName(childRole));
        child.connected = false;
        
        // Log additional debug info
        Serial.printf("HubClient: Current BLE connection count: %d\n", NimBLEDevice::getServer()->getConnectedCount());
        Serial.printf("HubClient: BLE stack status check...\n");
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
    
    // Update hub's own quaternion data (called from main.cpp with current IMU data)
    // This is handled by updateHubQuaternionData() function
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

void updateHubQuaternionData(float w, float x, float y, float z) {
    hubClientService.updateHubQuaternionData(w, x, y, z);
}

bool isChildConnected(DeviceRole childRole) {
    return hubClientService.isChildConnected(childRole);
}

AggregatedQuaternionData* getAggregatedData() {
    return hubClientService.getAggregatedData();
} 