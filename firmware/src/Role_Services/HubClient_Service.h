#ifndef HUB_CLIENT_SERVICE_H
#define HUB_CLIENT_SERVICE_H

#include <NimBLEDevice.h>
#include <NimBLEClient.h>
#include "DeviceConfig.h"
#include "Hub_Structures.h"

// Hub client configuration
#define MAX_CHILDREN 2
#define CHILD_CONNECTION_TIMEOUT_MS 5000

// Service and characteristic UUIDs
#define EIDON_SERVICE_UUID        "E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define QUATERNION_CHAR_UUID      "E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE"

// New characteristics for hub devices only (child data)
#define HAND_QUATERNION_CHAR_UUID     "E1D00008-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define FOREARM_QUATERNION_CHAR_UUID  "E1D00009-8B5A-3E5B-9E23-4F9B5C91BBDE"

// Hub client service class
class HubClientService {
private:
    ChildConnection childConnections[MAX_CHILDREN];
    int childConnectionCount;
    AggregatedQuaternionData aggregatedData;
    
    // Helper functions
    int findChildSlot(DeviceRole childRole);
    int createChildSlot();
    void syncConnectionStatus();
    
public:
    HubClientService();
    ~HubClientService();
    
    // Main interface
    void begin();
    void update();
    
    // Connection management
    void connectToChild(const NimBLEAddress& address, DeviceRole childRole);
    void disconnectFromChild(DeviceRole childRole);
    bool isChildConnected(DeviceRole childRole);
    
    // Data management
    void updateChildData();
    void updateHubQuaternionData(float w, float x, float y, float z);
    AggregatedQuaternionData* getAggregatedData() { return &aggregatedData; }
    
    // Access to child connections for external use
    ChildConnection* getChildConnections() { return childConnections; }
    int getChildConnectionCount() const { return childConnectionCount; }
};

// Global instance
extern HubClientService hubClientService;

// Function declarations for integration with main.cpp
void setupHubClientService();
void updateHubClientService();
void connectToChild(const NimBLEAddress& address, DeviceRole childRole);
void disconnectFromChild(DeviceRole childRole);
void updateChildData();
void updateHubQuaternionData(float w, float x, float y, float z);
bool isChildConnected(DeviceRole childRole);
AggregatedQuaternionData* getAggregatedData();

#endif // HUB_CLIENT_SERVICE_H 