#ifndef HUB_CLIENT_SERVICE_H
#define HUB_CLIENT_SERVICE_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "DeviceConfig.h"
#include "Hub_Structures.h"

// Hub ESP-NOW receiver configuration
#define MAX_CHILDREN 2
#define ESP_NOW_CHANNEL 1
#define CHILD_DATA_TIMEOUT_MS 30000  // Consider child disconnected if no data for 30 seconds

// ESP-NOW receiver service class
class HubClientService {
private:
    ESPNowChildDevice childDevices[MAX_CHILDREN];
    int childDeviceCount;
    AggregatedQuaternionData aggregatedData;
    bool espNowInitialized;
    
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
    
    // ESP-NOW management
    bool initializeESPNow();
    void registerChildDevice(const uint8_t* macAddress, DeviceRole childRole);
    void unregisterChildDevice(DeviceRole childRole);
    bool isChildConnected(DeviceRole childRole);
    void processESPNowPacket(const uint8_t* macAddr, const uint8_t* data, int dataLen);
    
    // Data management
    void updateChildData();
    void updateHubQuaternionData(float w, float x, float y, float z);
    AggregatedQuaternionData* getAggregatedData() { return &aggregatedData; }
    
    // Access to child devices for external use
    ESPNowChildDevice* getChildDevices() { return childDevices; }
    int getChildDeviceCount() const { return childDeviceCount; }
};

// Global instance
extern HubClientService hubClientService;

// Function declarations for integration with main.cpp
void setupHubClientService();
void updateHubClientService();
void registerChildDevice(const uint8_t* macAddress, DeviceRole childRole);
void unregisterChildDevice(DeviceRole childRole);
void updateChildData();
void updateHubQuaternionData(float w, float x, float y, float z);
bool isChildConnected(DeviceRole childRole);
AggregatedQuaternionData* getAggregatedData();

// ESP-NOW callback function declaration
void onESPNowDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int dataLen);

#endif // HUB_CLIENT_SERVICE_H 