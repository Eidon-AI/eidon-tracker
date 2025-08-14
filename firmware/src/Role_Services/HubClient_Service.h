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

// ESP-NOW receiver service class
class HubClientService {
private:
    ESPNowChildDevice childDevices[MAX_CHILDREN];
    int childDeviceCount;
    AggregatedQuaternionData aggregatedData;
    bool espNowInitialized;
    
    // Simple disconnection tracking - one counter per type
    unsigned int handMissedPolls;
    unsigned int forearmMissedPolls;
    
    // Periodic logging variables
    unsigned long lastLogTime;
    
public:
    // Periodic logging access
    int packetCounter;
    
    // Helper functions
    int findChildSlot(DeviceRole childRole);
    int findChildByMac(const uint8_t* macAddress);
    int createChildSlot();
    void syncConnectionStatus();
    
    // ESP-NOW peer management
    bool registerChildAsESPNowPeer(const uint8_t* macAddress);
    
public:
    HubClientService();
    ~HubClientService();
    
    // Main interface
    void begin();
    void update(bool bleConnected = false);
    
    // ESP-NOW management
    bool initializeESPNow();
    void registerChildDevice(const uint8_t* macAddress, DeviceRole childRole);
    void unregisterChildDevice(DeviceRole childRole);
    bool isChildConnected(DeviceRole childRole);
    void processESPNowPacket(const uint8_t* macAddr, const uint8_t* data, int dataLen);
    void processQuaternionPacket(const uint8_t* macAddr, const uint8_t* data, int dataLen);
    void restoreESPNowPeers();  // New recovery function
    
    // Command sending
    void sendCalibrationCommand();
    
    // Data management
    void updateChildData();
    void updateHubQuaternionData(float w, float x, float y, float z);
    void checkChildDisconnections();
    AggregatedQuaternionData* getAggregatedData() { return &aggregatedData; }
    
    // Access to child devices for external use
    ESPNowChildDevice* getChildDevices() { return childDevices; }
    int getChildDeviceCount() const { return childDeviceCount; }
};

// Global instance
extern HubClientService hubClientService;

// Function declarations for integration with main.cpp
void setupHubClientService();
void updateHubClientService(bool bleConnected = false);
void checkChildDisconnections();
void registerChildDevice(const uint8_t* macAddress, DeviceRole childRole);
void unregisterChildDevice(DeviceRole childRole);
void updateChildData();
void updateHubQuaternionData(float w, float x, float y, float z);
bool isChildConnected(DeviceRole childRole);
AggregatedQuaternionData* getAggregatedData();
void sendCalibrationCommand();
bool registerChildAsESPNowPeer(const uint8_t* macAddress);
void restoreESPNowPeers();  // Global recovery function

// ESP-NOW callback function declaration (defined in main.cpp)
void onESPNowDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int dataLen);

#endif // HUB_CLIENT_SERVICE_H 