#ifndef HUB_SCANNING_SERVICE_H
#define HUB_SCANNING_SERVICE_H

#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEClient.h>
#include "DeviceConfig.h"
#include "Hub_Structures.h"

// Scanning configuration
#define SCAN_INTERVAL_MS 1000        // How often to scan (1 second)
#define SCAN_DURATION_MS 500         // How long each scan lasts (500ms)
#define MAX_SCAN_RESULTS 10          // Maximum number of scan results to process
#define CHILD_CONNECTION_TIMEOUT_MS 5000  // Timeout for connection attempts

// Child device identification
#define CHILD_NAME_PREFIX "Eidon"    // Child devices must have "Eidon" in their name
#define EXPECTED_CHILDREN 2          // Hub expects 2 children (hand + forearm)

// Scanning service class
class HubScanningService {
private:
    NimBLEScan* pScan;
    bool scanningEnabled;
    unsigned long lastScanTime;
    unsigned long lastConnectionAttempt;
    int connectionAttempts;
    
    // Helper functions
    bool shouldScan();
    bool hasAllChildren();
    bool isChildDevice(const NimBLEAdvertisedDevice* device);
    bool isPairableChildDevice(DeviceRole childRole, DeviceRole hubRole);
    DeviceRole getChildRoleFromDevice(const NimBLEAdvertisedDevice* device);
    void processScanResults();
    void attemptChildConnections();
    
public:
    HubScanningService();
    ~HubScanningService();
    
    // Main interface
    void begin();
    void update();
    void startScanning();
    void stopScanning();
    bool isScanning() const { return scanningEnabled; }
    
    // Connection management
    void resetConnectionAttempts();
    bool canAttemptConnection();
};

// Global instance
extern HubScanningService hubScanningService;

// Function declarations for integration with main.cpp
void setupHubScanningService();
void updateHubScanning();

#endif // HUB_SCANNING_SERVICE_H 