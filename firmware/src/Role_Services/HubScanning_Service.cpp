#include "HubScanning_Service.h"
#include "HubClient_Service.h"
#include <Arduino.h>

// External dependencies
extern DeviceConfig deviceConfig;
extern void connectToChild(const NimBLEAddress& address, DeviceRole childRole);
extern HubClientService hubClientService;

// Global instance
HubScanningService hubScanningService;

// Constructor
HubScanningService::HubScanningService() 
    : pScan(nullptr)
    , scanningEnabled(false)
    , lastScanTime(0)
    , lastConnectionAttempt(0)
    , connectionAttempts(0) {
    // Initialize timing to allow immediate scanning
    lastScanTime = 0;
    lastConnectionAttempt = 0;
}

// Destructor
HubScanningService::~HubScanningService() {
    if (pScan) {
        pScan->stop();
        // Don't delete pScan - it's managed by NimBLEDevice
    }
}

// Initialize the scanning service
void HubScanningService::begin() {
    if (!deviceConfig.isHubMode()) {
        Serial.println("HubScanning: Not a hub, scanning disabled");
        return;
    }
    
    Serial.println("HubScanning: Initializing scanning service...");
    
    // Create scan instance
    pScan = NimBLEDevice::getScan();
    if (!pScan) {
        Serial.println("HubScanning: ERROR - Failed to create scan instance");
        return;
    }
    
    // Configure scanning parameters
    pScan->setInterval(SCAN_INTERVAL_MS);
    pScan->setWindow(SCAN_DURATION_MS);
    pScan->setActiveScan(true);
    pScan->setMaxResults(MAX_SCAN_RESULTS);
    
    // Enable scan response to get device names
    pScan->setDuplicateFilter(false);
    
    Serial.println("HubScanning: Service initialized successfully");
}

// Main update function - called from main loop
void HubScanningService::update() {
    if (!deviceConfig.isHubMode()) {
        return; // Not a hub, no scanning needed
    }
    
    // Check if we should scan
    if (shouldScan()) {
        startScanning();
    }
    
    // Process scan results if scanning is complete
    if (scanningEnabled && pScan && pScan->isScanning() == false) {
        processScanResults();
        scanningEnabled = false;
    }
}

// Determine if we should start scanning
bool HubScanningService::shouldScan() {
    // Don't scan if we already have all children
    if (hasAllChildren()) {
        return false;
    }
    
    // Don't scan if we're currently scanning
    if (scanningEnabled) {
        return false;
    }
    
    // Don't scan if we recently attempted connections
    if (millis() - lastConnectionAttempt < CHILD_CONNECTION_TIMEOUT_MS) {
        return false;
    }
    
    // Scan periodically
    bool shouldScanNow = (millis() - lastScanTime > SCAN_INTERVAL_MS);
    
    return shouldScanNow;
}

// Check if we have all expected children connected
bool HubScanningService::hasAllChildren() {
    // Check hub client service for connected children
    bool hasHand = false;
    bool hasForearm = false;
    
    for (int i = 0; i < hubClientService.getChildConnectionCount(); i++) {
        ChildConnection* connections = hubClientService.getChildConnections();
        if (connections[i].connected) {
            if (connections[i].role == ROLE_LEFT_HAND || connections[i].role == ROLE_RIGHT_HAND) {
                hasHand = true;
            } else if (connections[i].role == ROLE_LEFT_FOREARM || connections[i].role == ROLE_RIGHT_FOREARM) {
                hasForearm = true;
            }
        }
    }
    
    // For now, we'll scan if we don't have both hand and forearm
    // This can be made more sophisticated based on the hub's specific role
    bool allConnected = hasHand && hasForearm;
    
    // Only log when status changes
    static bool lastAllConnected = false;
    if (allConnected != lastAllConnected) {
        if (allConnected) {
            Serial.println("HubScanning: All expected children connected, stopping scan");
        } else {
            Serial.printf("HubScanning: Missing children - Hand: %s, Forearm: %s\n", 
                         hasHand ? "YES" : "NO", hasForearm ? "YES" : "NO");
        }
        lastAllConnected = allConnected;
    }
    
    return allConnected;
}

// Start scanning for child devices
void HubScanningService::startScanning() {
    if (!pScan) {
        Serial.println("HubScanning: ERROR - Scan instance not initialized");
        return;
    }
    
    if (scanningEnabled) {
        return; // Already scanning
    }
    
    if (pScan->start(SCAN_DURATION_MS, false)) {
        scanningEnabled = true;
        lastScanTime = millis();
        // Don't log every scan start - too verbose
    } else {
        Serial.println("HubScanning: ERROR - Failed to start scan");
    }
}

// Stop scanning
void HubScanningService::stopScanning() {
    if (pScan && scanningEnabled) {
        pScan->stop();
        scanningEnabled = false;
        Serial.println("HubScanning: Scan stopped");
    }
}

// Process scan results and attempt connections
void HubScanningService::processScanResults() {
    if (!pScan) {
        return;
    }
    
    NimBLEScanResults results = pScan->getResults();
    int resultCount = results.getCount();
    
    // Process each scan result
    int eidonDevicesFound = 0;
    int pairableChildrenFound = 0;
    
    for (int i = 0; i < resultCount && i < MAX_SCAN_RESULTS; i++) {
        const NimBLEAdvertisedDevice* device = results.getDevice(i);
        
        // Count and log Eidon devices (regardless of role)
        if (device->haveManufacturerData()) {
            std::string manufacturerData = device->getManufacturerData();
            if (manufacturerData.length() >= 3) {
                uint8_t companyIdLow = manufacturerData[0];
                uint8_t companyIdHigh = manufacturerData[1];
                uint16_t companyId = (companyIdHigh << 8) | companyIdLow;
                
                if (companyId == 0xE1D0) {
                    eidonDevicesFound++;
                    uint8_t roleByte = manufacturerData[2];
                    DeviceRole role = (DeviceRole)roleByte;
                    
                    // Check if this is a pairable device for our hub
                    DeviceRole hubRole = deviceConfig.getRole();
                    bool isPairable = isPairableChildDevice(role, hubRole);
                    
                    if (isPairable) {
                        pairableChildrenFound++;
                    }
                }
            }
        }
        
        if (isChildDevice(device)) {
            DeviceRole childRole = getChildRoleFromDevice(device);
            
            // Check if we already have this child connected
            bool alreadyConnected = hubClientService.isChildConnected(childRole);
            
            if (!alreadyConnected && canAttemptConnection()) {
                Serial.printf("HubScanning: Attempting connection to %s (Role: %s)\n", 
                             device->getName().c_str(), deviceConfig.getRoleName(childRole));
                connectToChild(device->getAddress(), childRole);
                lastConnectionAttempt = millis();
                connectionAttempts++;
            } else if (alreadyConnected) {
                // Don't log this - too verbose
            } else if (!canAttemptConnection()) {
                // Don't log this - too verbose
            }
        }
    }
    
    // Only log scan summary if we found pairable children or if this is a periodic update
    static unsigned long lastScanSummary = 0;
    if (pairableChildrenFound > 0 || (millis() - lastScanSummary > 10000)) { // Every 10 seconds
        Serial.printf("HubScanning: Found %d total devices, %d Eidon devices, %d pairable children\n", 
                     resultCount, eidonDevicesFound, pairableChildrenFound);
        lastScanSummary = millis();
    }
    
    // Clear scan results
    pScan->clearResults();
}

// Check if a child role is pairable with a given hub role
bool HubScanningService::isPairableChildDevice(DeviceRole childRole, DeviceRole hubRole) {
    // LEFT_HUB can only pair with LEFT_HAND and LEFT_FOREARM
    if (hubRole == ROLE_LEFT_HUB) {
        return (childRole == ROLE_LEFT_HAND || childRole == ROLE_LEFT_FOREARM);
    }
    
    // RIGHT_HUB can only pair with RIGHT_HAND and RIGHT_FOREARM
    if (hubRole == ROLE_RIGHT_HUB) {
        return (childRole == ROLE_RIGHT_HAND || childRole == ROLE_RIGHT_FOREARM);
    }
    
    return false; // Not a hub role
}

// Check if a device is a potential child device
bool HubScanningService::isChildDevice(const NimBLEAdvertisedDevice* device) {
    if (!device) {
        return false;
    }
    
    // Primary identification: Check manufacturer data for Eidon company ID
    if (!device->haveManufacturerData()) {
        return false; // No manufacturer data, skip
    }
    
    std::string manufacturerData = device->getManufacturerData();
    
    // Expected format: [Company ID Low, Company ID High, Role Data]
    if (manufacturerData.length() < 3) {
        return false; // Not enough data
    }
    
    // Check company ID (E1D0)
    uint8_t companyIdLow = manufacturerData[0];
    uint8_t companyIdHigh = manufacturerData[1];
    uint16_t companyId = (companyIdHigh << 8) | companyIdLow;
    
    if (companyId != 0xE1D0) {
        return false; // Wrong company ID
    }
    
    // Extract and validate role
    uint8_t roleByte = manufacturerData[2];
    DeviceRole role = (DeviceRole)roleByte;
    
    // Check if this is a pairable child for our hub
    DeviceRole hubRole = deviceConfig.getRole();
    if (!isPairableChildDevice(role, hubRole)) {
        return false; // Not pairable with this hub
    }
    
    return true;
}

// Extract child role from device manufacturer data
DeviceRole HubScanningService::getChildRoleFromDevice(const NimBLEAdvertisedDevice* device) {
    if (!device || !device->haveManufacturerData()) {
        return ROLE_UNKNOWN;
    }
    
    std::string manufacturerData = device->getManufacturerData();
    
    // Expected format: [Company ID Low, Company ID High, Role Data]
    if (manufacturerData.length() < 3) {
        return ROLE_UNKNOWN;
    }
    
    // Check company ID (E1D0)
    uint8_t companyIdLow = manufacturerData[0];
    uint8_t companyIdHigh = manufacturerData[1];
    uint16_t companyId = (companyIdHigh << 8) | companyIdLow;
    
    if (companyId != 0xE1D0) {
        return ROLE_UNKNOWN; // Wrong company ID
    }
    
    // Extract role from third byte
    uint8_t roleByte = manufacturerData[2];
    DeviceRole role = (DeviceRole)roleByte;
    
    // Validate role
    if (!deviceConfig.isValidRole(role)) {
        return ROLE_UNKNOWN;
    }
    
    // Only accept hand and forearm roles as children
    if (role != ROLE_LEFT_HAND && role != ROLE_RIGHT_HAND && 
        role != ROLE_LEFT_FOREARM && role != ROLE_RIGHT_FOREARM) {
        return ROLE_UNKNOWN;
    }
    
    return role;
}

// Check if we can attempt a new connection
bool HubScanningService::canAttemptConnection() {
    // Limit connection attempts to prevent spam
    if (connectionAttempts >= 5) {
        return false;
    }
    
    // Rate limit connection attempts
    return (millis() - lastConnectionAttempt > CHILD_CONNECTION_TIMEOUT_MS);
}

// Reset connection attempt counter
void HubScanningService::resetConnectionAttempts() {
    connectionAttempts = 0;
    lastConnectionAttempt = 0;
}

// Setup function for integration with main.cpp
void setupHubScanningService() {
    hubScanningService.begin();
}

// Update function for integration with main.cpp
void updateHubScanning() {
    hubScanningService.update();
} 