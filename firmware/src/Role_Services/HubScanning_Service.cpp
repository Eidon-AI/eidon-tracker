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
    
    Serial.printf("HubScanning: Initializing scanning service for hub role: %s\n", 
                 deviceConfig.getRoleName(deviceConfig.getRole()));
    Serial.println("HubScanning: Initializing scanning service...");
    
    // Create scan instance
    pScan = NimBLEDevice::getScan();
    if (!pScan) {
        Serial.println("HubScanning: ERROR - Failed to create scan instance");
        return;
    }
    
    // Configure scanning parameters
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
                
                // Log BLE stack status before connection attempt
                Serial.printf("HubScanning: BLE stack status - Server connections: %d\n", 
                             NimBLEDevice::getServer()->getConnectedCount());
                
                connectToChild(device->getAddress(), childRole);
                lastConnectionAttempt = millis();
                connectionAttempts++;
            } else if (alreadyConnected) {
                Serial.printf("HubScanning: Child %s already connected\n", deviceConfig.getRoleName(childRole));
            } else if (!canAttemptConnection()) {
                Serial.printf("HubScanning: Cannot attempt connection to %s (rate limited or max attempts)\n", 
                             device->getName().c_str());
            }
        } else {
            // Debug: Log why device is not considered a child
            if (device->haveManufacturerData()) {
                std::string manufacturerData = device->getManufacturerData();
                if (manufacturerData.length() >= 3) {
                    uint8_t companyIdLow = manufacturerData[0];
                    uint8_t companyIdHigh = manufacturerData[1];
                    uint16_t companyId = (companyIdHigh << 8) | companyIdLow;
                    uint8_t roleByte = manufacturerData[2];
                    
                    if (companyId == 0xE1D0) {
                        DeviceRole role = (DeviceRole)roleByte;
                        DeviceRole hubRole = deviceConfig.getRole();
                        bool isPairable = isPairableChildDevice(role, hubRole);
                        
                        Serial.printf("HubScanning: Found Eidon device %s (Role: %s) - Pairable: %s\n", 
                                     device->getName().c_str(), deviceConfig.getRoleName(role), 
                                     isPairable ? "YES" : "NO");
                    }
                }
            }
        }
    }
    
    // Log scan results every scan
    Serial.printf("HubScanning: Found %d total devices, %d Eidon devices, %d pairable children\n", 
                 resultCount, eidonDevicesFound, pairableChildrenFound);
    
    // Log all devices found (every scan)
    Serial.println("HubScanning: Devices found:");
    for (int i = 0; i < resultCount && i < MAX_SCAN_RESULTS; i++) {
        const NimBLEAdvertisedDevice* device = results.getDevice(i);
        std::string deviceNameStr = device->getName();
        String deviceName = String(deviceNameStr.c_str());
        if (deviceName.length() == 0) {
            deviceName = "Unknown";
        }
        Serial.printf("  %d: %s (RSSI: %d)", i, deviceName.c_str(), device->getRSSI());
        
        if (device->haveManufacturerData()) {
            std::string manufacturerData = device->getManufacturerData();
            Serial.printf(" - Manufacturer data: ");
            for (size_t j = 0; j < manufacturerData.length() && j < 5; j++) {
                Serial.printf("%02X ", (uint8_t)manufacturerData[j]);
            }
            
            // Check if this could be an Eidon device
            if (manufacturerData.length() >= 3) {
                uint8_t companyIdLow = manufacturerData[0];
                uint8_t companyIdHigh = manufacturerData[1];
                uint16_t companyId = (companyIdHigh << 8) | companyIdLow;
                Serial.printf(" -> Company ID: 0x%04X", companyId);
                
                if (companyId == 0xE1D0) {
                    uint8_t roleByte = manufacturerData[2];
                    Serial.printf(" (EIDON - Role: %d)", roleByte);
                }
            }
        } else {
            Serial.print(" - No manufacturer data");
        }
        Serial.println();
    }
    
    // Clear scan results
    pScan->clearResults();
}

// Check if a child role is pairable with a given hub role
bool HubScanningService::isPairableChildDevice(DeviceRole childRole, DeviceRole hubRole) {
    // LEFT_HUB can only pair with LEFT_HAND and LEFT_FOREARM
    if (hubRole == ROLE_LEFT_HUB) {
        bool pairable = (childRole == ROLE_LEFT_HAND || childRole == ROLE_LEFT_FOREARM);
        if (pairable) {
            Serial.printf("HubScanning: LEFT_HUB can pair with %s\n", deviceConfig.getRoleName(childRole));
        }
        return pairable;
    }
    
    // RIGHT_HUB can only pair with RIGHT_HAND and RIGHT_FOREARM
    if (hubRole == ROLE_RIGHT_HUB) {
        bool pairable = (childRole == ROLE_RIGHT_HAND || childRole == ROLE_RIGHT_FOREARM);
        if (pairable) {
            Serial.printf("HubScanning: RIGHT_HUB can pair with %s\n", deviceConfig.getRoleName(childRole));
        }
        return pairable;
    }
    
    Serial.printf("HubScanning: Hub role %s is not a valid hub role\n", deviceConfig.getRoleName(hubRole));
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
        Serial.printf("HubScanning: Max connection attempts reached (%d)\n", connectionAttempts);
        return false;
    }
    
    // Rate limit connection attempts
    bool canConnect = (millis() - lastConnectionAttempt > CHILD_CONNECTION_TIMEOUT_MS);
    if (!canConnect) {
        Serial.printf("HubScanning: Rate limited - %lu ms since last attempt\n", 
                     millis() - lastConnectionAttempt);
    }
    
    return canConnect;
}

// Reset connection attempt counter
void HubScanningService::resetConnectionAttempts() {
    connectionAttempts = 0;
    lastConnectionAttempt = 0;
    Serial.println("HubScanning: Connection attempts reset");
    
    // Also reset any existing child connections to force fresh connection attempts
    for (int i = 0; i < hubClientService.getChildConnectionCount(); i++) {
        ChildConnection* connections = hubClientService.getChildConnections();
        if (connections[i].connected) {
            Serial.printf("HubScanning: Resetting connection to child %s\n", 
                         deviceConfig.getRoleName(connections[i].role));
            connections[i].connected = false;
            connections[i].dataAvailable = false;
            if (connections[i].client != nullptr) {
                connections[i].client->disconnect();
            }
        }
    }
}

// Setup function for integration with main.cpp
void setupHubScanningService() {
    hubScanningService.begin();
}

// Update function for integration with main.cpp
void updateHubScanning() {
    hubScanningService.update();
}

// Reset function for integration with main.cpp
void resetHubScanningConnections() {
    hubScanningService.resetConnectionAttempts();
} 