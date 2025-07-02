#include "HubScanning_Service.h"
#include <Arduino.h>

// External dependencies
extern DeviceConfig deviceConfig;
extern void connectToChild(const NimBLEAddress& address, DeviceRole childRole);

// Global instance
HubScanningService hubScanningService;

// Constructor
HubScanningService::HubScanningService() 
    : pScan(nullptr)
    , scanningEnabled(false)
    , lastScanTime(0)
    , lastConnectionAttempt(0)
    , connectionAttempts(0) {
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
    return (millis() - lastScanTime > SCAN_INTERVAL_MS);
}

// Check if we have all expected children connected
bool HubScanningService::hasAllChildren() {
    // This will be updated to use the hub client service
    // For now, return false to allow scanning
    return false;
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
    
    Serial.println("HubScanning: Starting scan for child devices...");
    
    if (pScan->start(SCAN_DURATION_MS, false)) {
        scanningEnabled = true;
        lastScanTime = millis();
        Serial.println("HubScanning: Scan started successfully");
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
    
    Serial.printf("HubScanning: Found %d devices\n", resultCount);
    
    // Process each scan result
    for (int i = 0; i < resultCount && i < MAX_SCAN_RESULTS; i++) {
        const NimBLEAdvertisedDevice* device = results.getDevice(i);
        
        if (isChildDevice(device)) {
            DeviceRole childRole = getChildRoleFromDevice(device);
            
            Serial.printf("HubScanning: Found child device - Name: %s, Role: %s\n", 
                         device->getName().c_str(), 
                         deviceConfig.getRoleName(childRole));
            
            // Check if we already have this child connected
            // This will be updated to use the hub client service
            bool alreadyConnected = false;
            
            if (!alreadyConnected && canAttemptConnection()) {
                Serial.printf("HubScanning: Attempting connection to %s\n", device->getName().c_str());
                connectToChild(device->getAddress(), childRole);
                lastConnectionAttempt = millis();
                connectionAttempts++;
            }
        }
    }
    
    // Clear scan results
    pScan->clearResults();
}

// Check if a device is a potential child device
bool HubScanningService::isChildDevice(const NimBLEAdvertisedDevice* device) {
    if (!device) {
        return false;
    }
    
    // Check if device name contains "Eidon" (case-insensitive)
    String deviceName = String(device->getName().c_str());
    if (deviceName.length() == 0) {
        return false; // No name, skip
    }
    
    deviceName.toLowerCase();
    if (deviceName.indexOf(CHILD_NAME_PREFIX) == -1) {
        return false; // Doesn't contain "Eidon"
    }
    
    // Check if device has manufacturer data (for role information)
    if (!device->haveManufacturerData()) {
        return false; // No manufacturer data, skip
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