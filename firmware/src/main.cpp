#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLECharacteristic.h>
#include <WiFi.h>
#include <esp_now.h>
#include "BNO085.h"
#include "BLE_Services/BLE_Callbacks.h"
#include "Role_Services/RoleConfig_Service.h"
#include "Role_Services/HubClient_Service.h"
#include "BLE_Services/BLE_Polling_Service.h" //TODO: Move this from BLE Services
#include "DeviceConfig.h"
#include "Role_Services/Hub_Structures.h"

// Function declarations
void sendQuaternionReport();
void updateLEDStatus();
void startIMUResetPattern();
bool isConnected();
void updateAdvertisingData();

// ESP-NOW sender functions for child devices
bool initializeESPNowSender();
void sendESPNowQuaternionData();
void updateESPNowHubMacAddress();

// Hub client functions are now in Role_Services/HubClient_Service.h

// Polling system functions (implemented in BLE_Polling_Service.cpp)
void setupPollingSystem();
void handleRoleChange(const std::string& value, bool success);

// Polling manager instance (defined in BLE_Polling_Service.h)
extern BLEPollingManager pollingManager;

// Static callback instances to prevent memory deallocation issues
static ServerCallbacks serverCallbacksInstance;
static QuaternionCharCallbacks quaternionCallbacksInstance;
// HubClientCallbacks moved to HubClient_Service.cpp
// RoleConfig callbacks removed - using polling instead

// BNO085 IMU instance
BNO085 imu;

// Custom GATT Service UUIDs
#define EIDON_SERVICE_UUID        "E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define QUATERNION_CHAR_UUID      "E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define CALIBRATION_CHAR_UUID     "E1D00003-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define DEVICE_INFO_CHAR_UUID     "E1D00005-8B5A-3E5B-9E23-4F9B5C91BBDE"

// Message type constants for ESP-NOW packets
#define MESSAGE_TYPE_QUAT 0x01    // Quaternion data
#define MESSAGE_TYPE_CMD  0x02    // Command

// New characteristics for hub devices only (child data)
#define HAND_QUATERNION_CHAR_UUID     "E1D00008-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define FOREARM_QUATERNION_CHAR_UUID  "E1D00009-8B5A-3E5B-9E23-4F9B5C91BBDE"

// QuaternionData structure is now defined in Role_Services/Hub_Structures.h
QuaternionData gattQuaternionData;

// Global quaternion variables (for backward compatibility)
float quaternion_x = 0;
float quaternion_y = 0;
float quaternion_z = 0;
float quaternion_w = 1;

// Update interval (milliseconds)
const unsigned long UPDATE_INTERVAL = 1;
unsigned long lastUpdate = 0;

// Add rate limiting for BLE notifications
const unsigned long BLE_NOTIFICATION_INTERVAL = 20; // Send every 20ms (50Hz) for stability instead of 10ms
unsigned long lastNotificationTime = 0;

// Add transmission rate limiting for main loop
const unsigned long TRANSMISSION_INTERVAL = 42; // 24Hz max (41.67ms interval) to match ESP-NOW
unsigned long lastTransmission = 0;

// Track subscription status
bool quaternionSubscribed = false;

// ESP-NOW sender variables for child devices
bool espNowInitialized = false;
uint8_t hubMacAddress[6];
bool hubMacAssigned = false;
unsigned long lastESPNowTransmission = 0;
const unsigned long ESP_NOW_INTERVAL = 21; // 48 Hz (20.83ms interval) - redundancy approach for packet loss

// ESP-NOW error handling and timeout
unsigned long lastESPNowError = 0;
const unsigned long ESP_NOW_ERROR_TIMEOUT = 5000; // 5 seconds between error logs
int espNowErrorCount = 0;

// Child device periodic logging system
const unsigned long CHILD_LOG_INTERVAL = 30000; // 30 seconds for periodic updates (optimized for battery)
unsigned long lastChildLogTime = 0;
unsigned long imuUpdateCount = 0;
unsigned long espNowSendCount = 0;
unsigned long lastHubStatusCheck = 0;
const unsigned long HUB_STATUS_CHECK_INTERVAL = 40000; // 40 seconds (5x the regular interval)

// BLE advertising timeout for child devices
const unsigned long BLE_STARTUP_TIMEOUT = 30000; // 30 seconds for devices that start as children
const unsigned long BLE_DISCONNECT_TIMEOUT = 60000; // 60 seconds after role change to child
unsigned long startupTime = 0;
unsigned long disconnectTime = 0; // Time when device last disconnected
bool startupTimedOut = false;
bool disconnectTimedOut = false;

// IMU rate limiting for both child and hub devices
unsigned long lastIMUUpdate = 0;
const unsigned long IMU_UPDATE_INTERVAL = 21; // 48 Hz (20.83ms interval) - matches ESP-NOW rate for redundancy

// LED pin - changed from 5 to 2 to avoid conflict with switch pin
#define LED_PIN 2

// LED status variables
unsigned long ledLastUpdate = 0;
// LED patterns
enum LEDPattern {
    LED_ADVERTISING,  // Strobing brightness when advertising
    LED_CONNECTED,    // Fast blinking when connected
    LED_IMU_RESET     // Solid LED when resetting IMU
};
LEDPattern currentLEDPattern = LED_ADVERTISING;
unsigned long imuResetStartTime = 0;
const unsigned long IMU_RESET_DURATION = 300; // Solid LED duration in ms
int ledBrightness = 0;
bool ledState = false;

// Simple timer for debugging LED
unsigned long debugLedTimer = 0;
const unsigned long LED_FLASH_INTERVAL = 50; // Very slow flash for debugging

// LED brightness settings
#define LED_DIM_BRIGHTNESS 50  // Dim brightness level (0-255) when connected

// Add interrupt flag for faster sensor reading
volatile bool sensorDataReady = false;

// BLE connection state variables (moved up for LED functions)
bool deviceConnected = false;

// Interrupt service routine
void IRAM_ATTR sensorISR() {
    sensorDataReady = true;
}

// Function to update LED status based on current state
void updateLEDStatus() {
    unsigned long currentTime = millis();
    
    // Special test mode for connected state LED
    if (isConnected()) {
        // SIMPLIFIED: Just toggle LED every LED_FLASH_INTERVAL ms when connected
        if (currentTime - debugLedTimer >= LED_FLASH_INTERVAL) {
            debugLedTimer = currentTime;
            // Toggle between full on and full off for debugging
            ledState = !ledState;
            
            if (ledState) {
                digitalWrite(LED_PIN, HIGH); // Full ON for testing
            } else {
                digitalWrite(LED_PIN, LOW);  // Full OFF
            }
        }
        return; // Skip normal LED logic when connected
    }
    
    // Handle IMU reset pattern with priority
    if (currentLEDPattern == LED_IMU_RESET) {
        digitalWrite(LED_PIN, HIGH); // Solid ON during IMU reset
        
        // Check if IMU reset period is over
        if (currentTime - imuResetStartTime >= IMU_RESET_DURATION) {
            // Return to appropriate pattern based on connection state
            currentLEDPattern = isConnected() ? LED_CONNECTED : LED_ADVERTISING;
            ledLastUpdate = currentTime; // Reset timer to start new pattern immediately
        }
        return;
    }
    
    // Only handle advertising when not connected - simplified and less frequent
    if (currentLEDPattern == LED_ADVERTISING) {
        // Simple blinking pattern instead of complex sine wave
        if (currentTime - ledLastUpdate >= 100) { // Update every 100ms instead of 20ms
            ledLastUpdate = currentTime;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        }
    }
}

// Function to trigger IMU reset LED pattern
void startIMUResetPattern() {
    currentLEDPattern = LED_IMU_RESET;
    imuResetStartTime = millis();
}

// BLE objects
NimBLEServer* pServer = nullptr;

// Custom GATT Service
NimBLEService* eidonService = nullptr;
NimBLECharacteristic* quaternionChar = nullptr;
NimBLECharacteristic* calibrationChar = nullptr;
NimBLECharacteristic* deviceInfoChar = nullptr;

// New characteristics for hub devices only (child data)
NimBLECharacteristic* handQuaternionChar = nullptr;
NimBLECharacteristic* forearmQuaternionChar = nullptr;

// Function to get current BLE connection state (similar to Bluefruit.connected())
bool isConnected() {
    return deviceConnected; // Simple state tracking like reference code
}

// Function to update advertising data with current role information
void updateAdvertisingData() {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising == nullptr) return;
    
    // Stop current advertising
    pAdvertising->stop();
    
    // Update device name
    String deviceName = deviceConfig.generateDeviceName();
    NimBLEDevice::setDeviceName(deviceName.c_str());
    pAdvertising->setName(deviceName.c_str());
    
    // Update role information in manufacturer data
    // Format: [Company ID Low, Company ID High, Role Data]
    uint8_t manufacturerDataBytes[3];
    manufacturerDataBytes[0] = 0xD0;        // Company ID low byte (0xD0)
    manufacturerDataBytes[1] = 0xE1;        // Company ID high byte (0xE1)
    manufacturerDataBytes[2] = (uint8_t)deviceConfig.getRole(); // Role data
    
    NimBLEAdvertisementData manufacturerData;
    manufacturerData.setManufacturerData(manufacturerDataBytes, 3);
    pAdvertising->setAdvertisementData(manufacturerData);
    
    // Update scan response data
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(deviceName.c_str());
    pAdvertising->setScanResponseData(scanResponse);
    
    // Restart advertising with updated data
    pAdvertising->start();
    

}

void sendQuaternionReport() {
    // Rate limiting is now handled in the main loop to prevent double rate limiting
    
    // Get current quaternion data from the library
    float qw_sensor, qx_sensor, qy_sensor, qz_sensor;
    imu.getQuaternion(qw_sensor, qx_sensor, qy_sensor, qz_sensor);
    
    // Apply 180-degree rotation around Z-axis to correct for IMU mounting
    float corrected_w = qw_sensor;
    float corrected_x = -qx_sensor;
    float corrected_y = -qy_sensor;
    float corrected_z = qz_sensor;
    
    // Update global variables for backward compatibility
    quaternion_w = qw_sensor;
    quaternion_x = qx_sensor;
    quaternion_y = qy_sensor;
    quaternion_z = qz_sensor;
    
    // Update child data if in hub mode
    if (deviceConfig.isHubMode()) {
        // Update hub's own quaternion data in aggregated structure
        updateHubQuaternionData(corrected_w, corrected_x, corrected_y, corrected_z);
        
        // Update child data
        updateChildData();
    }
    
    // Send via GATT service if connected
    if (isConnected()) {
        // Only send quaternion data if BNO085 is available
        if (imu.isAvailable()) {
            // Send via GATT service
            if (deviceConfig.isHubMode()) {
                // Send aggregated data for hub
                AggregatedQuaternionData* agg = getAggregatedData();
                gattQuaternionData.w = agg->hubData.w;
                gattQuaternionData.x = agg->hubData.x;
                gattQuaternionData.y = agg->hubData.y;
                gattQuaternionData.z = agg->hubData.z;
            } else {
                // Send individual quaternion data for non-hub devices
                gattQuaternionData.w = corrected_w;
                gattQuaternionData.x = corrected_x;
                gattQuaternionData.y = corrected_y;
                gattQuaternionData.z = corrected_z;
            }
            
            // Send GATT notification with rate limiting
            if (quaternionChar != nullptr) {
                quaternionChar->notify((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
            }
            
            // Send child data via separate characteristics (only populated when device is hub)
            if (deviceConfig.isHubMode() && handQuaternionChar != nullptr && forearmQuaternionChar != nullptr) {
                AggregatedQuaternionData* agg = getAggregatedData();
                
                // Debug logging removed for performance
                
                // Send hand quaternion data
                if (agg->handConnected) {
                    QuaternionData handData = agg->handData;
                    handQuaternionChar->notify((uint8_t*)&handData, sizeof(handData));
                }
                
                // Send forearm quaternion data
                if (agg->forearmConnected) {
                    QuaternionData forearmData = agg->forearmData;
                    forearmQuaternionChar->notify((uint8_t*)&forearmData, sizeof(forearmData));
                }
            }
        } else {
            // Send zero quaternion via GATT when BNO085 is not available
            gattQuaternionData.w = 1.0f;  // Identity quaternion
            gattQuaternionData.x = 0.0f;
            gattQuaternionData.y = 0.0f;
            gattQuaternionData.z = 0.0f;
            
            if (quaternionChar != nullptr) {
                quaternionChar->notify((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
            }
        }
        
        // LED is now controlled by updateLEDStatus() in the main loop
    }
}

// ESP-NOW sender functions for child devices
bool initializeESPNowSender() {
    if (espNowInitialized) {
        return true; // Already initialized
    }
    
    // Force WiFi channel to ESP-NOW channel first
    WiFi.setChannel(1);
    delay(50); // Give WiFi time to settle
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        return false;
    }
    
    // Set ESP-NOW PMK (required for all ESP-NOW operations)
    if (esp_now_set_pmk((uint8_t*)"pmk1234567890123") != ESP_OK) {
        return false;
    }
    
    // ESP-NOW automatically supports both sending and receiving
    // No need to set a specific role - it can do both
    
    espNowInitialized = true;
    Serial.println("CHILD: ESP-NOW initialized on channel 1 (bidirectional)");
    return true;
}

void updateESPNowHubMacAddress() {
    // Check if we have a hub MAC address assigned
    if (deviceConfig.isHubMacAssigned()) {
        deviceConfig.getHubMacAddress(hubMacAddress);
        
        // Register hub as ESP-NOW peer
        esp_now_peer_info_t peerInfo;
        memset(&peerInfo, 0, sizeof(peerInfo));
        memcpy(peerInfo.peer_addr, hubMacAddress, 6);
        peerInfo.channel = 1; // Use channel 1
        peerInfo.encrypt = false; // No encryption for now
        
        esp_err_t result = esp_now_add_peer(&peerInfo);
        if (result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST) {
            hubMacAssigned = true;
        } else {
            hubMacAssigned = false;
        }
    } else {
        hubMacAssigned = false;
    }
}

// Process incoming ESP-NOW packets (for children to receive commands)
void onESPNowDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int dataLen) {
    // Check minimum packet size (header size)
    if (dataLen < sizeof(ESPNowPacketHeader)) {
        return; // Packet too small, ignore silently
    }
    
    // Extract header to determine packet type
    ESPNowPacketHeader* header = (ESPNowPacketHeader*)data;
    
    // Handle different packet types
    if (header->messageType == MESSAGE_TYPE_CMD) {
        // Process command packet
        if (dataLen == sizeof(ESPNowCommandPacket)) {
            ESPNowCommandPacket* cmd = (ESPNowCommandPacket*)data;
            
            if (cmd->commandType == 0x01) { // IMU reset command
                Serial.println("=== CALIBRATION COMMAND RECEIVED ===");
                
                if (imu.isAvailable()) {
                    Serial.println("Resetting IMU...");
                    imu.reset();
                    Serial.println("IMU reset completed successfully");
                } else {
                    Serial.println("ERROR: IMU not available for reset");
                }
            }
        }
    } else if (header->messageType == MESSAGE_TYPE_QUAT) {
        // Ignore quaternion packets silently
    } else {
        Serial.printf("ESP-NOW: Unknown message type: 0x%02X\n", header->messageType);
    }
}

void sendESPNowQuaternionData() {
    if (!espNowInitialized || !hubMacAssigned) {
        return; // Not ready to send
    }
    
    // Rate limiting - now handled by IMU update interval (48Hz)
    // Removed redundant rate limiting since IMU updates at 48Hz
    
    // Get current quaternion data from IMU
    float qw_sensor, qx_sensor, qy_sensor, qz_sensor;
    imu.getQuaternion(qw_sensor, qx_sensor, qy_sensor, qz_sensor);
    
    // Apply 180-degree rotation around Z-axis to correct for IMU mounting
    float corrected_w = qw_sensor;
    float corrected_x = -qx_sensor;
    float corrected_y = -qy_sensor;
    float corrected_z = qz_sensor;
    
    // Create ESP-NOW packet with simplified header structure
    ESPNowQuaternionPacket packet;
    packet.header.messageType = MESSAGE_TYPE_QUAT;  // QUAT
    packet.header.senderRole = (uint8_t)deviceConfig.getRole();  // Include role for categorization
    packet.header.reserved[0] = 0;
    packet.header.reserved[1] = 0;
    packet.quaternion.w = corrected_w;
    packet.quaternion.x = corrected_x;
    packet.quaternion.y = corrected_y;
    packet.quaternion.z = corrected_z;
    
    // Send packet to hub
    esp_err_t result = esp_now_send(hubMacAddress, (uint8_t*)&packet, sizeof(packet));
    
    if (result == ESP_OK) {
        lastESPNowTransmission = millis();
        espNowSendCount++;
        // Reset error count on successful send
        espNowErrorCount = 0;
    } else {
        // Rate limit error logging to prevent spam
        unsigned long currentTime = millis();
        if (currentTime - lastESPNowError >= ESP_NOW_ERROR_TIMEOUT) {
            espNowErrorCount++;
            Serial.printf("ESP-NOW: Failed to send data, error: %d (count: %d)\n", result, espNowErrorCount);
            lastESPNowError = currentTime;
            
            // If we've had many errors, try to reinitialize ESP-NOW
            if (espNowErrorCount >= 10) {
                Serial.println("ESP-NOW: Too many errors, attempting reinitialization...");
                esp_now_deinit();
                delay(100);
                if (esp_now_init() == ESP_OK) {
                    esp_now_set_pmk((uint8_t*)"pmk1234567890123");
                    updateESPNowHubMacAddress();
                    espNowErrorCount = 0;
                    Serial.println("ESP-NOW: Reinitialization successful");
                } else {
                    Serial.println("ESP-NOW: Reinitialization failed");
                }
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n----- Eidon Tracker Starting -----");
    
    // Record startup time for advertising timeout
    startupTime = millis();
    
    // Initialize device configuration first
    if (!deviceConfig.begin()) {
        Serial.println("Failed to initialize device configuration!");
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
    
    // Initialize WiFi for ESP-NOW support and MAC address retrieval
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(); // Start WiFi (no need to connect to network for ESP-NOW)
    
    // Setup LED pin
    pinMode(LED_PIN, OUTPUT);

    // Initialize IMU
    if (!imu.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
        
    // Initialize Bluetooth
    NimBLEDevice::init("");
    
    // Set device name for advertising
    String deviceName = deviceConfig.generateDeviceName();
    NimBLEDevice::setDeviceName(deviceName.c_str());
    
    // Enable proper security to fix write callbacks on encrypted connections
    NimBLEDevice::setSecurityAuth(true, true, true);  // Enable authentication, encryption, and authorization
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);  // No input/output - Just Works pairing, no prompt
    
    // Set consistent power level
    NimBLEDevice::setPower(9); // Use integer value instead of ESP_PWR_LVL_P9
    
    // Create server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacksInstance);
    
    // ---------- Custom GATT Service Setup -----------------------------
    eidonService = pServer->createService(EIDON_SERVICE_UUID);
    
    // Configure Quaternion characteristic
    quaternionChar = eidonService->createCharacteristic(
        QUATERNION_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    quaternionChar->setValue((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
    quaternionChar->setCallbacks(&quaternionCallbacksInstance);
    
    // Configure Calibration characteristic
    calibrationChar = eidonService->createCharacteristic(
        CALIBRATION_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
    );
    
    // Configure Device Info characteristic
    deviceInfoChar = eidonService->createCharacteristic(
        DEVICE_INFO_CHAR_UUID,
        NIMBLE_PROPERTY::READ
    );
    
    // Get device's WiFi MAC address
    uint8_t deviceMac[6];
    WiFi.macAddress(deviceMac);
    
    // Extended device info with MAC address (14 bytes total)
    uint8_t deviceInfo[14] = {
        0x01, 0x00,  // Device ID
        0x01, 0x02,  // Firmware version 1.2
        100,         // Battery level
        (uint8_t)deviceConfig.getRole(),  // Device role
        deviceMac[0], deviceMac[1], deviceMac[2], deviceMac[3], deviceMac[4], deviceMac[5]  // MAC address
    };
    deviceInfoChar->setValue(deviceInfo, sizeof(deviceInfo));
    
    // Add child data characteristics for all devices (populated when device becomes hub)
    // Configure Hand Quaternion characteristic
    handQuaternionChar = eidonService->createCharacteristic(
        HAND_QUATERNION_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    handQuaternionChar->setValue((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
    
    // Configure Forearm Quaternion characteristic
    forearmQuaternionChar = eidonService->createCharacteristic(
        FOREARM_QUATERNION_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    forearmQuaternionChar->setValue((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
    
    // Start custom service
    eidonService->start();
    
    // ---------- Role Configuration Service Setup -----------------------------
    createRoleConfigService(pServer);
    
    // ---------- BLE Polling System Setup -----------------------------
    setupPollingSystem();

    // Add Role target to polling system
    pollingManager.addTarget(roleConfigChar, 1000, handleRoleChange, "Role"); // 1 Hz (1000ms)
    
    // Add Calibration target to polling system (HUB devices only)
    if (deviceConfig.isHubMode()) {
        pollingManager.addTarget(calibrationChar, 100, handleCalibration, "Calibration"); // 10 Hz (100ms)
        Serial.println("HUB: Calibration polling enabled");
    } else {
        Serial.println("CHILD: Calibration polling disabled (not a hub)");
    }
    
    // ---------- Hub Client Setup -----------------------------
    setupHubClientService();
    
    // Configure advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(eidonService->getUUID());
    pAdvertising->addServiceUUID(roleConfigService->getUUID());
    
    // Add role information to manufacturer data
    // Format: [Company ID Low, Company ID High, Role Data]
    uint8_t manufacturerDataBytes[3];
    manufacturerDataBytes[0] = 0xD0;        // Company ID low byte (0xD0)
    manufacturerDataBytes[1] = 0xE1;        // Company ID high byte (0xE1)
    manufacturerDataBytes[2] = (uint8_t)deviceConfig.getRole(); // Role data
    
    NimBLEAdvertisementData manufacturerData;
    manufacturerData.setManufacturerData(manufacturerDataBytes, 3);
    pAdvertising->setAdvertisementData(manufacturerData);
    
    // Set device name in main advertising data (not just scan response)
    pAdvertising->setName(deviceName.c_str());
    

    
    // Set conservative advertising intervals for stable connection
    pAdvertising->setMinInterval(160);  // 100ms minimum (more conservative)
    pAdvertising->setMaxInterval(320);  // 200ms maximum (more conservative)
    
    // Create scan response data (additional data for active scanning)
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(deviceName.c_str());
    pAdvertising->setScanResponseData(scanResponse);
    
    // Start advertising
    pAdvertising->start();
    
    // Check for stored hub MAC address if this is a child device
    if (deviceConfig.isNodeMode() && (deviceConfig.getRole() == ROLE_LEFT_HAND || 
                                      deviceConfig.getRole() == ROLE_RIGHT_HAND ||
                                      deviceConfig.getRole() == ROLE_LEFT_FOREARM || 
                                      deviceConfig.getRole() == ROLE_RIGHT_FOREARM)) {
        if (deviceConfig.isHubMacAssigned()) {
            uint8_t hubMac[6];
            deviceConfig.getHubMacAddress(hubMac);
            
            // Initialize ESP-NOW sender for child device
            if (initializeESPNowSender()) {
                updateESPNowHubMacAddress();
            }
        }
    }
    
    // Essential initialization summary
    Serial.printf("INIT: Device: %s, Role: %s, Polling: 1Hz\n", 
                 deviceName.c_str(), 
                 deviceConfig.getRoleName(deviceConfig.getRole()));
    
    // Add ESP-NOW initialization status for child devices
    if (deviceConfig.isNodeMode() && deviceConfig.isHubMacAssigned()) {
        Serial.println("INIT: ESP-NOW ready");
    }
    
    Serial.println("----- Initialization Complete -----");
}

void loop() {
    // Single millis() call for all timing operations
    unsigned long currentTime = millis();
    
    // Update LED status first - DISABLED for performance
    // updateLEDStatus();
    
    // Simplified connection state management for maximum performance (like reference code)
    bool actuallyConnected = (pServer->getConnectedCount() > 0);
    if (actuallyConnected != deviceConnected) {
        deviceConnected = actuallyConnected;
        // Minimal logging to avoid delays
        if (deviceConnected) {
            Serial.println("BLE: Connected");
            
            // Re-initialize ESP-NOW after BLE connection to prevent conflicts
            if (deviceConfig.isHubMode()) {
                // Force WiFi channel back to ESP-NOW channel
                WiFi.setChannel(1);
                delay(100); // Give WiFi time to settle
                
                // Force complete ESP-NOW re-initialization
                esp_now_deinit();
                delay(100);
                
                if (esp_now_init() == ESP_OK) {
                    esp_now_set_pmk((uint8_t*)"pmk1234567890123");
                    esp_now_register_recv_cb(onESPNowDataRecv);
                }
            }
            
            // Force reset LED state and start fresh pattern
            digitalWrite(LED_PIN, LOW);  // Start with LED OFF
            ledState = false;
            ledLastUpdate = 0; // Force immediate update
            currentLEDPattern = LED_CONNECTED;
        } else {
            Serial.println("BLE: Disconnected");
            // Record disconnection time for child devices
            if (deviceConfig.isNodeMode()) {
                disconnectTime = currentTime;
                disconnectTimedOut = false; // Reset disconnect timeout flag
            }
            NimBLEDevice::startAdvertising();
            currentLEDPattern = LED_ADVERTISING; // Switch to advertising LED pattern
        }
    }
    
    // Rate-limited IMU updates for both child and hub devices (48Hz)
    if (currentTime - lastIMUUpdate >= IMU_UPDATE_INTERVAL) {
        imu.update();
        imuUpdateCount++; // Track IMU updates for periodic logging
        lastIMUUpdate = currentTime;
        
        // For child devices: Send ESP-NOW data immediately after every IMU update (48Hz redundancy approach)
        if (deviceConfig.isNodeMode() && (deviceConfig.getRole() == ROLE_LEFT_HAND || 
                                          deviceConfig.getRole() == ROLE_RIGHT_HAND ||
                                          deviceConfig.getRole() == ROLE_LEFT_FOREARM || 
                                          deviceConfig.getRole() == ROLE_RIGHT_FOREARM)) {
            sendESPNowQuaternionData();
        }
    }
    
    // Update BLE polling system
    pollingManager.update();
    
    // Update hub client service (only if we're a hub)
    if (deviceConfig.isHubMode()) {
        updateHubClientService(deviceConnected);
        
        // Check for child disconnections every second
        static unsigned long lastDisconnectCheck = 0;
        if (currentTime - lastDisconnectCheck >= 1000) {
            checkChildDisconnections();
            lastDisconnectCheck = currentTime;
        }
    }
    
    // Send data if connected
    if (deviceConnected) {
        // Rate limit data transmission to prevent overwhelming BLE connection
        static unsigned long lastTransmission = 0;
        const unsigned long TRANSMISSION_INTERVAL = 42; // 24Hz max (41.67ms interval) to match ESP-NOW
        
        if (currentTime - lastTransmission >= TRANSMISSION_INTERVAL) {
            lastTransmission = currentTime;
            // Send quaternion report if connected
            sendQuaternionReport();
            
            // Log BLE transmission for hub devices
            if (deviceConfig.isHubMode()) {
                static unsigned long lastBleLogTime = 0;
                static unsigned long bleTransmissionCount = 0;
                
                bleTransmissionCount++;
                
                // Log BLE transmission rate every 30 seconds
                if (currentTime - lastBleLogTime >= 30000) {
                    float bleRate = (float)bleTransmissionCount / 30.0; // transmissions per second
                    Serial.printf("HUB: Transmitting Quaternion Data over BLE. Rate: %.1f Hz\n", bleRate);
                    bleTransmissionCount = 0;
                    lastBleLogTime = currentTime;
                }
            }
        }
    } else {
        // No periodic debug output when not connected - connection events are logged above
    }
    
    // Periodic logging for child devices (ESP-NOW transmission now handled in IMU update section)
    if (deviceConfig.isNodeMode() && (deviceConfig.getRole() == ROLE_LEFT_HAND || 
                                      deviceConfig.getRole() == ROLE_RIGHT_HAND ||
                                      deviceConfig.getRole() == ROLE_LEFT_FOREARM || 
                                      deviceConfig.getRole() == ROLE_RIGHT_FOREARM)) {
        // Periodic logging for child devices
        if (currentTime - lastChildLogTime >= CHILD_LOG_INTERVAL) {
            float imuRate = (float)imuUpdateCount / (CHILD_LOG_INTERVAL / 1000.0);
            float espNowRate = (float)espNowSendCount / (CHILD_LOG_INTERVAL / 1000.0);
            
            if (startupTimedOut || disconnectTimedOut) {
                Serial.printf("CHILD: IMU %.0f Hz, Sent data to Hub: %.1f Hz, Role: %s (ESP-NOW only mode)\n", 
                             imuRate, espNowRate, deviceConfig.getRoleName(deviceConfig.getRole()));
            } else {
                // Check which timeout is closer
                unsigned long startupTimeLeft = BLE_STARTUP_TIMEOUT - (currentTime - startupTime);
                unsigned long disconnectTimeLeft = (disconnectTime > 0) ? BLE_DISCONNECT_TIMEOUT - (currentTime - disconnectTime) : 0;
                
                if (startupTimeLeft > 0 && (disconnectTime == 0 || startupTimeLeft <= disconnectTimeLeft)) {
                    Serial.printf("CHILD: IMU %.0f Hz, Sent data to Hub: %.1f Hz, Role: %s (Startup timeout in %lus)\n", 
                                 imuRate, espNowRate, deviceConfig.getRoleName(deviceConfig.getRole()), startupTimeLeft / 1000);
                } else if (disconnectTimeLeft > 0) {
                    Serial.printf("CHILD: IMU %.0f Hz, Sent data to Hub: %.1f Hz, Role: %s (Disconnect timeout in %lus)\n", 
                                 imuRate, espNowRate, deviceConfig.getRoleName(deviceConfig.getRole()), disconnectTimeLeft / 1000);
                } else {
                    Serial.printf("CHILD: IMU %.0f Hz, Sent data to Hub: %.1f Hz, Role: %s (BLE timeout imminent)\n", 
                                 imuRate, espNowRate, deviceConfig.getRoleName(deviceConfig.getRole()));
                }
            }
            
            // Reset counters
            imuUpdateCount = 0;
            espNowSendCount = 0;
            lastChildLogTime = currentTime;
        }
        
        // Hub status check (every 5x the regular interval)
        if (currentTime - lastHubStatusCheck >= HUB_STATUS_CHECK_INTERVAL) {
            if (hubMacAssigned) {
                Serial.printf("CHILD: Hub MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                             hubMacAddress[0], hubMacAddress[1], hubMacAddress[2], 
                             hubMacAddress[3], hubMacAddress[4], hubMacAddress[5]);
            } else {
                Serial.println("CHILD: No hub MAC assigned");
            }
            lastHubStatusCheck = currentTime;
        }
    }
    
    // BLE advertising timeout management for child devices
    if (deviceConfig.isNodeMode() && !deviceConnected) {
        // Check startup timeout (30 seconds) - only for devices that start as children
        if (!startupTimedOut && deviceConfig.isRoleAssigned()) {
            unsigned long timeSinceStartup = currentTime - startupTime;
            
            // Debug: Log timeout status every 10 seconds
            static unsigned long lastTimeoutDebug = 0;
            if (currentTime - lastTimeoutDebug >= 10000) {
                Serial.printf("CHILD: Startup timeout debug - timeSinceStartup: %lus, timeout: %lus\n", 
                             timeSinceStartup / 1000, BLE_STARTUP_TIMEOUT / 1000);
                lastTimeoutDebug = currentTime;
            }
            
            if (timeSinceStartup >= BLE_STARTUP_TIMEOUT) {
                // Stop advertising after 30 seconds from startup if no connection
                NimBLEDevice::getAdvertising()->stop();
                startupTimedOut = true;
                Serial.println("CHILD: BLE advertising stopped after 30s startup timeout (no phone connection)");
                Serial.println("CHILD: ESP-NOW only mode active - phone connection requires device restart");
            }
        }
        
        // Check disconnect timeout (60 seconds from last disconnection)
        if (!disconnectTimedOut && disconnectTime > 0) {
            unsigned long timeSinceDisconnect = currentTime - disconnectTime;
            
            // Debug: Log disconnect timeout status every 10 seconds
            static unsigned long lastDisconnectDebug = 0;
            if (currentTime - lastDisconnectDebug >= 10000) {
                Serial.printf("CHILD: Disconnect timeout debug - timeSinceDisconnect: %lus, timeout: %lus\n", 
                             timeSinceDisconnect / 1000, BLE_DISCONNECT_TIMEOUT / 1000);
                lastDisconnectDebug = currentTime;
            }
            
            if (timeSinceDisconnect >= BLE_DISCONNECT_TIMEOUT) {
                // Stop advertising after 60 seconds from disconnection
                NimBLEDevice::getAdvertising()->stop();
                disconnectTimedOut = true;
                Serial.println("CHILD: BLE advertising stopped after 60s disconnect timeout (no phone reconnection)");
                Serial.println("CHILD: ESP-NOW only mode active - phone connection requires device restart");
            }
        }
    } else if (deviceConfig.isNodeMode() && deviceConnected) {
        // Debug: When connected, reset disconnect timeout
        if (disconnectTime > 0) {
            Serial.printf("CHILD: Connected, resetting disconnect timeout (was %lus ago)\n", 
                         (currentTime - disconnectTime) / 1000);
            disconnectTime = 0; // Reset disconnect time when connected
        }
    }
    
    // Only restart advertising if truly disconnected, not advertising, and not timed out
    static unsigned long lastAdvertisingCheck = 0;
    static unsigned long lastAdvertisingRestart = 0;
    const unsigned long ADVERTISING_CHECK_INTERVAL = 1000; // Check every 1 second
    const unsigned long ADVERTISING_RESTART_COOLDOWN = 5000; // Wait 5 seconds between restarts
    
    if (!deviceConnected && !startupTimedOut && !disconnectTimedOut && (currentTime - lastAdvertisingCheck > ADVERTISING_CHECK_INTERVAL)) {
        lastAdvertisingCheck = currentTime;
        
        bool isCurrentlyAdvertising = NimBLEDevice::getAdvertising()->isAdvertising();
        
        if (!isCurrentlyAdvertising && (currentTime - lastAdvertisingRestart > ADVERTISING_RESTART_COOLDOWN)) {
            NimBLEDevice::startAdvertising();
            lastAdvertisingRestart = currentTime;
        }
    }
}
