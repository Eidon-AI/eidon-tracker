#include <Arduino.h>
#include <string>
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
#include "Version.h"

// Function declarations
void sendQuaternionReport();
void updateLEDStatus();
void updateStatusLED();
void startIMUResetPattern();
bool isConnected();
void updateAdvertisingData();
float readBatteryVoltage();
uint8_t calculateBatteryPercentage(float voltage);
void updateBatteryLevel();

// ESP-NOW sender functions for child devices
bool initializeESPNowSender();
void sendESPNowQuaternionData();
void sendESPNowRawData(); // Add declaration for raw data sender
void updateESPNowHubMacAddress();
void restoreESPNowPeers();  // ESP-NOW peer recovery function

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
#define QUATERNION_CHAR_UUID      "E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE"  // MAIN quaternion (device's own data)
#define CALIBRATION_CHAR_UUID     "E1D00003-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define DEVICE_INFO_CHAR_UUID     "E1D00005-8B5A-3E5B-9E23-4F9B5C91BBDE"

// MAIN Raw Data Characteristic (device's own raw data - all devices use this)
#define HUB_RAW_DATA_CHAR_UUID      "E1D0000B-8B5A-3E5B-9E23-4F9B5C91BBDE"  // Also known as MAIN_RAW_DATA_CHAR_UUID

// Message type constants for ESP-NOW packets
#define MESSAGE_TYPE_QUAT 0x01    // Quaternion data
#define MESSAGE_TYPE_CMD  0x02    // Command

// Characteristic UUIDs for device data
// MAIN characteristics: Device's own data (all devices use these)
// Note: MAIN quaternion uses QUATERNION_CHAR_UUID (already defined above)
// Note: MAIN raw data uses HUB_RAW_DATA_CHAR_UUID (already defined above)

// LEFT characteristics: Left child data (right hubs only - receives ESP-NOW from left child)
#define LEFT_QUATERNION_CHAR_UUID  "E1D00008-8B5A-3E5B-9E23-4F9B5C91BBDE"  // Left child quaternion data
#define LEFT_RAW_DATA_CHAR_UUID    "E1D0000C-8B5A-3E5B-9E23-4F9B5C91BBDE"  // Left child raw data
#define LEFT_BATTERY_CHAR_UUID     "E1D0000D-8B5A-3E5B-9E23-4F9B5C91BBDE"  // Left child battery level

// QuaternionData structure is now defined in Role_Services/Hub_Structures.h
QuaternionData gattQuaternionData;
// RawMotionData gattRawData; // Removed redundant global variable - using local vars or member access

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

// Raw data logging interval for debugging
const unsigned long RAW_DATA_LOG_INTERVAL = 5000; // 5 seconds for raw data logs
unsigned long lastRawDataLogTime = 0;

// BLE advertising timeout for child devices
const unsigned long BLE_STARTUP_TIMEOUT = 45000; // 45 seconds for devices that start as children
const unsigned long BLE_DISCONNECT_TIMEOUT = 60000; // 60 seconds after role change to child
unsigned long startupTime = 0;
unsigned long disconnectTime = 0; // Time when device last disconnected
bool startupTimedOut = false;
bool disconnectTimedOut = false;

// IMU rate limiting for both child and hub devices
unsigned long lastIMUUpdate = 0;
const unsigned long IMU_UPDATE_INTERVAL = 21; // 48 Hz (20.83ms interval) - matches ESP-NOW rate for redundancy

// LED pin - Seeed XIAO ESP32-C6 onboard LED is on GPIO15
#define LED_PIN 15

// Status LED pin (second LED for simple status indication on GPIO1)
#define STATUS_LED_PIN A1

// Status LED variables
unsigned long statusLedLastUpdate = 0;
bool statusLedState = false;
const unsigned long STATUS_LED_BLINK_INTERVAL = 500; // Blink every 500ms when advertising

// Battery level monitoring (following Seeed XIAO ESP32 wiki)
#define BATTERY_ADC_PIN 0  // GPIO0 (A0)
const float VOLTAGE_DIVIDER_RATIO = 2.0;  // Voltage divider with two equal resistors (R1 = R2)
const float ADC_REFERENCE_VOLTAGE = 3.3;  // ESP32-C3 ADC reference voltage
const int ADC_RESOLUTION = 4095;  // 12-bit ADC (0-4095)
unsigned long lastBatteryRead = 0;
const unsigned long BATTERY_READ_INTERVAL = 5000; // Read battery every 5 seconds (for debugging)
float batteryVoltage = 0.0;
uint8_t batteryPercentage = 100;
bool batteryPresent = true;

// Battery detection thresholds
const float BATTERY_MIN_VALID_VOLTAGE = 2.5;  // Below this = no battery present
const float BATTERY_FLOATING_MIN = 3.8;       // Floating ADC typically reads in this range
const float BATTERY_FLOATING_MAX = 4.0;       // when USB powered but no battery

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

// Function to update status LED (second LED) - simple on/off control
// Blinks when advertising, solid when connected
void updateStatusLED() {
    unsigned long currentTime = millis();
    static bool lastConnectedState = false;

    if (isConnected()) {
        // Solid ON when connected
        digitalWrite(STATUS_LED_PIN, HIGH);
        lastConnectedState = true;
    } else {
        // Blink when advertising (not connected)
        if (currentTime - statusLedLastUpdate >= STATUS_LED_BLINK_INTERVAL) {
            statusLedLastUpdate = currentTime;
            statusLedState = !statusLedState;
            digitalWrite(STATUS_LED_PIN, statusLedState ? HIGH : LOW);
        }
        lastConnectedState = false;
    }
}

// Function to read battery voltage from ADC
// Returns battery voltage in volts
// Following official Seeed XIAO ESP32C6 wiki example
float readBatteryVoltage() {
    // Read ADC value in millivolts (average of 16 samples as per Seeed wiki)
    uint32_t Vbatt = 0;
    const int numSamples = 16;

    for (int i = 0; i < numSamples; i++) {
        Vbatt += analogReadMilliVolts(A0); // Read and accumulate ADC voltage in mV
    }

    // Adjust for 1:2 voltage divider and convert to volts
    float Vbattf = 2.0 * Vbatt / numSamples / 1000.0;

    // Detect battery presence
    // When USB powered but no battery, ADC floats around 3.8-4.0V (giving false 80% reading)
    if (Vbattf < BATTERY_MIN_VALID_VOLTAGE) {
        batteryPresent = false;
    } else if (Vbattf >= BATTERY_FLOATING_MIN && Vbattf <= BATTERY_FLOATING_MAX) {
        // Likely floating ADC reading (USB powered, no battery)
        batteryPresent = false;
    } else {
        batteryPresent = true;
    }

    return Vbattf;
}

// Function to calculate battery percentage from voltage
// LiPo battery: 4.2V (100%) to 3.0V (0%)
uint8_t calculateBatteryPercentage(float voltage) {
    const float BATTERY_MAX_VOLTAGE = 4.2;  // Fully charged LiPo
    const float BATTERY_MIN_VOLTAGE = 3.0;  // Discharged LiPo (safe cutoff)

    uint8_t result;
    if (voltage >= BATTERY_MAX_VOLTAGE) {
        result = 100;
    } else if (voltage <= BATTERY_MIN_VOLTAGE) {
        result = 0;
    } else {
        // Linear interpolation
        float percentage = ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
        result = (uint8_t)percentage;
    }


    return result;
}

// BLE objects
NimBLEServer* pServer = nullptr;

// Custom GATT Service
NimBLEService* eidonService = nullptr;
NimBLECharacteristic* quaternionChar = nullptr;  // MAIN quaternion (device's own data - all devices use this)
NimBLECharacteristic* calibrationChar = nullptr;
NimBLECharacteristic* deviceInfoChar = nullptr;

// MAIN characteristics (device's own data):
NimBLECharacteristic* hubRawDataChar = nullptr;  // MAIN raw data (device's own raw data - all devices use this)

// LEFT characteristics (left child data - right hubs only)
NimBLECharacteristic* leftQuaternionChar = nullptr;  // Left child quaternion data
NimBLECharacteristic* leftRawDataChar = nullptr;     // Left child raw data
NimBLECharacteristic* leftBatteryChar = nullptr;     // Left child battery level

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
            
            // Send LEFT child data via LEFT characteristics (right hubs only)
            // Note: Chest hub has no children, so LEFT characteristics won't send data for chest
            if (deviceConfig.isHubMode() && leftQuaternionChar != nullptr) {
                AggregatedQuaternionData* agg = getAggregatedData();
                
                // Send left child quaternion data (each right hub has exactly one left child)
                // Check which child type is connected and send its data
                if (agg->handConnected) {
                    QuaternionData leftData = agg->handData;
                    leftQuaternionChar->notify((uint8_t*)&leftData, sizeof(leftData));
                } else if (agg->forearmConnected) {
                    QuaternionData leftData = agg->forearmData;
                    leftQuaternionChar->notify((uint8_t*)&leftData, sizeof(leftData));
                } else if (agg->shoulderConnected) {
                    QuaternionData leftData = agg->shoulderData;
                    leftQuaternionChar->notify((uint8_t*)&leftData, sizeof(leftData));
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

// Function to update battery level (called periodically)
void updateBatteryLevel() {
    unsigned long currentTime = millis();

    if (currentTime - lastBatteryRead >= BATTERY_READ_INTERVAL) {
        lastBatteryRead = currentTime;

        // Read battery voltage
        batteryVoltage = readBatteryVoltage();
        
        static bool firstRun = true;

        // Only calculate percentage if battery is present
        if (batteryPresent) {
            batteryPercentage = calculateBatteryPercentage(batteryVoltage);
        } else {
            // No battery detected (USB powered only or disconnected)
            batteryPercentage = 0;
        }

        // Update device info characteristic with new battery level
        if (deviceInfoChar != nullptr) {
            uint8_t deviceMac[6];
            WiFi.macAddress(deviceMac);

            // Extended device info with MAC address (14 bytes total)
            uint8_t deviceInfo[14] = {
                0x01, 0x00,  // Device ID
                FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH, // Firmware version
                batteryPercentage,  // Updated battery level (0 if no battery)
                (uint8_t)deviceConfig.getRole(),  // Device role
                deviceMac[0], deviceMac[1], deviceMac[2], deviceMac[3], deviceMac[4], deviceMac[5]  // MAC address
            };
            deviceInfoChar->setValue(deviceInfo, sizeof(deviceInfo));
        }

        // Update left child battery characteristic (right hubs only)
        if (leftBatteryChar != nullptr) {
            uint8_t childBattery = getChildBatteryLevel();
            leftBatteryChar->setValue(&childBattery, 1);
        }
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
    
    // Register callback for receiving commands from hub
    esp_now_register_recv_cb(onESPNowDataRecv);
    Serial.println("CHILD: ESP-NOW receive callback registered - ready to receive calibration commands");
    
    // ESP-NOW automatically supports both sending and receiving
    // No need to set a specific role - it can do both
    
    espNowInitialized = true;
    return true;
}

void updateESPNowHubMacAddress() {
    // Check if we have a hub MAC address assigned
    if (deviceConfig.isHubMacAssigned()) {
        if (deviceConfig.getHubMacAddress(hubMacAddress)) {
            // Register hub as ESP-NOW peer
            esp_now_peer_info_t peerInfo;
            memset(&peerInfo, 0, sizeof(peerInfo));
            memcpy(peerInfo.peer_addr, hubMacAddress, 6);
            peerInfo.channel = 1; // Use channel 1
            peerInfo.encrypt = false; // No encryption for now
            
            esp_err_t result = esp_now_add_peer(&peerInfo);
            if (result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST) {
                hubMacAssigned = true;
                Serial.printf("CHILD: Hub MAC registered for ESP-NOW: %02X:%02X:%02X:%02X:%02X:%02X\n",
                             hubMacAddress[0], hubMacAddress[1], hubMacAddress[2],
                             hubMacAddress[3], hubMacAddress[4], hubMacAddress[5]);
            } else {
                hubMacAssigned = false;
                Serial.printf("CHILD: Failed to register hub MAC for ESP-NOW (error: %d)\n", result);
            }
        } else {
            hubMacAssigned = false;
            Serial.println("CHILD: Failed to get hub MAC address from device config");
        }
    } else {
        hubMacAssigned = false;
        Serial.println("CHILD: No hub MAC address assigned in device config");
    }
}

/**
 * Unified ESP-NOW callback function for both hub and child devices
 * 
 * This function handles all incoming ESP-NOW packets and routes them based on device role:
 * - HUB devices: Process MESSAGE_TYPE_QUAT (quaternion packets) and MESSAGE_TYPE_RAW (raw data packets) from children
 * - CHILD devices: Process MESSAGE_TYPE_CMD (calibration commands) from hub
 * 
 * Packet Flow:
 * 1. Children send quaternions (MESSAGE_TYPE_QUAT) → Hub receives and processes
 * 2. Children send raw data (MESSAGE_TYPE_RAW) → Hub receives and processes
 * 3. Hub sends calibration (MESSAGE_TYPE_CMD) → Children receive and process
 * 
 * Expected Behavior:
 * - Hub ignores command packets (it only sends them)
 * - Children ignore quaternion and raw data packets (they only send them)
 * - All packets are validated for minimum size before processing
 * 
 * @param esp_now_info ESP-NOW receive information including source MAC address
 * @param data Raw packet data
 * @param dataLen Length of packet data in bytes
 */
void onESPNowDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int dataLen) {
    // Check minimum packet size (header size)
    if (dataLen < sizeof(ESPNowPacketHeader)) {
        Serial.printf("ESP-NOW: Packet too small: %d bytes\n", dataLen);
        return; // Packet too small, ignore silently
    }
    
    // Extract header to determine packet type
    ESPNowPacketHeader* header = (ESPNowPacketHeader*)data;
    
    // Route packets based on device role
    if (deviceConfig.isHubMode()) {
        // HUB: Process quaternion and raw data packets from children
        if (header->messageType == MESSAGE_TYPE_QUAT || header->messageType == MESSAGE_TYPE_RAW) {
            // Forward to hub client service for processing
            hubClientService.processESPNowPacket(esp_now_info->src_addr, data, dataLen);
        }
        // Hub ignores command packets (it only sends them)
    } else {
        // CHILD: Process command packets only (calibration commands from right hub)
        // Left children (hand, forearm, shoulder) receive calibration from their corresponding right hub
        if (header->messageType == MESSAGE_TYPE_CMD) {
            Serial.printf("CHILD: ESP-NOW packet received - type: 0x%02X, from: %02X:%02X:%02X:%02X:%02X:%02X\n",
                         header->messageType,
                         esp_now_info->src_addr[0], esp_now_info->src_addr[1], esp_now_info->src_addr[2],
                         esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5]);
            
            // Process calibration command
            if (dataLen == sizeof(ESPNowCommandPacket)) {
                ESPNowCommandPacket* cmd = (ESPNowCommandPacket*)data;
                
                if (cmd->commandType == 0x01) { // IMU reset command
                    Serial.println("CHILD: Calibration command received from hub");
                    
                    if (imu.isAvailable()) {
                        imu.reset();
                        Serial.println("CHILD: IMU reset completed");
                    }
                }
            }
        }
        // Children ignore quaternion and raw data packets (they only send them)
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
    packet.header.reserved[0] = batteryPercentage;
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

void sendESPNowRawData() {
    if (!espNowInitialized || !hubMacAssigned) {
        return; // Not ready to send
    }

    // Get current raw data from IMU
    RawMotionData raw;
    imu.getRawData(raw);

    // Create ESP-NOW packet
    ESPNowRawPacket packet;
    packet.header.messageType = MESSAGE_TYPE_RAW;
    packet.header.senderRole = (uint8_t)deviceConfig.getRole();
    packet.header.reserved[0] = batteryPercentage;
    packet.header.reserved[1] = 0;
    packet.data = raw;

    // Send packet to hub
    // Note: We don't track errors/stats separately for raw data to avoid log spam
    // The quaternion packet handles the connection health tracking
    esp_now_send(hubMacAddress, (uint8_t*)&packet, sizeof(packet));
}

void setup() {
    // Setup LED pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);

    // LED TEST CODE - Commented out but kept for hardware debugging
    // Uncomment below to test LED functionality with different resistor values
    /*
    // Rapid blinking test - should be visible even without serial
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }
    */

    Serial.begin(115200);
    delay(2000); // Longer delay for serial to initialize

    Serial.println("\n\n----- Eidon Tracker Starting -----");
    Serial.flush();
    // Serial.printf("LED: Onboard LED pin = %d, Status LED pin = %d\n", LED_PIN, STATUS_LED_PIN);
    // Serial.flush();

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
    WiFi.begin(); // Required for ESP-NOW - initializes WiFi stack

    // LED TEST CODE - Commented out but kept for hardware debugging
    /*
    // Longer LED test with serial logging
    Serial.println("LED: Extended test - both LEDs should blink 3 times");
    Serial.flush();
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(STATUS_LED_PIN, HIGH);
        Serial.printf("LED: ON - GPIO15=%d, GPIO1=%d\n", digitalRead(LED_PIN), digitalRead(STATUS_LED_PIN));
        Serial.flush();
        delay(500);
        digitalWrite(LED_PIN, LOW);
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial.printf("LED: OFF - GPIO15=%d, GPIO1=%d\n", digitalRead(LED_PIN), digitalRead(STATUS_LED_PIN));
        Serial.flush();
        delay(500);
    }

    // Test GPIO1 specifically with more aggressive toggling
    Serial.println("LED: Testing GPIO1 specifically (watch external LED)");
    Serial.flush();
    for (int i = 0; i < 10; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        Serial.printf("GPIO1 set HIGH, read=%d\n", digitalRead(STATUS_LED_PIN));
        Serial.flush();
        delay(300);
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial.printf("GPIO1 set LOW, read=%d\n", digitalRead(STATUS_LED_PIN));
        Serial.flush();
        delay(300);
    }

    Serial.println("LED: Startup test complete");
    Serial.flush();
    */

    // Setup battery ADC pin (following Seeed wiki)
    pinMode(A0, INPUT); // Configure A0 as ADC input

    // Initial battery reading
    batteryVoltage = readBatteryVoltage();
    batteryPercentage = calculateBatteryPercentage(batteryVoltage);
    if (batteryPresent) {
        Serial.printf("BATTERY: %.2fV (%d%%)\n", batteryVoltage, batteryPercentage);
    } else {
        Serial.println("BATTERY: No battery detected");
    }

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
    
    // Configure MAIN Quaternion characteristic (device's own quaternion data - all devices use this)
    quaternionChar = eidonService->createCharacteristic(
        QUATERNION_CHAR_UUID,  // This is the MAIN quaternion characteristic
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    quaternionChar->setValue((const uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
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
        FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH, // Firmware version
        batteryPercentage,  // Battery level (actual reading from ADC)
        (uint8_t)deviceConfig.getRole(),  // Device role
        deviceMac[0], deviceMac[1], deviceMac[2], deviceMac[3], deviceMac[4], deviceMac[5]  // MAC address
    };
    deviceInfoChar->setValue((const uint8_t*)deviceInfo, sizeof(deviceInfo));
    
    // Configure MAIN Raw Data characteristic (device's own raw data - all devices use this)
    // Note: MAIN quaternion uses quaternionChar (already configured above)
    hubRawDataChar = eidonService->createCharacteristic(
        HUB_RAW_DATA_CHAR_UUID,  // This is the MAIN raw data characteristic
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    // Initialize with zeros using a local variable
    RawMotionData zeroRawData = {0};
    hubRawDataChar->setValue((const uint8_t*)&zeroRawData, sizeof(zeroRawData));

    // Configure LEFT characteristics (left child data - right hubs only)
    // Note: Chest has no children, so these won't be used for chest
    if (deviceConfig.isHubMode() && (deviceConfig.getRole() == ROLE_RIGHT_HAND || 
                                     deviceConfig.getRole() == ROLE_RIGHT_FOREARM || 
                                     deviceConfig.getRole() == ROLE_RIGHT_SHOULDER)) {
        // Configure LEFT Quaternion characteristic (for left child quaternion data)
        leftQuaternionChar = eidonService->createCharacteristic(
            LEFT_QUATERNION_CHAR_UUID,
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
        );
        leftQuaternionChar->setValue((const uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));

        // Configure LEFT Raw Data characteristic (for left child raw data)
        leftRawDataChar = eidonService->createCharacteristic(
            LEFT_RAW_DATA_CHAR_UUID,
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
        );
        leftRawDataChar->setValue((const uint8_t*)&zeroRawData, sizeof(zeroRawData));

        // Configure LEFT Battery characteristic (for left child battery level)
        leftBatteryChar = eidonService->createCharacteristic(
            LEFT_BATTERY_CHAR_UUID,
            NIMBLE_PROPERTY::READ
        );
        uint8_t zeroBattery = 0;
        leftBatteryChar->setValue(&zeroBattery, 1);
    }

    // Start custom service
    eidonService->start();
    
    // ---------- Role Configuration Service Setup -----------------------------
    createRoleConfigService(pServer);
    
    // ---------- BLE Polling System Setup -----------------------------
    setupPollingSystem();

    // Add Role target to polling system
    pollingManager.addTarget(roleConfigChar, 1000, handleRoleChange, "Role"); // 1 Hz (1000ms)
    
    // Add Calibration target to polling system (HUB devices only)
    // Right hubs (hand, forearm, shoulder) and chest all poll for calibration commands
    if (deviceConfig.isHubMode()) {
        pollingManager.addTarget(calibrationChar, 100, handleCalibration, "Calibration"); // 10 Hz (100ms)
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
    
    // Check for stored hub MAC address if this is a left child device
    // Left children (hand, forearm, shoulder) send data to their corresponding right hub
    if (deviceConfig.isNodeMode() && (deviceConfig.getRole() == ROLE_LEFT_HAND || 
                                      deviceConfig.getRole() == ROLE_LEFT_FOREARM || 
                                      deviceConfig.getRole() == ROLE_LEFT_SHOULDER)) {
        if (deviceConfig.isHubMacAssigned()) {
            uint8_t hubMac[6];
            deviceConfig.getHubMacAddress(hubMac);
            
            // Initialize ESP-NOW sender for left child device to send to right hub
            if (initializeESPNowSender()) {
                updateESPNowHubMacAddress();
            }
        }
    }
    
    // Note: Hub client service initialization happens in setupHubClientService() above
    // This is called for all hubs (right hand, right forearm, right shoulder, chest)
    
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

    // Update status LED (second LED) - simple on/off control
    // updateStatusLED(); // Disabled to reduce log noise

    // Update battery level (periodic reading - updates device info characteristic silently)
    updateBatteryLevel();

    // Simplified connection state management for maximum performance (like reference code)
    bool actuallyConnected = (pServer->getConnectedCount() > 0);
    if (actuallyConnected != deviceConnected) {
        deviceConnected = actuallyConnected;
        // Minimal logging to avoid delays
        if (deviceConnected) {
            Serial.println("----- BLE Connected -----");
            
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
                    
                    // Restore ESP-NOW peer registrations after reinitialization
                    restoreESPNowPeers();
                }
            }
            
            // Force reset LED state and start fresh pattern
            digitalWrite(LED_PIN, LOW);  // Start with LED OFF
            ledState = false;
            ledLastUpdate = 0; // Force immediate update
            currentLEDPattern = LED_CONNECTED;
            
            // Reset polling disabled log flag for children
            if (deviceConfig.isNodeMode()) {
                // Reset the static flag by calling a function that can access it
                // This will be handled in the polling update section
            }
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
                                          deviceConfig.getRole() == ROLE_LEFT_FOREARM || 
                                          deviceConfig.getRole() == ROLE_LEFT_SHOULDER)) {
            sendESPNowQuaternionData();
            
            // Send raw data at half rate (24Hz) since sensor only updates at 25Hz
            static bool sendRawNext = true;
            if (sendRawNext) {
                sendESPNowRawData(); 
            }
            sendRawNext = !sendRawNext;
        }
    }
    
    // Update BLE polling system (skip when BLE timeout is up on children)
    if (!deviceConfig.isNodeMode() || deviceConnected || (!startupTimedOut && !disconnectTimedOut)) {
        // Reset polling disabled log flag when polling is active
        static bool pollingDisabledLogged = false;
        if (pollingDisabledLogged) {
            pollingDisabledLogged = false;
        }
        pollingManager.update();
    } else if (deviceConfig.isNodeMode() && !deviceConnected) {
        // Log once when polling is disabled for children
        static bool pollingDisabledLogged = false;
        if (!pollingDisabledLogged) {
            Serial.println("CHILD: BLE polling disabled - ESP-NOW only mode active");
            pollingDisabledLogged = true;
        }
    }
    
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
            
            // Send raw data report if connected
            if (isConnected()) {
                // 1. Hub Raw Data (hub devices send their own raw data)
                if (deviceConfig.isHubMode() && hubRawDataChar != nullptr && imu.isAvailable()) {
                    RawMotionData hubRaw;
                    imu.getRawData(hubRaw);
                    hubRawDataChar->notify((uint8_t*)&hubRaw, sizeof(hubRaw));
                }
                
                // 2. Child Raw Data (child devices send their own raw data when connected to phone)
                if (deviceConfig.isNodeMode() && hubRawDataChar != nullptr && imu.isAvailable()) {
                    RawMotionData childRaw;
                    imu.getRawData(childRaw);
                    hubRawDataChar->notify((uint8_t*)&childRaw, sizeof(childRaw));
                }

                // 3. LEFT Child Raw Data from ESP-NOW (right hubs forward left child data received via ESP-NOW)
                // Note: Chest hub has no children, so this section will not send data for chest
                if (deviceConfig.isHubMode() && leftRawDataChar != nullptr) {
                    // Access child devices to get their latest raw data
                    // Each right hub has exactly one left child (hand, forearm, or shoulder)
                    ESPNowChildDevice* children = getChildDevices();
                    int childCount = MAX_CHILDREN;

                    for (int i = 0; i < childCount; i++) {
                        if (children[i].dataAvailable) {
                            // Send left child raw data to LEFT characteristic
                            leftRawDataChar->notify((uint8_t*)&children[i].lastRawData, sizeof(RawMotionData));
                            break; // Each hub only has one child, so we can break after finding it
                        }
                    }
                }
            }
            
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
                                      deviceConfig.getRole() == ROLE_LEFT_FOREARM || 
                                      deviceConfig.getRole() == ROLE_LEFT_SHOULDER)) {
        // Periodic logging for child devices
        if (currentTime - lastChildLogTime >= CHILD_LOG_INTERVAL) {
            float imuRate = (float)imuUpdateCount / (CHILD_LOG_INTERVAL / 1000.0);
            float espNowRate = (float)espNowSendCount / (CHILD_LOG_INTERVAL / 1000.0);
            
            // Format hub MAC address for display
            char hubMacStr[20] = "NOT ASSIGNED";
            if (deviceConfig.isHubMacAssigned()) {
                uint8_t hubMac[6];
                if (deviceConfig.getHubMacAddress(hubMac)) {
                    snprintf(hubMacStr, sizeof(hubMacStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                            hubMac[0], hubMac[1], hubMac[2], hubMac[3], hubMac[4], hubMac[5]);
                }
            }
            
            if (startupTimedOut || disconnectTimedOut) {
                Serial.printf("CHILD: IMU %.0f Hz, Sent data to Hub: %.1f Hz, Role: %s, Hub: %s (ESP-NOW only mode)\n", 
                             imuRate, espNowRate, deviceConfig.getRoleName(deviceConfig.getRole()), hubMacStr);
            } else {
                // Check which timeout is closer
                unsigned long startupTimeLeft = BLE_STARTUP_TIMEOUT - (currentTime - startupTime);
                unsigned long disconnectTimeLeft = (disconnectTime > 0) ? BLE_DISCONNECT_TIMEOUT - (currentTime - disconnectTime) : 0;
                
                if (startupTimeLeft > 0 && (disconnectTime == 0 || startupTimeLeft <= disconnectTimeLeft)) {
                    Serial.printf("CHILD: IMU %.0f Hz, Sent data to Hub: %.1f Hz, Role: %s, Hub: %s (Startup timeout in %lus)\n", 
                                 imuRate, espNowRate, deviceConfig.getRoleName(deviceConfig.getRole()), hubMacStr, startupTimeLeft / 1000);
                } else if (disconnectTimeLeft > 0) {
                    Serial.printf("CHILD: IMU %.0f Hz, Sent data to Hub: %.1f Hz, Role: %s, Hub: %s (Disconnect timeout in %lus)\n", 
                                 imuRate, espNowRate, deviceConfig.getRoleName(deviceConfig.getRole()), hubMacStr, disconnectTimeLeft / 1000);
                } else {
                    Serial.printf("CHILD: IMU %.0f Hz, Sent data to Hub: %.1f Hz, Role: %s, Hub: %s (BLE timeout imminent)\n", 
                                 imuRate, espNowRate, deviceConfig.getRoleName(deviceConfig.getRole()), hubMacStr);
                }
            }
            
            // Reset counters
            imuUpdateCount = 0;
            espNowSendCount = 0;
            lastChildLogTime = currentTime;
            
            // Raw Data Debug removed to reduce serial noise
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
        // Check startup timeout (45 seconds) - only for devices that start as children
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
                // Stop advertising after 45 seconds from startup if no connection
                NimBLEDevice::getAdvertising()->stop();
                startupTimedOut = true;
                Serial.println("CHILD: BLE advertising stopped after 45s startup timeout (no phone connection)");
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
