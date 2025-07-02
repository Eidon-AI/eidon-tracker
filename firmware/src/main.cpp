#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>
#include "BNO085.h"
#include "BLE_Services/BLE_Callbacks.h"
#include "BLE_Services/HID_Descriptor.h"
#include "BLE_Services/RoleConfig_Service.h"
#include "BLE_Services/BLE_Polling_Service.h"
#include "DeviceConfig.h"

// Function declarations
void sendQuaternionReport();
void updateLEDStatus();
void startIMUResetPattern();
bool isConnected();

// Polling system functions (implemented in BLE_Polling_Service.cpp)
void setupPollingSystem();
void handleRoleChange(const std::string& value, bool success);

// Polling manager instance (defined in BLE_Polling_Service.h)
extern BLEPollingManager pollingManager;

// Static callback instances to prevent memory deallocation issues
static ServerCallbacks serverCallbacksInstance;
static OutputReportCallbacks outputReportCallbacksInstance;
static QuaternionCharCallbacks quaternionCallbacksInstance;
static CalibrationCallbacks calibrationCallbacksInstance;
// RoleConfig callbacks removed - using polling instead

// BNO085 IMU instance
BNO085 imu;

// Vendor and Product IDs
#define VENDOR_ID  0xE1D0 // Eidon AI vendor ID
#define PRODUCT_ID 0x0002 // Eidon Tracker product ID

// Custom GATT Service UUIDs
#define EIDON_SERVICE_UUID        "E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define QUATERNION_CHAR_UUID      "E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define CALIBRATION_CHAR_UUID     "E1D00003-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define DEVICE_INFO_CHAR_UUID     "E1D00005-8B5A-3E5B-9E23-4F9B5C91BBDE"

// Quaternion data structure for GATT (20 bytes)
struct QuaternionData {
    float w;
    float x;
    float y;
    float z;
    uint8_t switches;    // bit 0: isLeft, bit 1: isUpper
    uint8_t reserved[3]; // padding to 20 bytes
} __attribute__((packed));

QuaternionData gattQuaternionData;

// HID report map - now 8 bytes total (8 bytes for quaternion, no switch states)
uint8_t report_data[8] = {0};

// Add output report buffer
uint8_t output_report[1] = {0};

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
const unsigned long TRANSMISSION_INTERVAL = 20; // 50Hz max (20ms interval) for stability
unsigned long lastTransmission = 0;

// Track subscription status
bool quaternionSubscribed = false;

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
NimBLEHIDDevice* hid = nullptr;
NimBLECharacteristic* inputReport = nullptr;
NimBLECharacteristic* outputReport = nullptr;

// Custom GATT Service
NimBLEService* eidonService = nullptr;
NimBLECharacteristic* quaternionChar = nullptr;
NimBLECharacteristic* calibrationChar = nullptr;
NimBLECharacteristic* deviceInfoChar = nullptr;

// Function to get current BLE connection state (similar to Bluefruit.connected())
bool isConnected() {
    return deviceConnected; // Simple state tracking like reference code
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
    
    // Send via HID if connected
    if (isConnected()) {
        // Only send quaternion data if BNO085 is available
        if (imu.isAvailable()) {
            // Map corrected quaternion values (-1 to 1) to unsigned HID range (0 to 65535)
            uint16_t x = (uint16_t)((corrected_x + 1.0f) * 32767.5f);
            uint16_t y = (uint16_t)((corrected_y + 1.0f) * 32767.5f);
            uint16_t z = (uint16_t)((corrected_z + 1.0f) * 32767.5f);
            uint16_t w = (uint16_t)((corrected_w + 1.0f) * 32767.5f);
            
            // Create HID report - store 16-bit values in little-endian format
            report_data[0] = x & 0xFF;
            report_data[1] = (x >> 8) & 0xFF;
            report_data[2] = y & 0xFF;
            report_data[3] = (y >> 8) & 0xFF;
            report_data[4] = z & 0xFF;
            report_data[5] = (z >> 8) & 0xFF;
            report_data[6] = w & 0xFF;
            report_data[7] = (w >> 8) & 0xFF;
            
            // Send HID report with error handling
            if (inputReport != nullptr) {
                inputReport->setValue(report_data, sizeof(report_data));
                if (inputReport->notify()) {
                    // Successful transmission
                    static unsigned long successCount = 0;
                    successCount++;
                    
                    // Print success rate occasionally
                    static unsigned long lastSuccessReport = 0;
                    if (millis() - lastSuccessReport >= 10000) { // Every 10 seconds instead of 5
                        Serial.print("Successful HID transmissions: ");
                        Serial.println(successCount);
                        lastSuccessReport = millis();
                    }
                } else {
                    // Failed to send notification
                    Serial.println("Warning: Failed to send HID notification");
                    static unsigned long failCount = 0;
                    failCount++;
                    if (failCount % 10 == 0) {
                        Serial.print("Failed HID transmissions: ");
                        Serial.println(failCount);
                    }
                }
            }
            
            // Also send via GATT service
            gattQuaternionData.w = corrected_w;
            gattQuaternionData.x = corrected_x;
            gattQuaternionData.y = corrected_y;
            gattQuaternionData.z = corrected_z;
            gattQuaternionData.switches = 0;
            memset(gattQuaternionData.reserved, 0, sizeof(gattQuaternionData.reserved));
            
            // Send GATT notification with rate limiting
            if (quaternionChar != nullptr) {
                // Rate limit notifications to prevent overwhelming the BLE stack
                unsigned long currentTime = millis();
                if (currentTime - lastNotificationTime >= BLE_NOTIFICATION_INTERVAL) {
                    bool notifyResult = quaternionChar->notify((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
                    lastNotificationTime = currentTime;
                    
                    static unsigned long lastDebugPrint = 0;
                    if (currentTime - lastDebugPrint >= 10000) { // Print every 10 seconds (increased from 5 seconds)
                        Serial.print("GATT: Quaternion notification sent, result=");
                        Serial.print(notifyResult);
                        Serial.print(", subscribed="); Serial.print(quaternionSubscribed ? "YES" : "NO");
                        Serial.print(", data: W="); Serial.print(corrected_w, 4);
                        Serial.print(" X="); Serial.print(corrected_x, 4);
                        Serial.print(" Y="); Serial.print(corrected_y, 4);
                        Serial.print(" Z="); Serial.print(corrected_z, 4);
                        Serial.println();
                        lastDebugPrint = currentTime;
                    }
                }
            } else {
                Serial.println("GATT: quaternionChar is null!");
            }
        } else {
            // Send zero quaternion when BNO085 is not available
            memset(report_data, 0, sizeof(report_data));
            
            if (inputReport != nullptr) {
                inputReport->setValue(report_data, sizeof(report_data));
                inputReport->notify();
            }
            
            // Also send zero quaternion via GATT
            gattQuaternionData.w = 1.0f;  // Identity quaternion
            gattQuaternionData.x = 0.0f;
            gattQuaternionData.y = 0.0f;
            gattQuaternionData.z = 0.0f;
            gattQuaternionData.switches = 0;
            memset(gattQuaternionData.reserved, 0, sizeof(gattQuaternionData.reserved));
            
            if (quaternionChar != nullptr) {
                quaternionChar->notify((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
                static unsigned long lastDebugPrint = 0;
                if (millis() - lastDebugPrint >= 5000) { // Increased from 1 second
                    Serial.println("GATT: Sent zero quaternion (BNO085 not available)");
                    lastDebugPrint = millis();
                }
            }
        }
        
        // LED is now controlled by updateLEDStatus() in the main loop
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n----- Eidon Tracker Starting -----");
    
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
    
    // Generate unique device name based on role
    String deviceName = deviceConfig.generateDeviceName();
    NimBLEDevice::setDeviceName(deviceName.c_str());
    
    Serial.print("Advertising as: ");
    Serial.println(deviceName.c_str());
    
    // Enable proper security to fix write callbacks on encrypted connections
    NimBLEDevice::setSecurityAuth(true, true, true);  // Enable authentication, encryption, and authorization
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);  // No input/output - Just Works pairing, no prompt
    
    // Set consistent power level
    NimBLEDevice::setPower(9); // Use integer value instead of ESP_PWR_LVL_P9
    
    // Create server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacksInstance);
    
    // Create HID device
    hid = new NimBLEHIDDevice(pServer);
    inputReport = hid->getInputReport(1);
    outputReport = hid->getOutputReport(1);
    outputReport->setCallbacks(&outputReportCallbacksInstance);
    
    // Configure HID device
    hid->setManufacturer("Eidon AI");
    hid->setPnp(0x02, VENDOR_ID, PRODUCT_ID, 0x0110);
    hid->setHidInfo(0x00, 0x01);
    hid->setReportMap((uint8_t*)hid_report_descriptor, hid_report_descriptor_size);
    
    // Start HID services
    hid->startServices();
    
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
    calibrationChar->setCallbacks(&calibrationCallbacksInstance);
    
    // Configure Device Info characteristic
    deviceInfoChar = eidonService->createCharacteristic(
        DEVICE_INFO_CHAR_UUID,
        NIMBLE_PROPERTY::READ
    );
    uint8_t deviceInfo[8] = {
        0x01, 0x00,  // Device ID
        0x01, 0x02,  // Firmware version 1.2
        100,         // Battery level
        (uint8_t)deviceConfig.getRole(),  // Device role
        0, 0          // Reserved
    };
    deviceInfoChar->setValue(deviceInfo, sizeof(deviceInfo));
    
    // Start custom service
    eidonService->start();
    
    // ---------- Role Configuration Service Setup -----------------------------
    createRoleConfigService(pServer);
    
    // ---------- BLE Polling System Setup -----------------------------
    setupPollingSystem();

    // Add Role target to polling system
    pollingManager.addTarget(roleConfigChar, 200, handleRoleChange, "Role");
    
    Serial.println("BLE Polling System setup complete");
    
    // Configure advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setAppearance(HID_GAMEPAD);
    pAdvertising->addServiceUUID(hid->getHidService()->getUUID());
    pAdvertising->addServiceUUID(eidonService->getUUID());
    pAdvertising->addServiceUUID(roleConfigService->getUUID());
    
    // Set conservative advertising intervals for stable connection
    pAdvertising->setMinInterval(160);  // 100ms minimum (more conservative)
    pAdvertising->setMaxInterval(320);  // 200ms maximum (more conservative)
    
    // Create scan response data
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(deviceName.c_str());
    pAdvertising->setScanResponseData(scanResponse);
    
    // Start advertising
    pAdvertising->start();
}

void loop() {
    // Update LED status first
    updateLEDStatus();
    
    // Simplified connection state management for maximum performance (like reference code)
    bool actuallyConnected = (pServer->getConnectedCount() > 0);
    if (actuallyConnected != deviceConnected) {
        deviceConnected = actuallyConnected;
        // Minimal logging to avoid delays
        if (deviceConnected) {
            Serial.println("=== MAIN LOOP: Connection detected ===");
            Serial.println("Connected - starting to send data");
            // Force reset LED state and start fresh pattern
            digitalWrite(LED_PIN, LOW);  // Start with LED OFF
            ledState = false;
            ledLastUpdate = 0; // Force immediate update
            currentLEDPattern = LED_CONNECTED;
        } else {
            Serial.println("=== MAIN LOOP: Disconnection detected ===");
            Serial.println("Disconnected - restarting advertising");
            NimBLEDevice::startAdvertising();
            currentLEDPattern = LED_ADVERTISING; // Switch to advertising LED pattern
        }
    }
    
    // Read sensor data directly (polling-based instead of interrupt-driven)
    imu.update();
    
    // Update BLE polling system
    pollingManager.update();
    
    // Send data if connected
    if (deviceConnected) {
        // Rate limit data transmission to prevent overwhelming BLE connection
        static unsigned long lastTransmission = 0;
        const unsigned long TRANSMISSION_INTERVAL = 20; // 50Hz max (20ms interval) for stability
        
        if (millis() - lastTransmission < TRANSMISSION_INTERVAL) {
            // Skip this transmission cycle to maintain stable rate
            delay(1); // Small delay to prevent busy waiting
            return; // Early return to avoid rest of loop processing
        }
        
        lastTransmission = millis();
        
        // Send quaternion report if connected
        sendQuaternionReport();
    } else {
        // Add debug output when not connected - every 60 seconds (increased from 30 seconds)
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint >= 60000) {
            Serial.print("DEBUG: Not connected, waiting for client... Role: ");
            Serial.print(deviceConfig.getRoleName(deviceConfig.getRole()));
            Serial.print(", Assigned: ");
            Serial.print(deviceConfig.isRoleAssigned() ? "YES" : "NO");
            Serial.print(", Mode: ");
            Serial.println(deviceConfig.isHubMode() ? "HUB" : "NODE");
            lastDebugPrint = millis();
        }
    }
    
    // Only restart advertising if truly disconnected and not advertising
    // Add some debugging and rate limiting to prevent spam
    static unsigned long lastAdvertisingCheck = 0;
    static unsigned long lastAdvertisingRestart = 0;
    const unsigned long ADVERTISING_CHECK_INTERVAL = 1000; // Check every 1 second
    const unsigned long ADVERTISING_RESTART_COOLDOWN = 5000; // Wait 5 seconds between restarts
    
    if (!deviceConnected && (millis() - lastAdvertisingCheck > ADVERTISING_CHECK_INTERVAL)) {
        lastAdvertisingCheck = millis();
        
        bool isCurrentlyAdvertising = NimBLEDevice::getAdvertising()->isAdvertising();
        
        if (!isCurrentlyAdvertising && (millis() - lastAdvertisingRestart > ADVERTISING_RESTART_COOLDOWN)) {
            Serial.println("Restarting advertising to reconnect...");
            NimBLEDevice::startAdvertising();
            lastAdvertisingRestart = millis();
        }
    }
}
