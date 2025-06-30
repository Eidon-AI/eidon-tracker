#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>
#include "BNO085.h"

// Function declarations
void sendQuaternionReport();
void updateLEDStatus();
void startIMUResetPattern();
bool isConnected();

// BNO085 IMU instance
BNO085 imu;

// BNO085 I2C pins and Address (for external IMU) - now using constants from BNO085.h
// #define BNO085_I2C_SDA 9  // Now using I2C_SDA from BNO085.h
// #define BNO085_I2C_SCL 10 // Now using I2C_SCL from BNO085.h  
// #define BNO085_I2C_ADDR 0x4B // Now using I2C_ADDR from BNO085.h

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

/* One top-level application collection, Usage = Orientation                */
/*  ├─ Input  (Quaternion + 2 switch bits)                                  */
/*  ├─ Output (Vendor byte)                                                 */
/*  └─ Feature(RGB)                                                         */

const uint8_t hid_report_descriptor[] = {

  /* -----------------------------------------------------------------------
   * Top-level collection : sensor orientation + vendor channel, ID = 1
   * ---------------------------------------------------------------------*/
  0x05, 0x20,             /* UsagePage (Sensor)                    */
  0x09, 0x80,             /* Usage     (Orientation)               */
  0xA1, 0x01,             /* Collection (Application)              */

    0x85, 0x01,           /*   Report ID (1)                       */

    /* --- quaternion : 4 × 16-bit -------------------------------------- */
    0x0A, 0x83, 0x04,     /*   Usage 0x0483 – Quaternion           */
    0x75, 0x10,           /*   ReportSize 16                       */
    0x95, 0x04,           /*   ReportCount 4                       */
    0x17, 0x00,0x00,0x00,0x00, /* Logical Min 0                    */
    0x27, 0xFF,0xFF,0x00,0x00, /* Logical Max 65535                */
    0x81, 0x02,           /*   Input (Data,Var,Abs)                */

    /* --- two switch bits on the Button page --------------------------- */
    0x05, 0x09,           /*   UsagePage (Button)                  */
    0x19, 0x01, 0x29, 0x02, /* Usage Min/Max (Button 1-2)         */
    0x95, 0x02, 0x75, 0x01, /* ReportCount 2, ReportSize 1        */
    0x15, 0x00, 0x25, 0x01, /* Logical 0-1                        */
    0x81, 0x02,           /*   Input (Data,Var,Abs)                */

    /* --- six padding bits --------------------------------------------- */
    0x95, 0x06, 0x75, 0x01,
    0x81, 0x03,           /*   Input (Cnst,Var,Abs)                */

    /* ------------------------------------------------------------------
     * Vendor-defined channel : Output (1 byte)
     * ---------------------------------------------------------------- */
    0x06, 0x00, 0xFF,     /*   UsagePage (Vendor 0xFF00)           */
    0x09, 0x01,           /*   Usage      (Vendor 1)               */
    0x15, 0x00, 0x26, 0xFF, 0x00,   /* Logical 0-255               */
    0x75, 0x08, 0x95, 0x01,         /* ReportSize 8, Count 1       */
    0x91, 0x02,           /*   Output (Data,Var,Abs)               */

    /* ------------------------------------------------------------------
     * Vendor-defined Feature report : saved RGB (3 bytes)
     * ---------------------------------------------------------------- */
    0x09, 0x02,           /*   Usage (Vendor 2)                    */
    0x95, 0x03,           /*   ReportCount 3                       */
    0xB1, 0x02,           /*   Feature (Data,Var,Abs)              */

  0xC0                  /* End Collection                         */
};

// HID report map - now 9 bytes total (8 bytes for quaternion + 1 byte for switch states)
uint8_t report_data[9] = {0};

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

// Server callbacks
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("Client connected!");
        Serial.println("Connection established successfully");
        Serial.println("CALLBACK: onConnect fired!");
        deviceConnected = true;
        
        // Stop advertising when connected to prevent conflicts
        if (NimBLEDevice::getAdvertising()->isAdvertising()) {
            Serial.println("Stopping advertising after connection");
            NimBLEDevice::getAdvertising()->stop();
        }
        
        // Request stable connection parameters to prevent disconnections
        NimBLEConnInfo connInfo = pServer->getPeerInfo(0);
        Serial.println("Requesting stable connection parameters...");
        // Use conservative parameters: 12-24ms interval, latency 0, timeout 400ms
        pServer->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
    };

    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected");
        Serial.println("Connection terminated");
        Serial.println("CALLBACK: onDisconnect fired!");
        deviceConnected = false;
        
        // Immediately restart advertising when disconnected
        Serial.println("Disconnect callback: restarting advertising");
        NimBLEDevice::startAdvertising();
    };
    
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.print("MTU changed to: ");
        Serial.println(MTU);
    }
    
    // Security callbacks moved to ServerCallbacks in NimBLE 2.0+
    void onPassKeyDisplay(uint32_t pass_key) {
        Serial.print("Passkey Display: ");
        Serial.println(pass_key);
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) {
        Serial.println("Authentication Complete");
        Serial.print("Secure: ");
        Serial.println(connInfo.isEncrypted() ? "Yes" : "No");
        
        // Print connection parameters for debugging
        Serial.print("Connection interval: ");
        Serial.print(connInfo.getConnInterval() * 1.25);
        Serial.println("ms");
        Serial.print("Connection latency: ");
        Serial.println(connInfo.getConnLatency());
        Serial.print("Supervision timeout: ");
        Serial.print(connInfo.getConnTimeout() * 10);
        Serial.println("ms");
    }
};

// Output report callback
class OutputReportCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, const std::string& value) {
        if (!value.empty()) {
            uint8_t cmd = static_cast<uint8_t>(value[0]);
            Serial.print("Output report received, cmd=0x");
            Serial.println(cmd, HEX);

            if (cmd == 0x01) {
                if (imu.isAvailable()) {
                    Serial.println("Reset command: resetting BNO085");
                    // Reset BNO085 using the library
                    imu.reset();
                    startIMUResetPattern(); // Start IMU reset LED pattern
                } else {
                    Serial.println("Reset command received, but BNO085 is not available");
                }
            }
        }
    }
};

// GATT Calibration characteristic write callback
class CalibrationCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr) {
        std::string value = chr->getValue();
        if (value.empty()) return;
        
        uint8_t cmd = static_cast<uint8_t>(value[0]);
        switch (cmd) {
            case 0x01: { // Reset/calibrate IMU
                Serial.println("GATT: IMU calibration requested");
                if (imu.isAvailable()) {
                    imu.reset();
                    startIMUResetPattern(); // Start IMU reset LED pattern
                } else {
                    Serial.println("GATT: IMU calibration requested, but BNO085 is not available");
                }
                
                // Send acknowledgment
                uint8_t ack = 0x01;
                calibrationChar->setValue(&ack, 1);
                break;
            }
            default:
                Serial.print("GATT: Unknown calibration command 0x");
                Serial.println(cmd, HEX);
                break;
        }
    }
};

// GATT Quaternion characteristic callback
class QuaternionCharCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pChar) {
        Serial.println("GATT: Quaternion characteristic read request");
    }
    
    void onWrite(NimBLECharacteristic* pChar) {
        Serial.println("GATT: Quaternion characteristic write request (unexpected)");
    }
    
    void onNotify(NimBLECharacteristic* pChar) {
        Serial.println("GATT: Quaternion notification sent");
    }
    
    void onStatus(NimBLECharacteristic* pChar, int status, int code) {
        Serial.print("GATT: Quaternion characteristic status: ");
        Serial.print(status);
        Serial.print(" code: ");
        Serial.println(code);
    }
    
    void onSubscribe(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc, uint16_t subValue) {
        Serial.print("GATT: Quaternion notifications ");
        Serial.print(subValue == 0 ? "disabled" : "enabled");
        Serial.print(" for connection: ");
        Serial.println(desc->conn_handle);
        
        // Update global subscription status
        quaternionSubscribed = (subValue != 0);
    }
};

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
            report_data[8] = 0; // No switch states
            
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
            report_data[8] = 0; // No switch states
            
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

// Helper to generate unique BLE name
static std::string generateUniqueName() {
    NimBLEAddress addr = NimBLEDevice::getAddress();
    std::string mac = addr.toString();
    std::string suffix;
    for (int i = mac.size() - 2; i >= 0 && suffix.size() < 4; --i) {
        if (mac[i] != ':') suffix.insert(suffix.begin(), (char)toupper(mac[i]));
    }
    char name[32];
    snprintf(name, sizeof(name), "Eidon Tracker-%s", suffix.c_str());
    return std::string(name);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n----- Eidon Tracker Starting -----");
    
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
    
    // Generate unique device name
    std::string deviceName = generateUniqueName();
    NimBLEDevice::setDeviceName(deviceName);
    
    Serial.print("Advertising as: ");
    Serial.println(deviceName.c_str());
    
    // Configure security for reliable pairing - simplified for NimBLE 2.0+
    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    
    // Set consistent power level
    NimBLEDevice::setPower(9); // Use integer value instead of ESP_PWR_LVL_P9
    
    // Create server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create HID device
    hid = new NimBLEHIDDevice(pServer);
    inputReport = hid->getInputReport(1);
    outputReport = hid->getOutputReport(1);
    outputReport->setCallbacks(new OutputReportCallbacks());
    
    // Configure HID device
    hid->setManufacturer("Eidon AI");
    hid->setPnp(0x02, VENDOR_ID, PRODUCT_ID, 0x0110);
    hid->setHidInfo(0x00, 0x01);
    hid->setReportMap((uint8_t*)hid_report_descriptor, sizeof(hid_report_descriptor));
    
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
    Serial.println("GATT: Quaternion characteristic created");
    
    // Add callback to track subscription status
    quaternionChar->setCallbacks(new QuaternionCharCallbacks());
    
    // Configure Calibration characteristic
    calibrationChar = eidonService->createCharacteristic(
        CALIBRATION_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
    );
    calibrationChar->setCallbacks(new CalibrationCallbacks());
    Serial.println("GATT: Calibration characteristic created");
    
    // Configure Device Info characteristic
    deviceInfoChar = eidonService->createCharacteristic(
        DEVICE_INFO_CHAR_UUID,
        NIMBLE_PROPERTY::READ
    );
    Serial.println("GATT: Device Info characteristic created");
    
    // Set initial device info
    uint8_t deviceInfo[8] = {
        0x01, 0x00,  // Device ID
        0x01, 0x02,  // Firmware version 1.2
        100,         // Battery level
        0, 0, 0      // Reserved
    };
    deviceInfoChar->setValue(deviceInfo, sizeof(deviceInfo));
    
    // Start custom service
    eidonService->start();
    Serial.println("GATT: Eidon service started");
    
    // Configure advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setAppearance(HID_GAMEPAD);
    pAdvertising->addServiceUUID(hid->getHidService()->getUUID());
    pAdvertising->addServiceUUID(eidonService->getUUID());
    Serial.println("GATT: Added services to advertising");
    
    // Set conservative advertising intervals for stable connection
    pAdvertising->setMinInterval(160);  // 100ms minimum (more conservative)
    pAdvertising->setMaxInterval(320);  // 200ms maximum (more conservative)
    
    // Create scan response data
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(deviceName);
    pAdvertising->setScanResponseData(scanResponse);
    Serial.println("GATT: Scan response configured");
    
    // Start advertising
    pAdvertising->start();
    Serial.println("GATT: Advertising started");
    Serial.print("GATT: Device name: ");
    Serial.println(deviceName.c_str());
    Serial.print("GATT: Service UUID: ");
    Serial.println(EIDON_SERVICE_UUID);
    
    Serial.println("Setup complete");
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
            Serial.println("Connected - starting to send data");
            // Force reset LED state and start fresh pattern
            digitalWrite(LED_PIN, LOW);  // Start with LED OFF
            ledState = false;
            ledLastUpdate = 0; // Force immediate update
            currentLEDPattern = LED_CONNECTED;
        } else {
            Serial.println("Disconnected - restarting advertising");
            NimBLEDevice::startAdvertising();
            currentLEDPattern = LED_ADVERTISING; // Switch to advertising LED pattern
        }
    }
    
    // Read sensor data directly (polling-based instead of interrupt-driven)
    imu.update();;
    
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
        
        // Add basic debug output every 30 seconds (increased from 10 seconds)
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint >= 30000) {
            Serial.print("DEBUG: Connected="); Serial.print(deviceConnected);
            Serial.print(", BNO085_available="); Serial.print(imu.isAvailable());
            Serial.print(", quaternion: W="); Serial.print(quaternion_w, 4);
            Serial.print(" X="); Serial.print(quaternion_x, 4);
            Serial.print(" Y="); Serial.print(quaternion_y, 4);
            Serial.print(" Z="); Serial.print(quaternion_z, 4);
            Serial.println();
            lastDebugPrint = millis();
        }
    } else {
        // Add debug output when not connected - every 60 seconds (increased from 30 seconds)
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint >= 60000) {
            Serial.println("DEBUG: Not connected, waiting for client...");
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
