#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEHIDDevice.h>
#include <NimBLECharacteristic.h>
#include <Adafruit_BNO08x.h>
#include <Preferences.h>
#include "BNO085.h"

// Function declarations
void scanI2C();
bool testBNO085Direct();
bool initIMU();
void setReports();
void updateOrientation();
void sendQuaternionReport();
void readSwitches();
void updateLEDStatus();
void startIMUResetPattern();
bool isBNO085Available();
void quaternionToEuler();
void updateBNO085();
bool isConnected();

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
#define COLOR_CHAR_UUID           "E1D00004-8B5A-3E5B-9E23-4F9B5C91BBDE"
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

// BNO085 sensor
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Add BNO085 availability tracking
bool bno085_available = false;  // Track if BNO085 is working

// Orientation data
float quaternion_x = 0;
float quaternion_y = 0;
float quaternion_z = 0;
float quaternion_w = 1;

// Add euler angles structure for quaternion conversion
euler_t ypr = {0, 0, 0};

// Device color RGB stored (default white)
uint8_t device_color[3] = {0xFF, 0xFF, 0xFF};

// Update interval (milliseconds)
const unsigned long UPDATE_INTERVAL = 1;
unsigned long lastUpdate = 0;

// Add rate limiting for BLE notifications
const unsigned long BLE_NOTIFICATION_INTERVAL = 50; // Send every 50ms (20Hz) instead of every 1ms
unsigned long lastNotificationTime = 0;

// Track subscription status
bool quaternionSubscribed = false;

// Switch pins
#define SWITCH_OUT_LEFT_RIGHT 5  // D5 output for left/right switch
#define SWITCH_IN_LEFT_RIGHT 6   // D6 input for left/right switch
#define SWITCH_OUT_UPPER_LOWER 0 // D0 output for upper/lower switch
#define SWITCH_IN_UPPER_LOWER 1  // D1 input for upper/lower switch

// Switch states
bool isLeft = false;
bool isUpper = false;

// LED pin
#define LED_PIN 5

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
bool oldDeviceConnected = false;

// Add connection attempt tracking
static unsigned long connectionAttempts = 0;
static unsigned long lastConnectionCheck = 0;

// Add more robust connection state tracking
static unsigned long lastConnectionStateCheck = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 100; // Check every 100ms instead of 5 seconds

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
    
    // Only handle advertising when not connected
    if (currentLEDPattern == LED_ADVERTISING) {
        // Strobing brightness pattern (sine wave)
        if (currentTime - ledLastUpdate >= 20) { // Update every 20ms for smooth animation
            ledLastUpdate = currentTime;
            // Create a sine wave brightness pattern (0-255)
            ledBrightness = 128 + 127 * sin(currentTime / 500.0);
            analogWrite(LED_PIN, ledBrightness);
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
NimBLECharacteristic* featureReport = nullptr;

// Custom GATT Service
NimBLEService* eidonService = nullptr;
NimBLECharacteristic* quaternionChar = nullptr;
NimBLECharacteristic* calibrationChar = nullptr;
NimBLECharacteristic* colorChar = nullptr;
NimBLECharacteristic* deviceInfoChar = nullptr;

// Persistent storage
Preferences prefs;

// Server callbacks
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("Client connected!");
        Serial.println("Connection established successfully");
        Serial.println("CALLBACK: onConnect fired!");
        deviceConnected = true;
    };

    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected");
        Serial.println("Connection terminated");
        Serial.println("CALLBACK: onDisconnect fired!");
        deviceConnected = false;
    };
    
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.print("MTU changed to: ");
        Serial.println(MTU);
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
                Serial.println("Reset command: resetting BNO085");
                // Reset BNO085
                bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
                delay(100);
                bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000);
                startIMUResetPattern(); // Start IMU reset LED pattern
            }
        }
    }
};

// Feature report callback
class FeatureReportCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pChar, const std::string& value) {
        // Manually prepend report ID 1 to the color data
        uint8_t reportData[4];
        reportData[0] = 0x01;  // Report ID
        memcpy(reportData + 1, device_color, 3);
        pChar->setValue(reportData, 4);
    }
    
    void onWrite(NimBLECharacteristic* pChar, const std::string& value) {
        size_t len = value.size();
        if (len == 4) {
            // Host included Report ID as first byte – skip it
            memcpy(device_color, value.data() + 1, 3);
        } else if (len >= 3) {
            memcpy(device_color, value.data(), 3);
  } else {
            return; // invalid length
        }

        // Persist to NVS
        prefs.putBytes("color", device_color, 3);
        Serial.printf("Color updated to %02X %02X %02X and saved to flash\n", device_color[0], device_color[1], device_color[2]);
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
                bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
                delay(100);
                bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000);
                startIMUResetPattern(); // Start IMU reset LED pattern
                
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

// GATT Color characteristic write callback
class ColorCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr) {
        std::string value = chr->getValue();
        if (value.size() != 3) return;  // Expect RGB values
        
        // Store color
        memcpy(device_color, value.data(), 3);
        prefs.putBytes("color", device_color, 3);
        
        // Update the characteristic value
        colorChar->setValue(device_color, 3);
        
        Serial.print("GATT: Color set to #");
        for (uint8_t i = 0; i < 3; ++i) {
            if (device_color[i] < 16) Serial.print('0');
            Serial.print(device_color[i], HEX);
        }
        Serial.println();
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

void setReports() {
    // Enable game rotation vector at maximum rate (1000 Hz)
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000)) {
        Serial.println("Could not enable rotation vector");
    }

    // Enable Tap Detector (event-driven, report interval 0)
    if (!bno08x.enableReport(SH2_TAP_DETECTOR, 0)) {
        Serial.println("Could not enable tap detector");
    }
}

// Add I2C scanning function
void scanI2C() {
    Serial.println("Scanning I2C bus...");
    Serial.print("SDA: GPIO"); Serial.print(I2C_SDA);
    Serial.print(", SCL: GPIO"); Serial.println(I2C_SCL);
    
    int nDevices = 0;
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        }
        else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    } else {
        Serial.print("Found ");
        Serial.print(nDevices);
        Serial.println(" device(s)");
    }
    Serial.println("Scan complete\n");
}

// Add direct BNO085 communication test function
bool testBNO085Direct() {
    Serial.println("Testing direct BNO085 communication...");
    
    // BNO085 uses SHTP (Sensor Hub Transport Protocol)
    // Try to read the SHTP header to see if we can communicate
    Wire.requestFrom(I2C_ADDR, 4); // Request 4 bytes (SHTP header)
    
    if (Wire.available() >= 4) {
        byte header[4];
        for (int i = 0; i < 4; i++) {
            header[i] = Wire.read();
        }
        
        Serial.print("SHTP Header received: ");
        for (int i = 0; i < 4; i++) {
            if (header[i] < 16) Serial.print("0");
            Serial.print(header[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        // Check if this looks like a valid SHTP header
        uint16_t length = (header[1] << 8) | header[0];
        Serial.print("Packet length: ");
        Serial.println(length);
        
        if (length > 0 && length < 1000) { // Reasonable packet length
            Serial.println("Valid SHTP communication detected!");
            return true;
        } else {
            Serial.println("SHTP header format unexpected");
            return false;
        }
    } else {
        Serial.print("Only ");
        Serial.print(Wire.available());
        Serial.println(" bytes available from SHTP request");
        return false;
    }
}

bool initIMU() {
    Serial.println("Setting up I2C pins...");
    Serial.print("SDA: GPIO"); Serial.print(I2C_SDA);
    Serial.print(" (D9), SCL: GPIO"); Serial.print(I2C_SCL);
    Serial.println(" (D10)");
    
    // Initialize I2C with explicit pins for ESP32-C6
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    Wire.setClock(100000); // Set to 100kHz for better reliability
    
    Serial.println("I2C initialized, testing basic communication...");
    
    // Test basic I2C communication with retries
    int i2c_attempts = 0;
    bool i2c_success = false;
    
    while (!i2c_success && i2c_attempts < 5) {
        Wire.beginTransmission(I2C_ADDR);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.println("I2C communication successful!");
            i2c_success = true;
        } else {
            i2c_attempts++;
            Serial.print("I2C attempt "); Serial.print(i2c_attempts);
            Serial.print(" failed with error: "); Serial.println(error);
            if (i2c_attempts < 5) {
                Serial.println("Retrying I2C in 500ms...");
                delay(500);
            }
        }
    }
    
    if (!i2c_success) {
        Serial.println("I2C communication failed after 5 attempts, giving up on BNO085");
        return false;
    }
    
    // Test direct BNO085 communication
    if (testBNO085Direct()) {
        Serial.println("Direct BNO085 SHTP communication successful!");
    } else {
        Serial.println("Direct BNO085 SHTP communication failed!");
        // Don't return here - still try the Adafruit library
    }
    
    Serial.println("Initializing BNO085 with Adafruit library...");
    
    // Scan the I2C bus
    scanI2C();
    
    // Try multiple initialization approaches with retries
    bool bno_initialized = false;
    int total_attempts = 0;
    const int max_attempts = 15;
    
    while (!bno_initialized && total_attempts < max_attempts) {
        total_attempts++;
        
        Serial.print("BNO085 initialization attempt "); 
        Serial.print(total_attempts); 
        Serial.print("/"); 
        Serial.println(max_attempts);
        
        // Try different initialization methods
        if (total_attempts <= 5) {
            // First 5 attempts: Try with explicit Wire object
            Serial.println("  -> Trying with explicit Wire object...");
            if (bno08x.begin_I2C(I2C_ADDR, &Wire)) {
                Serial.println("BNO085 initialized successfully with explicit Wire object!");
                bno_initialized = true;
                break;
            }
        } else if (total_attempts <= 10) {
            // Next 5 attempts: Try with default address
            Serial.println("  -> Trying with default address...");
            if (bno08x.begin_I2C()) {
                Serial.println("BNO085 initialized successfully with default address!");
                bno_initialized = true;
                break;
            }
        } else {
            // Final attempts: Try with different I2C speeds
            if (total_attempts == 11) {
                Serial.println("  -> Trying with 400kHz I2C clock...");
                Wire.setClock(400000);
            } else if (total_attempts == 13) {
                Serial.println("  -> Trying with 50kHz I2C clock...");
                Wire.setClock(50000);
            }
            
            Serial.println("  -> Trying standard initialization...");
            if (bno08x.begin_I2C(I2C_ADDR)) {
                Serial.println("BNO085 initialized successfully!");
                bno_initialized = true;
                break;
            }
        }
        
        // Wait between attempts, with longer waits for later attempts
        int delay_ms = (total_attempts <= 5) ? 500 : 1000;
        Serial.print("  -> Failed, waiting "); Serial.print(delay_ms); Serial.println("ms...");
        delay(delay_ms);
        
        // Rescan I2C bus every 5 attempts to verify device is still there
        if (total_attempts % 5 == 0) {
            Serial.println("  -> Rescanning I2C bus...");
            scanI2C();
            
            // Reset I2C if device is not responding
            Wire.beginTransmission(I2C_ADDR);
            if (Wire.endTransmission() != 0) {
                Serial.println("  -> Device not responding, reinitializing I2C...");
                Wire.end();
                delay(100);
                Wire.setPins(I2C_SDA, I2C_SCL);
                Wire.begin();
                Wire.setClock(100000);
            }
        }
    }
    
    if (!bno_initialized) {
        Serial.print("BNO085 failed to initialize after ");
        Serial.print(max_attempts);
        Serial.println(" attempts. Continuing without IMU...");
        return false;
    }

    Serial.print("BNO085 successfully initialized on attempt ");
    Serial.print(total_attempts);
    Serial.println("!");

    // Enable the rotation vector report
    setReports();
    
    // Set availability flag
    bno085_available = true;

    return true;
}

// Function to check if BNO085 is available
bool isBNO085Available() {
    return bno085_available;
}

// Function to get current BLE connection state (similar to Bluefruit.connected())
bool isConnected() {
    if (pServer != nullptr) {
        return (pServer->getConnectedCount() > 0);
    }
    return deviceConnected; // Fallback to tracked state
}

void updateOrientation() {
    // Don't try to update if BNO085 is not available
    if (!bno085_available) {
        return;
    }
    
    if (bno08x.wasReset()) {
        Serial.println("BNO085 was reset");
        setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
                quaternion_x = sensorValue.un.gameRotationVector.i;
                quaternion_y = sensorValue.un.gameRotationVector.j;
                quaternion_z = sensorValue.un.gameRotationVector.k;
                quaternion_w = sensorValue.un.gameRotationVector.real;
                break;

            case SH2_TAP_DETECTOR: {
                uint8_t f = sensorValue.un.tapDetector.flags;
                bool isDouble = f & TAPDET_DOUBLE;
                Serial.print(isDouble ? "Double" : "Single");
                Serial.print(" tap detected on ");
                if      (f & TAPDET_X)     Serial.println("-X side");
                else if (f & TAPDET_X_POS) Serial.println("+X side");
                else if (f & TAPDET_Y)     Serial.println("-Y side");
                else if (f & TAPDET_Y_POS) Serial.println("+Y side");
                else if (f & TAPDET_Z)     Serial.println("-Z side");
                else if (f & TAPDET_Z_POS) Serial.println("+Z side");
                else                       Serial.println("unknown side");

                if (isDouble) {
                    Serial.println("Double tap detected");
                    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
                    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
                }
                break;
            }
        }
    }
}

void sendQuaternionReport() {
    // Apply 180-degree rotation around Z-axis to correct for IMU mounting
    float qw_sensor = quaternion_w;
    float qx_sensor = quaternion_x;
    float qy_sensor = quaternion_y;
    float qz_sensor = quaternion_z;
    
    // Apply 180-degree Z-rotation by negating X and Y components
    float corrected_w = qw_sensor;
    float corrected_x = -qx_sensor;
    float corrected_y = -qy_sensor;
    float corrected_z = qz_sensor;
    
    // Prepare switch states
    uint8_t switch_states = 0;
    if (isLeft) switch_states |= 0x01;
    if (isUpper) switch_states |= 0x02;
    
    // Send via HID if connected
    if (isConnected()) {
        // Only send quaternion data if BNO085 is available
        if (bno085_available) {
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
            report_data[8] = switch_states;
            
            // Send HID report
            if (inputReport != nullptr) {
                inputReport->setValue(report_data, sizeof(report_data));
                inputReport->notify();
            }
            
            // Also send via GATT service
            gattQuaternionData.w = corrected_w;
            gattQuaternionData.x = corrected_x;
            gattQuaternionData.y = corrected_y;
            gattQuaternionData.z = corrected_z;
            gattQuaternionData.switches = switch_states;
            memset(gattQuaternionData.reserved, 0, sizeof(gattQuaternionData.reserved));
            
            // Send GATT notification
            if (quaternionChar != nullptr) {
                // Rate limit notifications to prevent overwhelming the BLE stack
                unsigned long currentTime = millis();
                if (currentTime - lastNotificationTime >= BLE_NOTIFICATION_INTERVAL) {
                    bool notifyResult = quaternionChar->notify((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
                    lastNotificationTime = currentTime;
                    
                    static unsigned long lastDebugPrint = 0;
                    if (currentTime - lastDebugPrint >= 1000) { // Print every second
                        Serial.print("GATT: Quaternion notification sent, result=");
                        Serial.print(notifyResult);
                        Serial.print(", subscribed="); Serial.print(quaternionSubscribed ? "YES" : "NO");
                        Serial.print(", data: W="); Serial.print(corrected_w, 4);
                        Serial.print(" X="); Serial.print(corrected_x, 4);
                        Serial.print(" Y="); Serial.print(corrected_y, 4);
                        Serial.print(" Z="); Serial.print(corrected_z, 4);
                        Serial.print(" switches=0x"); Serial.print(switch_states, HEX);
                        Serial.print(", data_size="); Serial.print(sizeof(gattQuaternionData));
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
            report_data[8] = switch_states; // Still send switch states
            
            if (inputReport != nullptr) {
                inputReport->setValue(report_data, sizeof(report_data));
                inputReport->notify();
            }
            
            // Also send zero quaternion via GATT
            gattQuaternionData.w = 1.0f;  // Identity quaternion
            gattQuaternionData.x = 0.0f;
            gattQuaternionData.y = 0.0f;
            gattQuaternionData.z = 0.0f;
            gattQuaternionData.switches = switch_states;
            memset(gattQuaternionData.reserved, 0, sizeof(gattQuaternionData.reserved));
            
            if (quaternionChar != nullptr) {
                quaternionChar->notify((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
                static unsigned long lastDebugPrint = 0;
                if (millis() - lastDebugPrint >= 1000) {
                    Serial.println("GATT: Sent zero quaternion (BNO085 not available)");
                    lastDebugPrint = millis();
                }
            }
        }
        
        // LED is now controlled by updateLEDStatus() in the main loop
    }
}

// Function to read switch states
void readSwitches() {
    // Read left/right switch
    pinMode(SWITCH_OUT_LEFT_RIGHT, OUTPUT);
    digitalWrite(SWITCH_OUT_LEFT_RIGHT, HIGH);
    delayMicroseconds(10);
    pinMode(SWITCH_IN_LEFT_RIGHT, INPUT_PULLDOWN);
    isLeft = digitalRead(SWITCH_IN_LEFT_RIGHT) == HIGH;
    pinMode(SWITCH_OUT_LEFT_RIGHT, INPUT);
    
    // Read upper/lower switch
    pinMode(SWITCH_OUT_UPPER_LOWER, OUTPUT);
    digitalWrite(SWITCH_OUT_UPPER_LOWER, HIGH);
    delayMicroseconds(10);
    pinMode(SWITCH_IN_UPPER_LOWER, INPUT_PULLDOWN);
    isUpper = digitalRead(SWITCH_IN_UPPER_LOWER) == HIGH;
    pinMode(SWITCH_OUT_UPPER_LOWER, INPUT);
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
    if (!initIMU()) {
        Serial.println("Failed to initialize IMU!");
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
    
    // Remove interrupt setup - using polling instead
    // pinMode(BNO085_INT_PIN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(BNO085_INT_PIN), sensorISR, FALLING);
    
    // Initialize Bluetooth
    NimBLEDevice::init("");
    
    // Generate unique device name
    std::string deviceName = generateUniqueName();
    NimBLEDevice::setDeviceName(deviceName);
    
    Serial.print("Advertising as: ");
    Serial.println(deviceName.c_str());
    
    // Create server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create HID device
    hid = new NimBLEHIDDevice(pServer);
    inputReport = hid->getInputReport(1);
    outputReport = hid->getOutputReport(1);
    outputReport->setCallbacks(new OutputReportCallbacks());
    
    // Feature report for RGB color
    featureReport = hid->getFeatureReport(1);
    featureReport->setCallbacks(new FeatureReportCallbacks());
    
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
    
    // Configure Color characteristic
    colorChar = eidonService->createCharacteristic(
        COLOR_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
    );
    colorChar->setCallbacks(new ColorCallbacks());
    Serial.println("GATT: Color characteristic created");
    
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
    
    // Open NVS and load saved color
    prefs.begin("tracker", false);
    if (prefs.getBytes("color", device_color, 3) != 3) {
        device_color[0] = 0xFF;
        device_color[1] = 0xFF;
        device_color[2] = 0xFF;
    }
    
    // Set initial color values
    featureReport->setValue(device_color, 3);
    colorChar->setValue(device_color, 3);
    
    // Configure advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setAppearance(HID_GAMEPAD);
    pAdvertising->addServiceUUID(hid->getHidService()->getUUID());
    pAdvertising->addServiceUUID(eidonService->getUUID());
    Serial.println("GATT: Added services to advertising");
    
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
    
    // Initialize switch pins
    pinMode(SWITCH_OUT_LEFT_RIGHT, INPUT);
    pinMode(SWITCH_IN_LEFT_RIGHT, INPUT_PULLDOWN);
    pinMode(SWITCH_OUT_UPPER_LOWER, INPUT);
    pinMode(SWITCH_IN_UPPER_LOWER, INPUT_PULLDOWN);
}

void loop() {
    // Update LED status first
    updateLEDStatus();
    
    // Add BLE stack debugging every 2 seconds
    static unsigned long lastBleDebug = 0;
    if (millis() - lastBleDebug >= 2000) {
        Serial.print("BLE DEBUG: deviceConnected="); Serial.print(deviceConnected);
        Serial.print(", isConnected()="); Serial.print(isConnected());
        Serial.print(", oldDeviceConnected="); Serial.print(oldDeviceConnected);
        Serial.print(", advertising="); Serial.print(NimBLEDevice::getAdvertising()->isAdvertising());
        Serial.print(", server="); Serial.print(pServer != nullptr ? "OK" : "NULL");
        Serial.print(", eidonService="); Serial.print(eidonService != nullptr ? "OK" : "NULL");
        Serial.print(", quaternionChar="); Serial.print(quaternionChar != nullptr ? "OK" : "NULL");
        Serial.print(", connectionAttempts="); Serial.print(connectionAttempts);
        Serial.println();
        lastBleDebug = millis();
    }
    
    // Check for connection attempts (simple heuristic)
    if (!deviceConnected && millis() - lastConnectionCheck >= 1000) {
        // If advertising stops briefly, it might be a connection attempt
        static bool wasAdvertising = true;
        bool isAdvertising = NimBLEDevice::getAdvertising()->isAdvertising();
        
        if (wasAdvertising && !isAdvertising) {
            connectionAttempts++;
            Serial.print("BLE: Possible connection attempt detected! #");
            Serial.println(connectionAttempts);
        }
        wasAdvertising = isAdvertising;
        lastConnectionCheck = millis();
    }
    
    // Robust connection state checking - handle NimBLE callback issues
    if (millis() - lastConnectionStateCheck >= CONNECTION_CHECK_INTERVAL) {
        lastConnectionStateCheck = millis();
        
        // Get actual connection state from server
        bool serverHasConnections = false;
        if (pServer != nullptr) {
            serverHasConnections = (pServer->getConnectedCount() > 0);
        }
        
        // Update deviceConnected state based on server state
        if (serverHasConnections != deviceConnected) {
            if (serverHasConnections && !deviceConnected) {
                // Connection established but callback didn't fire
                Serial.println("BLE: Connection detected via server state check");
                Serial.println("BLE: NimBLE callback issue detected - connection established");
                deviceConnected = true;
                oldDeviceConnected = false; // Force connection change detection
            } else if (!serverHasConnections && deviceConnected) {
                // Disconnection detected
                Serial.println("BLE: Disconnection detected via server state check");
                deviceConnected = false;
                oldDeviceConnected = true; // Force disconnection change detection
            }
        }
    }
    
    // Handle connection state changes
    if (isConnected() && !oldDeviceConnected) {
        Serial.println("Connected - starting to send data");
        oldDeviceConnected = true;
        
        // Force reset LED state and start fresh pattern
        digitalWrite(LED_PIN, LOW);  // Start with LED OFF
        ledState = false;
        ledLastUpdate = 0; // Force immediate update
        currentLEDPattern = LED_CONNECTED;
        Serial.println("Switching to CONNECTED LED pattern - should flash dimmed");
    }
    
    if (!isConnected() && oldDeviceConnected) {
        Serial.println("Disconnected - restarting advertising");
        NimBLEDevice::startAdvertising();
        oldDeviceConnected = false;
        currentLEDPattern = LED_ADVERTISING; // Switch to advertising LED pattern
    }
    
    // Read sensor data directly (polling-based instead of interrupt-driven)
    updateBNO085();
    
    // Send quaternion report if connected
    if (isConnected()) {
        sendQuaternionReport();
        
        // Add basic debug output every 2 seconds
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint >= 2000) {
            Serial.print("DEBUG: Connected="); Serial.print(isConnected());
            Serial.print(", BNO085_available="); Serial.print(bno085_available);
            Serial.print(", quaternionChar="); Serial.print(quaternionChar != nullptr ? "OK" : "NULL");
            Serial.print(", quaternion: W="); Serial.print(quaternion_w, 4);
            Serial.print(" X="); Serial.print(quaternion_x, 4);
            Serial.print(" Y="); Serial.print(quaternion_y, 4);
            Serial.print(" Z="); Serial.print(quaternion_z, 4);
            Serial.println();
            lastDebugPrint = millis();
        }
    } else {
        // Add debug output when not connected
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint >= 5000) { // Every 5 seconds when not connected
            Serial.println("DEBUG: Not connected, waiting for client...");
            lastDebugPrint = millis();
        }
    }
    
    // Read switch states
    readSwitches();
    
    // Small delay
    delay(1);
}

// Function to convert quaternion to euler angles
void quaternionToEuler() {
    float sqr = sq(quaternion_w);
    float sqi = sq(quaternion_x);
    float sqj = sq(quaternion_y);
    float sqk = sq(quaternion_z);

    ypr.yaw = asin(-2.0 * (quaternion_x * quaternion_z - quaternion_y * quaternion_w) /
                     (sqi + sqj + sqk + sqr));
    ypr.pitch = atan2(2.0 * (quaternion_x * quaternion_y + quaternion_z * quaternion_w),
                    (sqi - sqj - sqk + sqr));
    ypr.roll = atan2(2.0 * (quaternion_y * quaternion_z + quaternion_x * quaternion_w),
                     (-sqi - sqj + sqk + sqr));

    // Convert to degrees
    ypr.yaw = ypr.yaw * RAD_TO_DEG;
    ypr.pitch = ypr.pitch * RAD_TO_DEG;
    ypr.roll = ypr.roll * RAD_TO_DEG;
}

// Function to update BNO085 sensor data
void updateBNO085() {
    // Don't try to update if BNO085 is not available
    if (!bno085_available) {
        return;
    }
    
    static unsigned long lastPrint = 0;
    const unsigned long PRINT_INTERVAL = 500; // Print every 500ms

    if (bno08x.wasReset()) {
        Serial.println("BNO085 was reset");
        setReports();
    }
    
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
                quaternion_x = sensorValue.un.gameRotationVector.i;
                quaternion_y = sensorValue.un.gameRotationVector.j;
                quaternion_z = sensorValue.un.gameRotationVector.k;
                quaternion_w = sensorValue.un.gameRotationVector.real;
                break;

            case SH2_TAP_DETECTOR: {
                uint8_t f = sensorValue.un.tapDetector.flags;
                bool isDouble = f & TAPDET_DOUBLE;
                Serial.print(isDouble ? "Double" : "Single");
                Serial.print(" tap detected on ");
                if      (f & TAPDET_X)     Serial.println("-X side");
                else if (f & TAPDET_X_POS) Serial.println("+X side");
                else if (f & TAPDET_Y)     Serial.println("-Y side");
                else if (f & TAPDET_Y_POS) Serial.println("+Y side");
                else if (f & TAPDET_Z)     Serial.println("-Z side");
                else if (f & TAPDET_Z_POS) Serial.println("+Z side");
                else                       Serial.println("unknown side");

                if (isDouble) {
                    Serial.println("Double tap detected");
                    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
                    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
                }
                break;
            }
        }

        // Print quaternion values every PRINT_INTERVAL milliseconds
        if (millis() - lastPrint >= PRINT_INTERVAL) {
            Serial.print("Quaternion - X: "); Serial.print(quaternion_x, 4);
            Serial.print(" Y: "); Serial.print(quaternion_y, 4);
            Serial.print(" Z: "); Serial.print(quaternion_z, 4);
            Serial.print(" W: "); Serial.print(quaternion_w, 4);
            
            // Also show magnitude to verify it's normalized (should be ~1.0)
            float magnitude = sqrt(quaternion_x*quaternion_x + quaternion_y*quaternion_y + 
                                 quaternion_z*quaternion_z + quaternion_w*quaternion_w);
            Serial.print(" |Mag: "); Serial.print(magnitude, 4);
            Serial.println("|");
            
            lastPrint = millis();
        }
    }
}
