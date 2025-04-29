#include <bluefruit.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

// For the built-in LED
#define LED_PIN PIN_LED

// I2C pins for XIAO nRF52840 Sense
#define I2C_SDA 4
#define I2C_SCL 5

// BNO085 I2C address
#define BNO085_I2C_ADDR 0x4B

// Vendor and Product IDs
#define VENDOR_ID 0x303A  // Adafruit's vendor ID
#define PRODUCT_ID 0xABCE // Custom product ID for Eidon Tracker

// HID Report Descriptor for a custom device with 4 quaternion values
// Using a custom usage page to avoid keyboard/gamepad interpretation
uint8_t const hid_report_descriptor[] = {
  0x06, 0xFF, 0x00,  // Usage Page (Vendor Defined)
  0x09, 0x01,        // Usage (1)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x01,        //   Report ID (1)
  
  // 4 values for quaternion (w, x, y, z)
  0x09, 0x30,        //   Usage (X)
  0x09, 0x31,        //   Usage (Y)
  0x09, 0x32,        //   Usage (Z)
  0x09, 0x33,        //   Usage (W)
  0x15, 0x81,        //   Logical Minimum (-127)
  0x25, 0x7F,        //   Logical Maximum (127)
  0x75, 0x08,        //   Report Size (8)
  0x95, 0x04,        //   Report Count (4)
  0x81, 0x02,        //   Input (Data, Variable, Absolute)
  
  0xC0               // End Collection
};

// HID report map
uint8_t report_data[4] = {0};  // 4 values for quaternion

// BNO085 sensor
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Orientation data
struct euler_t {
    float yaw;
    float pitch;
    float roll;
} ypr = {0, 0, 0};

float quaternion_x = 0;
float quaternion_y = 0;
float quaternion_z = 0;
float quaternion_w = 1;

// Bluetooth HID
BLEDis bledis;
BLEHidGeneric hid(1, 0, 0);

// Battery Service
BLEBas blebas;

// Complementary filter variables
unsigned long prevTime = 0;

// Update interval (milliseconds)
const unsigned long UPDATE_INTERVAL = 1;
unsigned long lastUpdate = 0;

// Debug counter
unsigned long debugCounter = 0;
const unsigned long DEBUG_INTERVAL = 1000; // Print debug info every second

// Add these global variables
bool isMagCalibrated = false;
uint8_t magAccuracy = 0;

// Function to read battery voltage using internal ADC
float readVBAT(void) {
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
  
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12);
  
  // Read the internal voltage reference
  float vref = 3.0;
  float measuredvbat = analogReadVDD();
  
  // Convert the voltage to actual battery voltage
  measuredvbat *= vref;
  measuredvbat /= 4095.0F;
  measuredvbat *= 2;  // Multiply by 2 as voltage is divided by 2 internally
  
  return measuredvbat;
}

// Convert voltage to rough battery percentage
uint8_t mvToPercent(float voltage) {
  if(voltage < 3.3) return 0;
  if(voltage < 3.6) {
    voltage -= 3.3;
    return (voltage * 100) / 0.3;  // Linear from 3.3V to 3.6V
  }
  if(voltage < 4.2) {
    return 100;  // Consider full between 3.6V and 4.2V
  }
  return 100;
}

void enterDFU() {
    // Enter DFU mode
    #if defined(ARDUINO_NRF52_ADAFRUIT)
        enterOTADfu();
    #else
        NRF_POWER->GPREGRET = 0x01; // Set the GPREGRET register to indicate DFU mode
        NVIC_SystemReset();         // Perform a system reset
    #endif
}

void  fxx() {
    if (Serial.available()) {
        if (Serial.read() == 'D') {
            enterDFU();
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    // Wait up to 5 seconds for serial connection
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 5000)) {
        delay(100);
    }
    
    Serial.println("\n\n=== XIAO nRF52840 IMU Tracker Starting ===");
    
    // Check for DFU trigger command
    while (Serial.available()) {
        if (Serial.read() == 'D') {  // 'D' for DFU
            enterDFU();
        }
    }
    
    // Set the LED pin as output
    pinMode(LED_PIN, OUTPUT);

    // Wait for serial port to open (up to 2 seconds)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000));
    
    Serial.println("XIAO nRF52840 IMU Bluetooth Orientation Tracker");
    Serial.println("Using BNO085 sensor");
    
    // Initialize IMU
    if (!initIMU()) {
        Serial.println("Failed to initialize IMU!");
        // Flash LED rapidly to indicate error
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
    
    // Initialize Bluetooth
    Bluefruit.begin();
    
    // Set device name
    Bluefruit.setName("Eidon Tracker");
    
    // Configure and Start Device Information Service
    bledis.begin();
    bledis.setModel("Eidon Tracker");
    bledis.setManufacturer("Eidon");
    bledis.setHardwareRev("1.0");
    bledis.setFirmwareRev("1.0");
    bledis.setSerialNum("123456");
    bledis.setSystemID(VENDOR_ID, PRODUCT_ID);
    
    // Configure HID
    hid.enableKeyboard(false);  // Explicitly disable keyboard
    hid.enableMouse(false);     // Explicitly disable mouse
    
    // Set our custom report map (descriptor)
    hid.setReportMap(hid_report_descriptor, sizeof(hid_report_descriptor));
    
    // Set the length of our input report (4 bytes for quaternion)
    uint16_t input_len[] = {4};  // Length of our single input report
    hid.setReportLen(input_len, NULL, NULL);
    
    // Start HID Service
    hid.begin();
    
    // Initialize Battery Service
    blebas.begin();
    blebas.write(100);
    
    // Start advertising
    startAdv();
    
    Serial.println("Setup complete");
    
    // Initialize time for complementary filter
    prevTime = millis();
}

void loop() {
    // checkDFU();  // Check for DFU command
    // Update orientation at regular intervals
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        updateOrientation();
        sendQuaternionReport();
        lastUpdate = millis();
    }
    
    // Print debug info periodically
    if (millis() - debugCounter >= DEBUG_INTERVAL) {
        debugCounter = millis();
        Serial.print("Connected: ");
        Serial.println(Bluefruit.connected() ? "Yes" : "No");
        Serial.print("Report data: ");
        Serial.print(report_data[0]);
        Serial.print(", ");
        Serial.print(report_data[1]);
        Serial.print(", ");
        Serial.print(report_data[2]);
        Serial.print(", ");
        Serial.println(report_data[3]);
    }
    
    // updateBatteryLevel();
}

bool initIMU() {
    // Initialize I2C with explicit pins for XIAO nRF52840 Sense
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    
    // Try to initialize the BNO085
    if (!bno08x.begin_I2C(BNO085_I2C_ADDR)) {
        Serial.println("Failed to find BNO085 chip");
        return false;
    }
    
    Serial.println("BNO085 Found!");
    
    // Enable the rotation vector report
    setReports();
    
    // Print calibration instructions
    Serial.println("\nCalibration Instructions:");
    Serial.println("1. Wave the device in a figure-8 pattern");
    Serial.println("2. Rotate slowly through all orientations");
    Serial.println("3. Keep away from magnetic interference");
    Serial.println("4. Wait for 'Calibrated' message\n");
    
    return true;
}

void setReports() {
    // Use ROTATION_VECTOR instead of GAME_ROTATION_VECTOR for magnetic north reference
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 5000)) { // 5ms (200Hz)
        Serial.println("Could not enable rotation vector");
    }
}

void updateOrientation() {
    if (bno08x.wasReset()) {
        Serial.println("BNO085 was reset");
        setReports();
    }
    
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ROTATION_VECTOR:
                // Update quaternion values
                quaternion_x = sensorValue.un.rotationVector.i;
                quaternion_y = sensorValue.un.rotationVector.j;
                quaternion_z = sensorValue.un.rotationVector.k;
                quaternion_w = sensorValue.un.rotationVector.real;
                
                // Get accuracy and status
                float accuracy = sensorValue.un.rotationVector.accuracy;
                magAccuracy = sensorValue.status;
                
                // Check if calibrated
                if (magAccuracy >= 2 && !isMagCalibrated) {
                    isMagCalibrated = true;
                    Serial.println("Magnetometer Calibrated!");
                }
                
                // Debug output
                static uint32_t lastPrint = 0;
                if (millis() - lastPrint >= 1000) { // Print every second
                    lastPrint = millis();
                    Serial.print("Quaternion - X: "); Serial.print(quaternion_x);
                    Serial.print(" Y: "); Serial.print(quaternion_y);
                    Serial.print(" Z: "); Serial.print(quaternion_z);
                    Serial.print(" W: "); Serial.println(quaternion_w);
                }
                break;
        }
    }
}

void sendQuaternionReport() {
    // Map quaternion values (-1 to 1) to HID range (-127 to 127)
    int8_t x = constrain((int8_t)(quaternion_x * 127.0f), -127, 127);
    int8_t y = constrain((int8_t)(quaternion_y * 127.0f), -127, 127);
    int8_t z = constrain((int8_t)(quaternion_z * 127.0f), -127, 127);
    int8_t w = constrain((int8_t)(quaternion_w * 127.0f), -127, 127);
    
    // Create report
    report_data[0] = x;
    report_data[1] = y;
    report_data[2] = z;
    report_data[3] = w;
    
    // Debug output every second
    static uint32_t lastDebugPrint = 0;
    if (millis() - lastDebugPrint >= 1000) {
        lastDebugPrint = millis();
        Serial.println("Raw quaternion values:");
        Serial.print("X: "); Serial.print(quaternion_x);
        Serial.print(" Y: "); Serial.print(quaternion_y);
        Serial.print(" Z: "); Serial.println(quaternion_z);
        Serial.print(" W: "); Serial.print(quaternion_w);
        
        Serial.println("Mapped HID values:");
        Serial.print("X: "); Serial.print((int)x);
        Serial.print(" Y: "); Serial.print((int)y);
        Serial.print(" Z: "); Serial.println((int)z);
        Serial.print(" W: "); Serial.print((int)w);
    }
    
    // Send the report if connected
    if (Bluefruit.connected()) {
        hid.inputReport(1, report_data, sizeof(report_data));
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

void startAdv() {
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    
    // Set appearance to HID Device (not specifically a gamepad)
    Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);
    
    // Include HID service
    Bluefruit.Advertising.addService(hid);
    
    // Include Device Information Service
    Bluefruit.Advertising.addService(bledis);
    
    // Include Name
    Bluefruit.Advertising.addName();
    
    // Include Battery Service
    Bluefruit.Advertising.addService(blebas);
    
    // Start advertising
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
    
    Serial.println("Advertising started");
    
    // Debug print the vendor and product IDs
    Serial.print("Vendor ID: 0x");
    Serial.println(VENDOR_ID, HEX);
    Serial.print("Product ID: 0x");
    Serial.println(PRODUCT_ID, HEX);
}

// Update battery level periodically
void updateBatteryLevel() {
  static uint32_t lastUpdate = 0;
  
  // Update every 10 seconds
  if(millis() - lastUpdate >= 10000) {
    lastUpdate = millis();
    
    // Read battery voltage
    float vbat = readVBAT();
    
    // Convert to percentage
    uint8_t battery_level = mvToPercent(vbat);
    
    // Update Battery Service
    blebas.write(battery_level);
    
    // Debug output
    Serial.print("Battery Voltage: ");
    Serial.print(vbat);
    Serial.print("V (");
    Serial.print(battery_level);
    Serial.println("%)");
  }
}

void quaternionToEuler() {
    // Different quaternion to euler conversion that might reduce axis coupling
    ypr.yaw = atan2(2.0f * (quaternion_w * quaternion_z + quaternion_x * quaternion_y),
                    1.0f - 2.0f * (quaternion_y * quaternion_y + quaternion_z * quaternion_z));
    ypr.pitch = asin(2.0f * (quaternion_w * quaternion_y - quaternion_z * quaternion_x));
    ypr.roll = atan2(2.0f * (quaternion_w * quaternion_x + quaternion_y * quaternion_z),
                     1.0f - 2.0f * (quaternion_x * quaternion_x + quaternion_y * quaternion_y));

    // Convert to degrees
    ypr.yaw = ypr.yaw * RAD_TO_DEG;
    ypr.pitch = ypr.pitch * RAD_TO_DEG;
    ypr.roll = ypr.roll * RAD_TO_DEG;
}

// Optional: Add a function to save calibration data
void saveCalibration() {
    // You could save the quaternion values when fully calibrated
    // to use as a reference point
    if (isMagCalibrated) {
        // Save current orientation as reference
        // This is just an example - you'd need to implement the actual storage
        Serial.println("Saving calibration reference point");
    }
}
