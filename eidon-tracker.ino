#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>

// For the built-in LED
#define LED_PIN PIN_LED

// I2C pins for XIAO nRF52840 Sense
#define I2C_SDA 6  // SDA pin
#define I2C_SCL 7  // SCL pin

// LSM6DS3TR-C I2C address on XIAO nRF52840 Sense
#define LSM6DS_I2C_ADDR 0x6A

// HID Report Descriptor for a custom device with 3 axes
// Using a custom usage page to avoid keyboard/gamepad interpretation
uint8_t const hid_report_descriptor[] = {
  0x06, 0xFF, 0x00,  // Usage Page (Vendor Defined)
  0x09, 0x01,        // Usage (1)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x01,        //   Report ID (1)
  
  // 3 axes for roll, pitch, yaw
  0x09, 0x30,        //   Usage (X)
  0x09, 0x31,        //   Usage (Y)
  0x09, 0x32,        //   Usage (Z)
  0x15, 0x81,        //   Logical Minimum (-127)
  0x25, 0x7F,        //   Logical Maximum (127)
  0x75, 0x08,        //   Report Size (8)
  0x95, 0x03,        //   Report Count (3)
  0x81, 0x02,        //   Input (Data, Variable, Absolute)
  
  0xC0               // End Collection
};

// HID report map
uint8_t report_data[4] = {0};  // Report ID + 3 axis values

// IMU sensor - Seeed Studio LSM6DS3
LSM6DS3 myIMU(I2C_MODE, LSM6DS_I2C_ADDR);

// Bluetooth HID
BLEDis bledis;
BLEHidGeneric hid(1, 0, 0);

// Battery Service
BLEBas blebas;

// Complementary filter variables
float accelRoll = 0, accelPitch = 0;
float gyroRoll = 0, gyroPitch = 0, gyroYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
float gyroXrate = 0, gyroYrate = 0, gyroZrate = 0;
float dt = 0;
unsigned long prevTime = 0;

// Update interval (milliseconds)
const unsigned long UPDATE_INTERVAL = 20;
unsigned long lastUpdate = 0;

// Debug counter
unsigned long debugCounter = 0;
const unsigned long DEBUG_INTERVAL = 1000; // Print debug info every second

// Function to read battery voltage using internal ADC
float readVBAT(void) {
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
  
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12);
  
  // Read the internal voltage reference
  float vref = 3.0;
  float measuredvbat = analogReadVDDH();
  
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

void checkDFU() {
    if (Serial.available()) {
        if (Serial.read() == 'D') {
            enterDFU();
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    // Wait a bit for serial connection
    delay(500);
    
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
    Serial.println("Using LSM6DS3 sensor with Seeed Studio library");
    
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
    
    // Configure HID
    hid.enableKeyboard(false);  // Explicitly disable keyboard
    hid.enableMouse(false);     // Explicitly disable mouse
    
    // Set our custom report map (descriptor)
    hid.setReportMap(hid_report_descriptor, sizeof(hid_report_descriptor));
    
    // Set the length of our input report (3 bytes for roll, pitch, yaw)
    uint16_t input_len[] = {3};  // Length of our single input report
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
        sendGamepadReport();
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
    
    updateBatteryLevel();
}

bool initIMU() {
    // Initialize I2C with explicit pins for XIAO nRF52840 Sense
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    
    // I2C Scanner - to help diagnose issues
    Serial.println("Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;
    
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("!");
            nDevices++;
        }
    }
    
    if (nDevices == 0) {
        Serial.println("No I2C devices found!");
    }
    
    // Initialize the LSM6DS3
    Serial.println("Trying to initialize LSM6DS3...");
    
    if (myIMU.begin() != 0) {
        Serial.println("Failed to initialize LSM6DS3!");
        return false;
    } else {
        Serial.println("LSM6DS3 initialized successfully!");
    }
    
    return true;
}

void updateOrientation() {
    // Calculate dt for integration
    unsigned long currentTime = millis();
    dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;
    
    // Get accelerometer data
    float accelX = myIMU.readFloatAccelX();
    float accelY = myIMU.readFloatAccelY();
    float accelZ = myIMU.readFloatAccelZ();
    
    // Get gyroscope data (in degrees per second)
    float gyroX = myIMU.readFloatGyroX();
    float gyroY = myIMU.readFloatGyroY();
    float gyroZ = myIMU.readFloatGyroZ();
    
    // Calculate roll and pitch from accelerometer (angles in degrees)
    // Note: atan2 returns radians, we convert to degrees
    accelRoll = atan2(accelY, accelZ) * RAD_TO_DEG;
    accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
    
    // Integrate gyro rates to get angles
    gyroXrate = gyroX; // degrees per second
    gyroYrate = gyroY;
    gyroZrate = gyroZ;
    
    // Complementary filter - combine accelerometer and gyroscope angles
    // Use high % of gyro and low % of accel for smoother results
    const float GYRO_WEIGHT = 0.96;
    const float ACCEL_WEIGHT = 0.04;
    
    // Update roll, pitch, and yaw
    gyroRoll += gyroXrate * dt;
    gyroPitch += gyroYrate * dt;
    gyroYaw += gyroZrate * dt;
    
    // Correct for drift with accelerometer data
    gyroRoll = gyroRoll * GYRO_WEIGHT + accelRoll * ACCEL_WEIGHT;
    gyroPitch = gyroPitch * GYRO_WEIGHT + accelPitch * ACCEL_WEIGHT;
    // Note: No accelerometer correction for yaw as it would require a magnetometer
    
    // Transfer filtered angles to global variables
    roll = gyroRoll;
    pitch = gyroPitch;
    yaw = gyroYaw;
    
    // Keep yaw in -180 to 180 range
    while (yaw > 180) yaw -= 360;
    while (yaw < -180) yaw += 360;
    
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Yaw: ");
    Serial.println(yaw);
}

void sendGamepadReport() {
    // Map orientation to values (-127 to 127)
    int8_t rollAxis = constrain(map(roll, -180, 180, -127, 127), -127, 127);
    int8_t pitchAxis = constrain(map(pitch, -180, 180, -127, 127), -127, 127);
    int8_t yawAxis = constrain(map(yaw, -180, 180, -127, 127), -127, 127);
    
    // Create report
    // report_data[0] = 1;  // Report ID
    report_data[0] = rollAxis;
    report_data[1] = pitchAxis;
    report_data[2] = yawAxis;
    
    // Send the report if connected
    if (Bluefruit.connected()) {
        hid.inputReport(1, report_data, sizeof(report_data));
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED to indicate data sent
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