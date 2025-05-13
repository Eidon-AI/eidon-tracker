#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <bluefruit.h>
#include <stdint.h>

// Forward declarations
void printCalibrationStatus(uint8_t status);
void saveCalibrationData();

// Calibration data structure
struct sh2_CalibrationData_t {
    float mag_bias[3];
    float mag_scale[3];
    float accel_bias[3];
    float gyro_bias[3];
};

// For the built-in LED
#define LED_PIN PIN_LED

// I2C pins for XIAO nRF52840 Sense
#define I2C_SDA 9
#define I2C_SCL 10

// BNO085 I2C address
#define BNO085_I2C_ADDR 0x4B

// Vendor and Product IDs
#define VENDOR_ID 0x303A  // Adafruit's vendor ID
#define PRODUCT_ID 0xABCE // Custom product ID for Eidon Tracker

// System ID string (8 bytes: 4 bytes vendor ID + 4 bytes product ID)
const char system_id[] = {0x3A, 0x30, 0xCE, 0xAB, 0x00, 0x00, 0x00, 0x00};

// HID Report Descriptor for a custom device with 4 quaternion values and switch states
uint8_t const hid_report_descriptor[] = {
  0x05, 0x20,                  //  UsagePage (Sensor)
  0x09, 0x80,                  //  Usage (Orientation)
  0xA1, 0x01,                  //  Collection (Application)

  0x85, 0x01,                  //  Report ID (1)

  // 4×16-bit quaternion components (i, j, k, real)
  0x0A, 0x83, 0x04,            //  Usage 0x0483 – Data Field: Quaternion
  0x75, 0x10,                  //  ReportSize 16
  0x95, 0x04,                  //  ReportCount 4
  0x17, 0x00, 0x00, 0x00, 0x00,// Logical Minimum  0      (32-bit form)
  0x27, 0xFF, 0xFF, 0x00, 0x00,// Logical Maximum  65535  (32-bit form)
  0x81, 0x02,                  //  Input (Data,Var,Abs)

  // Optional switch byte – put it on the Button page so hosts know it’s a button
  0x05, 0x09,                  //  UsagePage (Button)
  0x19, 0x01,                  //  UsageMinimum (Button 1)
  0x29, 0x02,                  //  UsageMaximum (Button 2)  ← up to you
  0x95, 0x02,                  //  ReportCount 2
  0x75, 0x01,                  //  ReportSize 1
  0x15, 0x00, 0x25, 0x01,      //  LogicalMin 0, LogicalMax 1
  0x81, 0x02,                  //  Input (Data,Var,Abs)
  0x95, 0x06, 0x75, 0x01, 0x81, 0x03, // padding bits

0xC0                            // End Collection

};

// HID report map - now 9 bytes total (8 bytes for quaternion + 1 byte for switch states)
uint8_t report_data[9] = {0};

// Add output report buffer
uint8_t output_report[1] = {0};

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

// Battery monitoring constants
const double VREF = 3.3;  // ADC reference voltage
const unsigned int NUM_READINGS = 1024;  // 10-bit ADC readings 0-1023
const double VOLTAGE_DIVIDER_RATIO = 1510.0/510.0;  // Voltage divider ratio from VBAT to ADC
const int BAT_MONITOR_EN_PIN = 14;  // P0.14 for battery monitoring enable

// Switch pins
#define SWITCH_OUT_LEFT_RIGHT 5  // D5 output for left/right switch
#define SWITCH_IN_LEFT_RIGHT 6   // D6 input for left/right switch
#define SWITCH_OUT_UPPER_LOWER 0 // D0 output for upper/lower switch
#define SWITCH_IN_UPPER_LOWER 1  // D1 input for upper/lower switch

// Switch states
bool isLeft = false;
bool isUpper = false;

// Add calibration status variables
struct calibration_status_t {
    uint8_t mag_status;      // 0-3: Unreliable to High accuracy
    uint8_t accel_status;    // 0-3: Unreliable to High accuracy
    uint8_t gyro_status;     // 0-3: Unreliable to High accuracy
    bool needs_calibration;  // True if any sensor needs calibration
} calibration_status = {0, 0, 0, true};

// Add calibration data storage
struct calibration_data_t {
    float mag_bias[3];
    float mag_scale[3];
    float accel_bias[3];
    float gyro_bias[3];
    uint32_t timestamp;
} calibration_data;

// Global to hold last tap time
volatile uint32_t lastTapMillis = 0;

// Function to read battery voltage using internal ADC
float readVBAT(void) {
  // Enable battery monitoring
  pinMode(BAT_MONITOR_EN_PIN, OUTPUT);
  digitalWrite(BAT_MONITOR_EN_PIN, LOW);
  delay(1);  // Small delay to ensure pin state is stable
  
  // Read ADC value
  unsigned int adcCount = analogRead(PIN_VBAT);
  
  // Debug raw ADC reading
//   Serial.print("Raw ADC reading: ");
//   Serial.println(adcCount);
  
  // Convert ADC count to voltage
  double adcVoltage = (adcCount * VREF) / NUM_READINGS;
  
  // Calculate actual battery voltage using voltage divider ratio
  double vBat = adcVoltage * VOLTAGE_DIVIDER_RATIO;
  
  // Disable battery monitoring to save power
  digitalWrite(BAT_MONITOR_EN_PIN, HIGH);
  
  return vBat;
}

// Convert voltage to battery percentage with more accurate mapping
uint8_t mvToPercent(float voltage) {
  // Debug the input voltage
//   Serial.print("Input voltage: ");
//   Serial.print(voltage, 3);
//   Serial.println("V");
  
  // For LiPo battery
  if (voltage >= 4.2) return 100;
  if (voltage <= 3.3) return 0;
  
  // Linear mapping between 3.3V and 4.2V
  // 3.3V = 0%, 3.6V = 20%, 3.7V = 40%, 3.8V = 60%, 3.9V = 80%, 4.2V = 100%
  uint8_t percentage;
  if (voltage < 3.6) {
    percentage = (voltage - 3.3) * 66.67;  // 20% over 0.3V
  } else if (voltage < 3.7) {
    percentage = 20 + (voltage - 3.6) * 200;  // 20% over 0.1V
  } else if (voltage < 3.8) {
    percentage = 40 + (voltage - 3.7) * 200;  // 20% over 0.1V
  } else if (voltage < 3.9) {
    percentage = 60 + (voltage - 3.8) * 200;  // 20% over 0.1V
  } else {
    percentage = 80 + (voltage - 3.9) * 66.67;  // 20% over 0.3V
  }
  
  // Debug the calculated percentage
//   Serial.print("Calculated percentage: ");
//   Serial.print(percentage);
//   Serial.println("%");
  
  return percentage;
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

void setReports() {
    // Enable game rotation vector (orientation)
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) {
        Serial.println("Could not enable rotation vector");
    }

    // Enable Tap Detector (event-driven, report interval 0)
    if (!bno08x.enableReport(SH2_TAP_DETECTOR, 0)) {
        Serial.println("Could not enable tap detector");
    }
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
    
    // // Print calibration instructions
    // Serial.println("\nCalibration Instructions:");
    // Serial.println("1. Wave the device in a figure-8 pattern");
    // Serial.println("2. Rotate slowly through all orientations");
    // Serial.println("3. Keep away from magnetic interference");
    // Serial.println("4. Wait for 'Calibrated' message\n");
    
    return true;
}

void updateOrientation() {
    if (bno08x.wasReset()) {
        Serial.println("BNO085 was reset");
        setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
                // existing quaternion handling
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

                // simple visual feedback
                Serial.println("Tap detected");

                lastTapMillis = millis();
                digitalWrite(LED_GREEN, LOW);   // turn blue on
                bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);   // disable
                bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000); // re-enable (200 Hz)
                delay(2000);
                digitalWrite(LED_GREEN, HIGH);  // turn blue off
                break;
            }
        }
    }
}

void sendQuaternionReport() {
    // Map quaternion values (-1 to 1) to unsigned HID range (0 to 65535)
    // This maps -1 to 0, 0 to 32768, and 1 to 65535
    uint16_t x = (uint16_t)((quaternion_x + 1.0f) * 32767.5f);
    uint16_t y = (uint16_t)((quaternion_y + 1.0f) * 32767.5f);
    uint16_t z = (uint16_t)((quaternion_z + 1.0f) * 32767.5f);
    uint16_t w = (uint16_t)((quaternion_w + 1.0f) * 32767.5f);
    
    // Create report - store 16-bit values in little-endian format
    report_data[0] = x & 0xFF;        // LSB of x
    report_data[1] = (x >> 8) & 0xFF; // MSB of x
    report_data[2] = y & 0xFF;        // LSB of y
    report_data[3] = (y >> 8) & 0xFF; // MSB of y
    report_data[4] = z & 0xFF;        // LSB of z
    report_data[5] = (z >> 8) & 0xFF; // MSB of z
    report_data[6] = w & 0xFF;        // LSB of w
    report_data[7] = (w >> 8) & 0xFF; // MSB of w
    
    // Add switch states to the report
    uint8_t switch_states = 0;
    if (isLeft) switch_states |= 0x01;  // Set bit 0 for left
    if (isUpper) switch_states |= 0x02; // Set bit 1 for upper
    report_data[8] = switch_states;
    
    // Debug output every second
    static uint32_t lastDebugPrint = 0;
    if (millis() - lastDebugPrint >= 1000) {
        lastDebugPrint = millis();
        // Serial.println("Raw quaternion values:");
        // Serial.print("X: "); Serial.print(quaternion_x);
        // Serial.print(" Y: "); Serial.print(quaternion_y);
        // Serial.print(" Z: "); Serial.print(quaternion_z);
        // Serial.print(" W: "); Serial.println(quaternion_w);
        
        // Serial.print("Mapped 16-bit values: ");
        // Serial.print(x); Serial.print(", ");
        // Serial.print(y); Serial.print(", ");
        // Serial.print(z); Serial.print(", ");
        // Serial.println(w);
        
        // Serial.print("Switch States: ");
        // Serial.print(isLeft ? "Left" : "Right");
        // Serial.print(", ");
        // Serial.print(isUpper ? "Upper" : "Lower");
        // Serial.print(" (0x");
        // Serial.print(switch_states, HEX);
        // Serial.println(")");
        
        // Debug connection and report sending
        // Serial.print("Connected: ");
        // Serial.println(Bluefruit.connected() ? "Yes" : "No");
        // Serial.print("Report data: ");
        // for (int i = 0; i < 9; i++) {
        //     Serial.print(report_data[i], HEX);
        //     Serial.print(" ");
        // }
        // Serial.println();
    }
    
    // Send the report if connected
    if (Bluefruit.connected()) {
        if (!hid.inputReport(1, report_data, sizeof(report_data))) {
            Serial.println("Failed to send HID report!");
        }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

void startAdv() {
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    
    // Set appearance to HID Device (not specifically a gamepad)
    Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_GENERIC_HID);
    
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
    
    // Serial.println("\nBattery Reading:");
    
    // Read battery voltage
    float vbat = readVBAT();
    
    // Convert to percentage
    uint8_t battery_level = mvToPercent(vbat);
    
    // Update Battery Service
    blebas.write(battery_level);
    
    // Debug output
    // Serial.print("Final Battery Level: ");
    // Serial.print(battery_level);
    // Serial.println("%");
    // Serial.println("-------------------");
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

// Function to read switch states
void readSwitches() {
  // Read left/right switch
  pinMode(SWITCH_OUT_LEFT_RIGHT, OUTPUT);
  digitalWrite(SWITCH_OUT_LEFT_RIGHT, HIGH);
  delayMicroseconds(10);  // Small delay for signal to stabilize
  pinMode(SWITCH_IN_LEFT_RIGHT, INPUT_PULLDOWN);  // Use pulldown to ensure clean low state
  isLeft = digitalRead(SWITCH_IN_LEFT_RIGHT) == HIGH;
  pinMode(SWITCH_OUT_LEFT_RIGHT, INPUT);  // Set back to input to prevent floating
  
  // Read upper/lower switch
  pinMode(SWITCH_OUT_UPPER_LOWER, OUTPUT);
  digitalWrite(SWITCH_OUT_UPPER_LOWER, HIGH);
  delayMicroseconds(10);  // Small delay for signal to stabilize
  pinMode(SWITCH_IN_UPPER_LOWER, INPUT_PULLDOWN);  // Use pulldown to ensure clean low state
  isUpper = digitalRead(SWITCH_IN_UPPER_LOWER) == HIGH;
  pinMode(SWITCH_OUT_UPPER_LOWER, INPUT);  // Set back to input to prevent floating
}

// Update the calibration status monitoring
void updateCalibrationStatus() {
    static uint32_t lastCalibrationCheck = 0;
    if (millis() - lastCalibrationCheck >= 1000) {  // Check every second
        lastCalibrationCheck = millis();
        
        // Get calibration status
        sh2_SensorValue_t sensorValue;
        if (bno08x.getSensorEvent(&sensorValue)) {
            switch (sensorValue.sensorId) {
                case SH2_MAGNETIC_FIELD_CALIBRATED:
                    calibration_status.mag_status = sensorValue.status;
                    break;
                case SH2_ACCELEROMETER:
                    calibration_status.accel_status = sensorValue.status;
                    break;
                case SH2_RAW_GYROSCOPE:
                    calibration_status.gyro_status = sensorValue.status;
                    break;
            }
        }

        // Print calibration status
        Serial.println("\nCalibration Status:");
        Serial.print("Magnetometer: ");
        switch (calibration_status.mag_status) {
            case 0: Serial.println("Uncalibrated"); break;
            case 1: Serial.println("Poor"); break;
            case 2: Serial.println("Good"); break;
            case 3: Serial.println("Excellent"); break;
            default: Serial.println("Unknown"); break;
        }
        Serial.print("Accelerometer: ");
        switch (calibration_status.accel_status) {
            case 0: Serial.println("Uncalibrated"); break;
            case 1: Serial.println("Poor"); break;
            case 2: Serial.println("Good"); break;
            case 3: Serial.println("Excellent"); break;
            default: Serial.println("Unknown"); break;
        }
        Serial.print("Gyroscope: ");
        switch (calibration_status.gyro_status) {
            case 0: Serial.println("Uncalibrated"); break;
            case 1: Serial.println("Poor"); break;
            case 2: Serial.println("Good"); break;
            case 3: Serial.println("Excellent"); break;
            default: Serial.println("Unknown"); break;
        }
    }
}

// Helper function to print calibration status
void printCalibrationStatus(uint8_t status) {
    switch (status) {
        case 0:
            Serial.println("Uncalibrated");
            break;
        case 1:
            Serial.println("Poor");
            break;
        case 2:
            Serial.println("Good");
            break;
        case 3:
            Serial.println("Excellent");
            break;
        default:
            Serial.println("Unknown");
            break;
    }
}

// Update the calibration command handler
void handleCalibrationCommand(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len >= 1) {
        switch (data[0]) {
            case 1:  // Magnetometer calibration
                Serial.println("\nStarting magnetometer calibration...");
                Serial.println("Please move the device in a figure-8 pattern");
                Serial.println("Keep away from magnetic interference");
                calibration_status.mag_status = 0;
                break;
            case 2:  // Accelerometer calibration
                Serial.println("\nStarting accelerometer calibration...");
                Serial.println("Please place the device in 6 different stable positions");
                Serial.println("Hold each position for 2-3 seconds");
                calibration_status.accel_status = 0;
                break;
            case 3:  // Gyroscope calibration
                Serial.println("\nStarting gyroscope calibration...");
                Serial.println("Please keep the device completely still");
                Serial.println("This will take about 5 seconds");
                calibration_status.gyro_status = 0;
                break;
            case 4:  // Full calibration
                Serial.println("\nStarting full calibration sequence...");
                Serial.println("1. Keep device still for gyroscope calibration (5s)");
                Serial.println("2. Place in 6 positions for accelerometer calibration");
                Serial.println("3. Move in figure-8 pattern for magnetometer calibration");
                calibration_status.mag_status = 0;
                calibration_status.accel_status = 0;
                calibration_status.gyro_status = 0;
                break;
            case 5:  // Save calibration data
                saveCalibrationData();
                break;
        }
    }
}

// Function to save calibration data
void saveCalibrationData() {
    // The BNO085 automatically saves calibration data to non-volatile memory
    // when calibration is complete. We just need to wait for the calibration
    // to finish and verify the status.
    Serial.println("Calibration data is automatically saved when calibration is complete.");
    Serial.println("Please check the calibration status using the updateCalibrationStatus function.");
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
    bledis.setSystemID(system_id, 8);
    
    // Configure HID
    hid.enableKeyboard(false);  // Explicitly disable keyboard
    hid.enableMouse(false);     // Explicitly disable mouse
    
    // Set our custom report map (descriptor)
    hid.setReportMap(hid_report_descriptor, sizeof(hid_report_descriptor));
    
    // Set the length of our reports
    uint16_t input_len[] = {9};  // Length of our input report
    uint16_t output_len[] = {1}; // Length of our output report
    hid.setReportLen(input_len, output_len, NULL);
    
    // Set the output report callback
    hid.setOutputReportCallback(2, handleCalibrationCommand);
    
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
    
    // Initialize switch pins
    pinMode(SWITCH_OUT_LEFT_RIGHT, INPUT);
    pinMode(SWITCH_IN_LEFT_RIGHT, INPUT_PULLDOWN);
    pinMode(SWITCH_OUT_UPPER_LOWER, INPUT);
    pinMode(SWITCH_IN_UPPER_LOWER, INPUT_PULLDOWN);

    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_GREEN, HIGH); // off (assuming active-low RGB LED)
}

void loop() {
    // checkDFU();  // Check for DFU command
    // Update orientation at regular intervals
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        updateOrientation();
        sendQuaternionReport();
        lastUpdate = millis();
    }
    
    // Update battery level periodically
    updateBatteryLevel();
    
    // Read switch states
    readSwitches();
    
    // Monitor calibration status
    // updateCalibrationStatus();
}
