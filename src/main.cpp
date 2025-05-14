#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <bluefruit.h>
#include <stdint.h>

// For the built-in LED
#define LED_PIN PIN_LED

// LSM6DS3TR-C I2C pins and Address (for XIAO nRF52840 Sense built-in IMU)
// #define LSM6DS_I2C_SDA 6  // SDA pin
// #define LSM6DS_I2C_SCL 7  // SCL pin
// #define LSM6DS_I2C_ADDR 0x6A // Address

// BNO085 I2C pins and Address (for external IMU)
#define BNO085_I2C_SDA 9
#define BNO085_I2C_SCL 10
#define BNO085_I2C_ADDR 0x4B // Address

// Vendor and Product IDs
#define VENDOR_ID  0xE1D0 // Eidon AI vendor ID
#define PRODUCT_ID 0x0002 // Eidon Tracker product ID

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

  // Optional switch byte – put it on the Button page so hosts know it's a button
  0x05, 0x09,                  //  UsagePage (Button)
  0x19, 0x01,                  //  UsageMinimum (Button 1)
  0x29, 0x02,                  //  UsageMaximum (Button 2)  ← up to you
  0x95, 0x02,                  //  ReportCount 2
  0x75, 0x01,                  //  ReportSize 1
  0x15, 0x00, 0x25, 0x01,      //  LogicalMin 0, LogicalMax 1
  0x81, 0x02,                  //  Input (Data,Var,Abs)
  0x95, 0x06, 0x75, 0x01, 0x81, 0x03, // padding bits

  0xC0,                           // End Orientation Collection

  // ── Second top-level collection : vendor I/O ─────────────────────────
  0x06, 0x00, 0xFF,            //  UsagePage (Vendor-defined 0xFF00)
  0x09, 0x01,                  //  Usage (Vendor usage 1)
  0xA1, 0x01,                  //  Collection (Application)

  0x85, 0x01,                  //    Report ID (1) – command channel
  0x15, 0x00, 0x26, 0xFF, 0x00,//    Logical Min 0, Logical Max 255
  0x75, 0x08,                  //    Report Size 8 bits
  0x95, 0x01,                  //    Report Count 1
  0x09, 0x01,                  //    Usage (Vendor usage 1)
  0x91, 0x02,                  //    Output (Data,Var,Abs)

  0xC0                            //  End Vendor Collection
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
BLEHidGeneric blehid(1, 2, 0);

// Battery Service
BLEBas blebas;

// Update interval (milliseconds)
const unsigned long UPDATE_INTERVAL = 1;
unsigned long lastUpdate = 0;

// -----------------------------------------------------------------------------
// Battery-monitoring constants and helpers
// Xiao nRF52840 Sense routes VBAT through a resistor divider ( ≈ 2.961 : 1 ) to
// pin P0.31 (alias PIN_VBAT).
// The nRF52 ADC is configured for a 0.6 V reference with gain ×6 → 3.6 V full-scale
// and 12-bit resolution (0–4095).  Use the same settings recommended by Seeed.
// -----------------------------------------------------------------------------
const float ADC_REF_VOLTAGE   = 3.6f;       // 0.6 V × 6 gain
const uint16_t ADC_MAX_COUNT  = 4095;       // 12-bit ADC
const float VBAT_DIVIDER_RATIO = 2.961f;    // Empirically measured divider ratio
const int   BAT_MONITOR_EN_PIN = 14;        // P0.14 controls divider (LOW = measure)

// Battery monitoring pins
#define PIN_VBAT        (32)  // D32 battery voltage
#define PIN_VBAT_ENABLE (14)  // D14 LOW:read anable
#define PIN_HICHG       (22)  // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23)  // D23 charge indicatore LOW:charge HIGH:no charge

// Switch pins
#define SWITCH_OUT_LEFT_RIGHT 5  // D5 output for left/right switch
#define SWITCH_IN_LEFT_RIGHT 6   // D6 input for left/right switch
#define SWITCH_OUT_UPPER_LOWER 0 // D0 output for upper/lower switch
#define SWITCH_IN_UPPER_LOWER 1  // D1 input for upper/lower switch

// Switch states
bool isLeft = false;
bool isUpper = false;

float readVBAT(void) {
  // Enable voltage divider (active LOW)
//   pinMode(BAT_MONITOR_EN_PIN, OUTPUT);
//   digitalWrite(BAT_MONITOR_EN_PIN, LOW);
//   delayMicroseconds(300);                 // allow voltage to settle

//   (void)analogRead(PIN_VBAT);             // dummy read to discard first sample
  uint16_t adcCount = analogRead(PIN_VBAT);

  // Disable divider to save power
  digitalWrite(BAT_MONITOR_EN_PIN, HIGH);

  // Convert to volts
  float vBat = ( (float)adcCount / ADC_MAX_COUNT ) * ADC_REF_VOLTAGE * VBAT_DIVIDER_RATIO;

  return vBat;
}

// Convert voltage to battery percentage with more accurate mapping
uint8_t mvToPercent(float voltage) {
  // Debug the input voltage
  Serial.print("Input voltage: ");
  Serial.print(voltage, 3);
  Serial.println("V");
  
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
  Serial.print("Calculated percentage: ");
  Serial.print(percentage);
  Serial.println("%");
  
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
    Wire.setPins(BNO085_I2C_SDA, BNO085_I2C_SCL);
    Wire.begin();
    
    // Try to initialize the BNO085
    if (!bno08x.begin_I2C(BNO085_I2C_ADDR)) {
        Serial.println("Failed to find BNO085 chip");
        return false;
    }

    Serial.println("BNO085 Found!");

    // Enable the rotation vector report
    setReports();

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

                if (isDouble) {
                    Serial.println("Double tap detected");

                    digitalWrite(LED_GREEN, LOW);   // turn blue on
                    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);   // disable
                    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000); // re-enable (200 Hz)
                    delay(2000);
                    digitalWrite(LED_GREEN, HIGH);  // turn blue off
                } else {
                    Serial.println("Single tap detected");
                }

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
    static unsigned long lastDebugPrint = 0;
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
        if (!blehid.inputReport(1, report_data, sizeof(report_data))) {
            Serial.println("Failed to send HID report!");
        }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

void appendUniqueToName() {
  // 1. Fetch the STATIC RANDOM address the SoftDevice is using
  uint8_t addr[6];
  Bluefruit.getAddr(addr);            // LSByte = addr[0]

  // 2. Build "Eidon Tracker-xxxx", where xxxx = low 16 bits of the address
  char advName[32];
  sprintf(advName, "Eidon Tracker-%02X%02X", addr[1], addr[0]); // 4 hex chars

  // 3. Replace the default name and put it in the scan-response
  Bluefruit.setName(advName);
  Bluefruit.ScanResponse.clearData(); // keep other SR fields if you added any
  Bluefruit.ScanResponse.addName();   // full name lives in scan response
}

void startAdv() {
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    
    // Set appearance to HID Device (not specifically a gamepad)
    Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_GENERIC_HID);
    
    // Include HID service
    Bluefruit.Advertising.addService(blehid);
    
    // Include Device Information Service
    Bluefruit.Advertising.addService(bledis);
    
    // Include Name
    // Bluefruit.ScanResponse.addName();
    appendUniqueToName();
    
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
  static unsigned long lastUpdate = 0;
  
  // Update every 10 seconds
  if(millis() - lastUpdate >= 10000) {
    lastUpdate = millis();
    
    Serial.println("\nBattery Reading:");
    
    // Read battery voltage
    float vbat = readVBAT();
    
    // Convert to percentage
    uint8_t battery_level = mvToPercent(vbat);
    
    // Update Battery Service
    blebas.write(battery_level);
    
    // Debug output
    Serial.print("Final Battery Level: ");
    Serial.print(battery_level);
    Serial.println("%");
    Serial.println("-------------------");
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

void initBatteryMonitoring() {
    pinMode(PIN_VBAT, INPUT);
    pinMode(PIN_VBAT_ENABLE, OUTPUT);
    pinMode(PIN_HICHG, OUTPUT);
    pinMode(PIN_CHG, INPUT);

    digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
    digitalWrite(PIN_HICHG, LOW);       // charge current 100mA

    // -----------------------------------------------------------------
    // ADC configuration for battery monitoring
    // -----------------------------------------------------------------
    analogReference(AR_DEFAULT);   // 0.6 V ×6 = 3.6 V
    analogReadResolution(12);      // 0-4095 counts
}

void handleCommand(uint16_t conn_hdl,
                   BLECharacteristic* chr,
                   uint8_t* data, uint16_t len)
{
  if (len == 0) return;

  switch (data[0])
  {
    case 0x01:               // soft-reset IMU
      Serial.println("Host requested IMU reset");
      digitalWrite(LED_GREEN, LOW);   // turn blue on
      bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);   // disable
      bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000); // re-enable (200 Hz)
      delay(2000);
      digitalWrite(LED_GREEN, HIGH);  // turn blue off
      break;

    // Add more command bytes here if you wish
    default:
      Serial.print("Unknown command 0x");
      Serial.println(data[0], HEX);
      break;
  }
}

void setup() {
    Serial.begin(115200);

    // Wait for serial port to open (up to 2 seconds)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000));

    // Check for DFU trigger command
    while (Serial.available()) {
        if (Serial.read() == 'D') {  // 'D' for DFU
            enterDFU();
        }
    }

    Serial.println("XIAO nRF52840 IMU Bluetooth Orientation Tracker");
    Serial.println("Using BNO085 sensor");

    // Set the LED pin as output
    pinMode(LED_PIN, OUTPUT);

    initBatteryMonitoring();

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
    
    // ---------- Device-information service -----------------------------

    // PnP-ID (see Core Spec vol 3, part C §12.1)
    static const uint8_t pnp_id[7] = {
      0x02,                             // 0x01 = BT-SIG, 0x02 = USB-IF
      (uint8_t)(VENDOR_ID  & 0xFF),
      (uint8_t)(VENDOR_ID  >> 8),
      (uint8_t)(PRODUCT_ID & 0xFF),
      (uint8_t)(PRODUCT_ID >> 8),
      0x00, 0x01                        // product / firmware version
    };

    bledis.setPNPID(reinterpret_cast<const char*>(pnp_id), sizeof(pnp_id));
    bledis.setModel("Eidon Tracker");
    bledis.setManufacturer("Eidon AI");
    bledis.setHardwareRev("v1.0");
    bledis.setFirmwareRev("v1.0");

    char uid[17];                              // 16 hex digits + NUL
    sprintf(uid, "%08lX%08lX",
            NRF_FICR->DEVICEID[1],
            NRF_FICR->DEVICEID[0]);
    bledis.setSerialNum(uid);

    // const char* uid = getMcuUniqueID();   // returns NUL-terminated C-string
    Serial.print("Board UID = "); Serial.println(uid);

    // CREATE the characteristics now
    bledis.begin();
    // -------------------------------------------------------------------
    
    // Configure HID
    blehid.enableKeyboard(false);  // Explicitly disable keyboard
    blehid.enableMouse(false);     // Explicitly disable mouse
    
    // Set our custom report map (descriptor)
    blehid.setReportMap(hid_report_descriptor, sizeof(hid_report_descriptor));
    
    // Set the length of our reports
    uint16_t input_len[]  = { 9 };   // Quaternion + switches
    uint16_t output_len[] = { 1, 1 };   // 1-byte dummy, 1-byte command (ID 2)
    blehid.setReportLen(input_len, output_len, NULL);
    
    // Start HID Service
    blehid.begin();
    
    // Set the output report callback
    blehid.setOutputReportCallback(1, handleCommand);
    
    // Initialize Battery Service
    blebas.begin();
    blebas.write(100);
    
    // Start advertising
    startAdv();
    
    Serial.println("Setup complete");
    
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
}
