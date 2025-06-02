#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <bluefruit.h>
#include <stdint.h>
#include "Adafruit_SPIFlash.h"

// Built from the P25Q16H datasheet.
SPIFlash_Device_t const P25Q16H {
  .total_size = (1UL << 21), // 2MiB
  .start_up_time_us = 10000, // Don't know where to find that value

  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,

  .max_clock_speed_mhz = 55,
  .quad_enable_bit_mask = 0x02, // Datasheet p. 27
  .has_sector_protection = 1,   // Datasheet p. 27
  .supports_fast_read = 1,      // Datasheet p. 29
  .supports_qspi = 1,           // Obviously
  .supports_qspi_writes = 1,    // Datasheet p. 41
  .write_status_register_split = 1, // Datasheet p. 28
  .single_status_byte = 0,      // 2 bytes
  .is_fram = 0,                 // Flash Memory
};

// LSM6DS3TR-C I2C pins and Address (for XIAO nRF52840 Sense built-in IMU)
// #define LSM6DS_I2C_SDA 6  // SDA pin
// #define LSM6DS_I2C_SCL 7  // SCL pin
// #define LSM6DS_I2C_ADDR 0x6A // Address

// BNO085 I2C pins and Address (for external IMU)
#define BNO085_I2C_SDA 9
#define BNO085_I2C_SCL 10
#define BNO085_I2C_ADDR 0x4B // Address
#define BNO085_INT_PIN 8     // Interrupt pin for data ready

// Vendor and Product IDs
#define VENDOR_ID  0xE1D0 // Eidon AI vendor ID
#define PRODUCT_ID 0x0002 // Eidon Tracker product ID

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

// Orientation data
float quaternion_x = 0;
float quaternion_y = 0;
float quaternion_z = 0;
float quaternion_w = 1;

// Bluetooth HID
BLEDis bledis;
BLEHidGeneric blehid(1, 2, 1);

#define COLOR_MAGIC   0xE7          // any value ≠ 0xFF

// Device color RGB stored (default white)
uint8_t device_color[3] = {0xFF, 0xFF, 0xFF};

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
  // Serial.print("Input voltage: ");
  // Serial.print(voltage, 3);
  // Serial.println("V");
  
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
  // Serial.print("Calculated percentage: ");
  // Serial.print(percentage);
  // Serial.println("%");
  
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
    // Enable game rotation vector at maximum rate (1000 Hz)
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000)) {
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
    // Apply 180-degree rotation around Z-axis to correct for IMU mounting
    // For 180° rotation around Z: negate X and Y components, keep Z and W unchanged
    
    // Original sensor quaternion
    float qw_sensor = quaternion_w;
    float qx_sensor = quaternion_x;
    float qy_sensor = quaternion_y;
    float qz_sensor = quaternion_z;
    
    // Apply 180-degree Z-rotation by negating X and Y components
    float corrected_w = qw_sensor;   // W stays the same
    float corrected_x = -qx_sensor;  // Negate X (East becomes West)
    float corrected_y = -qy_sensor;  // Negate Y (North becomes South)
    float corrected_z = qz_sensor;   // Z stays the same (Up is still Up)
    
    // Map corrected quaternion values (-1 to 1) to unsigned HID range (0 to 65535)
    // This maps -1 to 0, 0 to 32768, and 1 to 65535
    uint16_t x = (uint16_t)((corrected_x + 1.0f) * 32767.5f);
    uint16_t y = (uint16_t)((corrected_y + 1.0f) * 32767.5f);
    uint16_t z = (uint16_t)((corrected_z + 1.0f) * 32767.5f);
    uint16_t w = (uint16_t)((corrected_w + 1.0f) * 32767.5f);
    
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
    // static unsigned long lastDebugPrint = 0;
    // if (millis() - lastDebugPrint >= 1000) {
    //     lastDebugPrint = millis();
    //     // Debug the quaternion correction
    //     Serial.println("Raw sensor quaternion:");
    //     Serial.print("W: "); Serial.print(qw_sensor, 4);
    //     Serial.print(" X: "); Serial.print(qx_sensor, 4);
    //     Serial.print(" Y: "); Serial.print(qy_sensor, 4);
    //     Serial.print(" Z: "); Serial.println(qz_sensor, 4);
        
    //     Serial.println("Corrected quaternion:");
    //     Serial.print("W: "); Serial.print(corrected_w, 4);
    //     Serial.print(" X: "); Serial.print(corrected_x, 4);
    //     Serial.print(" Y: "); Serial.print(corrected_y, 4);
    //     Serial.print(" Z: "); Serial.println(corrected_z, 4);
    //     Serial.println("---");
    // }
    
    // Send the report if connected
    if (Bluefruit.connected()) {
        if (!blehid.inputReport(1, report_data, sizeof(report_data))) {
            Serial.println("Failed to send HID report!");
        }
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
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
  
  // Update every 5 minutes to minimize performance impact
  if(millis() - lastUpdate >= 300000) {  // 300 seconds = 5 minutes
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


// QSPI flash transport and object for XIAO nRF52840 Sense (external 2-MiB P25Q16H)
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash qspiFlash(&flashTransport);

// Sector/offset inside external flash that holds the color record
#define COLOR_SECTOR      0          // last sector in 2-MiB device
#define COLOR_ADDR        (COLOR_SECTOR * 4096)

// ── Helper to write color to external flash ───────────────────────────────
static void color_store_write(uint8_t rgb[3]) {
    uint8_t buf[4] = { COLOR_MAGIC, rgb[0], rgb[1], rgb[2] };

    // Erase sector 0 (first 4-kB) — pass sector *number*, not byte address
    if (!qspiFlash.eraseSector(COLOR_SECTOR)) {
        Serial.println("QSPI eraseSector() failed");
        return;
    }
    qspiFlash.waitUntilReady();
    if (qspiFlash.writeBuffer(COLOR_ADDR, buf, sizeof(buf)) != sizeof(buf)) {
        Serial.println("QSPI writeBuffer() failed");
        return;
    }
    qspiFlash.waitUntilReady(); // ensure data is on flash before power-down

    // read back for verification during development
    uint8_t verify[4];
    qspiFlash.readBuffer(COLOR_ADDR, verify, sizeof(verify));
    Serial.print("Color written / verify: ");
    for(int i=0;i<4;i++){ Serial.print(verify[i], HEX); Serial.print(" "); }
    Serial.println();
}

static bool color_store_read(uint8_t rgb[3]) {
    uint8_t buf[4];
    qspiFlash.readBuffer(COLOR_ADDR, buf, sizeof(buf));
    if (buf[0] != COLOR_MAGIC) return false;

    rgb[0] = buf[1];
    rgb[1] = buf[2];
    rgb[2] = buf[3];
    return true;
}

void handleColorFeature(uint16_t         /*conn*/,
                         BLECharacteristic* chr,
                         uint8_t*          data,
                         uint16_t          len)
{
  if (len != 3) return;                 // expect exactly R-G-B

  // 1. store locally
  memcpy(device_color, data, 3);      // keep it in RAM
  color_store_write(device_color);

  // 2. update the GATT database value of *this* characteristic
  //    (so the next Get-Feature Read returns the new bytes)
  chr->write(device_color, 3);

  // optional debug
  Serial.print  ("Color set to #");
  for (uint8_t i=0; i<3; ++i) {
      if (device_color[i] < 16) Serial.print('0');
      Serial.print(device_color[i], HEX);
  }
  Serial.println();
}

void sendColorFeature()
{
  blehid.inputReport(   /*ID*/ 2, device_color, 3);   // echoes new value once
}

// Add interrupt flag for faster sensor reading
volatile bool sensorDataReady = false;

// Interrupt service routine
void sensorISR() {
    sensorDataReady = true;
}

void setup() {
    Serial.begin(115200);

    // Reduced wait time for faster startup (500ms max)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 500));

    // Check for DFU trigger command
    while (Serial.available()) {
        if (Serial.read() == 'D') {  // 'D' for DFU
            enterDFU();
        }
    }

    Serial.println("XIAO nRF52840 IMU Bluetooth Orientation Tracker");
    Serial.println("Using BNO085 sensor");

    // Set the LED pin as output
    pinMode(PIN_LED, OUTPUT);

    initBatteryMonitoring();

    // -------------------------------------------------
    // Initialise external QSPI flash
    // -------------------------------------------------
    if (!qspiFlash.begin(&P25Q16H, 1)) {
        Serial.println("QSPI Flash init FAILED – color will not persist");
    }

    // Initialize IMU
    if (!initIMU()) {
        Serial.println("Failed to initialize IMU!");
        // Flash LED rapidly to indicate error
        while (1) {
            digitalWrite(PIN_LED, HIGH);
            delay(100);
            digitalWrite(PIN_LED, LOW);
            delay(100);
        }
    }
    
    // Configure interrupt pin for sensor data ready
    pinMode(BNO085_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BNO085_INT_PIN), sensorISR, FALLING);
    
    // Initialize Bluetooth
    Bluefruit.begin();
    
    // Set device name
    Bluefruit.setName("Eidon Tracker");
    
    // Optimize BLE for minimum latency
    Bluefruit.Periph.setConnInterval(6, 12);   // 7.5-15ms intervals (fast as possible)
    // Note: setConnSupervision and setConnSlaveLatency may not be available in this library version
    
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
    bledis.setHardwareRev("1.1");
    bledis.setFirmwareRev("1.1");

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
    uint16_t feat_len[] = { 3 };     // RGB
    blehid.setReportLen(input_len, output_len, feat_len);
    
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

    blehid.setFeatureReportCallback(1, handleColorFeature);
    
    // Send initial feature report to host (optional)
    blehid.featureReport(1 /*ID*/, device_color, 3);
    
    // Read the color from flash
    color_store_read(device_color);

    Serial.print("Saved Color: #");
    Serial.print(device_color[0], HEX);
    Serial.print(device_color[1], HEX);
    Serial.println(device_color[2], HEX);

    // Update color feature report
    blehid.featureReport(1 /*ID*/, device_color, 3);
}

void loop() {
    // Interrupt-driven sensor reading for minimum latency
    // if (sensorDataReady) {
    //     sensorDataReady = false;
        updateOrientation();
        sendQuaternionReport();
    // }
    
    // Update battery level very infrequently to avoid performance impact
    updateBatteryLevel();
    
    // Read switch states (keep this frequent for responsiveness)
    readSwitches();
}
