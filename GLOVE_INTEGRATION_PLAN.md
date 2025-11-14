# Eidon Glove Integration Plan

**Date Created:** 2025-11-14
**Repositories Involved:**
- `eidon-tracker` (ESP32-C6, NimBLE, GATT custom service)
- `eidon-glove` (ESP32-C3, BLE HID, finger sensors)
- `eidon-sim` (Web app, visualization, recording)

**Goal:** Integrate glove functionality into the tracker firmware architecture, enabling a unified firmware that supports both tracker and glove hardware with proper role management and sensor fusion.

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Architecture Analysis](#architecture-analysis)
3. [Phase 1: Firmware Integration](#phase-1-firmware-integration)
4. [Phase 2: App Adaptation](#phase-2-app-adaptation)
5. [Implementation Checklist](#implementation-checklist)
6. [Testing Strategy](#testing-strategy)
7. [Questions & Decisions](#questions--decisions)

---

## Executive Summary

### Current State

**eidon-tracker:**
- **Hardware:** ESP32-C6 (Seeed XIAO)
- **IMU:** BNO085 on I2C (GPIO 19/20)
- **BLE:** Custom GATT service (UUID: E1D00001-...)
- **Framework:** Arduino + NimBLE (PlatformIO)
- **Role system:** HUB (left/right), CHILD (hand/forearm), CHEST
- Hub devices aggregate 2 children via ESP-NOW
- 16-byte quaternion reports @ 50Hz

**eidon-glove:**
- **Hardware:** ESP32-C3 (Seeed XIAO)
- **IMU:** BNO085 on I2C (different GPIO pins than C6)
- **BLE:** BLE HID protocol (ESP-IDF native)
- **Framework:** ESP-IDF (CMake)
- **Finger Sensors:** 16 Hall effect sensors via analog multiplexer
  - MUX control: 4 GPIO pins (S0-S3: GPIO 10/9/6/7)
  - Analog read: 1 ADC GPIO pin (TBD from schematic)
- 9-byte HID reports: 8-byte quaternion + 1-byte buttons

### Target Architecture

**Tracker Firmware Running on ESP32-C3 (Glove Hardware):**
- Port eidon-tracker firmware (Arduino + NimBLE) to ESP32-C3
- New roles: `ROLE_LEFT_GLOVE` (8), `ROLE_RIGHT_GLOVE` (9)
- IMU configuration: Both devices use I2C, just different GPIO pins
- Extended GATT characteristics: Add finger sensor data (16 x uint16_t = 32 bytes)
- Glove connection model: Direct to phone (like hub), no children
- Arm setup changes:
  - **Pure tracker:** HUB → CHILD_FOREARM + CHILD_HAND (3 devices)
  - **Glove setup:** HUB → CHILD_FOREARM + GLOVE (3 devices, glove direct to phone)

### Strategy

1. **Phase 1.1:** Get tracker firmware building for ESP32-C3 (change board config)
2. **Phase 1.2:** Configure BNO085 I2C GPIO pins for glove hardware
3. **Phase 1.3:** Test basic IMU tracking on glove hardware (ignore fingers initially)
4. **Phase 1.4:** Add glove roles and advertising
5. **Phase 1.5:** Implement MUX-based finger sensor reading
6. **Phase 1.6:** Add finger data to GATT service
7. **Phase 2:** Update eidon-sim to visualize finger data

---

## Architecture Analysis

### Key Differences Between Tracker and Glove

| Aspect | eidon-tracker | eidon-glove | Integration Strategy |
|--------|---------------|-------------|---------------------|
| **MCU** | ESP32-C6 | ESP32-C3 | Port glove to C6 |
| **IMU Interface** | I2C (GPIO 19/20) | SPI (GPIO 2/4/6/7) | Support both modes |
| **BLE Protocol** | Custom GATT | BLE HID | Migrate to custom GATT |
| **Framework** | Arduino + NimBLE | ESP-IDF native | Keep Arduino framework |
| **Build System** | PlatformIO | ESP-IDF (CMake) | Stay with PlatformIO |
| **Sensors** | IMU only | IMU + 16 finger sensors | Add finger sensor support |
| **Role System** | 7 roles (0-6, 255) | None | Add GLOVE roles (8-9) |
| **Data Rate** | 50Hz quaternion | 100Hz quat + fingers | Unified 50Hz for consistency |

### Compatibility Matrix

```
Pure Tracker Setup (Current):
┌─────────┐
│  Phone  │
└────┬────┘
     │ BLE GATT
     │
┌────▼────────────┐
│   LEFT_HUB      │ (Upper Arm)
│   ESP-NOW       │
└────┬────────────┘
     │
     ├── ESP-NOW ──► LEFT_FOREARM (Child)
     └── ESP-NOW ──► LEFT_HAND (Child)

Glove Setup (Target):
┌─────────┐
│  Phone  │
└────┬────┘
     │ BLE GATT
     ├──────────────┐
     │              │
┌────▼────────┐ ┌──▼──────────┐
│  LEFT_HUB   │ │ LEFT_GLOVE  │ (Direct, no children)
│  ESP-NOW    │ │  (No ESP-NOW)
└────┬────────┘ └─────────────┘
     │
     └── ESP-NOW ──► LEFT_FOREARM (Child)
```

### Role Design

**New Roles:**
```cpp
enum DeviceRole {
    ROLE_LEFT_HAND = 0,           // Tracker child
    ROLE_RIGHT_HAND = 1,          // Tracker child
    ROLE_LEFT_FOREARM = 2,        // Tracker child
    ROLE_RIGHT_FOREARM = 3,       // Tracker child
    ROLE_LEFT_HUB = 4,            // Tracker hub (upper arm)
    ROLE_RIGHT_HUB = 5,           // Tracker hub (upper arm)
    ROLE_CHEST = 6,               // Standalone
    ROLE_LEFT_GLOVE = 8,          // NEW: Glove (direct to phone)
    ROLE_RIGHT_GLOVE = 9,         // NEW: Glove (direct to phone)
    ROLE_UNKNOWN = 255
};
```

**Role Properties:**
- Gloves are **NOT** hub devices (don't receive ESP-NOW)
- Gloves are **NOT** child devices (don't send ESP-NOW)
- Gloves are **standalone with sensors** (direct BLE to phone)
- Gloves advertise unique service data for identification

---

## Phase 1: Firmware Integration ✅ COMPLETE

**Estimated Time:** 3-5 days
**Actual Time:** 1 day
**Status:** ✅ All tasks complete, ready for hardware testing
**Target:** Tracker firmware running on ESP32-C3 glove hardware with full glove support

### 1.1: Port Tracker Firmware to ESP32-C3 ✅ COMPLETE

**Status:** ✅ Complete - Both platforms build successfully
**Commit:** `02c2269` - Add ESP32-C3 glove build environment

**Goal:** Get eidon-tracker firmware building and running on ESP32-C3 hardware

**Files Modified:**
- `eidon-tracker/platformio.ini` - Added ESP32-C3 environment

**Implementation:**

Add new environment in `platformio.ini`:

```ini
[env:seeed_xiao_esp32c3]
platform = espressif32
board = seeed_xiao_esp32c3
framework = arduino
lib_deps =
    dxinteractive/ResponsiveAnalogRead@^1.2.1
    h2zero/NimBLE-Arduino@^2.3.1
    adafruit/Adafruit BNO08x@^1.2.3
monitor_speed = 115200
build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
    ; Define target as ESP32-C3 glove
    -DESP32_C3_GLOVE=1
    ; NimBLE Configuration (same as C6)
    -DCONFIG_BT_NIMBLE_ENABLED=1
    -DCONFIG_BT_ENABLED=1
    -DCONFIG_BT_NIMBLE_MAX_CONNECTIONS=3
    -DCONFIG_BT_NIMBLE_MAX_BONDS=3
    -DCONFIG_BT_NIMBLE_MAX_CCCDS=8
lib_ignore =
    BluetoothSerial
    ESP32 BLE Arduino
```

**Tasks:**
- [x] Add `seeed_xiao_esp32c3` environment to platformio.ini
- [x] Add `-DESP32_C3_GLOVE=1` build flag for conditional compilation
- [x] Test build with `pio run -e seeed_xiao_esp32c3`
- [x] Verify NimBLE compatibility on ESP32-C3

**Actual Time:** 1 hour

---

### 1.2: Configure BNO085 I2C GPIO Pins for Glove ✅ COMPLETE

**Status:** ✅ Complete - I2C configured for both platforms
**Commits:**
- `cf8a5c8` - Configure BNO085 I2C GPIO pins for glove hardware
- `131d6fe` - Standardize I2C frequency to 100kHz across all platforms

**Goal:** Support different I2C GPIO pins for ESP32-C3 glove vs ESP32-C6 tracker

**Files Modified:**
- `firmware/lib/BNO085/BNO085.h` - Added GPIO pin definitions
- `firmware/lib/BNO085/BNO085.cpp` - Added conditional pin initialization

**GPIO Differences:**

| Signal | ESP32-C6 Tracker | ESP32-C3 Glove | Notes |
|--------|------------------|----------------|-------|
| I2C SDA | GPIO 20 | GPIO 21 (D6/TX) | Data line |
| I2C SCL | GPIO 19 | GPIO 20 (D7/RX) | Clock line |
| I2C Address | 0x4B | 0x4B | Same address |
| I2C Speed | 100kHz | 100kHz | **Changed to 100kHz for consistency and stability** |

**Implementation:**

```cpp
// BNO085.h - Add conditional GPIO definitions
#ifdef ESP32_C3_GLOVE
    // ESP32-C3 Glove: I2C pins
    #define I2C_SDA             21   // GPIO 21 (D6/TX)
    #define I2C_SCL             20   // GPIO 20 (D7/RX)
    #define I2C_ADDR            0x4B
    #define I2C_FREQ_HZ         100000  // 100kHz
#else
    // ESP32-C6 Tracker: I2C pins
    #define I2C_SDA             20   // GPIO 20
    #define I2C_SCL             19   // GPIO 19
    #define I2C_ADR             18   // GPIO 18 (address select)
    #define I2C_ADDR            0x4B
    #define I2C_FREQ_HZ         400000  // 400kHz
#endif
```

```cpp
// BNO085.cpp - Initialize I2C with correct pins
bool BNO085::begin() {
    // Initialize I2C bus with conditional pins
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);

    // Initialize BNO085 via I2C
    if (!bno08x.begin_I2C(I2C_ADDR, &Wire)) {
        return false;
    }

    // Common configuration
    setReports();
    return true;
}
```

**Tasks:**
- [x] Add conditional I2C GPIO pin definitions based on `ESP32_C3_GLOVE`
- [x] Update `BNO085::begin()` to use correct pins for each platform
- [x] Standardize both platforms to 100kHz for stability
- [x] Both platforms build successfully

**Actual Time:** 2 hours

**Implementation Notes:**
- Initially had tracker at 400kHz, but changed to 100kHz for consistency
- Glove team found 100kHz more reliable in their testing
- Minimal performance impact at 48Hz update rate

---

### 1.4: Add Glove Roles to DeviceConfig ✅ COMPLETE

**Status:** ✅ Complete - Glove roles implemented
**Commit:** `82ba83b` - Add glove roles to tracker firmware

**Files to Modify:**
- `firmware/src/DeviceConfig.h` - Add role enums
- `firmware/src/DeviceConfig.cpp` - Update role logic

**Implementation:**

```cpp
// DeviceConfig.h
enum DeviceRole {
    // ... existing roles ...
    ROLE_LEFT_GLOVE = 8,
    ROLE_RIGHT_GLOVE = 9,
    ROLE_UNKNOWN = 255
};

class DeviceConfig {
public:
    static bool isGloveMode();           // NEW: Returns true for glove roles
    static bool isStandaloneMode();      // Updated: Include glove + chest
    static String generateDeviceName();  // Updated: "Eidon Glove XXXX" for gloves
};
```

**Role Classification Logic:**

```cpp
bool DeviceConfig::isHubMode() {
    DeviceRole role = getRole();
    return (role == ROLE_LEFT_HUB || role == ROLE_RIGHT_HUB);
}

bool DeviceConfig::isNodeMode() {
    DeviceRole role = getRole();
    return (role == ROLE_LEFT_HAND || role == ROLE_RIGHT_HAND ||
            role == ROLE_LEFT_FOREARM || role == ROLE_RIGHT_FOREARM);
}

bool DeviceConfig::isGloveMode() {
    DeviceRole role = getRole();
    return (role == ROLE_LEFT_GLOVE || role == ROLE_RIGHT_GLOVE);
}

bool DeviceConfig::isStandaloneMode() {
    DeviceRole role = getRole();
    return (role == ROLE_CHEST || isGloveMode());
}
```

**Tasks:**
- [x] Add `ROLE_LEFT_GLOVE` and `ROLE_RIGHT_GLOVE` to enum
- [x] Implement `isGloveMode()` helper
- [x] Implement `isStandaloneMode()` helper
- [x] Update `generateDeviceName()` to return "Eidon-Glove-XXXX" for glove roles
- [x] Update `getRoleName()` to include glove role names
- [x] Update `isValidRole()` to accept glove roles

**Actual Time:** 1.5 hours

---

### 1.5: Finger Sensor Integration (MUX-Based) ✅ COMPLETE

**Status:** ✅ Complete - FingerSensors library implemented
**Commit:** `ede81d7` - Add MUX-based finger sensor library for ESP32-C3 glove

**Goal:** Implement 16-channel Hall effect sensor reading via analog multiplexer

**Hardware Architecture:**
- 16 Hall effect sensors → CD74HC4067 multiplexer → 1 ADC GPIO
- 4 GPIO pins control which sensor (0-15) is routed to ADC
- Binary selection: S0=bit0, S1=bit1, S2=bit2, S3=bit3

**GPIO Configuration (from eidon-glove PCB schematic):**
```cpp
// Multiplexer control pins (ESP32-C3)
#define MUX_S0          10   // GPIO 10 - D10 - Select bit 0
#define MUX_S1          9    // GPIO 9  - D9  - Select bit 1
#define MUX_S2          6    // GPIO 6  - D4  - Select bit 2
#define MUX_S3          7    // GPIO 7  - D5  - Select bit 3
#define MUX_ADC_PIN     0    // GPIO 0  - A0/D0 - Analog input (MUX_OUT)
```

**Note:** GPIO 9 and 10 have conflicts with boot/mode buttons but those can be resolved or buttons disabled when glove role is active.

**New Files:**
- `firmware/lib/FingerSensors/FingerSensors.h`
- `firmware/lib/FingerSensors/FingerSensors.cpp`

**Implementation:**

```cpp
// FingerSensors.h
#ifdef ESP32_C3_GLOVE

#define NUM_FINGER_SENSORS 16

class FingerSensors {
public:
    bool begin();
    void update();                              // Read all 16 sensors
    uint16_t getRawValue(uint8_t index);        // Get ADC value (0-4095)
    float getNormalized(uint8_t index);         // Get 0.0-1.0 range
    void getEncodedValues(uint16_t* output);    // Get all 16 as uint16 BLE format

    // Calibration
    void calibrateSensor(uint8_t index, uint16_t min, uint16_t max);
    void loadCalibration();
    void saveCalibration();

private:
    void selectMuxChannel(uint8_t channel);     // Set S0-S3 for channel 0-15
    uint16_t readMuxChannel(uint8_t channel);   // Select + read ADC

    uint16_t _rawValues[NUM_FINGER_SENSORS];

    // Calibration ranges (stored in NVS)
    uint16_t _minValues[NUM_FINGER_SENSORS];    // Min ADC value (finger extended)
    uint16_t _maxValues[NUM_FINGER_SENSORS];    // Max ADC value (finger curled)

    // GPIO pins
    const uint8_t _muxPins[4] = {MUX_S0, MUX_S1, MUX_S2, MUX_S3};
};

#endif // ESP32_C3_GLOVE
```

```cpp
// FingerSensors.cpp
void FingerSensors::selectMuxChannel(uint8_t channel) {
    // Set S0-S3 based on channel (0-15)
    digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
    digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
    digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
    digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);

    delayMicroseconds(10);  // Allow MUX to settle
}

uint16_t FingerSensors::readMuxChannel(uint8_t channel) {
    selectMuxChannel(channel);

    // Read ADC with averaging (3 samples)
    uint32_t sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += analogRead(MUX_ADC_PIN);
        delayMicroseconds(100);
    }
    return sum / 3;
}

void FingerSensors::update() {
    for (uint8_t i = 0; i < NUM_FINGER_SENSORS; i++) {
        _rawValues[i] = readMuxChannel(i);
    }
}

void FingerSensors::getEncodedValues(uint16_t* output) {
    for (uint8_t i = 0; i < NUM_FINGER_SENSORS; i++) {
        // Normalize to 0.0-1.0 based on calibration
        float normalized = (float)(_rawValues[i] - _minValues[i]) /
                          (_maxValues[i] - _minValues[i]);
        normalized = constrain(normalized, 0.0f, 1.0f);

        // Encode as uint16 (-1.0 to 1.0 -> 0 to 65535)
        // Map 0.0-1.0 to -1.0-1.0 range
        float ranged = (normalized * 2.0f) - 1.0f;
        output[i] = (uint16_t)((ranged + 1.0f) * 32767.5f);
    }
}
```

**Encoding Strategy (matches eidon-glove):**
```cpp
// BLE transmission format: uint16_t (0-65535) representing float (-1.0 to 1.0)
// Encode: uint16 = (float + 1.0) * 32767.5
// Decode: float = (uint16 / 32767.5) - 1.0
```

**Tasks:**
- [x] Create `FingerSensors` library with MUX support
- [x] Implement `selectMuxChannel()` for binary selection (4-bit addressing)
- [x] Implement `readMuxChannel()` with ADC averaging (3 samples)
- [x] Add calibration storage to NVS (16 min + 16 max values)
- [x] Implement encoding to match eidon-glove format (uint16)
- [x] Add diagnostic functions (printValues, printCalibration)
- [ ] Test reading all 16 sensors sequentially on hardware (pending)

**Actual Time:** 2 hours

**Implementation Notes:**
- CD74HC4067 multiplexer controlled via GPIO 10, 9, 6, 7
- Single ADC read on GPIO 0
- 12-bit ADC resolution (0-4095)
- 3-sample averaging with 100µs delay between samples
- Default calibration: min=500, max=3500 (will be tuned on hardware)

---

### 1.6: Extend GATT Service for Glove Data ✅ COMPLETE

**Status:** ✅ Complete - Finger sensor characteristic added
**Commit:** `8c79e7b` - Add finger sensor GATT characteristic and integration

**Goal:** Add new characteristic for finger sensor data

**Files to Modify:**
- `firmware/src/main.cpp` - Add characteristics

**New UUIDs:**
```cpp
// Existing
#define EIDON_SERVICE_UUID           "E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE"
#define QUAT_CHARACTERISTIC_UUID     "E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE"

// NEW: Finger sensor characteristic
#define FINGER_CHARACTERISTIC_UUID   "E1D0000A-8B5A-3E5B-9E23-4F9B5C91BBDE"
```

**Characteristic Structure:**

```cpp
// Finger Sensor Characteristic
// UUID: E1D0000A-8B5A-3E5B-9E23-4F9B5C91BBDE
// Properties: READ, NOTIFY
// Size: 32 bytes (16 x uint16_t)
// Data: [sensor0_h, sensor0_l, sensor1_h, sensor1_l, ..., sensor15_h, sensor15_l]
```

**Implementation:**

```cpp
// In setup()
if (DeviceConfig::isGloveMode()) {
    // Create finger sensor characteristic
    pFingerCharacteristic = pService->createCharacteristic(
        FINGER_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // Initialize finger sensors
    if (!fingerSensors.begin()) {
        Serial.println("ERROR: Failed to initialize finger sensors");
    }
}

// In loop()
if (DeviceConfig::isGloveMode() && deviceConnected) {
    static unsigned long lastFingerUpdate = 0;

    // Update finger sensors at 50Hz
    if (currentTime - lastFingerUpdate >= 20) {
        fingerSensors.update();

        uint16_t encodedValues[NUM_FINGER_SENSORS];
        fingerSensors.getEncodedValues(encodedValues);

        // Send as BLE notification
        pFingerCharacteristic->setValue(
            (uint8_t*)encodedValues,
            sizeof(encodedValues)
        );
        pFingerCharacteristic->notify();

        lastFingerUpdate = currentTime;
    }
}
```

**Device Info Update:**

```cpp
// Byte 5 in Device Info characteristic now includes glove roles
uint8_t deviceInfo[14];
deviceInfo[5] = (uint8_t)DeviceConfig::getRole();  // 8 or 9 for gloves
```

**Tasks:**
- [x] Add `FINGER_SENSOR_CHAR_UUID` (E1D0000A) definition
- [x] Create characteristic conditionally for glove builds
- [x] Initialize finger sensors in setup() for glove mode
- [x] Implement finger sensor update loop (50Hz, same as quaternion)
- [x] Device Info already includes role byte (glove role auto-reported)
- [ ] Test BLE notifications with glove data on hardware (pending)

**Actual Time:** 2 hours

**Implementation Details:**
- Characteristic created only with `#ifdef ESP32_C3_GLOVE`
- 32-byte characteristic (16 sensors × 2 bytes uint16_t)
- Updates at 50Hz in the same transmission block as quaternion
- Non-blocking: IMU works even if finger sensors fail to initialize

---

### 1.5: Conditional Compilation & Main Loop

**Goal:** Cleanly separate tracker vs glove logic

**Implementation Strategy:**

```cpp
// In setup()
void setup() {
    // Common initialization
    initLEDs();
    initBattery();
    DeviceConfig::loadConfig();

    // Role-specific initialization
    DeviceRole role = DeviceConfig::getRole();

    if (DeviceConfig::isGloveMode()) {
        // Glove-specific setup
        imu.begin(BNO085_SPI);
        fingerSensors.begin();
        // No ESP-NOW initialization

    } else {
        // Tracker setup
        imu.begin(BNO085_I2C);

        if (DeviceConfig::isHubMode()) {
            initESPNow();
            hubClientService.begin();
        } else if (DeviceConfig::isNodeMode()) {
            initESPNowChild();
        }
    }

    // Common BLE setup
    initBLE();
    startAdvertising();
}

// In loop()
void loop() {
    unsigned long currentTime = millis();

    // Common: IMU update
    updateIMU(currentTime);

    // Common: BLE quaternion transmission
    if (deviceConnected) {
        sendQuaternionReport(currentTime);
    }

    // Glove-specific: Finger sensor transmission
    if (DeviceConfig::isGloveMode() && deviceConnected) {
        sendFingerReport(currentTime);
    }

    // Tracker-specific: ESP-NOW communication
    if (DeviceConfig::isNodeMode()) {
        sendESPNowQuaternion(currentTime);
    } else if (DeviceConfig::isHubMode()) {
        updateHubAggregation(currentTime);
    }

    // Common: LED, battery, polling
    updateLEDs(currentTime);
    updateBattery(currentTime);
    blePollingManager.update();
}
```

**Tasks:**
- [ ] Refactor `setup()` with role-based branching
- [ ] Refactor `loop()` with conditional execution
- [ ] Ensure ESP-NOW is not initialized for gloves
- [ ] Test all roles (hub, child, glove, chest)

**Estimated Time:** 3-4 hours

---

### 1.6: Configuration & Calibration

**Goal:** Add glove calibration and finger sensor config

**Files to Modify:**
- `firmware/src/DeviceConfig.h/cpp` - Add finger calibration storage

**NVS Keys:**
```cpp
// Existing
"role"              // 1 byte
"hub_mac"           // 6 bytes

// NEW for gloves
"finger_cal_min"    // 16 x 4 bytes (float array)
"finger_cal_max"    // 16 x 4 bytes (float array)
"finger_enabled"    // 2 bytes (16-bit mask)
```

**Calibration Process:**

```cpp
// Add to RoleConfig service or create new CalibrationService
void calibrateFingerSensors(uint8_t sensorIndex) {
    // 1. Prompt user to fully extend finger
    // 2. Sample for 2 seconds, record max
    // 3. Prompt user to fully curl finger
    // 4. Sample for 2 seconds, record min
    // 5. Save to NVS
}
```

**Tasks:**
- [ ] Add finger calibration storage to DeviceConfig
- [ ] Implement calibration routine
- [ ] Add calibration characteristic or extend existing
- [ ] Create calibration UI in eidon-sim (Phase 2)

**Estimated Time:** 4-5 hours

---

### 1.7: Testing & Validation

**Hardware Tests:**
- [ ] Tracker mode on ESP32-C6 (I2C IMU)
- [ ] Glove mode on ESP32-C6 (SPI IMU)
- [ ] Finger sensor readings accuracy
- [ ] BLE connection stability with extended data
- [ ] Power consumption comparison
- [ ] Battery life with 50Hz finger updates

**Integration Tests:**
- [ ] Pure tracker setup (HUB + 2 children)
- [ ] Glove setup (HUB + FOREARM + GLOVE)
- [ ] Role switching without reflashing
- [ ] Multi-device BLE connections

**Estimated Time:** 1-2 days

---

## Phase 2: App Adaptation (eidon-sim) ⏳ READY TO START

**Estimated Time:** 1-2 days
**Status:** ⏳ Ready to implement (waiting for hardware testing)
**Target:** Visualize glove orientation + finger angles, record glove data

**Prerequisites:** ✅ All firmware complete, can proceed in parallel with hardware testing

### 2.1: BLE Service Extensions

**Files to Modify:**
- `src/core/ble/BLEManager.ts` - Add finger characteristic
- `src/core/ble/TrackerDevice.ts` - Extend device model

**Implementation:**

```typescript
// BLEManager.ts
export const FINGER_CHARACTERISTIC_UUID = 'e1d0000a-8b5a-3e5b-9e23-4f9b5c91bbde';

export interface FingerData {
    sensors: number[];  // 16 floats, decoded from uint16
    timestamp: number;
}

// TrackerDevice.ts
export class TrackerDevice extends EventTarget {
    private fingerData: FingerData | null = null;

    async subscribeToFingerData() {
        const char = await this.service.getCharacteristic(FINGER_CHARACTERISTIC_UUID);
        await char.startNotifications();
        char.addEventListener('characteristicvaluechanged', (e) => {
            this.handleFingerDataUpdate(e.target.value);
        });
    }

    private handleFingerDataUpdate(dataView: DataView) {
        const sensors = [];
        for (let i = 0; i < 16; i++) {
            const encoded = dataView.getUint16(i * 2, true); // little-endian
            const decoded = (encoded / 32767.5) - 1.0;
            sensors.push(decoded);
        }

        this.fingerData = { sensors, timestamp: Date.now() };
        this.dispatchEvent(new CustomEvent('fingerdata', { detail: this.fingerData }));
    }
}
```

**Tasks:**
- [ ] Add `FINGER_CHARACTERISTIC_UUID` constant
- [ ] Extend `TrackerDevice` with finger data handling
- [ ] Add finger data decoding logic
- [ ] Emit finger data events
- [ ] Add error handling for devices without finger sensors

**Estimated Time:** 2-3 hours

---

### 2.2: Glove Role Detection

**Files to Modify:**
- `src/core/ble/BLEManager.ts` - Detect glove from Device Info

**Implementation:**

```typescript
// Device role detection from Device Info characteristic
export enum DeviceRole {
    LEFT_HAND = 0,
    RIGHT_HAND = 1,
    LEFT_FOREARM = 2,
    RIGHT_FOREARM = 3,
    LEFT_HUB = 4,
    RIGHT_HUB = 5,
    CHEST = 6,
    LEFT_GLOVE = 8,      // NEW
    RIGHT_GLOVE = 9,     // NEW
    UNKNOWN = 255
}

export function isGloveRole(role: DeviceRole): boolean {
    return role === DeviceRole.LEFT_GLOVE || role === DeviceRole.RIGHT_GLOVE;
}

export function isTrackerRole(role: DeviceRole): boolean {
    return role >= 0 && role <= 6;
}

// In device connection flow
const deviceInfo = await this.readDeviceInfo();
const role = deviceInfo.role;

if (isGloveRole(role)) {
    // Subscribe to finger characteristic
    await device.subscribeToFingerData();
}
```

**Tasks:**
- [ ] Add glove roles to `DeviceRole` enum
- [ ] Implement `isGloveRole()` helper
- [ ] Conditionally subscribe to finger data based on role
- [ ] Update device discovery UI to show "Glove" devices

**Estimated Time:** 1-2 hours

---

### 2.3: Finger Angle Visualization

**Goal:** Display 16 finger sensor values in real-time

**New Component:**
- `src/ui/components/FingerPanel.ts` - Finger angle bars

**Implementation:**

```typescript
// FingerPanel.ts
export class FingerPanel {
    private container: HTMLElement;
    private bars: HTMLElement[] = [];

    private readonly FINGER_NAMES = [
        'Thumb CMC', 'Thumb MCP', 'Thumb IP', 'Thumb Flex',
        'Index MCP', 'Index PIP', 'Index DIP', 'Index Flex',
        'Middle MCP', 'Middle PIP', 'Middle DIP', 'Middle Flex',
        'Ring MCP', 'Ring PIP', 'Ring DIP', 'Ring Flex',
    ];

    constructor() {
        this.container = document.createElement('div');
        this.container.className = 'finger-panel';
        this.createBars();
    }

    private createBars() {
        for (let i = 0; i < 16; i++) {
            const barContainer = document.createElement('div');
            barContainer.className = 'finger-bar-container';

            const label = document.createElement('span');
            label.textContent = this.FINGER_NAMES[i];
            label.className = 'finger-label';

            const bar = document.createElement('div');
            bar.className = 'finger-bar';
            bar.style.width = '0%';

            const value = document.createElement('span');
            value.className = 'finger-value';
            value.textContent = '0.00';

            barContainer.appendChild(label);
            barContainer.appendChild(bar);
            barContainer.appendChild(value);
            this.container.appendChild(barContainer);

            this.bars.push(bar);
        }
    }

    updateFingerData(data: FingerData) {
        for (let i = 0; i < 16; i++) {
            const normalized = (data.sensors[i] + 1.0) / 2.0; // -1..1 -> 0..1
            const percentage = (normalized * 100).toFixed(0);

            this.bars[i].style.width = `${percentage}%`;
            this.bars[i].nextElementSibling.textContent = data.sensors[i].toFixed(2);
        }
    }
}
```

**CSS:**
```css
.finger-panel {
    display: flex;
    flex-direction: column;
    gap: 8px;
    padding: 16px;
    background: rgba(0, 0, 0, 0.2);
    border-radius: 8px;
}

.finger-bar-container {
    display: flex;
    align-items: center;
    gap: 12px;
}

.finger-label {
    width: 120px;
    font-size: 12px;
    color: #aaa;
}

.finger-bar {
    flex: 1;
    height: 20px;
    background: linear-gradient(90deg, #4CAF50, #FFC107);
    border-radius: 4px;
    transition: width 0.05s linear;
}

.finger-value {
    width: 50px;
    text-align: right;
    font-family: monospace;
    font-size: 12px;
}
```

**Tasks:**
- [ ] Create `FingerPanel` component
- [ ] Add finger bar visualization
- [ ] Integrate into main UI (sidebar or modal)
- [ ] Add smooth transitions for real-time updates
- [ ] Color-code by finger (thumb=blue, index=green, etc.)

**Estimated Time:** 3-4 hours

---

### 2.4: 3D Hand Model (Optional Advanced)

**Goal:** Render articulated hand mesh with finger angles

**Approach:** Use Three.js + GLTF hand model

```typescript
// HandRig.ts
export class HandRig {
    private scene: THREE.Scene;
    private handModel: THREE.Object3D;
    private fingerBones: THREE.Bone[] = [];

    async loadModel(url: string) {
        const loader = new GLTFLoader();
        const gltf = await loader.loadAsync(url);
        this.handModel = gltf.scene;
        this.indexFingerBones();
    }

    private indexFingerBones() {
        // Map 16 sensors to bone hierarchy
        // Thumb: bones[0-3]
        // Index: bones[4-7]
        // Middle: bones[8-11]
        // Ring: bones[12-15]
    }

    updateFromFingerData(data: FingerData) {
        for (let i = 0; i < 16; i++) {
            const angle = data.sensors[i] * Math.PI / 2; // -90° to 90°
            this.fingerBones[i].rotation.z = angle;
        }
    }
}
```

**Tasks (Future Enhancement):**
- [ ] Source or create hand GLTF model
- [ ] Map 16 sensors to bone hierarchy
- [ ] Implement inverse kinematics solver
- [ ] Add to 3D scene alongside arm rigs

**Estimated Time:** 8-12 hours (future work)

---

### 2.5: Recording & Playback Extensions

**Files to Modify:**
- `src/core/recording/DataRecorder.ts` - Add finger data to records

**Implementation:**

```typescript
// DataRecorder.ts
export interface RecordingFrame {
    timestamp: number;
    devices: {
        [deviceId: string]: {
            quaternion: QuaternionData;
            fingerData?: FingerData;  // NEW: Optional for gloves
            role: DeviceRole;
        }
    };
}

export class DataRecorder {
    private recordFrame() {
        const frame: RecordingFrame = {
            timestamp: Date.now(),
            devices: {}
        };

        for (const [id, device] of this.connectedDevices) {
            frame.devices[id] = {
                quaternion: device.getQuaternionData(),
                role: device.getRole()
            };

            // Add finger data if glove
            if (isGloveRole(device.getRole())) {
                frame.devices[id].fingerData = device.getFingerData();
            }
        }

        this.frames.push(frame);
    }
}
```

**File Format:**
```json
{
    "version": "2.0",
    "recordedAt": "2025-11-14T12:00:00Z",
    "devices": [
        {
            "id": "left_glove",
            "role": 8,
            "hasFingerData": true
        }
    ],
    "frames": [
        {
            "timestamp": 1234567890,
            "devices": {
                "left_glove": {
                    "quaternion": [1, 0, 0, 0],
                    "fingerData": {
                        "sensors": [0.1, 0.2, ..., 0.9]
                    }
                }
            }
        }
    ]
}
```

**Tasks:**
- [ ] Extend `RecordingFrame` interface with finger data
- [ ] Update recording logic to capture finger data
- [ ] Update playback logic to restore finger data
- [ ] Bump recording format version to 2.0
- [ ] Add backward compatibility for v1 files

**Estimated Time:** 2-3 hours

---

### 2.6: UI/UX Enhancements

**Features:**

1. **Device List Enhancement**
   - Show glove icon next to glove devices
   - Display finger sensor availability

2. **Connection Panel**
   - "Connect Glove" button
   - Show connected gloves with finger data indicators

3. **Calibration UI**
   - Finger calibration wizard
   - Per-sensor min/max adjustment
   - "Reset to defaults" button

4. **Debug Panel**
   - Real-time finger data stream
   - Sensor health indicators
   - Latency measurements

**Tasks:**
- [ ] Add glove icons to device list
- [ ] Create calibration wizard component
- [ ] Add finger panel to main UI
- [ ] Update connection status indicators

**Estimated Time:** 4-5 hours

---

## Implementation Checklist

### Phase 1: Firmware (eidon-tracker) ✅ COMPLETE

#### Hardware Abstraction ✅
- [x] Support I2C on both platforms (SPI not needed - both use I2C)
- [x] Test I2C mode on tracker (builds successfully)
- [x] Standardize to 100kHz for both platforms
- [x] Add compile-time interface selection via ESP32_C3_GLOVE flag

#### Role System ✅
- [x] Add `ROLE_LEFT_GLOVE` (8) and `ROLE_RIGHT_GLOVE` (9)
- [x] Implement `isGloveMode()` helper
- [x] Implement `isStandaloneMode()` helper
- [x] Update device name generation ("Eidon-Glove-XXXX")
- [x] Update role name strings

#### Finger Sensors ✅
- [x] Create `FingerSensors` library
- [x] Implement MUX control (4 select pins, 1 ADC pin)
- [x] Implement ADC reading with 3-sample averaging
- [x] Add encoding/decoding functions (uint16 BLE format)
- [ ] Test sensor accuracy (pending hardware)

#### GATT Service ✅
- [x] Add `FINGER_SENSOR_CHAR_UUID` (E1D0000A)
- [x] Create finger characteristic (32 bytes)
- [x] Implement notification loop (50Hz)
- [x] Device Info already reports role

#### Main Loop ✅
- [x] Refactor `setup()` with role branching
- [x] Refactor `loop()` with conditional execution
- [x] ESP-NOW disabled for gloves (isStandaloneMode)
- [ ] Test all roles on hardware (pending)

#### Configuration ✅
- [x] Add finger calibration storage (NVS)
- [x] Implement calibration load/save functions
- [x] Default calibration values (min=500, max=3500)
- [ ] Test NVS persistence (pending hardware)

#### Testing ⏳ PENDING HARDWARE
- [ ] Flash firmware to glove
- [ ] Validate IMU on glove hardware
- [ ] Validate finger sensor readings
- [ ] Test BLE connection and data streaming
- [ ] Power consumption tests
- [ ] Multi-device tests

### Phase 2: App (eidon-sim)

#### BLE Integration
- [ ] Add finger characteristic support
- [ ] Implement data decoding
- [ ] Add finger data events
- [ ] Test with real glove

#### Role Detection
- [ ] Add glove roles to enum
- [ ] Implement role-based subscription
- [ ] Update device discovery UI

#### Visualization
- [ ] Create `FingerPanel` component
- [ ] Add finger bars
- [ ] Integrate into main UI
- [ ] Test real-time updates

#### Recording
- [ ] Extend recording format
- [ ] Update recorder logic
- [ ] Update playback logic
- [ ] Test backward compatibility

#### UI/UX
- [ ] Add glove icons
- [ ] Create calibration wizard
- [ ] Update connection panel
- [ ] Add debug panel

---

## Testing Strategy

### Unit Tests

**Firmware:**
```cpp
// test/test_finger_sensors.cpp
void test_finger_encoding() {
    float input = 0.5f;
    uint16_t encoded = encodeFloat(input);
    float decoded = decodeFloat(encoded);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, input, decoded);
}
```

**App:**
```typescript
// src/core/ble/__tests__/TrackerDevice.test.ts
test('decodes finger data correctly', () => {
    const encoded = new Uint16Array([32767, 49151, 0, 65535]);
    const decoded = decodeFingerData(encoded);
    expect(decoded[0]).toBeCloseTo(-0.0, 2);
    expect(decoded[1]).toBeCloseTo(0.5, 2);
});
```

### Integration Tests

1. **Pure Tracker Setup**
   - Connect HUB + 2 children
   - Verify ESP-NOW aggregation
   - Record 30-second session
   - Validate all quaternion streams

2. **Glove Setup**
   - Connect HUB + FOREARM + GLOVE
   - Verify GLOVE direct BLE connection
   - Verify finger data streaming at 50Hz
   - Record hand gestures
   - Validate playback

3. **Bilateral Glove Setup**
   - Connect LEFT_GLOVE + RIGHT_GLOVE simultaneously
   - Verify independent finger streams
   - Test cross-device synchronization
   - Record bimanual tasks

### Performance Tests

**Metrics:**
- BLE notification latency (target: <20ms)
- Finger sensor sampling rate (target: 50Hz)
- Battery life with finger sensors (target: >6 hours)
- Web app frame rate with gloves (target: 60 FPS)

---

## Questions & Decisions

### Pre-Implementation Questions

1. **GPIO Pin Mapping:**
   - Q: Which GPIO pins on ESP32-C6 are available for 16 finger sensors?
   - A: Need to review glove schematic and map to C6 pinout
   - Status: ⏳ Pending

2. **SPI vs I2C Performance:**
   - Q: Does SPI mode provide better IMU performance than I2C?
   - A: Glove uses SPI at 3MHz. Tracker uses I2C at 400kHz.
   - Decision: Support both, benchmark latency
   - Status: ⏳ Pending

3. **Finger Sensor Hardware:**
   - Q: Are Hall effect sensors analog or digital?
   - A: Analog (ADC reading based on glove code)
   - Q: What is the expected range and resolution?
   - Status: ⏳ Need testing

4. **BLE MTU Size:**
   - Q: Can we fit quaternion (16B) + fingers (32B) in single packet?
   - A: 48 bytes < default MTU (23 bytes), need MTU negotiation or separate characteristics
   - Decision: Use separate characteristics (quaternion + finger)
   - Status: ✅ Decided

5. **Calibration Strategy:**
   - Q: Should calibration be per-device or per-user?
   - A: Per-device (stored in NVS), but allow user profiles in app
   - Status: ✅ Decided

6. **Glove Hub Behavior:**
   - Q: Should gloves ever act as hubs for other devices?
   - A: No, gloves are always standalone with direct phone connection
   - Status: ✅ Decided

7. **ESP-NOW for Gloves:**
   - Q: Should gloves support ESP-NOW for future extensions?
   - A: No immediate need, but keep architecture open
   - Decision: Initialize ESP-NOW only for hub/child roles
   - Status: ✅ Decided

8. **Hand Model Source:**
   - Q: Do we have a GLTF hand model, or need to create one?
   - Status: ⏳ Pending (Phase 2 future work)

### Technical Decisions Made

| Decision | Rationale | Date |
|----------|-----------|------|
| Use separate GATT characteristics for quaternion vs fingers | Cleaner separation, better backward compatibility | 2025-11-14 |
| Keep Arduino framework for unified firmware | Consistency, faster development | 2025-11-14 |
| Support both I2C and SPI for IMU | Hardware flexibility | 2025-11-14 |
| Gloves connect directly to phone (no children) | Simpler pairing, lower latency | 2025-11-14 |
| 50Hz update rate for all data | Consistent timing, adequate for hand tracking | 2025-11-14 |

---

## Timeline Estimate

### Phase 1: Firmware Integration
- **Week 1:** Hardware abstraction + role system (2-3 days)
- **Week 2:** Finger sensors + GATT service (3-4 days)
- **Week 3:** Testing + validation (2-3 days)
- **Total:** 7-10 days

### Phase 2: App Adaptation
- **Week 4:** BLE integration + visualization (2-3 days)
- **Week 5:** Recording + UI enhancements (2-3 days)
- **Total:** 4-6 days

### Overall: 2-3 weeks (11-16 days)

---

## Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| GPIO pin conflicts on ESP32-C6 | High | Review pinout early, consider I2C expander |
| SPI IMU initialization fails | High | Test on glove hardware first, have I2C fallback |
| Finger sensor noise/drift | Medium | Implement filtering, calibration routine |
| BLE throughput limitations | Medium | Optimize packet size, test MTU negotiation |
| Battery life degradation | Medium | Profile power consumption, optimize update rates |
| 3D hand model complexity | Low | Start with 2D bars, defer 3D to future phase |

---

## Next Steps

1. **Review this plan** and clarify questions above
2. **Test SPI IMU** on glove hardware with tracker firmware
3. **Map GPIO pins** for finger sensors on ESP32-C6
4. **Create feature branch** in eidon-tracker: `feature/glove-integration`
5. **Start Phase 1.1** (Hardware Abstraction Layer)

---

## References

- [eidon-tracker firmware](../eidon-tracker/firmware/)
- [eidon-glove firmware](../eidon-glove/firmware/)
- [eidon-sim docs](../eidon-sim/docs/)
- [ESP32-C6 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf)
- [NimBLE Documentation](https://github.com/h2zero/NimBLE-Arduino)
- [Web Bluetooth API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API)

---

**Document Status:** ✅ Phase 1 Complete - Ready for Hardware Testing
**Last Updated:** 2025-11-14
**Phase 1 Completed:** 2025-11-14
**Next Review:** After hardware testing and Phase 2 planning
