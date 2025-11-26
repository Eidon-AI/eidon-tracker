#include "FingerSensors.h"

#ifdef ESP32_C3_GLOVE

#include <Preferences.h>

// Static member initialization
const char* FingerSensors::CONFIG_NAMESPACE = "finger_cal";
const char* FingerSensors::CAL_MIN_KEY = "cal_min";
const char* FingerSensors::CAL_MAX_KEY = "cal_max";

// ResponsiveAnalogRead instance - same as eidon-glove
ResponsiveAnalogRead analog(MUX_ADC_PIN, true);

// Polynomial coefficients from eidon-glove HallEffectSensors.cpp
// These convert raw ADC values to "proto_angles"
static const float polyVals[16][3] = {
    {-0.000087481431887f, 0.549306011516565f, -709.440158534950912f},  // thumb 0
    {0.000043188683603f, -0.288631646308703f, 518.26001946236131f},    // thumb 1
    {0.000081944493116f, -0.48715545187493f, 753.215310445897971f},    // thumb 2
    {0.000029299702805f, -0.235958663536102f, 473.892439373476074f},   // thumb 3
    {-0.000005288207298f, 0.121258593336859f, -147.245901639344262f},  // pointer 4
    {-0.000135942468348f, 0.817456387325188f, -1033.093236601650843f}, // pointer 5
    {0.000101643291297f, -0.646716069346575f, 1031.971761445997989f},  // pointer 6
    {-0.000041474654378f, 0.275529953917051f, -295.161290322580645f},  // middle 7
    {-0.000155663598998f, 0.846081469596033f, -994.321241823930591f},  // middle 8
    {0.000170233984067f, -1.128460118194487f, 1768.951835332448657f},  // middle 9
    {-0.000041474654378f, 0.275529953917051f, -295.161290322580645f},  // ring 10
    {-0.000155663598998f, 0.846081469596033f, -994.321241823930591f},  // ring 11
    {0.000170233984067f, -1.128460118194487f, 1768.951835332448657f},  // ring 12
    {-0.000050156739812f, 0.308087774294671f, -325.54858934169279f},   // pinkie 13
    {-0.000204869267408f, 1.180238586110067f, -1522.071698458919325f}, // pinkie 14
    {0.00009027900176f, -0.57849114376526f, 925.953643298021097f},     // pinkie 15
};

bool FingerSensors::begin() {
    Serial.println("FingerSensors: Initializing...");

    // Configure multiplexer select pins as outputs
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);

    // Configure ADC pin
    pinMode(MUX_ADC_PIN, INPUT);

    // Set ADC resolution to 12-bit (0-4095)
    analogReadResolution(12);

    // Initialize arrays
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        _rawValues[i] = 0;
        _protoAngles[i] = 0.0f;
        _angles[i] = 0;
    }

    // Initialize sensor inversion flags (from eidon-glove main.cpp)
    // Index 0-3: Thumb (none inverted)
    // Index 4-6: Index (5 inverted)
    // Index 7-9: Middle (8 inverted)
    // Index 10-12: Ring (11 inverted)
    // Index 13-15: Pinky (14 inverted)
    _invertedSensors[0] = false;  // Thumb CMC Flex
    _invertedSensors[1] = false;  // Thumb CMC Abd
    _invertedSensors[2] = false;  // Thumb PIP
    _invertedSensors[3] = false;  // Thumb DIP
    _invertedSensors[4] = false;  // Index Abd
    _invertedSensors[5] = true;   // Index MCP (inverted)
    _invertedSensors[6] = false;  // Index PIP
    _invertedSensors[7] = false;  // Middle Abd
    _invertedSensors[8] = true;   // Middle MCP (inverted)
    _invertedSensors[9] = false;  // Middle PIP
    _invertedSensors[10] = false; // Ring Abd
    _invertedSensors[11] = true;  // Ring MCP (inverted)
    _invertedSensors[12] = false; // Ring PIP
    _invertedSensors[13] = false; // Pinky Abd
    _invertedSensors[14] = true;  // Pinky MCP (inverted)
    _invertedSensors[15] = false; // Pinky PIP

    _lastCalibSave = 0;

    // Load calibration from NVS
    loadCalibration();

    Serial.println("FingerSensors: Initialized with eidon-glove polynomial calibration");
    return true;
}

void FingerSensors::selectMuxChannel(uint8_t channel) {
    // Set S0-S3 based on channel (0-15) - matching eidon-glove bit order
    digitalWrite(MUX_S0, channel & 0x01);
    digitalWrite(MUX_S1, (channel >> 1) & 0x01);
    digitalWrite(MUX_S2, (channel >> 2) & 0x01);
    digitalWrite(MUX_S3, (channel >> 3) & 0x01);

    // Allow MUX to settle - 1ms like eidon-glove
    delay(1);
}

uint16_t FingerSensors::readMuxChannel(uint8_t channel) {
    selectMuxChannel(channel);

    // Use ResponsiveAnalogRead for smoothing - same as eidon-glove
    analog.update();
    return analog.getRawValue();
}

float FingerSensors::poly(float x, float a, float b, float c) {
    return a * x * x + b * x + c;
}

void FingerSensors::measureSensors() {
    // Read all 16 sensors sequentially
    for (uint8_t i = 0; i < NUM_FINGER_SENSORS; i++) {
        _rawValues[i] = readMuxChannel(i);
    }

    // Convert raw values to proto_angles using polynomial coefficients
    for (uint8_t i = 0; i < NUM_FINGER_SENSORS; i++) {
        _protoAngles[i] = poly(_rawValues[i], polyVals[i][0], polyVals[i][1], polyVals[i][2]);
    }
}

void FingerSensors::calibrateDynamic() {
    bool calibrationChanged = false;

    for (uint8_t i = 0; i < NUM_FINGER_SENSORS; i++) {
        if (_protoAngles[i] < _minAngles[i]) {
            _minAngles[i] = _protoAngles[i];
            calibrationChanged = true;
        } else if (_protoAngles[i] > _maxAngles[i]) {
            _maxAngles[i] = _protoAngles[i];
            calibrationChanged = true;
        }
    }

    // Save calibration periodically if it has changed
    if (calibrationChanged && (millis() - _lastCalibSave > CALIB_SAVE_INTERVAL)) {
        saveCalibration();
        _lastCalibSave = millis();
    }
}

int32_t FingerSensors::adjustMCPAbductionAngle(int32_t i) {
    float angle = _protoAngles[i];
    float max_angle = _maxAngles[i];
    float min_angle = _minAngles[i];

    // Avoid division by zero
    if (max_angle <= min_angle) return MCP_ABDUCTION_MAX;

    int32_t adjusted_angle = (int32_t)((angle - min_angle) / (max_angle - min_angle) * (2 * MCP_ABDUCTION_MAX));

    // Per-sensor adjustments from eidon-glove
    if (i == 4) {
        adjusted_angle += 10;
    } else if (i == 7) {
        adjusted_angle -= 10;
    } else if (i == 10 || i == 13) {
        adjusted_angle += 50;
    }

    // Apply inversion if needed
    if (_invertedSensors[i]) {
        adjusted_angle = (2 * MCP_ABDUCTION_MAX) - adjusted_angle;
    }

    return adjusted_angle;
}

int32_t FingerSensors::adjustMCPFlexionAngle(int32_t i) {
    float angle = _protoAngles[i];
    float max_angle = _maxAngles[i];
    float min_angle = _minAngles[i];

    // Avoid division by zero
    if (max_angle <= min_angle) return 0;

    int32_t adjusted_angle = (int32_t)((angle - min_angle) / (max_angle - min_angle) * MCP_FLEXION_MAX);

    // Apply inversion if needed
    if (_invertedSensors[i]) {
        adjusted_angle = MCP_FLEXION_MAX - adjusted_angle;
    }

    return adjusted_angle;
}

int32_t FingerSensors::adjustPIPFlexionAngle(int32_t i) {
    float angle = _protoAngles[i];
    float max_angle = _maxAngles[i];
    float min_angle = _minAngles[i];

    // Avoid division by zero
    if (max_angle <= min_angle) return 0;

    int32_t adjusted_angle = (int32_t)((angle - min_angle) / (max_angle - min_angle) * PIP_FLEXION_MAX);

    // Apply inversion if needed
    if (_invertedSensors[i]) {
        adjusted_angle = PIP_FLEXION_MAX - adjusted_angle;
    }

    return adjusted_angle;
}

int32_t FingerSensors::adjustThumbCMCFlexionAngle(int32_t i) {
    float angle = _protoAngles[i];
    float max_angle = _maxAngles[i];
    float min_angle = _minAngles[i];

    // Avoid division by zero
    if (max_angle <= min_angle) return 0;

    int32_t adjusted_angle = (int32_t)((angle - min_angle) / (max_angle - min_angle) * THUMB_CMC_FLEXION_MAX);

    // Apply inversion if needed
    if (_invertedSensors[i]) {
        adjusted_angle = THUMB_CMC_FLEXION_MAX - adjusted_angle;
    }

    return adjusted_angle;
}

int32_t FingerSensors::adjustThumbCMCAbductionAngle(int32_t i) {
    float angle = _protoAngles[i];
    float max_angle = _maxAngles[i];
    float min_angle = _minAngles[i];

    // Avoid division by zero
    if (max_angle <= min_angle) return THUMB_CMC_ABDUCTION_MAX;

    int32_t adjusted_angle = (int32_t)((angle - min_angle) / (max_angle - min_angle) * (2 * THUMB_CMC_ABDUCTION_MAX));

    // Apply inversion if needed
    if (_invertedSensors[i]) {
        adjusted_angle = (2 * THUMB_CMC_ABDUCTION_MAX) - adjusted_angle;
    }

    return adjusted_angle;
}

int32_t FingerSensors::adjustThumbPIPFlexionAngle(int32_t i) {
    float angle = _protoAngles[i];
    float max_angle = _maxAngles[i];
    float min_angle = _minAngles[i];

    // Avoid division by zero
    if (max_angle <= min_angle) return 0;

    int32_t adjusted_angle = (int32_t)((angle - min_angle) / (max_angle - min_angle) * THUMB_PIP_FLEXION_MAX);

    // Apply inversion if needed
    if (_invertedSensors[i]) {
        adjusted_angle = THUMB_PIP_FLEXION_MAX - adjusted_angle;
    }

    return adjusted_angle;
}

void FingerSensors::adjustAngles() {
    // Apply per-joint angle adjustments exactly like eidon-glove FingerTracking.cpp

    // Thumb (0-3)
    _angles[0] = adjustThumbCMCFlexionAngle(0);
    _angles[1] = adjustThumbCMCAbductionAngle(1);
    _angles[2] = adjustThumbPIPFlexionAngle(2);
    _angles[3] = adjustThumbPIPFlexionAngle(3); // Not using this data currently (like eidon-glove)

    // Index (4-6)
    _angles[4] = adjustMCPAbductionAngle(4);
    _angles[5] = adjustMCPFlexionAngle(5);
    _angles[6] = adjustPIPFlexionAngle(6);

    // Middle (7-9)
    _angles[7] = adjustMCPAbductionAngle(7);
    _angles[8] = adjustMCPFlexionAngle(8);
    _angles[9] = adjustPIPFlexionAngle(9);

    // Ring (10-12)
    _angles[10] = adjustMCPAbductionAngle(10);
    _angles[11] = adjustMCPFlexionAngle(11);
    _angles[12] = adjustPIPFlexionAngle(12);

    // Pinkie (13-15)
    _angles[13] = adjustMCPAbductionAngle(13);
    _angles[14] = adjustMCPFlexionAngle(14);
    _angles[15] = adjustPIPFlexionAngle(15);
}

void FingerSensors::update() {
    // Complete update cycle matching eidon-glove calcFingerAngles()
    measureSensors();       // Read raw ADC and compute proto_angles
    calibrateDynamic();     // Update min/max dynamically
    adjustAngles();         // Apply per-joint adjustments
}

uint16_t FingerSensors::getRawValue(uint8_t index) {
    if (index >= NUM_FINGER_SENSORS) {
        return 0;
    }
    return _rawValues[index];
}

int32_t FingerSensors::getAngle(uint8_t index) {
    if (index >= NUM_FINGER_SENSORS) {
        return 0;
    }
    return _angles[index];
}

float FingerSensors::getNormalized(uint8_t index) {
    if (index >= NUM_FINGER_SENSORS) {
        return 0.0f;
    }

    // Different max values for different joint types
    float maxValue;
    if (index == 0) {
        maxValue = THUMB_CMC_FLEXION_MAX;
    } else if (index == 1) {
        maxValue = 2 * THUMB_CMC_ABDUCTION_MAX; // Abduction goes from -125 to +125
    } else if (index == 2 || index == 3) {
        maxValue = THUMB_PIP_FLEXION_MAX;
    } else if (index == 4 || index == 7 || index == 10 || index == 13) {
        maxValue = 2 * MCP_ABDUCTION_MAX; // Abduction goes from -80 to +80
    } else if (index == 5 || index == 8 || index == 11 || index == 14) {
        maxValue = MCP_FLEXION_MAX;
    } else {
        maxValue = PIP_FLEXION_MAX;
    }

    // Normalize to 0.0-1.0
    float normalized = (float)_angles[index] / maxValue;

    // Clamp to 0.0-1.0 range
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    return normalized;
}

void FingerSensors::getEncodedValues(uint16_t* output) {
    // Map adjusted angles to 0-255 (like HID) then encode to uint16
    // This matches the original eidon-glove HID output format

    for (uint8_t i = 0; i < NUM_FINGER_SENSORS; i++) {
        // Get the adjusted angle
        int32_t angle = _angles[i];

        // Get the appropriate min/max for this joint type
        int32_t minAngle, maxAngle;

        if (i == 0) {
            // Thumb CMC Flexion: 0-255
            minAngle = THUMB_CMC_FLEXION_MIN;
            maxAngle = THUMB_CMC_FLEXION_MAX;
        } else if (i == 1) {
            // Thumb CMC Abduction: 0 to 2*125 (mapped from -125 to +125)
            minAngle = 0;
            maxAngle = 2 * THUMB_CMC_ABDUCTION_MAX;
        } else if (i == 2 || i == 3) {
            // Thumb PIP Flexion: 0-255
            minAngle = THUMB_PIP_FLEXION_MIN;
            maxAngle = THUMB_PIP_FLEXION_MAX;
        } else if (i == 4 || i == 7 || i == 10 || i == 13) {
            // MCP Abduction: 0 to 2*80 (mapped from -80 to +80)
            minAngle = 0;
            maxAngle = 2 * MCP_ABDUCTION_MAX;
        } else if (i == 5 || i == 8 || i == 11 || i == 14) {
            // MCP Flexion: 0-240
            minAngle = MCP_FLEXION_MIN;
            maxAngle = MCP_FLEXION_MAX;
        } else {
            // PIP Flexion: 0-255
            minAngle = PIP_FLEXION_MIN;
            maxAngle = PIP_FLEXION_MAX;
        }

        // Constrain angle to valid range
        if (angle < minAngle) angle = minAngle;
        if (angle > maxAngle) angle = maxAngle;

        // Map to 0-255 (same as eidon-glove mapAngleToHID)
        uint8_t hidValue = map(angle, minAngle, maxAngle, 0, 255);

        // Encode as uint16 for BLE (0-255 -> 0-65535)
        // Using simple scaling: uint16 = hidValue * 257 (maps 0-255 to 0-65535)
        output[i] = (uint16_t)hidValue * 257;
    }
}

void FingerSensors::loadCalibration() {
    Preferences prefs;

    if (!prefs.begin(CONFIG_NAMESPACE, true)) { // Read-only
        Serial.println("FingerSensors: Failed to open calibration storage");
        resetCalibration();
        return;
    }

    // Load min_angles array
    size_t minSize = prefs.getBytesLength(CAL_MIN_KEY);
    size_t maxSize = prefs.getBytesLength(CAL_MAX_KEY);

    // If no calibration found or wrong size, use defaults
    if (minSize != sizeof(_minAngles) || maxSize != sizeof(_maxAngles)) {
        prefs.end();
        Serial.println("FingerSensors: No calibration found, using defaults");
        resetCalibration();
        return;
    }

    prefs.getBytes(CAL_MIN_KEY, _minAngles, sizeof(_minAngles));
    prefs.getBytes(CAL_MAX_KEY, _maxAngles, sizeof(_maxAngles));

    prefs.end();

    Serial.println("FingerSensors: Calibration loaded from NVS");
}

void FingerSensors::saveCalibration() {
    Preferences prefs;

    if (!prefs.begin(CONFIG_NAMESPACE, false)) { // Read-write
        Serial.println("FingerSensors: Failed to open calibration storage");
        return;
    }

    // Save min and max arrays
    prefs.putBytes(CAL_MIN_KEY, _minAngles, sizeof(_minAngles));
    prefs.putBytes(CAL_MAX_KEY, _maxAngles, sizeof(_maxAngles));

    prefs.end();

    Serial.println("FingerSensors: Calibration saved to NVS");
}

void FingerSensors::resetCalibration() {
    // Reset to initial values for dynamic calibration (like eidon-glove)
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        _minAngles[i] = 10000.0f;   // Will be updated to actual min
        _maxAngles[i] = -10000.0f;  // Will be updated to actual max
    }

    Serial.println("FingerSensors: Calibration reset to defaults (dynamic mode)");
}

void FingerSensors::clearCalibration() {
    Preferences prefs;

    if (!prefs.begin(CONFIG_NAMESPACE, false)) { // Read-write
        Serial.println("FingerSensors: Failed to open calibration storage");
        resetCalibration();
        return;
    }

    // Clear stored calibration
    prefs.clear();
    prefs.end();

    // Reset to default values
    resetCalibration();

    Serial.println("FingerSensors: Calibration cleared from NVS and reset to defaults");
}

void FingerSensors::printValues() {
    Serial.println("=== Finger Sensor Adjusted Angles ===");
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        Serial.printf("%2d %-15s: angle=%4d, norm=%.3f\n",
                      i, FINGER_NAMES[i], _angles[i], getNormalized(i));
    }
}

void FingerSensors::printRawValues() {
    Serial.println("=== Finger Sensor Raw Values ===");
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        Serial.printf("%2d %-15s: raw=%4d, proto=%.2f\n",
                      i, FINGER_NAMES[i], _rawValues[i], _protoAngles[i]);
    }
}

void FingerSensors::printCalibration() {
    Serial.println("=== Finger Sensor Calibration ===");
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        Serial.printf("%2d %-15s: min=%.2f, max=%.2f%s\n",
                      i, FINGER_NAMES[i], _minAngles[i], _maxAngles[i],
                      _invertedSensors[i] ? " (inverted)" : "");
    }
}

#endif // ESP32_C3_GLOVE
