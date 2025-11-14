#include "FingerSensors.h"

#ifdef ESP32_C3_GLOVE

#include <Preferences.h>

// Static member initialization
const char* FingerSensors::CONFIG_NAMESPACE = "finger_cal";
const char* FingerSensors::CAL_MIN_KEY = "cal_min";
const char* FingerSensors::CAL_MAX_KEY = "cal_max";

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

    // Set ADC attenuation to 11dB (0-3.3V range)
    analogSetAttenuation(ADC_11db);

    // Initialize raw values to 0
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        _rawValues[i] = 0;
    }

    // Load calibration from NVS
    loadCalibration();

    Serial.println("FingerSensors: Initialized");
    return true;
}

void FingerSensors::selectMuxChannel(uint8_t channel) {
    // Set S0-S3 based on channel (0-15)
    digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
    digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
    digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
    digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);

    // Allow MUX to settle (10µs should be enough for CD74HC4067)
    delayMicroseconds(10);
}

uint16_t FingerSensors::readMuxChannel(uint8_t channel) {
    selectMuxChannel(channel);

    // Read ADC with averaging (3 samples to reduce noise)
    uint32_t sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += analogRead(MUX_ADC_PIN);
        delayMicroseconds(100);
    }

    return sum / 3;
}

void FingerSensors::update() {
    // Read all 16 sensors sequentially
    for (uint8_t i = 0; i < NUM_FINGER_SENSORS; i++) {
        _rawValues[i] = readMuxChannel(i);
    }
}

uint16_t FingerSensors::getRawValue(uint8_t index) {
    if (index >= NUM_FINGER_SENSORS) {
        return 0;
    }
    return _rawValues[index];
}

float FingerSensors::getNormalized(uint8_t index) {
    if (index >= NUM_FINGER_SENSORS) {
        return 0.0f;
    }

    // Normalize to 0.0-1.0 based on calibration
    float normalized = (float)(_rawValues[index] - _minValues[index]) /
                      (_maxValues[index] - _minValues[index]);

    // Clamp to 0.0-1.0 range
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    return normalized;
}

void FingerSensors::getEncodedValues(uint16_t* output) {
    for (uint8_t i = 0; i < NUM_FINGER_SENSORS; i++) {
        // Normalize to 0.0-1.0 based on calibration
        float normalized = getNormalized(i);

        // Map 0.0-1.0 to -1.0-1.0 range (for consistency with glove encoding)
        float ranged = (normalized * 2.0f) - 1.0f;

        // Encode as uint16 (-1.0 to 1.0 -> 0 to 65535)
        // Encoding: uint16 = (float + 1.0) * 32767.5
        output[i] = (uint16_t)((ranged + 1.0f) * 32767.5f);
    }
}

void FingerSensors::calibrateSensor(uint8_t index, uint16_t min, uint16_t max) {
    if (index >= NUM_FINGER_SENSORS) {
        return;
    }

    _minValues[index] = min;
    _maxValues[index] = max;

    Serial.printf("FingerSensors: Calibrated sensor %d (%s): min=%d, max=%d\n",
                  index, FINGER_NAMES[index], min, max);
}

void FingerSensors::loadCalibration() {
    Preferences prefs;

    if (!prefs.begin(CONFIG_NAMESPACE, true)) { // Read-only
        Serial.println("FingerSensors: Failed to open calibration storage");
        resetCalibration();
        return;
    }

    // Load min values
    size_t minSize = prefs.getBytes(CAL_MIN_KEY, _minValues, sizeof(_minValues));
    size_t maxSize = prefs.getBytes(CAL_MAX_KEY, _maxValues, sizeof(_maxValues));

    prefs.end();

    // If no calibration found, use defaults
    if (minSize != sizeof(_minValues) || maxSize != sizeof(_maxValues)) {
        Serial.println("FingerSensors: No calibration found, using defaults");
        resetCalibration();
    } else {
        Serial.println("FingerSensors: Calibration loaded from NVS");
    }
}

void FingerSensors::saveCalibration() {
    Preferences prefs;

    if (!prefs.begin(CONFIG_NAMESPACE, false)) { // Read-write
        Serial.println("FingerSensors: Failed to open calibration storage");
        return;
    }

    // Save min and max values
    prefs.putBytes(CAL_MIN_KEY, _minValues, sizeof(_minValues));
    prefs.putBytes(CAL_MAX_KEY, _maxValues, sizeof(_maxValues));

    prefs.end();

    Serial.println("FingerSensors: Calibration saved to NVS");
}

void FingerSensors::resetCalibration() {
    // Set all sensors to default calibration
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        _minValues[i] = DEFAULT_MIN;
        _maxValues[i] = DEFAULT_MAX;
    }

    Serial.println("FingerSensors: Calibration reset to defaults");
}

void FingerSensors::printValues() {
    Serial.println("=== Finger Sensor Values ===");
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        Serial.printf("%2d %-12s: raw=%4d, norm=%.3f\n",
                      i, FINGER_NAMES[i], _rawValues[i], getNormalized(i));
    }
}

void FingerSensors::printCalibration() {
    Serial.println("=== Finger Sensor Calibration ===");
    for (int i = 0; i < NUM_FINGER_SENSORS; i++) {
        Serial.printf("%2d %-12s: min=%4d, max=%4d\n",
                      i, FINGER_NAMES[i], _minValues[i], _maxValues[i]);
    }
}

#endif // ESP32_C3_GLOVE
