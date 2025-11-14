#ifndef FINGER_SENSORS_H
#define FINGER_SENSORS_H

#include <Arduino.h>

// Only compile for ESP32-C3 glove
#ifdef ESP32_C3_GLOVE

// Multiplexer control pins (ESP32-C3)
#define MUX_S0          10   // GPIO 10 - D10 - Select bit 0
#define MUX_S1          9    // GPIO 9  - D9  - Select bit 1
#define MUX_S2          6    // GPIO 6  - D4  - Select bit 2
#define MUX_S3          7    // GPIO 7  - D5  - Select bit 3
#define MUX_ADC_PIN     0    // GPIO 0  - A0/D0 - Analog input (MUX_OUT)

#define NUM_FINGER_SENSORS 16

// Finger sensor names for debugging
static const char* FINGER_NAMES[NUM_FINGER_SENSORS] = {
    "Thumb CMC",    "Thumb MCP",    "Thumb IP",     "Thumb Flex",
    "Index MCP",    "Index PIP",    "Index DIP",    "Index Flex",
    "Middle MCP",   "Middle PIP",   "Middle DIP",   "Middle Flex",
    "Ring MCP",     "Ring PIP",     "Ring DIP",     "Ring Flex"
};

class FingerSensors {
public:
    // Initialization
    bool begin();

    // Update all sensor readings
    void update();

    // Data access
    uint16_t getRawValue(uint8_t index);        // Get ADC value (0-4095)
    float getNormalized(uint8_t index);         // Get 0.0-1.0 range
    void getEncodedValues(uint16_t* output);    // Get all 16 as uint16 BLE format

    // Calibration
    void calibrateSensor(uint8_t index, uint16_t min, uint16_t max);
    void loadCalibration();
    void saveCalibration();
    void resetCalibration();

    // Diagnostics
    void printValues();
    void printCalibration();

private:
    void selectMuxChannel(uint8_t channel);     // Set S0-S3 for channel 0-15
    uint16_t readMuxChannel(uint8_t channel);   // Select + read ADC

    uint16_t _rawValues[NUM_FINGER_SENSORS];

    // Calibration ranges (stored in NVS)
    uint16_t _minValues[NUM_FINGER_SENSORS];    // Min ADC value (finger extended)
    uint16_t _maxValues[NUM_FINGER_SENSORS];    // Max ADC value (finger curled)

    // Default calibration values (will be updated from testing)
    static const uint16_t DEFAULT_MIN = 500;    // Default min ADC
    static const uint16_t DEFAULT_MAX = 3500;   // Default max ADC

    // Configuration keys for NVS
    static const char* CONFIG_NAMESPACE;
    static const char* CAL_MIN_KEY;
    static const char* CAL_MAX_KEY;
};

#endif // ESP32_C3_GLOVE

#endif // FINGER_SENSORS_H
