#ifndef FINGER_SENSORS_H
#define FINGER_SENSORS_H

#include <Arduino.h>
#include <ResponsiveAnalogRead.h>

// Only compile for ESP32-C3 glove
#ifdef ESP32_C3_GLOVE

// Multiplexer control pins (ESP32-C3) - matching eidon-glove hardware
#define MUX_S0          10   // GPIO 10 - Select bit 0
#define MUX_S1          9    // GPIO 9  - Select bit 1
#define MUX_S2          6    // GPIO 6  - Select bit 2
#define MUX_S3          7    // GPIO 7  - Select bit 3
#define MUX_ADC_PIN     A2   // A2 - Analog input (MUX_OUT) - same as eidon-glove

#define NUM_FINGER_SENSORS 16

// Joint angle limits from eidon-glove FingerTracking.h
#define MCP_FLEXION_MIN 0
#define MCP_FLEXION_MAX 240

#define PIP_FLEXION_MIN 0
#define PIP_FLEXION_MAX 255

#define MCP_ABDUCTION_MIN -80
#define MCP_ABDUCTION_MAX 80

#define THUMB_CMC_FLEXION_MIN 0
#define THUMB_CMC_FLEXION_MAX 255

#define THUMB_CMC_ABDUCTION_MIN -125
#define THUMB_CMC_ABDUCTION_MAX 125

#define THUMB_PIP_FLEXION_MIN 0
#define THUMB_PIP_FLEXION_MAX 255

// Finger sensor names for debugging (Thumb 4 + Index/Middle/Ring/Pinky 3 each = 16)
static const char* FINGER_NAMES[NUM_FINGER_SENSORS] = {
    "Thumb CMC Flex", "Thumb CMC Abd", "Thumb PIP",    "Thumb DIP",
    "Index Abd",      "Index MCP",     "Index PIP",
    "Middle Abd",     "Middle MCP",    "Middle PIP",
    "Ring Abd",       "Ring MCP",      "Ring PIP",
    "Pinky Abd",      "Pinky MCP",     "Pinky PIP"
};

class FingerSensors {
public:
    // Initialization
    bool begin();

    // Update all sensor readings and calculate angles
    void update();

    // Data access
    uint16_t getRawValue(uint8_t index);        // Get ADC value (0-4095)
    int32_t getAngle(uint8_t index);            // Get adjusted angle value
    float getNormalized(uint8_t index);         // Get 0.0-1.0 range (for compatibility)
    void getEncodedValues(uint16_t* output);    // Get all 16 as uint16 BLE format

    // Calibration
    void loadCalibration();
    void saveCalibration();
    void resetCalibration();
    void clearCalibration();                    // Clear NVS and reset to defaults

    // Diagnostics
    void printValues();
    void printRawValues();
    void printCalibration();

private:
    void selectMuxChannel(uint8_t channel);     // Set S0-S3 for channel 0-15
    uint16_t readMuxChannel(uint8_t channel);   // Select + read ADC

    // Polynomial conversion (from eidon-glove)
    float poly(float x, float a, float b, float c);

    // Angle adjustment functions (from eidon-glove)
    void measureSensors();                      // Read raw values and compute proto_angles
    void calibrateDynamic();                    // Update min/max dynamically
    void adjustAngles();                        // Apply per-joint adjustments

    int32_t adjustMCPAbductionAngle(int32_t i);
    int32_t adjustMCPFlexionAngle(int32_t i);
    int32_t adjustPIPFlexionAngle(int32_t i);
    int32_t adjustThumbCMCFlexionAngle(int32_t i);
    int32_t adjustThumbCMCAbductionAngle(int32_t i);
    int32_t adjustThumbPIPFlexionAngle(int32_t i);

    // Raw ADC values
    uint16_t _rawValues[NUM_FINGER_SENSORS];

    // Proto angles (after polynomial conversion)
    float _protoAngles[NUM_FINGER_SENSORS];

    // Final adjusted angles
    int32_t _angles[NUM_FINGER_SENSORS];

    // Dynamic calibration min/max (from eidon-glove)
    float _minAngles[NUM_FINGER_SENSORS];
    float _maxAngles[NUM_FINGER_SENSORS];

    // Sensor inversion flags (from eidon-glove)
    bool _invertedSensors[NUM_FINGER_SENSORS];

    // Configuration keys for NVS
    static const char* CONFIG_NAMESPACE;
    static const char* CAL_MIN_KEY;
    static const char* CAL_MAX_KEY;

    // Track when calibration was last saved to avoid excessive writes
    unsigned long _lastCalibSave;
    static const unsigned long CALIB_SAVE_INTERVAL = 5000; // Save every 5 seconds max
};

#endif // ESP32_C3_GLOVE

#endif // FINGER_SENSORS_H
