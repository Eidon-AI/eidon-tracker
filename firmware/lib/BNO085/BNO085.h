#ifndef BNO085_H
#define BNO085_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

// Define I2C pins - conditional based on hardware platform
#ifdef ESP32_C3_GLOVE
    // ESP32-C3 Glove: I2C pins (matching eidon-glove hardware)
    #define I2C_SDA             21   // GPIO 21 - Data line (from eidon-glove)
    #define I2C_SCL             20   // GPIO 20 - Clock line (from eidon-glove)
    #define I2C_ADDR            0x4B // I2C address
    #define I2C_FREQ_HZ         100000  // 100kHz for glove
#else
    // ESP32-C6 Tracker: I2C pins
    #define I2C_SDA             20   // GPIO 20 (D9) - Data line
    #define I2C_SCL             19   // GPIO 19 (D8) - Clock line
    #define I2C_ADR             18   // GPIO 18 (D10) - Address select pin
    #define I2C_ADDR            0x4B // I2C address
    #define I2C_FREQ_HZ         100000  // 100kHz for stability and consistency
#endif

// Declare the struct type
struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

// BNO085 class definition
class BNO085 {
public:
    // Core functionality
    bool begin();
    void update();
    bool isAvailable();
    bool reset();
    
    // Data access
    void getQuaternion(float &w, float &x, float &y, float &z);
    void getEulerAngles(float &yaw, float &pitch, float &roll);
    
    // Configuration and utilities
    void enableReports();
    void scanI2C();
    bool testCommunication();
    
    // Quaternion math utilities
    void quaternionToEuler();
    
private:
    // Private implementation details
    // (All variables are currently global in the implementation for compatibility)
};

// Declare the variable as extern (for backward compatibility)
extern euler_t ypr;

// Legacy function declarations (for backward compatibility)
void setupBNO085();
void updateBNO085();
void resetBNO085();
void printBNO085Values();
void quaternionToEuler();

// Helper function to check if BNO085 is available
bool isBNO085Available();

// Declare external variables to store sensor data (for backward compatibility)
extern float quaternion_x;
extern float quaternion_y;
extern float quaternion_z;
extern float quaternion_w;
extern bool bno085_available;

#endif 