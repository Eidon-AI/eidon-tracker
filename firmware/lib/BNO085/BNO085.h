#ifndef BNO085_H
#define BNO085_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

// Define I2C pins
#define I2C_SCL 19  // D8 on XIAO ESP32-C6 (GPIO19)
#define I2C_SDA 20  // D9 on XIAO ESP32-C6 (GPIO20)
#define I2C_ADDR 0x4B

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