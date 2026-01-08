#include "BNO085.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// BNO085 sensor instance
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Sensor data storage
float quaternion_x = 0;
float quaternion_y = 0;
float quaternion_z = 0;
float quaternion_w = 1;

// Raw data storage
RawMotionData rawData = {0};

// Euler angles structure for quaternion conversion
euler_t ypr = {0, 0, 0};

// BNO085 availability tracking
bool bno085_available = false;

// Update interval tracking
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 1; // Not used in current implementation

// Debug print interval - match main.cpp periodic logging
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 3000; // Print every 3 seconds (increased from 5 seconds)

// Function to scan I2C bus
void BNO085::scanI2C() {
    Serial.println("Scanning I2C bus...");
    
    int nDevices = 0;
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    } else {
        Serial.print("Found ");
        Serial.print(nDevices);
        Serial.println(" device(s)");
    }
}

// Function to test direct BNO085 communication
bool BNO085::testCommunication() {
    // BNO085 uses SHTP (Sensor Hub Transport Protocol)
    // Try to read the SHTP header to see if we can communicate
    Wire.requestFrom(I2C_ADDR, 4); // Request 4 bytes (SHTP header)
    
    if (Wire.available() >= 4) {
        byte header[4];
        for (int i = 0; i < 4; i++) {
            header[i] = Wire.read();
        }
        
        // Check if this looks like a valid SHTP header
        uint16_t length = (header[1] << 8) | header[0];
        
        if (length > 0 && length < 1000) { // Reasonable packet length
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

// Function to enable sensor reports
void BNO085::enableReports() {
    // Use GAME_ROTATION_VECTOR for fast quaternion updates (no magnetic north reference)
    // Set to 48Hz (20.83ms) to match main.cpp IMU_UPDATE_INTERVAL for redundancy approach
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20830)) { // 20.83ms (48Hz) - target rate
        if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 25000)) { // 25ms (40Hz) fallback
            if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 16700)) { // 16.7ms (60Hz) fallback
                Serial.println("Could not enable rotation vector");
            }
        }
    }

    // Enable Tap Detector (event-driven, report interval 0)
    if (!bno08x.enableReport(SH2_TAP_DETECTOR, 0)) {
        Serial.println("Could not enable tap detector");
    }

    // Enable Accelerometer (SH2_ACCELEROMETER)
    // 40ms = 25Hz (Half of quaternion rate)
    if (!bno08x.enableReport(SH2_ACCELEROMETER, 40000)) {
        Serial.println("Could not enable accelerometer");
    }

    // Enable Calibrated Gyroscope (SH2_GYROSCOPE_CALIBRATED)
    // 40ms = 25Hz
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 40000)) {
        Serial.println("Could not enable gyroscope");
    }

    // Enable Calibrated Magnetometer (SH2_MAGNETIC_FIELD_CALIBRATED)
    // 40ms = 25Hz
    if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 40000)) {
        Serial.println("Could not enable magnetometer");
    }
}

// Function to initialize the BNO085 sensor
bool BNO085::begin() {    
    Serial.println("BNO085: Starting initialization...");
    
    // Configure ADR pin for I2C address selection
    // ADR pin HIGH = 0x4B, ADR pin LOW = 0x4A
    pinMode(I2C_ADR, OUTPUT);
    digitalWrite(I2C_ADR, HIGH); // Set to 0x4B address
    Serial.printf("BNO085: ADR pin set HIGH for I2C address 0x%02X\n", I2C_ADDR);
    
    // Initialize I2C with explicit pins for ESP32-C6
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    Wire.setClock(400000); // Set to 400kHz (standard fast mode) for stability
    

    
    // Test basic I2C communication with retries
    int i2c_attempts = 0;
    bool i2c_success = false;
    
    while (!i2c_success && i2c_attempts < 5) {
        Wire.beginTransmission(I2C_ADDR);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            i2c_success = true;
        } else {
            i2c_attempts++;
            if (i2c_attempts < 5) {
                delay(500);
            }
        }
    }
    
    if (!i2c_success) {
        Serial.println("BNO085: I2C communication failed");
        return false;
    }
    
    // Try multiple initialization approaches with retries
    bool bno_initialized = false;
    int total_attempts = 0;
    const int max_attempts = 10; // Reduced from 15
    
    while (!bno_initialized && total_attempts < max_attempts) {
        total_attempts++;
        
        // Try different initialization methods
        if (total_attempts <= 3) {
            // First 3 attempts: Try with explicit Wire object
            if (bno08x.begin_I2C(I2C_ADDR, &Wire)) {
                bno_initialized = true;
                break;
            }
        } else if (total_attempts <= 6) {
            // Next 3 attempts: Try with default address
            if (bno08x.begin_I2C()) {
                bno_initialized = true;
                break;
            }
        } else {
            // Final attempts: Try with different I2C speeds
            if (total_attempts == 7) {
                Wire.setClock(400000);
            } else if (total_attempts == 9) {
                Wire.setClock(50000);
            }
            
            if (bno08x.begin_I2C(I2C_ADDR)) {
                bno_initialized = true;
                break;
            }
        }
        
        // Wait between attempts
        int delay_ms = (total_attempts <= 5) ? 500 : 1000;
        delay(delay_ms);
        
        // Reset I2C if device is not responding (every 5 attempts)
        if (total_attempts % 5 == 0) {
            Wire.beginTransmission(I2C_ADDR);
            if (Wire.endTransmission() != 0) {
                Wire.end();
                delay(100);
                Wire.setPins(I2C_SDA, I2C_SCL);
                Wire.begin();
                Wire.setClock(100000);
            }
        }
    }
    
    if (!bno_initialized) {
        Serial.println("BNO085: Failed to initialize");
        return false;
    }

    Serial.println("BNO085: Initialized");

    // Enable the rotation vector report
    enableReports();
    
    // Set availability flag
    bno085_available = true;

    return true;
}

// Function to check if BNO085 is available
bool BNO085::isAvailable() {
    return bno085_available;
}

// Function to reset the BNO085 sensor
bool BNO085::reset() {
    if (!bno085_available) {
        return false;
    }
    
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
    delay(100);
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000);
    return true;
}

// Function to update BNO085 sensor data
void BNO085::update() {
    // Don't try to update if BNO085 is not available
    if (!bno085_available) {
        return;
    }

    if (bno08x.wasReset()) {
        Serial.println("IMU: Reset detected");
        enableReports();
    }
    
    // Process all available events
    while (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
                quaternion_x = sensorValue.un.gameRotationVector.i;
                quaternion_y = sensorValue.un.gameRotationVector.j;
                quaternion_z = sensorValue.un.gameRotationVector.k;
                quaternion_w = sensorValue.un.gameRotationVector.real;
                break;

            case SH2_TAP_DETECTOR: {
                uint8_t f = sensorValue.un.tapDetector.flags;
                bool isDouble = f & TAPDET_DOUBLE;
                
                if (isDouble) {
                    Serial.println("IMU: Double tap detected");
                    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
                    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
                }
                break;
            }

            case SH2_ACCELEROMETER:
                rawData.accel_x = sensorValue.un.accelerometer.x;
                rawData.accel_y = sensorValue.un.accelerometer.y;
                rawData.accel_z = sensorValue.un.accelerometer.z;
                break;

            case SH2_GYROSCOPE_CALIBRATED:
                rawData.gyro_x = sensorValue.un.gyroscope.x;
                rawData.gyro_y = sensorValue.un.gyroscope.y;
                rawData.gyro_z = sensorValue.un.gyroscope.z;
                break;

            case SH2_MAGNETIC_FIELD_CALIBRATED:
                rawData.mag_x = sensorValue.un.magneticField.x;
                rawData.mag_y = sensorValue.un.magneticField.y;
                rawData.mag_z = sensorValue.un.magneticField.z;
                break;
        }

        // Quaternion output disabled to reduce serial overhead
        // if (millis() - lastPrint >= PRINT_INTERVAL) {
        //     Serial.printf("IMU: W=%.4f X=%.4f Y=%.4f Z=%.4f\n", 
        //                  quaternion_w, quaternion_x, quaternion_y, quaternion_z);
        //     lastPrint = millis();
        // }
    }
}

// Function to get current quaternion values
void BNO085::getQuaternion(float &w, float &x, float &y, float &z) {
    w = quaternion_w;
    x = quaternion_x;
    y = quaternion_y;
    z = quaternion_z;
}

// Function to get current raw data
void BNO085::getRawData(RawMotionData &data) {
    data = rawData;
}

// Function to convert quaternion to euler angles
void BNO085::quaternionToEuler() {
    float sqr = sq(quaternion_w);
    float sqi = sq(quaternion_x);
    float sqj = sq(quaternion_y);
    float sqk = sq(quaternion_z);

    ypr.yaw = asin(-2.0 * (quaternion_x * quaternion_z - quaternion_y * quaternion_w) /
                     (sqi + sqj + sqk + sqr));
    ypr.pitch = atan2(2.0 * (quaternion_x * quaternion_y + quaternion_z * quaternion_w),
                    (sqi - sqj - sqk + sqr));
    ypr.roll = atan2(2.0 * (quaternion_y * quaternion_z + quaternion_x * quaternion_w),
                     (-sqi - sqj + sqk + sqr));

    // Convert to degrees
    ypr.yaw = ypr.yaw * RAD_TO_DEG;
    ypr.pitch = ypr.pitch * RAD_TO_DEG;
    ypr.roll = ypr.roll * RAD_TO_DEG;
}

// Function to get euler angles
void BNO085::getEulerAngles(float &yaw, float &pitch, float &roll) {
    quaternionToEuler();
    yaw = ypr.yaw;
    pitch = ypr.pitch;
    roll = ypr.roll;
} 