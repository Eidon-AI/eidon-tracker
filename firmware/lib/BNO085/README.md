# BNO085 Library

A comprehensive library for interfacing with the BNO085 IMU sensor on ESP32-C6, providing quaternion and euler angle data.

## Features

- **Quaternion Support**: Direct access to normalized quaternion data
- **Euler Angles**: Convert quaternions to yaw, pitch, roll angles
- **Robust Initialization**: Multiple fallback strategies for reliable sensor setup
- **I2C Communication**: Built-in I2C scanning and communication testing
- **Tap Detection**: Support for single and double tap detection
- **Error Handling**: Comprehensive error checking and recovery

## Installation

This library is designed to work with PlatformIO. Add it to your `lib/` directory or install via PlatformIO registry.

### Dependencies

- `adafruit/Adafruit BNO08x@^1.2.3`

## Hardware Setup

### Pin Configuration

```cpp
#define I2C_SCL 18  // D10 on XIAO ESP32-C6 (GPIO18)
#define I2C_SDA 20  // D9 on XIAO ESP32-C6 (GPIO20)
#define I2C_ADDR 0x4B
```

### Wiring

- Connect BNO085 SDA to GPIO20 (D9)
- Connect BNO085 SCL to GPIO18 (D10)
- Connect BNO085 VIN to 3.3V
- Connect BNO085 GND to GND

## Usage

### Basic Setup

```cpp
#include "BNO085.h"

BNO085 imu;

void setup() {
    Serial.begin(115200);
    
    if (!imu.begin()) {
        Serial.println("Failed to initialize BNO085!");
        while (1);
    }
}

void loop() {
    imu.update();
    
    if (imu.isAvailable()) {
        float w, x, y, z;
        imu.getQuaternion(w, x, y, z);
        
        Serial.printf("Quaternion: W=%.4f X=%.4f Y=%.4f Z=%.4f\n", w, x, y, z);
    }
}
```

### Getting Euler Angles

```cpp
void loop() {
    imu.update();
    
    if (imu.isAvailable()) {
        float yaw, pitch, roll;
        imu.getEulerAngles(yaw, pitch, roll);
        
        Serial.printf("Euler: Yaw=%.2f° Pitch=%.2f° Roll=%.2f°\n", yaw, pitch, roll);
    }
}
```

### Sensor Reset

```cpp
// Reset the sensor (useful for calibration)
if (imu.reset()) {
    Serial.println("Sensor reset successful");
}
```

## API Reference

### Core Methods

#### `bool begin()`
Initializes the BNO085 sensor. Returns `true` if successful, `false` otherwise.

#### `void update()`
Updates sensor data. Call this regularly in your main loop.

#### `bool isAvailable()`
Returns `true` if the sensor is available and working.

#### `bool reset()`
Resets the sensor. Returns `true` if successful.

### Data Access

#### `void getQuaternion(float &w, float &x, float &y, float &z)`
Gets the current quaternion values (normalized).

#### `void getEulerAngles(float &yaw, float &pitch, float &roll)`
Gets the current euler angles in degrees.

### Configuration

#### `void enableReports()`
Enables sensor reports with optimal settings.

#### `void scanI2C()`
Scans the I2C bus and reports found devices.

#### `bool testCommunication()`
Tests direct communication with the BNO085 sensor.

## Error Handling

The library includes comprehensive error handling:

- **I2C Communication**: Automatic retry with different speeds
- **Sensor Initialization**: Multiple fallback strategies
- **Data Validation**: Checks for valid sensor responses
- **Connection Monitoring**: Tracks sensor availability

## Troubleshooting

### Common Issues

1. **Sensor not found**: Check wiring and I2C address
2. **Communication errors**: Try different I2C speeds
3. **Initialization failures**: Ensure proper power supply

### Debug Output

The library provides detailed debug output via Serial. Enable Serial communication to see initialization progress and error messages.

## License

MIT License - see LICENSE file for details. 