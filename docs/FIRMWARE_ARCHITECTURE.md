# Firmware Architecture

## Overview

The Eidon Tracker firmware is built using the Arduino framework for the nRF52840 platform, leveraging the Adafruit nRF52 BSP and Bluefruit libraries for BLE functionality.

## Build System

### PlatformIO Configuration
- **Platform**: Seeed nRF52 boards
- **Board**: `seeed-xiao-afruitnrf52-nrf52840`
- **Framework**: Arduino
- **Key Dependencies**:
  - Adafruit BNO08x (v1.2.3+)
  - Adafruit Bluefruit nRF52 Libraries
  - Adafruit SPIFlash

### Build Process
```bash
# Build firmware
cd firmware
pio run

# Upload firmware
pio run --target upload

# Monitor serial output
pio device monitor
```

## Code Structure

### Main Components

1. **BLE Stack**
   - Dual-mode operation: HID + Custom GATT
   - Service advertisement and connection management
   - Optimized for low latency (7.5-15ms connection interval)

2. **IMU Interface**
   - BNO085 driver with I2C communication
   - Interrupt-driven data acquisition
   - Quaternion output with 180° Z-axis correction

3. **Data Flow**
   ```
   BNO085 IMU → Interrupt → Read Quaternion → Apply Correction → 
   → Send via HID Report → Send via GATT Notification
   ```

4. **Persistent Storage**
   - QSPI Flash for configuration storage
   - Color settings persistence
   - Wear leveling considerations

## Key Functions

### Initialization
- `setup()`: Hardware initialization, BLE service configuration
- `initIMU()`: BNO085 sensor initialization
- `initBatteryMonitoring()`: ADC configuration for battery monitoring

### Main Loop
- `loop()`: Continuous sensor reading and data transmission
- `updateOrientation()`: Process IMU data
- `sendQuaternionReport()`: Dual-protocol data transmission

### BLE Callbacks
- `handleCommand()`: HID output report handler
- `gattCalibrationCallback()`: GATT calibration commands
- `gattColorCallback()`: GATT color configuration
- `handleColorFeature()`: HID feature report handler

## Memory Management

### RAM Usage
- BLE stack: ~50KB
- Application: ~20KB
- Buffers: ~10KB
- Total: ~80KB of 256KB available

### Flash Usage
- Bootloader: 152KB
- Application: ~300KB
- Total: ~452KB of 1MB available

## Power Management

### Active Mode
- Continuous BLE transmission
- IMU sampling at 200Hz (post-calibration)
- LED status indication

### Power Optimization
- Battery monitoring every 5 minutes
- Efficient BLE connection parameters
- Hardware interrupt for IMU data ready

## Error Handling

### Startup Errors
- IMU initialization failure: Rapid LED flashing
- Flash initialization failure: Continue without persistence

### Runtime Errors
- BLE transmission failures: Logged but non-blocking
- Invalid commands: Ignored with serial logging

## Debugging

### Serial Output
- Baud rate: 115200
- Debug messages for all major operations
- Quaternion data can be enabled for debugging

### DFU Mode
- Triggered by sending 'D' over serial during startup
- Allows OTA firmware updates

## Performance Metrics

### Latency
- IMU to BLE transmission: <5ms typical
- Total system latency: <20ms

### Throughput
- Quaternion updates: 200Hz
- BLE notifications: Up to 200Hz
- HID reports: Up to 200Hz

## Future Considerations

### Planned Improvements
1. Dynamic IMU sampling rate adjustment
2. Advanced power management modes
3. Over-the-air configuration updates
4. Extended telemetry data

### Scalability
- Support for multiple IMU configurations
- Extensible GATT service design
- Modular architecture for feature additions 