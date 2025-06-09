# Hardware Overview

## Device Specifications

### Main Board: XIAO nRF52840 Sense
- **MCU**: Nordic nRF52840
- **CPU**: ARM Cortex-M4F @ 64MHz
- **Memory**: 256KB RAM, 1MB Flash
- **Bluetooth**: BLE 5.0
- **Power**: 3.3V operating voltage
- **Size**: 21 x 17.5mm

### IMU: BNO085
- **Type**: 9-axis IMU with sensor fusion
- **Interface**: I2C
- **Output**: Quaternion, Euler angles, Linear acceleration
- **Sample Rate**: Up to 1000Hz
- **Accuracy**: ±2° RMS (dynamic), ±1° RMS (static)

### Additional Components
- **Flash Storage**: 2MB QSPI Flash (P25Q16H)
- **Battery Monitoring**: Built-in ADC with voltage divider
- **LEDs**: RGB LED for status indication
- **Switches**: 2 position detection switches

## Pin Configuration

### I2C Bus (BNO085)
- **SDA**: Pin D9
- **SCL**: Pin D10
- **INT**: Pin D8 (Interrupt for data ready)
- **Address**: 0x4B

### Switch Inputs
- **Left/Right Switch**:
  - Output: D5
  - Input: D6
- **Upper/Lower Switch**:
  - Output: D0
  - Input: D1

### Power Management
- **VBAT**: Pin D32 (Battery voltage monitoring)
- **VBAT_ENABLE**: Pin D14 (LOW to enable reading)
- **HICHG**: Pin D22 (Charge current: LOW=100mA, HIGH=50mA)
- **CHG**: Pin D23 (Charge indicator: LOW=charging, HIGH=not charging)

### Status LEDs
- **Main LED**: PIN_LED (Built-in)
- **Green LED**: LED_GREEN (RGB LED, active low)

## Power Specifications

### Battery
- **Type**: LiPo single cell
- **Voltage Range**: 3.3V - 4.2V
- **Monitoring**: 12-bit ADC with 2.961:1 voltage divider
- **Charging**: Built-in charging circuit

### Power Consumption
- **Active BLE**: ~15mA typical
- **Sleep Mode**: <10µA
- **Peak**: ~30mA during BLE transmission

## Mechanical Design

### Enclosure Requirements
- Waterproof rating: IP67 recommended
- IMU mounting: Rigid mounting required for accurate readings
- Orientation: Device switches indicate position (left/right, upper/lower)

### Mounting Considerations
- IMU pre-calibrated for 180° Z-axis rotation
- Double-tap detection enabled for calibration trigger
- Switches must be accessible for position detection

## Communication Interfaces

### Bluetooth Low Energy
- **Profile**: Dual mode (HID + Custom GATT)
- **Connection Interval**: 7.5-15ms
- **TX Power**: Adjustable, default 0dBm
- **Range**: ~30m line of sight

### USB
- **Type**: USB-C
- **Purpose**: Charging, firmware updates, serial debug
- **DFU Mode**: Accessible via serial command 'D'

## Environmental Specifications

### Operating Conditions
- **Temperature**: -20°C to +70°C
- **Humidity**: 10% to 90% non-condensing
- **Vibration**: Suitable for wearable applications

### Storage Conditions
- **Temperature**: -40°C to +85°C
- **Humidity**: 5% to 95% non-condensing 