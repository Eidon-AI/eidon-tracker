# Eidon Tracker

A complete Bluetooth IMU tracking system featuring custom hardware, 3D printed enclosure, and web-based applications. The Eidon Tracker transmits real-time quaternion orientation data over Bluetooth HID, making it perfect for motion capture, gaming, and interactive applications.

<div align="center">
  <img src="images/eidon-tracker-closed.png" alt="Eidon Tracker - Closed" width="360px" />
</div>

## Features

- **High-precision IMU tracking** with BNO085 9-DOF sensor
- **Bluetooth HID device** - appears as standard input device
- **Real-time quaternion data** streaming at 1000Hz
- **Custom 3D printed enclosure** with integrated switches
- **Web-based applications** for visualization and interaction
- **Battery monitoring** with charging status
- **Device color customization** via HID feature reports

## Bill of Materials (BOM)

<div align="center">
  <img src="images/eidon-tracker-opened.png" alt="Eidon Tracker - Open" width="45%" />
</div>

### Required Components

| Component | Quantity | Description | Purchase Link |
|-----------|----------|-------------|---------------|
| [Seeed XIAO ESP32-C6](https://www.seeedstudio.com/Seeed-XIAO-ESP32C6-p-5884.html) | 1 | Main microcontroller with built-in BLE 5 and WiFi 6 | [Seeed Studio](https://www.seeedstudio.com/Seeed-XIAO-ESP32C6-p-5884.html) |
| BNO085 IMU Breakout | 1 | 9-DOF Absolute Orientation Sensor | [Amazon](https://amzn.to/4mKZSTC) |
| LiPo Battery (3.7V) | 1 | 250mAh recommended | [Amazon](https://amzn.to/4kpmNSK) |
| Toggle Switches (6mm) | 2 | For power and position config | [Amazon](https://amzn.to/3ZfAECM) |
| 30 AWG Wires | 7x ~5cm | Female-to-female, ~20cm | [Amazon](https://amzn.to/4kB8Z7V) |

### 3D Printing Materials

| Material | Quantity | Description |
|----------|----------|-------------|
| PLA Filament | ~50g | For enclosure parts |
| Support Material | As needed | If printing with overhangs |

### Tools Required

- 3D Printer
- Soldering iron and solder
- Small screwdriver
- Wire strippers

**Estimated Total Cost**: $80-120 USD

## Hardware Setup

### Wiring Diagram

Connect the BNO085 to the XIAO ESP32-C6 using SPI interface:

| BNO085 Pin | XIAO ESP32-C6 Pin | GPIO | Function |
|------------|-------------------|------|----------|
| VIN | 3V3 | - | Power (3.3V) |
| GND | GND | - | Ground |
| CS | D1 | GPIO17 | SPI Chip Select |
| SCK/SCL | D2 | GPIO19 | SPI Clock |
| MOSI/DI | D3 | GPIO18 | SPI Master Out |
| MISO/SDA | D4 | GPIO20 | SPI Master In |
| INT | D5 | GPIO22 | Interrupt |
| RST | D6 | GPIO23 | Reset |
| WAK | D0 | GPIO2 | Wake (optional) |

### Switch Connections

| Switch | Output Pin | Input Pin | Function |
|--------|------------|-----------|----------|
| Left/Right | D7 | D8 | Horizontal orientation |
| Upper/Lower | D9 | D10 | Vertical orientation |

### 3D Printing

Print the following files from the `cad/` directory:

1. `eidon-tracker-base.stl` - Main enclosure base
2. `eidon-tracker-top.stl` - Enclosure lid  
3. `eidon-tracker-mount.stl` - Optional mounting bracket

**Print Settings:**
- Layer Height: 0.2mm
- Infill: 20%
- Support: Yes (for overhangs)
- Material: PLA

## Software Setup

### ESP-IDF Installation

1. Install ESP-IDF v6.0 or later:
   ```bash
   # macOS/Linux
   mkdir -p ~/esp
   cd ~/esp
   git clone -b v6.0 --recursive https://github.com/espressif/esp-idf.git
   cd esp-idf
   ./install.sh esp32c6
   . ./export.sh
   ```

   For Windows, follow the [ESP-IDF Windows Installer guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html)

2. Install build dependencies:
   ```bash
   # Ubuntu/Debian
   sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
   
   # macOS
   brew install cmake ninja dfu-util
   ```

### Building and Uploading Firmware

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/eidon-tracker.git
   cd eidon-tracker
   ```

2. Initialize submodules:
   ```bash
   git submodule update --init --recursive
   ```

3. Build the firmware:
   ```bash
   make build
   # or
   cd firmware
   . ~/esp-idf/export.sh
   idf.py build
   ```

4. Connect your XIAO ESP32-C6 via USB

5. Flash the firmware:
   ```bash
   make flash
   # or with monitor
   make flash-monitor
   ```

### Dependencies

The firmware uses the following ESP-IDF components:

- **bno08x_driver** - BNO085 SPI driver (included as submodule)
- **ESP-IDF Bluetooth Stack** - BLE functionality
- **FreeRTOS** - Real-time operating system

## Web Applications

The `web/` directory contains the main interactive application:

### 3D Orientation Visualizer (`index.html`)

- **Real-time 3D visualization** of tracker orientation and movement
- **Multiple camera angles** and viewing modes
- **Bluetooth HID integration** for seamless device connectivity
- **Calibration status monitoring** and quaternion data display

### Running Web Applications

Simply open `web/index.html` directly in your web browser:

1. Navigate to the `web/` directory in your file explorer
2. Double-click `index.html` to open it in your default browser
3. Or right-click and "Open with" your preferred browser (Chrome/Edge recommended for Web Bluetooth support)
4. Connect to your Eidon Tracker via Bluetooth

## HID Reports

The Eidon Tracker implements a custom HID device with multiple report types for data exchange:

### Input Report (Report ID: 1)
**Size**: 9 bytes total
- **Quaternion Data** (8 bytes): Four 16-bit values representing orientation
  - `w` component (bytes 0-1): Scalar part of quaternion
  - `x` component (bytes 2-3): X-axis rotation
  - `y` component (bytes 4-5): Y-axis rotation  
  - `z` component (bytes 6-7): Z-axis rotation
- **Switch States** (1 byte): Two button states plus padding
  - Bit 0: Left/Right switch state
  - Bit 1: Upper/Lower switch state
  - Bits 2-7: Reserved (padding)

### Output Report (Report ID: 1)
**Size**: 1 byte
- **Vendor Command**: Custom command byte for device control
  - Used for device configuration and special commands
  - Values defined in firmware for various operations

### Feature Report (Report ID: 1)  
**Size**: 3 bytes
- **RGB Color Settings**: Device LED color configuration
  - Byte 0: Red component (0-255)
  - Byte 1: Green component (0-255)
  - Byte 2: Blue component (0-255)
  - Colors are stored in device flash memory
  - Allows customization of device LED color

### HID Usage Pages
- **Primary**: Sensor Page (0x20) - Orientation Usage (0x80)
- **Secondary**: Button Page (0x09) - Button 1-2
- **Vendor**: Custom Page (0xFF00) - Vendor-specific features

## Usage

### Initial Setup

1. **Power on** the device - LED will indicate status
2. **Pair via Bluetooth** - Device appears as "Eidon Tracker XXXX"
3. **Calibrate IMU** - Wave device in figure-8 pattern until calibrated
4. **Open web application** of choice
5. **Connect** to device in web browser

### Calibration Process

The BNO085 sensor requires calibration for optimal performance:

1. **Gyroscope**: Keep device stationary for 2-3 seconds
2. **Accelerometer**: Slowly rotate through all orientations  
3. **Magnetometer**: Move device in figure-8 pattern away from magnetic interference
4. Monitor calibration status in web application or serial output

### LED Status Indicators

| LED State | Meaning |
|-----------|---------|
| Solid Blue | Connected and ready |
| Blinking Blue | Advertising/pairing mode |
| Red | Low battery or error |
| Green | Charging |

## Development

### Project Structure

```
eidon-tracker/
├── firmware/              # ESP-IDF firmware project
│   ├── main/             # Main application source
│   ├── components/       # Custom components
│   │   └── bno08x_driver/ # BNO085 SPI driver submodule
│   ├── CMakeLists.txt    # CMake configuration
│   └── sdkconfig.defaults # Default configuration
├── web/                   # Web applications
│   ├── index.html         # 3D orientation visualizer
│   └── script.js          # Shared JavaScript libraries
├── cad/                   # 3D printing files
│   ├── eidon-tracker.f3d  # Fusion 360 source
│   ├── *.stl              # STL files for printing
│   └── *.gcode            # Pre-sliced files
├── images/                # Product photos
├── Makefile               # Root build automation
└── README.md              # This file
```

### Firmware Development

The firmware is built on ESP-IDF v6.0 with FreeRTOS. Key components:

- **HID Report Descriptor**: Custom HID device with quaternion data
- **Bluetooth Stack**: ESP-IDF BLE implementation
- **SPI IMU Driver**: High-speed BNO08x driver for reliable communication
- **Battery Management**: Built-in XIAO charging and monitoring

### Build Commands

The root Makefile provides convenient commands:

```bash
make build              # Build the firmware
make flash              # Flash the firmware to device
make monitor            # Open serial monitor
make clean              # Clean build files
make fullclean          # Full clean (remove all build artifacts)
make menuconfig         # Open ESP-IDF configuration menu
make flash-monitor      # Flash and then monitor
make build-flash        # Build and flash
make build-flash-monitor # Build, flash, and monitor
```

### Adding New Web Applications

1. Create new HTML file in `web/` directory
2. Include `script.js` for Bluetooth connectivity
3. Implement HID report parsing for quaternion data
4. Add visualization or interaction logic

## Troubleshooting

### Connection Issues

- **Device not found**: Ensure Bluetooth is enabled and device is in pairing mode
- **Connection drops**: Check battery level and distance from host device
- **Web Bluetooth not supported**: Use Chrome/Edge on desktop or Android

### Calibration Problems

- **Poor tracking accuracy**: Ensure proper IMU calibration
- **Drift over time**: Recalibrate magnetometer away from metal objects
- **Erratic behavior**: Check SPI wiring connections and signal integrity

### Build Issues

- **Target mismatch**: Run `make fullclean` then `make build` to ensure ESP32-C6 target
- **Submodule missing**: Run `git submodule update --init --recursive`
- **ESP-IDF not found**: Ensure you've sourced `~/esp-idf/export.sh`
- **Upload fails**: Hold BOOT button while connecting USB, then release

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is open source and available under the MIT License. See LICENSE file for details.

## Acknowledgments

- [Seeed Studio](https://www.seeedstudio.com/) for the excellent XIAO nRF52840 Sense board
- [Adafruit](https://www.adafruit.com/) for comprehensive sensor libraries and breakout boards
- [PlatformIO](https://platformio.org/) for the excellent development environment
