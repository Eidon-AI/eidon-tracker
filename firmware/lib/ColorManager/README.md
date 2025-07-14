# ColorManager Library

A comprehensive color management library for ESP32 with BLE integration, persistent storage, and RGB color support.

## Features

- **RGB Color Support**: Full RGB color management with 8-bit precision
- **Persistent Storage**: Automatic saving and loading of colors using ESP32 Preferences
- **BLE Integration**: Built-in BLE characteristic management for color updates
- **GATT Service Support**: Custom GATT characteristics for color control
- **Utility Functions**: Color printing, hex conversion, and validation

## Installation

This library is designed to work with PlatformIO. Add it to your `lib/` directory.

### Dependencies

- `h2zero/NimBLE-Arduino@^2.3.1`

## Usage

### Basic Setup

```cpp
#include "ColorManager.h"

ColorManager colorManager;

void setup() {
    Serial.begin(115200);
    
    if (!colorManager.begin()) {
        Serial.println("Failed to initialize ColorManager!");
        return;
    }
}

void loop() {
    // Set a new color
    colorManager.setColor(255, 0, 0); // Red
    delay(1000);
    
    // Get current color
    RGBColor current = colorManager.getColor();
    Serial.printf("Current color: R=%d G=%d B=%d\n", 
                  current.r, current.g, current.b);
}
```

### BLE Integration

```cpp
#include "ColorManager.h"
#include <NimBLEServer.h>

ColorManager colorManager;
NimBLEServer* pServer;
NimBLECharacteristic* colorChar;
NimBLECharacteristic* featureReport;

void setup() {
    // Initialize color manager
    colorManager.begin();
    
    // Create BLE server and characteristics
    pServer = NimBLEDevice::createServer();
    
    // Create color characteristic
    colorChar = pServer->createService("color-service")
        ->createCharacteristic("color", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    
    // Create feature report characteristic (legacy support)
    featureReport = pServer->createService("feature-service")
        ->createCharacteristic("feature-report", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    
    // Connect color manager to BLE characteristics
    colorManager.setColorCharacteristic(colorChar);
    colorManager.setFeatureReport(featureReport);
    
    // Set up callbacks
    colorChar->setCallbacks(new ColorManager::ColorCallbacks(&colorManager));
    featureReport->setCallbacks(new ColorManager::FeatureReportCallbacks(&colorManager));
}
```

### Color Persistence

```cpp
void setup() {
    colorManager.begin(); // Automatically loads saved color
    
    // Set a new color (automatically saved)
    colorManager.setColor(0, 255, 0); // Green
    
    // Color is automatically saved and will be loaded on next boot
}
```

## API Reference

### Core Methods

#### `bool begin()`
Initializes the ColorManager. Returns `true` if successful.

#### `void setColor(uint8_t r, uint8_t g, uint8_t b)`
Sets the current color to the specified RGB values.

#### `void setColor(const RGBColor& color)`
Sets the current color using an RGBColor structure.

#### `RGBColor getColor() const`
Returns the current color as an RGBColor structure.

#### `void getColor(uint8_t& r, uint8_t& g, uint8_t& b) const`
Gets the current color as individual RGB components.

### Persistence

#### `bool loadFromStorage()`
Loads the color from persistent storage. Returns `true` if successful.

#### `bool saveToStorage()`
Saves the current color to persistent storage. Returns `true` if successful.

### BLE Integration

#### `void setColorCharacteristic(NimBLECharacteristic* characteristic)`
Connects the color manager to a BLE characteristic for color updates.

#### `void setFeatureReport(NimBLECharacteristic* characteristic)`
Connects the color manager to a feature report characteristic (legacy support).

#### `void updateBLECharacteristics()`
Updates all connected BLE characteristics with the current color.

### Utility Functions

#### `void printColor() const`
Prints the current color to Serial in a readable format.

#### `String getColorHex() const`
Returns the current color as a hex string (e.g., "#FF0000").

## Data Structures

### RGBColor

```cpp
struct RGBColor {
    uint8_t r;  // Red component (0-255)
    uint8_t g;  // Green component (0-255)
    uint8_t b;  // Blue component (0-255)
    
    RGBColor();  // Default constructor (white)
    RGBColor(uint8_t red, uint8_t green, uint8_t blue);
};
```

## Callback Classes

### ColorManager::ColorCallbacks

Handles GATT color characteristic write operations.

### ColorManager::FeatureReportCallbacks

Handles feature report read/write operations (legacy support).

## Storage

The library uses ESP32 Preferences to store color data:
- **Namespace**: "tracker"
- **Key**: "color"
- **Format**: 3 bytes (RGB)

## License

MIT License - see LICENSE file for details. 