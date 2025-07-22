#ifndef COLOR_MANAGER_H
#define COLOR_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLECharacteristic.h>

// Color structure for RGB values
struct RGBColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    
    RGBColor() : r(0xFF), g(0xFF), b(0xFF) {} // Default white
    RGBColor(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
};

// ColorManager class definition
class ColorManager {
public:
    // Core functionality
    bool begin();
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setColor(const RGBColor& color);
    RGBColor getColor() const;
    void getColor(uint8_t& r, uint8_t& g, uint8_t& b) const;
    
    // Persistence
    bool loadFromStorage();
    bool saveToStorage();
    
    // BLE Integration
    void setColorCharacteristic(NimBLECharacteristic* characteristic);
    void setFeatureReport(NimBLECharacteristic* characteristic);
    void updateBLECharacteristics();
    
    // Utility functions
    void printColor() const;
    String getColorHex() const;
    
    // Callback classes for BLE
    class FeatureReportCallbacks;
    class ColorCallbacks;
    
private:
    RGBColor currentColor;
    Preferences prefs;
    NimBLECharacteristic* colorChar = nullptr;
    NimBLECharacteristic* featureReport = nullptr;
    bool initialized = false;
    
    static const char* STORAGE_NAMESPACE;
    static const char* STORAGE_KEY;
};

// Feature report callback for HID
class ColorManager::FeatureReportCallbacks : public NimBLECharacteristicCallbacks {
public:
    FeatureReportCallbacks(ColorManager* manager) : colorManager(manager) {}
    
    void onRead(NimBLECharacteristic* pChar, const std::string& value);
    void onWrite(NimBLECharacteristic* pChar, const std::string& value);
    
private:
    ColorManager* colorManager;
};

// GATT Color characteristic callback
class ColorManager::ColorCallbacks : public NimBLECharacteristicCallbacks {
public:
    ColorCallbacks(ColorManager* manager) : colorManager(manager) {}
    
    void onWrite(NimBLECharacteristic* chr);
    
private:
    ColorManager* colorManager;
};

#endif 