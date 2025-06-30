#include "ColorManager.h"

// Static member initialization
const char* ColorManager::STORAGE_NAMESPACE = "tracker";
const char* ColorManager::STORAGE_KEY = "color";

// ColorManager implementation
bool ColorManager::begin() {
    if (initialized) return true;
    
    // Initialize preferences
    prefs.begin(STORAGE_NAMESPACE, false);
    
    // Load saved color or use default
    if (!loadFromStorage()) {
        // Set default white color
        currentColor = RGBColor(0xFF, 0xFF, 0xFF);
        saveToStorage();
    }
    
    initialized = true;
    Serial.println("ColorManager initialized");
    return true;
}

void ColorManager::setColor(uint8_t r, uint8_t g, uint8_t b) {
    currentColor = RGBColor(r, g, b);
    saveToStorage();
    updateBLECharacteristics();
    printColor();
}

void ColorManager::setColor(const RGBColor& color) {
    currentColor = color;
    saveToStorage();
    updateBLECharacteristics();
    printColor();
}

RGBColor ColorManager::getColor() const {
    return currentColor;
}

void ColorManager::getColor(uint8_t& r, uint8_t& g, uint8_t& b) const {
    r = currentColor.r;
    g = currentColor.g;
    b = currentColor.b;
}

bool ColorManager::loadFromStorage() {
    uint8_t colorData[3];
    if (prefs.getBytes(STORAGE_KEY, colorData, 3) == 3) {
        currentColor = RGBColor(colorData[0], colorData[1], colorData[2]);
        Serial.print("Color loaded from storage: ");
        printColor();
        return true;
    }
    return false;
}

bool ColorManager::saveToStorage() {
    uint8_t colorData[3] = {currentColor.r, currentColor.g, currentColor.b};
    prefs.putBytes(STORAGE_KEY, colorData, 3);
    Serial.print("Color saved to storage: ");
    printColor();
    return true;
}

void ColorManager::setColorCharacteristic(NimBLECharacteristic* characteristic) {
    colorChar = characteristic;
    if (colorChar) {
        uint8_t colorData[3] = {currentColor.r, currentColor.g, currentColor.b};
        colorChar->setValue(colorData, 3);
    }
}

void ColorManager::setFeatureReport(NimBLECharacteristic* characteristic) {
    featureReport = characteristic;
    if (featureReport) {
        uint8_t colorData[3] = {currentColor.r, currentColor.g, currentColor.b};
        featureReport->setValue(colorData, 3);
    }
}

void ColorManager::updateBLECharacteristics() {
    if (colorChar) {
        uint8_t colorData[3] = {currentColor.r, currentColor.g, currentColor.b};
        colorChar->setValue(colorData, 3);
    }
    if (featureReport) {
        uint8_t colorData[3] = {currentColor.r, currentColor.g, currentColor.b};
        featureReport->setValue(colorData, 3);
    }
}

void ColorManager::printColor() const {
    Serial.printf("Color: R=%02X G=%02X B=%02X (%s)\n", 
                  currentColor.r, currentColor.g, currentColor.b, 
                  getColorHex().c_str());
}

String ColorManager::getColorHex() const {
    char hex[8];
    snprintf(hex, sizeof(hex), "#%02X%02X%02X", currentColor.r, currentColor.g, currentColor.b);
    return String(hex);
}

// FeatureReportCallbacks implementation
void ColorManager::FeatureReportCallbacks::onRead(NimBLECharacteristic* pChar, const std::string& value) {
    if (!colorManager) return;
    
    // Manually prepend report ID 1 to the color data
    uint8_t reportData[4];
    reportData[0] = 0x01;  // Report ID
    RGBColor color = colorManager->getColor();
    memcpy(reportData + 1, &color, 3);
    pChar->setValue(reportData, 4);
}

void ColorManager::FeatureReportCallbacks::onWrite(NimBLECharacteristic* pChar, const std::string& value) {
    if (!colorManager) return;
    
    size_t len = value.size();
    if (len == 4) {
        // Host included Report ID as first byte – skip it
        uint8_t r = value[1];
        uint8_t g = value[2];
        uint8_t b = value[3];
        colorManager->setColor(r, g, b);
    } else if (len >= 3) {
        uint8_t r = value[0];
        uint8_t g = value[1];
        uint8_t b = value[2];
        colorManager->setColor(r, g, b);
    }
}

// ColorCallbacks implementation
void ColorManager::ColorCallbacks::onWrite(NimBLECharacteristic* chr) {
    if (!colorManager) return;
    
    std::string value = chr->getValue();
    if (value.size() != 3) return;  // Expect RGB values
    
    uint8_t r = value[0];
    uint8_t g = value[1];
    uint8_t b = value[2];
    
    colorManager->setColor(r, g, b);
    
    // Update the characteristic value
    RGBColor color = colorManager->getColor();
    uint8_t colorData[3] = {color.r, color.g, color.b};
    chr->setValue(colorData, 3);
    
    Serial.print("GATT: Color set to ");
    Serial.println(colorManager->getColorHex());
} 