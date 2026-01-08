#include "BLE_OTA_Service.h"
#include <Arduino.h>

// State variables
static bool updateFlag = false;
static bool readyFlag = false;
static size_t bytesReceived = 0;
static size_t totalSize = 0;

// Callback for OTA Control Characteristic
class OtaControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) {
        std::string value = pChar->getValue();
        if (value.length() > 0) {
            uint8_t command = value[0];

            // Command: 0x00 = Start Update
            if (command == 0x00) {
                // Next 4 bytes are size
                if (value.length() >= 5) {
                    totalSize = (uint32_t)value[1] | ((uint32_t)value[2] << 8) | ((uint32_t)value[3] << 16) | ((uint32_t)value[4] << 24);
                    Serial.printf("OTA: Start request. Size: %d bytes\n", totalSize);

                    if (!Update.begin(totalSize, U_FLASH)) {
                        Serial.println("OTA: Not enough space to begin OTA");
                        // Send Error: 0x0F
                        uint8_t err[] = {0x0F};
                        pChar->notify(err, 1);
                    } else {
                        // Send ACK: 0x01
                        uint8_t ack[] = {0x01};
                        pChar->notify(ack, 1);
                        bytesReceived = 0;
                        updateFlag = true;
                        Serial.println("OTA: Update started");
                    }
                }
            } 
            // Command: 0x03 = End Update
            else if (command == 0x03) {
                if (updateFlag) {
                    Serial.println("OTA: End request received");
                    if (Update.end(true)) {
                        Serial.println("OTA: Update Success!");
                        // Send Success: 0x02
                        uint8_t success[] = {0x02};
                        pChar->notify(success, 1);
                        delay(1000);
                        ESP.restart();
                    } else {
                        Serial.println("OTA: Update Failed");
                        // Send Error: 0x0F
                        uint8_t err[] = {0x0F};
                        pChar->notify(err, 1);
                    }
                    updateFlag = false;
                }
            }
        }
    }
};

// Callback for OTA Data Characteristic
class OtaDataCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) {
        if (updateFlag) {
            std::string rxData = pChar->getValue();
            if (rxData.length() > 0) {
                if (Update.write((uint8_t*)rxData.data(), rxData.length()) != rxData.length()) {
                    Serial.println("OTA: Write failed");
                } else {
                    bytesReceived += rxData.length();
                    // Optional: Print progress sparingly
                    // Serial.printf("OTA: %d / %d\n", bytesReceived, totalSize);
                }
            }
        }
    }
};

void BleOtaService::begin(NimBLEServer* pServer) {
    NimBLEService* pOtaService = pServer->createService(OTA_SERVICE_UUID);

    // Control Characteristic
    NimBLECharacteristic* pOtaControlChar = pOtaService->createCharacteristic(
        OTA_CONTROL_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    pOtaControlChar->setCallbacks(new OtaControlCallbacks());

    // Data Characteristic
    NimBLECharacteristic* pOtaDataChar = pOtaService->createCharacteristic(
        OTA_DATA_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE_NR
    );
    pOtaDataChar->setCallbacks(new OtaDataCallbacks());

    pOtaService->start();
    
    // Add service UUID to advertising is handled in main.cpp, but we can access it here if needed
    // For now, we assume main.cpp will add it
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising) {
        pAdvertising->addServiceUUID(OTA_SERVICE_UUID);
    }
    
    Serial.println("OTA: Service started");
}

