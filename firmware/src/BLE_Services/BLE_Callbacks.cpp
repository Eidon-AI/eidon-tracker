#include "BLE_Callbacks.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include "DeviceConfig.h"

// External variables that the callbacks need access to
extern bool deviceConnected;
extern bool quaternionSubscribed;
extern BNO085 imu;
extern NimBLECharacteristic* calibrationChar;

// Server callbacks implementation
void ServerCallbacks::onConnect(NimBLEServer* pServer) {
    deviceConnected = true;
    if (NimBLEDevice::getAdvertising()->isAdvertising()) {
        NimBLEDevice::getAdvertising()->stop();
    }
    NimBLEConnInfo connInfo = pServer->getPeerInfo(0);
    pServer->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
}

void ServerCallbacks::onDisconnect(NimBLEServer* pServer) {
    deviceConnected = false;
    NimBLEDevice::startAdvertising();
}

void ServerCallbacks::onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {}
void ServerCallbacks::onPassKeyDisplay(uint32_t pass_key) {}
void ServerCallbacks::onAuthenticationComplete(NimBLEConnInfo& connInfo) {}



// GATT Quaternion characteristic callback implementation
void QuaternionCharCallbacks::onRead(NimBLECharacteristic* pChar) {}
void QuaternionCharCallbacks::onWrite(NimBLECharacteristic* pChar, const std::string& value) {}
void QuaternionCharCallbacks::onNotify(NimBLECharacteristic* pChar) {}
void QuaternionCharCallbacks::onStatus(NimBLECharacteristic* pChar, int status, int code) {}
void QuaternionCharCallbacks::onSubscribe(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc, uint16_t subValue) {
    quaternionSubscribed = (subValue != 0);
}

// Hub Client callbacks implementation
void HubClientCallbacks::onConnect(NimBLEClient* pClient) {
    Serial.println("=== HUB CLIENT: Child connected ===");
    Serial.printf("Connected to child at address: %s\n", pClient->getPeerAddress().toString().c_str());
}

void HubClientCallbacks::onDisconnect(NimBLEClient* pClient) {
    Serial.println("=== HUB CLIENT: Child disconnected ===");
    Serial.printf("Disconnected from child at address: %s\n", pClient->getPeerAddress().toString().c_str());
}

// Role configuration callbacks removed - using polling system instead 