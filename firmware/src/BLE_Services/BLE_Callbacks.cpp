#include "BLE_Callbacks.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>

// External variables that the callbacks need access to
extern bool deviceConnected;
extern bool quaternionSubscribed;
extern BNO085 imu;
extern NimBLECharacteristic* calibrationChar;

// Server callbacks implementation
void ServerCallbacks::onConnect(NimBLEServer* pServer) {
    Serial.println("Client connected!");
    Serial.println("Connection established successfully");
    Serial.println("CALLBACK: onConnect fired!");
    deviceConnected = true;
    
    // Stop advertising when connected to prevent conflicts
    if (NimBLEDevice::getAdvertising()->isAdvertising()) {
        Serial.println("Stopping advertising after connection");
        NimBLEDevice::getAdvertising()->stop();
    }
    
    // Request stable connection parameters to prevent disconnections
    NimBLEConnInfo connInfo = pServer->getPeerInfo(0);
    Serial.println("Requesting stable connection parameters...");
    // Use conservative parameters: 12-24ms interval, latency 0, timeout 400ms
    pServer->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
}

void ServerCallbacks::onDisconnect(NimBLEServer* pServer) {
    Serial.println("Client disconnected");
    Serial.println("Connection terminated");
    Serial.println("CALLBACK: onDisconnect fired!");
    deviceConnected = false;
    
    // Immediately restart advertising when disconnected
    Serial.println("Disconnect callback: restarting advertising");
    NimBLEDevice::startAdvertising();
}

void ServerCallbacks::onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
    Serial.print("MTU changed to: ");
    Serial.println(MTU);
}

void ServerCallbacks::onPassKeyDisplay(uint32_t pass_key) {
    Serial.print("Passkey Display: ");
    Serial.println(pass_key);
}

void ServerCallbacks::onAuthenticationComplete(NimBLEConnInfo& connInfo) {
    Serial.println("Authentication Complete");
    Serial.print("Secure: ");
    Serial.println(connInfo.isEncrypted() ? "Yes" : "No");
    
    // Print connection parameters for debugging
    Serial.print("Connection interval: ");
    Serial.print(connInfo.getConnInterval() * 1.25);
    Serial.println("ms");
    Serial.print("Connection latency: ");
    Serial.println(connInfo.getConnLatency());
    Serial.print("Supervision timeout: ");
    Serial.print(connInfo.getConnTimeout() * 10);
    Serial.println("ms");
}

// Output report callback implementation
void OutputReportCallbacks::onWrite(NimBLECharacteristic* pChar, const std::string& value) {
    if (!value.empty()) {
        uint8_t cmd = static_cast<uint8_t>(value[0]);
        Serial.print("Output report received, cmd=0x");
        Serial.println(cmd, HEX);

        if (cmd == 0x01) {
            if (imu.isAvailable()) {
                Serial.println("Reset command: resetting BNO085");
                // Reset BNO085 using the library
                imu.reset();
                startIMUResetPattern(); // Start IMU reset LED pattern
            } else {
                Serial.println("Reset command received, but BNO085 is not available");
            }
        }
    }
}

// GATT Calibration characteristic write callback implementation
void CalibrationCallbacks::onWrite(NimBLECharacteristic* chr) {
    std::string value = chr->getValue();
    if (value.empty()) return;
    
    uint8_t cmd = static_cast<uint8_t>(value[0]);
    switch (cmd) {
        case 0x01: { // Reset/calibrate IMU
            Serial.println("GATT: IMU calibration requested");
            if (imu.isAvailable()) {
                imu.reset();
                startIMUResetPattern(); // Start IMU reset LED pattern
            } else {
                Serial.println("GATT: IMU calibration requested, but BNO085 is not available");
            }
            
            // Send acknowledgment
            uint8_t ack = 0x01;
            calibrationChar->setValue(&ack, 1);
            break;
        }
        default:
            Serial.print("GATT: Unknown calibration command 0x");
            Serial.println(cmd, HEX);
            break;
    }
}

// GATT Quaternion characteristic callback implementation
void QuaternionCharCallbacks::onRead(NimBLECharacteristic* pChar) {
    Serial.println("GATT: Quaternion characteristic read request");
}

void QuaternionCharCallbacks::onWrite(NimBLECharacteristic* pChar) {
    Serial.println("GATT: Quaternion characteristic write request (unexpected)");
}

void QuaternionCharCallbacks::onNotify(NimBLECharacteristic* pChar) {
    Serial.println("GATT: Quaternion notification sent");
}

void QuaternionCharCallbacks::onStatus(NimBLECharacteristic* pChar, int status, int code) {
    Serial.print("GATT: Quaternion characteristic status: ");
    Serial.print(status);
    Serial.print(" code: ");
    Serial.println(code);
}

void QuaternionCharCallbacks::onSubscribe(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc, uint16_t subValue) {
    Serial.print("GATT: Quaternion notifications ");
    Serial.print(subValue == 0 ? "disabled" : "enabled");
    Serial.print(" for connection: ");
    Serial.println(desc->conn_handle);
    
    // Update global subscription status
    quaternionSubscribed = (subValue != 0);
} 