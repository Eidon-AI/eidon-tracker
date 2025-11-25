#ifndef BLE_CALLBACKS_H
#define BLE_CALLBACKS_H

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>
// DISABLED: Using stub in main.cpp for BLE-only mode
// #include "BNO085.h"

// Forward declarations
class BNO085;
void startIMUResetPattern();

// Server callbacks
class ServerCallbacks : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer* pServer);
    void onDisconnect(NimBLEServer* pServer);
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc);
    void onPassKeyDisplay(uint32_t pass_key);
    void onAuthenticationComplete(NimBLEConnInfo& connInfo);
};

// GATT Quaternion characteristic callback
class QuaternionCharCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic* pChar);
    void onWrite(NimBLECharacteristic* pChar, const std::string& value);
    void onNotify(NimBLECharacteristic* pChar);
    void onStatus(NimBLECharacteristic* pChar, int status, int code);
    void onSubscribe(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc, uint16_t subValue);
};

// Role configuration polling system (no callbacks needed)
// Polling will be implemented in main.cpp

#endif // BLE_CALLBACKS_H 