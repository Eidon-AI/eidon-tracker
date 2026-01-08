#ifndef BLE_OTA_SERVICE_H
#define BLE_OTA_SERVICE_H

#include <NimBLEDevice.h>
#include <Update.h>

// UUIDs for the OTA Service
// Use a custom 128-bit UUID for the service
#define OTA_SERVICE_UUID "E1D0000A-8B5A-3E5B-9E23-4F9B5C91BBDE" 

// Characteristics
#define OTA_CONTROL_CHAR_UUID "E1D0000E-8B5A-3E5B-9E23-4F9B5C91BBDE" // Write/Notify for commands
#define OTA_DATA_CHAR_UUID    "E1D0000F-8B5A-3E5B-9E23-4F9B5C91BBDE" // WriteNoResponse for data

class BleOtaService {
public:
    static void begin(NimBLEServer* pServer);
};

#endif // BLE_OTA_SERVICE_H

