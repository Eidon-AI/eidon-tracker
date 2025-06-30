// NOT IN USE

// #include "GATT_Service.h"
// #include <NimBLEServer.h>
// #include <NimBLECharacteristic.h>
// #include <NimBLEService.h>
// #include "BLE_Callbacks.h"

// // External variables
// extern NimBLEService* eidonService;
// extern NimBLECharacteristic* quaternionChar;
// extern NimBLECharacteristic* calibrationChar;
// extern NimBLECharacteristic* deviceInfoChar;
// extern QuaternionData gattQuaternionData;

// // Function to setup the GATT service
// void setupGATTService(NimBLEServer* pServer) {
//     // Create the Eidon service
//     eidonService = pServer->createService(EIDON_SERVICE_UUID);
    
//     // Configure Quaternion characteristic
//     quaternionChar = eidonService->createCharacteristic(
//         QUATERNION_CHAR_UUID,
//         NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
//     );
//     quaternionChar->setValue((uint8_t*)&gattQuaternionData, sizeof(gattQuaternionData));
//     Serial.println("GATT: Quaternion characteristic created");
    
//     // Add callback to track subscription status
//     quaternionChar->setCallbacks(new QuaternionCharCallbacks());
    
//     // Configure Calibration characteristic
//     calibrationChar = eidonService->createCharacteristic(
//         CALIBRATION_CHAR_UUID,
//         NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
//     );
//     calibrationChar->setCallbacks(new CalibrationCallbacks());
//     Serial.println("GATT: Calibration characteristic created");
    
//     // Configure Device Info characteristic
//     deviceInfoChar = eidonService->createCharacteristic(
//         DEVICE_INFO_CHAR_UUID,
//         NIMBLE_PROPERTY::READ
//     );
//     Serial.println("GATT: Device Info characteristic created");
    
//     // Set initial device info - using the same format as original working code
//     uint8_t deviceInfo[8] = {
//         0x01, 0x00,  // Device ID
//         0x01, 0x02,  // Firmware version 1.2
//         100,         // Battery level
//         0, 0, 0      // Reserved
//     };
//     deviceInfoChar->setValue(deviceInfo, sizeof(deviceInfo));
    
//     // Start the service
//     eidonService->start();
// } 