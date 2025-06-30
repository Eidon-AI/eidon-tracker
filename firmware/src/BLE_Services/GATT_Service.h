// NOT IN USE

// #ifndef GATT_SERVICE_H
// #define GATT_SERVICE_H

// #include <stdint.h>

// // Custom GATT Service UUIDs
// #define EIDON_SERVICE_UUID        "E1D00001-8B5A-3E5B-9E23-4F9B5C91BBDE"
// #define QUATERNION_CHAR_UUID      "E1D00002-8B5A-3E5B-9E23-4F9B5C91BBDE"
// #define CALIBRATION_CHAR_UUID     "E1D00003-8B5A-3E5B-9E23-4F9B5C91BBDE"
// #define DEVICE_INFO_CHAR_UUID     "E1D00005-8B5A-3E5B-9E23-4F9B5C91BBDE"

// // Quaternion data structure for GATT (20 bytes - matches original working version)
// struct QuaternionData {
//     float w;
//     float x;
//     float y;
//     float z;
//     uint8_t switches;    // bit 0: isLeft, bit 1: isUpper
//     uint8_t reserved[3]; // padding to 20 bytes
// } __attribute__((packed));

// // Device info structure (8 bytes)
// struct DeviceInfo {
//     uint16_t device_id;      // Device ID
//     uint16_t firmware_ver;   // Firmware version
//     uint8_t battery_level;   // Battery level
//     uint8_t reserved[3];     // Reserved for future use
// } __attribute__((packed));

// // Function declaration
// void setupGATTService(class NimBLEServer* pServer);

// #endif // GATT_SERVICE_H 