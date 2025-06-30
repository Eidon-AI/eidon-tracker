#ifndef HID_DESCRIPTOR_H
#define HID_DESCRIPTOR_H

#include <stdint.h>
#include <stddef.h>

// HID Report Descriptor for Eidon Tracker
// One top-level application collection, Usage = Orientation
//  ├─ Input  (Quaternion + 2 switch bits)
//  ├─ Output (Vendor byte)
//  └─ Feature(RGB)

extern const uint8_t hid_report_descriptor[];

// Size of the HID report descriptor
extern const size_t hid_report_descriptor_size;

#endif // HID_DESCRIPTOR_H 