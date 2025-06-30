#include "HID_Descriptor.h"

/* One top-level application collection, Usage = Orientation                */
/*  ├─ Input  (Quaternion + 2 switch bits)                                  */
/*  ├─ Output (Vendor byte)                                                 */
/*  └─ Feature(RGB)                                                         */

const uint8_t hid_report_descriptor[] = {

  /* -----------------------------------------------------------------------
   * Top-level collection : sensor orientation + vendor channel, ID = 1
   * ---------------------------------------------------------------------*/
  0x05, 0x20,             /* UsagePage (Sensor)                    */
  0x09, 0x80,             /* Usage     (Orientation)               */
  0xA1, 0x01,             /* Collection (Application)              */

    0x85, 0x01,           /*   Report ID (1)                       */

    /* --- quaternion : 4 × 16-bit -------------------------------------- */
    0x0A, 0x83, 0x04,     /*   Usage 0x0483 – Quaternion           */
    0x75, 0x10,           /*   ReportSize 16                       */
    0x95, 0x04,           /*   ReportCount 4                       */
    0x17, 0x00,0x00,0x00,0x00, /* Logical Min 0                    */
    0x27, 0xFF,0xFF,0x00,0x00, /* Logical Max 65535                */
    0x81, 0x02,           /*   Input (Data,Var,Abs)                */

    /* --- eight padding bits --------------------------------------------- */
    0x95, 0x08, 0x75, 0x01,
    0x81, 0x03,           /*   Input (Cnst,Var,Abs)                */

    /* ------------------------------------------------------------------
     * Vendor-defined channel : Output (1 byte)
     * ---------------------------------------------------------------- */
    0x06, 0x00, 0xFF,     /*   UsagePage (Vendor 0xFF00)           */
    0x09, 0x01,           /*   Usage      (Vendor 1)               */
    0x15, 0x00, 0x26, 0xFF, 0x00,   /* Logical 0-255               */
    0x75, 0x08, 0x95, 0x01,         /* ReportSize 8, Count 1       */
    0x91, 0x02,           /*   Output (Data,Var,Abs)               */

    /* ------------------------------------------------------------------
     * Vendor-defined Feature report : saved RGB (3 bytes)
     * ---------------------------------------------------------------- */
    0x09, 0x02,           /*   Usage (Vendor 2)                    */
    0x95, 0x03,           /*   ReportCount 3                       */
    0xB1, 0x02,           /*   Feature (Data,Var,Abs)              */

  0xC0                  /* End Collection                         */
};

// Size of the HID report descriptor
const size_t hid_report_descriptor_size = sizeof(hid_report_descriptor);