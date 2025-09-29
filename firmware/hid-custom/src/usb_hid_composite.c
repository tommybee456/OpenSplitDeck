#include "usb_hid_composite.h"
#include <sample_usbd.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(usb_hid_composite, LOG_LEVEL_INF);

static const uint8_t hid_report_desc[] =
    {
        // ====== DS4 GAMEPAD COLLECTION (Report ID 1) ======
        0x05, 0x01,       // Usage Page (Generic Desktop Ctrls)
        0x09, 0x05,       // Usage (Game Pad)
        0xA1, 0x01,       // Collection (Application)
        0x85, 0x01,       //   Report ID (1)
        0x09, 0x30,       //   Usage (X)
        0x09, 0x31,       //   Usage (Y)
        0x09, 0x32,       //   Usage (Z)
        0x09, 0x35,       //   Usage (Rz)
        0x15, 0x00,       //   Logical Minimum (0)
        0x26, 0xFF, 0x00, //   Logical Maximum (255)
        0x75, 0x08,       //   Report Size (8)
        0x95, 0x04,       //   Report Count (4)
        0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

        0x09, 0x39,       //   Usage (Hat switch)
        0x15, 0x00,       //   Logical Minimum (0)
        0x25, 0x07,       //   Logical Maximum (7)
        0x35, 0x00,       //   Physical Minimum (0)
        0x46, 0x3B, 0x01, //   Physical Maximum (315)
        0x65, 0x14,       //   Unit (System: English Rotation, Length: Centimeter)
        0x75, 0x04,       //   Report Size (4)
        0x95, 0x01,       //   Report Count (1)
        0x81, 0x42,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)

        0x65, 0x00, //   Unit (None)
        0x05, 0x09, //   Usage Page (Button)
        0x19, 0x01, //   Usage Minimum (0x01)
        0x29, 0x0E, //   Usage Maximum (0x0E)
        0x15, 0x00, //   Logical Minimum (0)
        0x25, 0x01, //   Logical Maximum (1)
        0x75, 0x01, //   Report Size (1)
        0x95, 0x0E, //   Report Count (14)
        0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

        0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
        0x09, 0x20,       //   Usage (0x20)
        0x75, 0x06,       //   Report Size (6)
        0x95, 0x01,       //   Report Count (1)
        0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

        0x05, 0x01,       //   Usage Page (Generic Desktop Ctrls)
        0x09, 0x33,       //   Usage (Rx)
        0x09, 0x34,       //   Usage (Ry)
        0x15, 0x00,       //   Logical Minimum (0)
        0x26, 0xFF, 0x00, //   Logical Maximum (255)
        0x75, 0x08,       //   Report Size (8)
        0x95, 0x02,       //   Report Count (2)
        0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

        0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
        0x09, 0x21,       //   Usage (0x21)
        0x95, 0x36,       //   Report Count (54)
        0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

        0x85, 0x05, //   Report ID (5)
        0x09, 0x22, //   Usage (0x22)
        0x95, 0x1F, //   Report Count (31)
        0x91, 0x02, //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

        0x85, 0x03,       //   Report ID (3)
        0x0A, 0x21, 0x27, //   Usage (0x2721)
        0x95, 0x2F,       //   Report Count (47)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

        0x85, 0x02,       //   Report ID (2)
        0x09, 0x24,       //   Usage (0x24)
        0x95, 0x24,       //   Report Count (36)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x08,       //   Report ID (8)
        0x09, 0x25,       //   Usage (0x25)
        0x95, 0x03,       //   Report Count (3)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x10,       //   Report ID (16)
        0x09, 0x26,       //   Usage (0x26)
        0x95, 0x04,       //   Report Count (4)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x11,       //   Report ID (17)
        0x09, 0x27,       //   Usage (0x27)
        0x95, 0x02,       //   Report Count (2)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x12,       //   Report ID (18)
        0x06, 0x02, 0xFF, //   Usage Page (Vendor Defined 0xFF02)
        0x09, 0x21,       //   Usage (0x21)
        0x95, 0x0F,       //   Report Count (15)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x13,       //   Report ID (19)
        0x09, 0x22,       //   Usage (0x22)
        0x95, 0x16,       //   Report Count (22)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x14,       //   Report ID (20)
        0x06, 0x05, 0xFF, //   Usage Page (Vendor Defined 0xFF05)
        0x09, 0x20,       //   Usage (0x20)
        0x95, 0x10,       //   Report Count (16)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x15,       //   Report ID (21)
        0x09, 0x21,       //   Usage (0x21)
        0x95, 0x2C,       //   Report Count (44)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x06, 0x80, 0xFF, //   Usage Page (Vendor Defined 0xFF80)
        0x85, 0x80,       //   Report ID (128)
        0x09, 0x20,       //   Usage (0x20)
        0x95, 0x06,       //   Report Count (6)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x81,       //   Report ID (129)
        0x09, 0x21,       //   Usage (0x21)
        0x95, 0x06,       //   Report Count (6)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x82,       //   Report ID (130)
        0x09, 0x22,       //   Usage (0x22)
        0x95, 0x05,       //   Report Count (5)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x83,       //   Report ID (131)
        0x09, 0x23,       //   Usage (0x23)
        0x95, 0x01,       //   Report Count (1)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x84,       //   Report ID (132)
        0x09, 0x24,       //   Usage (0x24)
        0x95, 0x04,       //   Report Count (4)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x85,       //   Report ID (133)
        0x09, 0x25,       //   Usage (0x25)
        0x95, 0x06,       //   Report Count (6)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x86,       //   Report ID (134)
        0x09, 0x26,       //   Usage (0x26)
        0x95, 0x06,       //   Report Count (6)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x87,       //   Report ID (135)
        0x09, 0x27,       //   Usage (0x27)
        0x95, 0x23,       //   Report Count (35)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x88,       //   Report ID (136)
        0x09, 0x28,       //   Usage (0x28)
        0x95, 0x22,       //   Report Count (34)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x89,       //   Report ID (137)
        0x09, 0x29,       //   Usage (0x29)
        0x95, 0x02,       //   Report Count (2)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x90,       //   Report ID (144)
        0x09, 0x30,       //   Usage (0x30)
        0x95, 0x05,       //   Report Count (5)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x91,       //   Report ID (145)
        0x09, 0x31,       //   Usage (0x31)
        0x95, 0x03,       //   Report Count (3)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x92,       //   Report ID (146)
        0x09, 0x32,       //   Usage (0x32)
        0x95, 0x03,       //   Report Count (3)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0x93,       //   Report ID (147)
        0x09, 0x33,       //   Usage (0x33)
        0x95, 0x0C,       //   Report Count (12)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA0,       //   Report ID (160)
        0x09, 0x40,       //   Usage (0x40)
        0x95, 0x06,       //   Report Count (6)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA1,       //   Report ID (161)
        0x09, 0x41,       //   Usage (0x41)
        0x95, 0x01,       //   Report Count (1)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA2,       //   Report ID (162)
        0x09, 0x42,       //   Usage (0x42)
        0x95, 0x01,       //   Report Count (1)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA3,       //   Report ID (163)
        0x09, 0x43,       //   Usage (0x43)
        0x95, 0x30,       //   Report Count (48)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA4,       //   Report ID (164)
        0x09, 0x44,       //   Usage (0x44)
        0x95, 0x0D,       //   Report Count (13)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA5,       //   Report ID (165)
        0x09, 0x45,       //   Usage (0x45)
        0x95, 0x15,       //   Report Count (21)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA6,       //   Report ID (166)
        0x09, 0x46,       //   Usage (0x46)
        0x95, 0x15,       //   Report Count (21)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA7,       //   Report ID (167)
        0x09, 0x4A,       //   Usage (0x4A)
        0x95, 0x01,       //   Report Count (1)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA8,       //   Report ID (168)
        0x09, 0x4B,       //   Usage (0x4B)
        0x95, 0x01,       //   Report Count (1)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xA9,       //   Report ID (169)
        0x09, 0x4C,       //   Usage (0x4C)
        0x95, 0x08,       //   Report Count (8)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xAA,       //   Report ID (170)
        0x09, 0x4E,       //   Usage (0x4E)
        0x95, 0x01,       //   Report Count (1)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xAB,       //   Report ID (171)
        0x09, 0x4F,       //   Usage (0x4F)
        0x95, 0x39,       //   Report Count (57)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xAC,       //   Report ID (172)
        0x09, 0x50,       //   Usage (0x50)
        0x95, 0x39,       //   Report Count (57)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xAD,       //   Report ID (173)
        0x09, 0x51,       //   Usage (0x51)
        0x95, 0x0B,       //   Report Count (11)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xAE,       //   Report ID (174)
        0x09, 0x52,       //   Usage (0x52)
        0x95, 0x01,       //   Report Count (1)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xAF,       //   Report ID (175)
        0x09, 0x53,       //   Usage (0x53)
        0x95, 0x02,       //   Report Count (2)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xB0,       //   Report ID (176)
        0x09, 0x54,       //   Usage (0x54)
        0x95, 0x3F,       //   Report Count (63)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0xC0,             // End Collection

        // ====== AUTHENTICATION COLLECTIONS (Report IDs -16 to -13) ======

        0x06, 0xF0, 0xFF, // Usage Page (Vendor Defined 0xFFF0)
        0x09, 0x40,       // Usage (0x40)
        0xA1, 0x01,       // Collection (Application)
        0x85, 0xF0,       //   Report ID (-16) AUTH F0
        0x09, 0x47,       //   Usage (0x47)
        0x95, 0x3F,       //   Report Count (63)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xF1,       //   Report ID (-15) AUTH F1
        0x09, 0x48,       //   Usage (0x48)
        0x95, 0x3F,       //   Report Count (63)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xF2,       //   Report ID (-14) AUTH F2
        0x09, 0x49,       //   Usage (0x49)
        0x95, 0x0F,       //   Report Count (15)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0x85, 0xF3,       //   Report ID (-13) Auth F3 (Reset)
        0x0A, 0x01, 0x47, //   Usage (0x4701)
        0x95, 0x07,       //   Report Count (7)
        0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0xC0,             // End Collection

        // ====== MOUSE COLLECTION (Report ID 201) ======
        0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
        0x09, 0x02, // Usage (Mouse)
        0xA1, 0x01, // Collection (Application)
        0x85, 0xC9, //   Report ID (201)
        0x09, 0x01, //   Usage (Pointer)
        0xA1, 0x00, //   Collection (Physical)
        0x05, 0x09, //     Usage Page (Button)
        0x19, 0x01, //     Usage Minimum (0x01)
        0x29, 0x03, //     Usage Maximum (0x03)
        0x15, 0x00, //     Logical Minimum (0)
        0x25, 0x01, //     Logical Maximum (1)
        0x95, 0x03, //     Report Count (3)
        0x75, 0x01, //     Report Size (1)
        0x81, 0x02, //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x95, 0x01, //     Report Count (1)
        0x75, 0x05, //     Report Size (5)
        0x81, 0x03, //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x05, 0x01, //     Usage Page (Generic Desktop Ctrls)
        0x09, 0x30, //     Usage (X)
        0x09, 0x31, //     Usage (Y)
        0x15, 0x81, //     Logical Minimum (-127)
        0x25, 0x7F, //     Logical Maximum (127)
        0x75, 0x08, //     Report Size (8)
        0x95, 0x02, //     Report Count (2)
        0x81, 0x06, //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
        0x09, 0x38, //     Usage (Wheel)
        0x95, 0x01, //     Report Count (1)
        0x81, 0x06, //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
        0xC0,       //   End Collection
        0xC0,       // End Collection

        // ====== KEYBOARD COLLECTION (Report ID 202) ======
        0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
        0x09, 0x06, // Usage (Keyboard)
        0xA1, 0x01, // Collection (Application)
        0x85, 0xCA, //   Report ID (202)
        0x05, 0x07, //   Usage Page (Kbrd/Keypad)
        0x19, 0xE0, //   Usage Minimum (0xE0)
        0x29, 0xE7, //   Usage Maximum (0xE7)
        0x15, 0x00, //   Logical Minimum (0)
        0x25, 0x01, //   Logical Maximum (1)
        0x75, 0x01, //   Report Size (1)
        0x95, 0x08, //   Report Count (8)
        0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x95, 0x01, //   Report Count (1)
        0x75, 0x08, //   Report Size (8)
        0x81, 0x03, //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x95, 0x06, //   Report Count (6)
        0x75, 0x08, //   Report Size (8)
        0x15, 0x00, //   Logical Minimum (0)
        0x25, 0x65, //   Logical Maximum (101)
        0x05, 0x07, //   Usage Page (Kbrd/Keypad)
        0x19, 0x00, //   Usage Minimum (0x00)
        0x29, 0x65, //   Usage Maximum (0x65)
        0x81, 0x00, //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0xC0,       // End Collection
};

static K_SEM_DEFINE(ep_write_sem, 0, 1);
static const struct device *hid_device = NULL;

// Global DS4 counters (shared between both report functions)
static uint16_t ds4_timestamp_counter = 0;
static uint8_t ds4_frame_counter = 0;
static uint8_t ds4_touch_counter = 0;

// PS4/DS4 Feature Report Data (neutral calibration values for better DS4Windows compatibility)
static const uint8_t feature_0x02_calibration[] = {
    0x02,  // Report ID
    0xfe, 0xff, 0x0e, 0x00, 0x04, 0x00, 0xd4, 0x22,
    0x2a, 0xdd, 0xbb, 0x22, 0x5e, 0xdd, 0x81, 0x22, 
    0x84, 0xdd, 0x1c, 0x02, 0x1c, 0x02, 0x85, 0x1f,
    0xb0, 0xe0, 0xc6, 0x20, 0xb5, 0xe0, 0xb1, 0x20,
    0x83, 0xdf, 0x0c, 0x00
};

static const uint8_t feature_0x03_definition[] = {
    0x03, 0x21, 0x27, 0x04, 0xcf, 0x00, 0x2c, 0x56,
    0x08, 0x00, 0x3d, 0x00, 0xe8, 0x03, 0x04, 0x00,
    0xff, 0x7f, 0x0d, 0x0d, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const uint8_t feature_0x12_mac_address[] = {
    0x12,                               // Report ID
    0xAC, 0xFD, 0x93, 0xBA, 0xD1, 0xF4, // Device MAC address (6 bytes)
    0x08, 0x25, 0x00,                   // BT device class (3 bytes)
    0x12, 0xC7, 0xFA, 0x32, 0x00, 0x00  // Host MAC address (6 bytes) - matches DS4Windows fallback
};

static const uint8_t feature_0xa3_version[] = {
    0xa3, 0x4a, 0x75, 0x6e, 0x20, 0x20, 0x39, 0x20, 0x32,
    0x30, 0x31, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x31, 0x32, 0x3a, 0x33, 0x36, 0x3a, 0x34, 0x31,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x08, 0xb4, 0x01, 0x00, 0x00, 0x00,
    0x07, 0xa0, 0x10, 0x20, 0x00, 0xa0, 0x02, 0x00,
    0x00 // Extra byte to make it 48 data bytes total
};

static const uint8_t feature_0xf3_auth_reset[] = {
    0xf3, 0x0, 0x38, 0x38, 0, 0, 0, 0};

// Serial number for DS4Windows/Linux compatibility (Report ID 0x81) //this doesn't work....yet
static const uint8_t feature_0x81_serial[] = {
    0x81, 0xAC, 0xFD, 0x93, 0xBA, 0xD1, 0xF4}; // 7 bytes: report ID + 6 byte MAC-like serial

static int enable_usb_device_next(void)
{
    struct usbd_context *sample_usbd;
    int err;

    sample_usbd = sample_usbd_init_device(NULL);
    if (sample_usbd == NULL)
    {
        // LOG_ERR("Failed to initialize USB device");
        return -ENODEV;
    }

    // Set VID and PID to match DS4
    usbd_device_set_vid(sample_usbd, 0x054C); // Sony VID
    usbd_device_set_pid(sample_usbd, 0x05C4); // DS4 PID

    err = usbd_enable(sample_usbd);
    if (err)
    {
        // LOG_ERR("Failed to enable device support");
        return err;
    }

    // LOG_DBG("USB device support enabled");

    return 0;
}

static void int_in_ready_cb(const struct device *dev)
{
    ARG_UNUSED(dev);
    k_sem_give(&ep_write_sem);
}

// Feature report callback for HID Get Report requests
static int get_report_cb(const struct device *dev, uint8_t type, uint8_t id, uint16_t len, uint8_t *buf)
{
    ARG_UNUSED(dev);

    // LOG_ERR("*** GET REPORT *** type=%u, id=0x%02x, len=%u", type, id, len);

    // Handle feature reports (type 3)
    if (type == 3)
    {

        switch (id)
        {
        case 0x02: // Calibration data
            if (len >= sizeof(feature_0x02_calibration))
            {
                memcpy(buf, feature_0x02_calibration, sizeof(feature_0x02_calibration));
                // LOG_ERR("*** SENT calibration data (0x02), len=%d", sizeof(feature_0x02_calibration));
                return sizeof(feature_0x02_calibration);
            }
            // LOG_ERR("*** FAILED calibration data (0x02), requested=%d, available=%d", len, sizeof(feature_0x02_calibration));
            return -ENOTSUP;

        case 0x03: // Controller definition
            if (len >= sizeof(feature_0x03_definition))
            {
                memcpy(buf, feature_0x03_definition, sizeof(feature_0x03_definition));
                // LOG_ERR("*** SENT controller definition (0x03), len=%d", sizeof(feature_0x03_definition));
                return sizeof(feature_0x03_definition);
            }
            // LOG_ERR("*** FAILED controller definition (0x03), requested=%d, available=%d", len, sizeof(feature_0x03_definition));
            return -ENOTSUP;

        case 0x12: // MAC address
            // LOG_ERR("*** DS4Windows requesting MAC address (0x12), wLength=%d", len);
            if (len >= sizeof(feature_0x12_mac_address))
            {
                memcpy(buf, feature_0x12_mac_address, sizeof(feature_0x12_mac_address));
                // LOG_ERR("*** SENT MAC address (0x12), len=%d, Device MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                //      sizeof(feature_0x12_mac_address),
                //        feature_0x12_mac_address[1], feature_0x12_mac_address[2], feature_0x12_mac_address[3],
                //        feature_0x12_mac_address[4], feature_0x12_mac_address[5], feature_0x12_mac_address[6]);
                return sizeof(feature_0x12_mac_address);
            }
            // LOG_ERR("*** FAILED MAC address (0x12), requested=%d, available=%d", len, sizeof(feature_0x12_mac_address));
            return -ENOTSUP;

        case 0xa3: // Firmware version
            if (len >= sizeof(feature_0xa3_version))
            {
                memcpy(buf, feature_0xa3_version, sizeof(feature_0xa3_version));
                // LOG_ERR("*** SENT firmware version (0xa3), len=%d", sizeof(feature_0xa3_version));
                return sizeof(feature_0xa3_version);
            }
            // LOG_ERR("*** FAILED firmware version (0xa3), requested=%d, available=%d", len, sizeof(feature_0xa3_version));
            return -ENOTSUP;

        case 0xf3: // Authentication reset
            if (len >= sizeof(feature_0xf3_auth_reset))
            {
                memcpy(buf, feature_0xf3_auth_reset, sizeof(feature_0xf3_auth_reset));
                // LOG_ERR("*** SENT auth reset (0xf3), len=%d", sizeof(feature_0xf3_auth_reset));
                return sizeof(feature_0xf3_auth_reset);
            }
            // LOG_ERR("*** FAILED auth reset (0xf3), requested=%d, available=%d", len, sizeof(feature_0xf3_auth_reset));
            return -ENOTSUP;

        case 0x81: // Serial number
            if (len >= sizeof(feature_0x81_serial))
            {
                memcpy(buf, feature_0x81_serial, sizeof(feature_0x81_serial));
                // LOG_ERR("*** SENT serial number (0x81), len=%d", sizeof(feature_0x81_serial));
                return sizeof(feature_0x81_serial);
            }
            // LOG_ERR("*** FAILED serial number (0x81), requested=%d, available=%d", len, sizeof(feature_0x81_serial));
            return -ENOTSUP;

        default:
            // LOG_ERR("*** UNHANDLED feature report ID: 0x%02x, len=%d", id, len);
            if (len > 0)
            {
                buf[0] = id;
                if (len > 1)
                {
                    memset(&buf[1], 0, len - 1);
                }
                return len;
            }
            return -ENOTSUP;
        }
    }

    // LOG_ERR("*** GET REPORT FAILED - returning ENOTSUP");
    return -ENOTSUP;
}

// Feature report callback for HID Set Report requests
static int set_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
                         const uint16_t len, const uint8_t *const buf)
{
    ARG_UNUSED(dev);

    // LOG_INF("Set Report: type=%u, id=0x%02x, len=%u", type, id, len);

    // Handle output reports (type 2) - for rumble, LED, etc.
    if (type == 2)
    {
        // Log the data being sent for debugging
        if (buf && len > 0)
        {
            // LOG_HEXDUMP_INF(buf, MIN(len, 16), "Output report data:");
        }

        switch (id)
        {
        case 0x01: // Possible device info request (seen in Wireshark)
            if (buf && len >= 2)
            {
                // LOG_ERR("*** OUTPUT REPORT 0x01 received, data[0]=0x%02x, data[1]=0x%02x",
                //        buf[0], buf[1]);
                // DS4Windows might be requesting device info with this command
                // We may need to respond with device information including serial
            }
            else
            {
                // LOG_ERR("*** OUTPUT REPORT 0x01 received with insufficient data");
            }
            break;

        case 0x05: // DS4 Output Report (rumble, LED, etc.)
            // LOG_INF("DS4 Output Report 0x05 received (rumble/LED control)");
            // TODO: Could implement actual rumble/LED control here
            break;

        default:
            // LOG_INF("Output report 0x%02x acknowledged", id);
            break;
        }

        // Acknowledge the output request
        return 0;
    }

    // Handle feature reports (type 3)
    if (type == 3)
    {
        // Log the data being sent for debugging
        if (buf && len > 0)
        {
            // LOG_HEXDUMP_INF(buf, MIN(len, 16), "Set report data:");
        }

        switch (id)
        {
        case 0xf0: // Authentication F0
        case 0xf1: // Authentication F1
        case 0xf2: // Authentication F2
            // LOG_INF("Authentication report 0x%02x received", id);
            break;

        default:
            // LOG_INF("Set report 0x%02x acknowledged", id);
            break;
        }
        return 0;
    }

    return -ENOTSUP;
}

static const struct hid_device_ops ops = {
    .input_report_done = int_in_ready_cb,
    .get_report = get_report_cb,
    .set_report = set_report_cb,
};

// Initialize USB HID composite device
int usb_hid_composite_init(void)
{
    int ret;

    hid_device = DEVICE_DT_GET_ONE(zephyr_hid_device);

    if (hid_device == NULL)
    {
        // LOG_ERR("Cannot get USB HID Device");
        return -ENODEV;
    }

    hid_device_register(hid_device,
                        hid_report_desc, sizeof(hid_report_desc),
                        &ops);

    //usb_hid_init(hid_device);

    ret = enable_usb_device_next();

    if (ret != 0)
    {
        // LOG_ERR("Failed to enable USB");
        return ret;
    }

    // LOG_ERR("*** USB HID composite device initialized - DS4 Feature Reports Ready ***");
    return 0;
}

// Get the HID device handle
const struct device *usb_hid_composite_get_device(void)
{
    return hid_device;
}

// Simple DS4 Report structure - matches exact byte layout
// https://controllers.fandom.com/wiki/Sony_DualShock_4/Data_Structures#HID_Report_0x05_Output_USB/Dongle
typedef struct __attribute__((packed))
{
    uint8_t report_id;        // Byte 0: Report ID (0x01)
    uint8_t left_stick_x;     // Byte 1: Left analog stick X
    uint8_t left_stick_y;     // Byte 2: Left analog stick Y
    uint8_t right_stick_x;    // Byte 3: Right analog stick X
    uint8_t right_stick_y;    // Byte 4: Right analog stick Y
    uint8_t buttons_dpad;     // Byte 5: D-pad (low 4 bits) + face buttons (high 4 bits)
    uint8_t buttons_shoulder; // Byte 6: Shoulder buttons + Share/Options + L3/R3
    uint8_t buttons_special;  // Byte 7: PS button + Touchpad + Counter (6 bits)
    uint8_t left_trigger;     // Byte 8: Left trigger (L2)
    uint8_t right_trigger;    // Byte 9: Right trigger (R2)
    uint16_t timestamp;       // Bytes 10-11: Timestamp
    uint8_t battery;          // Byte 12: Battery info
    int16_t gyro_x;           // Bytes 13-14: Gyroscope X
    int16_t gyro_y;           // Bytes 15-16: Gyroscope Y
    int16_t gyro_z;           // Bytes 17-18: Gyroscope Z
    int16_t accel_x;          // Bytes 19-20: Accelerometer X
    int16_t accel_y;          // Bytes 21-22: Accelerometer Y
    int16_t accel_z;          // Bytes 23-24: Accelerometer Z
    uint8_t reserved[5];      // Bytes 25-29: Reserved/unknown
    uint8_t extension;        // Byte 30: Extension byte
    uint8_t unknown1[2];      // Bytes 31-32: Unknown
    uint8_t touchpad_packets; // Byte 33: Number of touchpad packets
    uint8_t packet_counter;   // Byte 34: Packet counter
    uint8_t touch1_data[4];   // Bytes 35-38: Touch 1 data
    uint8_t touch2_data[4];   // Bytes 39-42: Touch 2 data
    uint8_t unknown2[21];     // Bytes 43-63: Unknown/padding
} SimpleDS4Report;


// Helper function to send DS4 gamepad report with touchpad and IMU data
void usb_hid_send_ds4_report_with_touchpad_and_imu(const struct device *hid_dev, uint8_t dpad, uint8_t buttons1, uint8_t buttons2,
                                                   uint8_t left_x, uint8_t left_y, uint8_t right_x, uint8_t right_y,
                                                   uint8_t left_trigger, uint8_t right_trigger,
                                                   bool touch1_active, uint16_t touch1_x, uint16_t touch1_y,
                                                   bool touch2_active, uint16_t touch2_x, uint16_t touch2_y,
                                                   int16_t accel_x, int16_t accel_y, int16_t accel_z,
                                                   int16_t gyro_x, int16_t gyro_y, int16_t gyro_z)
{
    // Ensure touch slot 1 is used before slot 2 (DS4 protocol requirement)
bool actual_touch1_active, actual_touch2_active;
uint16_t actual_touch1_x, actual_touch1_y, actual_touch2_x, actual_touch2_y;

if (touch1_active && touch2_active) {
    // Both active - keep original assignment
    actual_touch1_active = touch1_active;
    actual_touch1_x = touch1_x;
    actual_touch1_y = touch1_y;
    actual_touch2_active = touch2_active;
    actual_touch2_x = touch2_x;
    actual_touch2_y = touch2_y;
} else if (touch1_active) {
    // Only touch1 active - use slot 1
    actual_touch1_active = true;
    actual_touch1_x = touch1_x;
    actual_touch1_y = touch1_y;
    actual_touch2_active = false;
    actual_touch2_x = 0;
    actual_touch2_y = 0;
} else if (touch2_active) {
    // Only touch2 active - move to slot 1
    actual_touch1_active = true;
    actual_touch1_x = touch2_x;
    actual_touch1_y = touch2_y;
    actual_touch2_active = false;
    actual_touch2_x = 0;
    actual_touch2_y = 0;
} else {
    // Neither active
    actual_touch1_active = false;
    actual_touch1_x = 0;
    actual_touch1_y = 0;
    actual_touch2_active = false;
    actual_touch2_x = 0;
    actual_touch2_y = 0;
}
    SimpleDS4Report ds4_report;

    // Clear the report first
    memset(&ds4_report, 0, sizeof(ds4_report));

    // Fill basic controller data
    ds4_report.report_id = DS4_REPORT_ID;
    ds4_report.left_stick_x = left_x;
    ds4_report.left_stick_y = left_y;
    ds4_report.right_stick_x = right_x;
    ds4_report.right_stick_y = right_y;

    // Pack buttons manually (no bit fields)
    uint8_t buttons_byte1 = 0;
    uint8_t buttons_byte2 = 0;
    uint8_t buttons_byte3 = 0;

    // Byte 5: dpad (low 4 bits) + face buttons (high 4 bits)
    buttons_byte1 = (dpad & 0x0F) | ((buttons1 & 0x0F) << 4);

    // Byte 6: shoulder buttons (L1,R1,L2,R2) + share/options/L3/R3
    buttons_byte2 = ((buttons1 & 0xF0) >> 4) | ((buttons2 & 0x0F) << 4);

    // Byte 7: PS button + touchpad + frame counter
    buttons_byte3 = ((buttons2 & 0x30) >> 4) | ((ds4_frame_counter & 0x3F) << 2);

    ds4_report.buttons_dpad = buttons_byte1;
    ds4_report.buttons_shoulder = buttons_byte2;
    ds4_report.buttons_special = buttons_byte3;

    // Frame counter increment
    ds4_frame_counter = 0;
    if (ds4_frame_counter > 63)
    {
        ds4_frame_counter = 0;
    }

    // Trigger values (bytes 8-9)
    ds4_report.left_trigger = left_trigger;
    ds4_report.right_trigger = right_trigger;

    // Timing counter for authenticity (bytes 10-11)
    ds4_report.timestamp = ds4_timestamp_counter;
    ds4_timestamp_counter += 45;

    // Battery/USB state (byte 12)
    ds4_report.battery = 0x0B; // USB charging state

    // Sensor data - convert from int8_t to int16_t with scaling
    // Gyroscope data (bytes 13-18) - try larger scaling for gyro
    ds4_report.gyro_x = gyro_x; // Increased from 100 to 500
    ds4_report.gyro_y = gyro_y;
    ds4_report.gyro_z = gyro_z;

    // Accelerometer data (bytes 19-24)
    ds4_report.accel_x = accel_x;
    ds4_report.accel_y = accel_y;
    ds4_report.accel_z = accel_z;

    // Debug log the first time we have non-zero gyro data
    if ((gyro_x != 0 || gyro_y != 0 || gyro_z != 0))
    {
        static bool logged_gyro = false;
        if (!logged_gyro)
        {
            // LOG_INF("Gyro data: input(%d,%d,%d) -> output(%d,%d,%d)",
            //        gyro_x, gyro_y, gyro_z,
            //        ds4_report.gyro_x, ds4_report.gyro_y, ds4_report.gyro_z);
            logged_gyro = true;
        }
    }

    // Touchpad data (bytes 33-42)
    ds4_report.touchpad_packets = 1; // Always report 1 packet
    ds4_report.packet_counter = ds4_touch_counter;
    ds4_touch_counter++;
    if (ds4_touch_counter > 255)
    {
        ds4_touch_counter = 0;
    }

    // Touch 1 data (bytes 35-38)
    if (actual_touch1_active)
    {
        ds4_report.touch1_data[0] = ds4_touch_counter & 0x7F;                            // Counter with active bit clear (bit 7 = 0)
        ds4_report.touch1_data[1] = actual_touch1_x & 0xFF;                                     // X low 8 bits
        ds4_report.touch1_data[2] = ((actual_touch1_x >> 8) & 0x0F) | ((actual_touch1_y & 0x0F) << 4); // X high 4 bits + Y low 4 bits
        ds4_report.touch1_data[3] = (actual_touch1_y >> 4) & 0xFF;                              // Y high 8 bits
    }
    else
    {
        ds4_report.touch1_data[0] = 0x80; // Inactive touch (bit 7 set = 1)
        ds4_report.touch1_data[1] = 0;
        ds4_report.touch1_data[2] = 0;
        ds4_report.touch1_data[3] = 0;
    }

    // Touch 2 data (bytes 39-42) - use incremented counter for second touch
    uint8_t touch2_counter = (ds4_touch_counter + 1) & 0xFF;
    if (actual_touch2_active)
    {
        ds4_report.touch2_data[0] = touch2_counter & 0x7F;                               // Different counter with active bit clear (bit 7 = 0)
        ds4_report.touch2_data[1] = actual_touch2_x & 0xFF;                                     // X low 8 bits
        ds4_report.touch2_data[2] = ((actual_touch2_x >> 8) & 0x0F) | ((actual_touch2_y & 0x0F) << 4); // X high 4 bits + Y low 4 bits
        ds4_report.touch2_data[3] = (actual_touch2_y >> 4) & 0xFF;                              // Y high 8 bits
    }
    else
    {
        ds4_report.touch2_data[0] = 0x80; // Inactive touch (bit 7 set = 1)
        ds4_report.touch2_data[1] = 0;
        ds4_report.touch2_data[2] = 0;
        ds4_report.touch2_data[3] = 0;
    }

    // Send the complete DS4 report
    uint32_t usb_start = k_uptime_get_32();
    int ret = hid_int_ep_write(hid_dev, (uint8_t *)&ds4_report, sizeof(ds4_report), NULL);
    if (ret == 0)
    {
        k_sem_take(&ep_write_sem, K_FOREVER);
        
        // Monitor USB write timing - only log if really slow
        uint32_t usb_time = k_uptime_get_32() - usb_start;
        static uint32_t max_usb_time = 0;
        if (usb_time > max_usb_time) {
            max_usb_time = usb_time;
            if (usb_time > 5) { // Only warn if USB write takes over 5ms (was 1ms)
                LOG_WRN("USB write took %dms (new max)", usb_time);
            }
        }
    }
    else
    {
        LOG_ERR("HID write failed: %d", ret);
    }

    // Log successful report
    // LOG_DBG("DS4 report sent with touchpad and IMU: touch1(%s), touch2(%s), gyro(%d,%d,%d)",
    //        touch1_active ? "active" : "inactive", touch2_active ? "active" : "inactive",
    //        gyro_x, gyro_y, gyro_z);

    // Debug struct size and touchpad data when active
    static uint32_t debug_counter = 0;
    debug_counter++;

    // Log every 60 reports (1 second at 60Hz)
    if (debug_counter % 60 == 0)
    {
        // LOG_INF("SimpleDS4Report size: %d bytes (expected 64)", sizeof(SimpleDS4Report));
        if (touch1_active || touch2_active)
        {
            // LOG_INF("Touchpad debug: T1[%02X %02X %02X %02X] T2[%02X %02X %02X %02X]",
        //            ds4_report.touch1_data[0], ds4_report.touch1_data[1],
        //            ds4_report.touch1_data[2], ds4_report.touch1_data[3],
        //            ds4_report.touch2_data[0], ds4_report.touch2_data[1],
        //            ds4_report.touch2_data[2], ds4_report.touch2_data[3]);
        }
        // Log first few bytes of the report to see the structure
        // uint8_t *report_bytes = (uint8_t *)&ds4_report;
        // LOG_INF("Report bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        //        report_bytes[0], report_bytes[1], report_bytes[2], report_bytes[3], report_bytes[4],
        //        report_bytes[5], report_bytes[6], report_bytes[7], report_bytes[8], report_bytes[9]);
    }
}

// Helper function to send mouse report
void usb_hid_send_mouse_report(const struct device *hid_dev, int8_t x, int8_t y, int8_t wheel, uint8_t buttons)
{
    uint8_t mouse_report[5] = {
        MOUSE_REPORT_ID,
        buttons, // Button state (bit 0=left, bit 1=right, bit 2=middle)
        x,       // X movement (-127 to 127)
        y,       // Y movement (-127 to 127)
        wheel    // Wheel movement (-127 to 127)
    };

    int ret = hid_int_ep_write(hid_dev, mouse_report, sizeof(mouse_report), NULL);
    if (ret)
    {
        // LOG_DBG("Mouse HID write error: %d", ret);
    }
    else
    {
        k_sem_take(&ep_write_sem, K_FOREVER);
    }
}

// Helper function to send keyboard report
void usb_hid_send_keyboard_report(const struct device *hid_dev, uint8_t modifiers, uint8_t key1, uint8_t key2, uint8_t key3, uint8_t key4, uint8_t key5, uint8_t key6)
{
    uint8_t keyboard_report[9] = {
        KEYBOARD_REPORT_ID,
        modifiers,                         // Modifier keys (Ctrl, Shift, Alt, etc.)
        0x00,                              // Reserved
        key1, key2, key3, key4, key5, key6 // Up to 6 simultaneous keys
    };

    int ret = hid_int_ep_write(hid_dev, keyboard_report, sizeof(keyboard_report), NULL);
    if (ret)
    {
        // LOG_DBG("Keyboard HID write error: %d", ret);
    }
    else
    {
        k_sem_take(&ep_write_sem, K_FOREVER);
    }
}