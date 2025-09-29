#ifndef USB_HID_COMPOSITE_H
#define USB_HID_COMPOSITE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/class/usb_hid.h>

// Report IDs
#define DS4_REPORT_ID     1
#define MOUSE_REPORT_ID   201
#define KEYBOARD_REPORT_ID 202

// Initialize USB HID composite device
int usb_hid_composite_init(void);

// Get the HID device handle
const struct device* usb_hid_composite_get_device(void);

// Helper function to send DS4 gamepad report with touchpad and IMU data
void usb_hid_send_ds4_report_with_touchpad_and_imu(const struct device *hid_dev, uint8_t dpad, uint8_t buttons1, uint8_t buttons2, 
                             uint8_t left_x, uint8_t left_y, uint8_t right_x, uint8_t right_y, 
                             uint8_t left_trigger, uint8_t right_trigger,
                             bool touch1_active, uint16_t touch1_x, uint16_t touch1_y,
                             bool touch2_active, uint16_t touch2_x, uint16_t touch2_y,
                             int16_t accel_x, int16_t accel_y, int16_t accel_z,
                             int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);

// Helper function to send mouse report
void usb_hid_send_mouse_report(const struct device *hid_dev, int8_t x, int8_t y, int8_t wheel, uint8_t buttons);

// Helper function to send keyboard report  
void usb_hid_send_keyboard_report(const struct device *hid_dev, uint8_t modifiers, uint8_t key1, uint8_t key2, uint8_t key3, uint8_t key4, uint8_t key5, uint8_t key6);

#endif // USB_HID_COMPOSITE_H