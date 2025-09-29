/**
 * @file trackpad_driver.h
 * @brief IQS7211E Trackpad Driver Library
 * 
 * This driver provides a clean interface for the IQS7211E trackpad sensor,
 * wrapping the complex Arduino-style state machine in a simple API.
 * 
 * Features:
 * - Automatic initialization and state management
 * - Touch coordinate reading with scaling
 * - Gesture detection (tap, swipe, hold, etc.)
 * - Sleep/wake functionality for power management
 * - Clean controller data format integration
 * - Hardware abstraction for GPIO and I2C
 */

#ifndef TRACKPAD_DRIVER_H
#define TRACKPAD_DRIVER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "IQS7211E.h"

// Status codes for trackpad driver
typedef enum {
    TRACKPAD_STATUS_OK = 0,
    TRACKPAD_STATUS_ERROR,
    TRACKPAD_STATUS_NOT_INITIALIZED,
    TRACKPAD_STATUS_DEVICE_NOT_FOUND,
    TRACKPAD_STATUS_TIMEOUT,
    TRACKPAD_STATUS_INVALID_PARAM
} trackpad_status_t;

// Trackpad configuration structure
typedef struct {
    const struct device *i2c_dev;           // I2C device for communication
    uint8_t device_address;                 // I2C address (typically 0x56)
    const struct gpio_dt_spec *ready_pin;   // Ready/interrupt pin
    uint32_t coordinate_max_x;              // Maximum X coordinate
    uint32_t coordinate_max_y;              // Maximum Y coordinate
    uint8_t controller_scale_max;           // Controller coordinate scale (0-127)
    bool debug_logging;                     // Enable detailed debug logging
} trackpad_config_t;

// Trackpad touch data structure
typedef struct {
    uint8_t num_fingers;                    // Number of fingers detected (0-2)
    uint16_t finger1_x;                     // Finger 1 X coordinate (raw)
    uint16_t finger1_y;                     // Finger 1 Y coordinate (raw)
    uint16_t finger2_x;                     // Finger 2 X coordinate (raw)
    uint16_t finger2_y;                     // Finger 2 Y coordinate (raw)
    uint8_t scaled_x;                       // Scaled X coordinate for controller (0-127)
    uint8_t scaled_y;                       // Scaled Y coordinate for controller (0-127)
    bool valid_touch;                       // True if touch data is valid
} trackpad_touch_data_t;

// Trackpad gesture flags
typedef struct {
    bool single_tap;
    bool double_tap;
    bool two_finger_tap;
    bool press_and_hold;
    bool palm_gesture;
    bool swipe_up;
    bool swipe_down;
    bool swipe_left;
    bool swipe_right;
} trackpad_gestures_t;

// Combined trackpad data for controller integration
typedef struct {
    trackpad_touch_data_t touch;
    trackpad_gestures_t gestures;
    bool device_ready;
    bool new_data_available;
} trackpad_controller_data_t;

/**
 * @brief Initialize the trackpad driver
 * @param config Trackpad configuration structure
 * @return trackpad_status_t Status of initialization
 */
trackpad_status_t trackpad_driver_init(const trackpad_config_t *config);

/**
 * @brief Check if trackpad driver is initialized and device is ready
 * @return bool True if trackpad is available and ready
 */
bool trackpad_is_available(void);

/**
 * @brief Update trackpad state and read new data if available
 * 
 * This function should be called regularly to maintain the Arduino-style
 * state machine and process new touch data.
 * 
 * @return trackpad_status_t Status of the update operation
 */
trackpad_status_t trackpad_update(void);

/**
 * @brief Get current trackpad data for controller integration
 * @param data Pointer to structure to receive trackpad data
 * @return trackpad_status_t Status of data retrieval
 */
trackpad_status_t trackpad_get_controller_data(trackpad_controller_data_t *data);

/**
 * @brief Get only touch coordinates (simplified interface)
 * @param scaled_x Pointer to receive scaled X coordinate (0-127)
 * @param scaled_y Pointer to receive scaled Y coordinate (0-127)
 * @param num_fingers Pointer to receive number of fingers detected
 * @return trackpad_status_t Status of coordinate reading
 */
trackpad_status_t trackpad_get_coordinates(uint8_t *scaled_x, uint8_t *scaled_y, uint8_t *num_fingers);

/**
 * @brief Get gesture detection results
 * @param gestures Pointer to structure to receive gesture flags
 * @return trackpad_status_t Status of gesture reading
 */
trackpad_status_t trackpad_get_gestures(trackpad_gestures_t *gestures);

/**
 * @brief Enter trackpad sleep mode for power saving
 * @return trackpad_status_t Status of sleep operation
 */
trackpad_status_t trackpad_enter_sleep(void);

/**
 * @brief Wake up trackpad from sleep mode
 * @return trackpad_status_t Status of wake operation
 */
trackpad_status_t trackpad_wakeup(void);

/**
 * @brief Reset trackpad device
 * @return trackpad_status_t Status of reset operation
 */
trackpad_status_t trackpad_reset(void);

/**
 * @brief Force recalibration of trackpad (re-ATI)
 * @return trackpad_status_t Status of calibration operation
 */
trackpad_status_t trackpad_recalibrate(void);

/**
 * @brief Get trackpad device information
 * @param product_num Pointer to receive product number
 * @param version_major Pointer to receive major version
 * @param version_minor Pointer to receive minor version
 * @return trackpad_status_t Status of information retrieval
 */
trackpad_status_t trackpad_get_device_info(uint16_t *product_num, uint8_t *version_major, uint8_t *version_minor);

/**
 * @brief Get trackpad driver statistics
 * @param total_updates Pointer to receive total update count
 * @param successful_reads Pointer to receive successful read count
 * @param touch_events Pointer to receive touch event count
 * @param gesture_events Pointer to receive gesture event count
 * @return trackpad_status_t Status of statistics retrieval
 */
trackpad_status_t trackpad_get_statistics(uint32_t *total_updates, uint32_t *successful_reads, 
                                         uint32_t *touch_events, uint32_t *gesture_events);

/**
 * @brief Enable or disable debug logging
 * @param enabled True to enable debug logging, false to disable
 */
void trackpad_set_debug_logging(bool enabled);

/**
 * @brief Get current trackpad state (for debugging)
 * @param main_state Pointer to receive main state
 * @param init_state Pointer to receive initialization state
 * @return trackpad_status_t Status of state retrieval
 */
trackpad_status_t trackpad_get_state(uint8_t *main_state, uint8_t *init_state);

#endif // TRACKPAD_DRIVER_H
