/*
 * Controller Storage Module - NVS-based Configuration Storage
 * 
 * This module provides persistent storage for controller configuration data
 * including calibrations, custom bindings, and user preferences.
 */

#ifndef CONTROLLER_STORAGE_H
#define CONTROLLER_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Storage keys - keep these short to save space
#define STORAGE_KEY_CALIBRATION     "cal"
#define STORAGE_KEY_BINDINGS        "bind"
#define STORAGE_KEY_PREFERENCES     "pref"
#define STORAGE_KEY_IMU_OFFSET      "imu_off"
#define STORAGE_KEY_STICK_DEADZONE  "stick_dz"
#define STORAGE_KEY_TRIGGER_CURVE   "trig_crv"

// Calibration data structure
typedef struct {
    // Analog stick calibration
    int16_t stick_center_x;
    int16_t stick_center_y;
    int16_t stick_min_x;
    int16_t stick_max_x;
    int16_t stick_min_y;
    int16_t stick_max_y;
    uint16_t stick_deadzone;
    
    // Trigger calibration
    int16_t trigger_min;
    int16_t trigger_max;
    
    // IMU calibration offsets
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    
    // Trackpad calibration
    uint16_t trackpad_sensitivity;
    
    // Calibration flags
    bool stick_calibrated;
    bool trigger_calibrated;
    bool imu_calibrated;
    bool trackpad_calibrated;
} controller_calibration_t;

// Button binding structure
typedef struct {
    uint8_t button_map[16];     // Map physical buttons to logical buttons
    uint8_t trigger_mode;       // 0=analog, 1=digital, 2=hybrid
    uint8_t stick_mode;         // 0=gamepad, 1=mouse, 2=keyboard
    uint8_t trackpad_mode;      // 0=mouse, 1=scroll, 2=gestures
    bool invert_stick_x;
    bool invert_stick_y;
    bool swap_sticks;           // For left/right handed use
} controller_bindings_t;

// User preferences
typedef struct {
    uint8_t display_brightness; // 0-255
    uint8_t led_brightness;     // 0-255
    bool sleep_enabled;
    uint16_t sleep_timeout_ms;
    uint8_t controller_id;      // 0=RIGHT, 1=LEFT
    char controller_name[16];   // Custom name
} controller_preferences_t;

// Function prototypes

/**
 * Initialize the storage subsystem
 * @return 0 on success, negative on error
 */
int controller_storage_init(void);

/**
 * Save calibration data to flash
 * @param cal Pointer to calibration data
 * @return 0 on success, negative on error
 */
int controller_storage_save_calibration(const controller_calibration_t *cal);

/**
 * Load calibration data from flash
 * @param cal Pointer to store loaded calibration data
 * @return 0 on success, negative on error (use defaults)
 */
int controller_storage_load_calibration(controller_calibration_t *cal);

/**
 * Save button bindings to flash
 * @param bindings Pointer to bindings data
 * @return 0 on success, negative on error
 */
int controller_storage_save_bindings(const controller_bindings_t *bindings);

/**
 * Load button bindings from flash
 * @param bindings Pointer to store loaded bindings data
 * @return 0 on success, negative on error (use defaults)
 */
int controller_storage_load_bindings(controller_bindings_t *bindings);

/**
 * Save user preferences to flash
 * @param prefs Pointer to preferences data
 * @return 0 on success, negative on error
 */
int controller_storage_save_preferences(const controller_preferences_t *prefs);

/**
 * Load user preferences from flash
 * @param prefs Pointer to store loaded preferences data
 * @return 0 on success, negative on error (use defaults)
 */
int controller_storage_load_preferences(controller_preferences_t *prefs);

/**
 * Reset all stored data to factory defaults
 * @return 0 on success, negative on error
 */
int controller_storage_factory_reset(void);

/**
 * Get storage statistics
 * @param used_bytes Pointer to store used bytes count
 * @param free_bytes Pointer to store free bytes count
 * @return 0 on success, negative on error
 */
int controller_storage_get_stats(size_t *used_bytes, size_t *free_bytes);

/**
 * Initialize calibration data with defaults
 * @param cal Pointer to calibration structure to initialize
 */
void controller_storage_init_default_calibration(controller_calibration_t *cal);

/**
 * Initialize bindings data with defaults
 * @param bindings Pointer to bindings structure to initialize
 */
void controller_storage_init_default_bindings(controller_bindings_t *bindings);

/**
 * Initialize preferences data with defaults
 * @param prefs Pointer to preferences structure to initialize
 */
void controller_storage_init_default_preferences(controller_preferences_t *prefs);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_STORAGE_H */
