/*
 * Controller Storage Module - NVS-based Configuration Storage
 * 
 * This module provides persistent storage for controller configuration data
 * including calibrations, custom bindings, and user preferences.
 */

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "controller_storage.h"

LOG_MODULE_REGISTER(storage, LOG_LEVEL_INF);

// Storage is ready flag
static bool storage_initialized = false;

// Current configuration in RAM (cached for performance)
static controller_calibration_t current_calibration;
static controller_bindings_t current_bindings;
static controller_preferences_t current_preferences;

// Settings subsystem handlers
static int calibration_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (len != sizeof(controller_calibration_t)) {
        LOG_WRN("Calibration data size mismatch: expected %zu, got %zu", 
                sizeof(controller_calibration_t), len);
        return -EINVAL;
    }
    
    int ret = read_cb(cb_arg, &current_calibration, sizeof(current_calibration));
    if (ret >= 0) {
        LOG_INF("Loaded calibration data from flash");
    }
    return ret;
}

static int bindings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (len != sizeof(controller_bindings_t)) {
        LOG_WRN("Bindings data size mismatch: expected %zu, got %zu", 
                sizeof(controller_bindings_t), len);
        return -EINVAL;
    }
    
    int ret = read_cb(cb_arg, &current_bindings, sizeof(current_bindings));
    if (ret >= 0) {
        LOG_INF("Loaded bindings data from flash");
    }
    return ret;
}

static int preferences_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (len != sizeof(controller_preferences_t)) {
        LOG_WRN("Preferences data size mismatch: expected %zu, got %zu", 
                sizeof(controller_preferences_t), len);
        return -EINVAL;
    }
    
    int ret = read_cb(cb_arg, &current_preferences, sizeof(current_preferences));
    if (ret >= 0) {
        LOG_INF("Loaded preferences data from flash");
    }
    return ret;
}

// Settings handler registration
SETTINGS_STATIC_HANDLER_DEFINE(controller, "controller", NULL, calibration_set, NULL, NULL);
SETTINGS_STATIC_HANDLER_DEFINE(bindings, "bindings", NULL, bindings_set, NULL, NULL);
SETTINGS_STATIC_HANDLER_DEFINE(preferences, "preferences", NULL, preferences_set, NULL, NULL);

int controller_storage_init(void)
{
    int ret;
    
    LOG_INF("Initializing controller storage...");
    
    // Initialize settings subsystem
    ret = settings_subsys_init();
    if (ret) {
        LOG_ERR("Failed to initialize settings subsystem: %d", ret);
        return ret;
    }
    
    // Initialize with defaults first
    controller_storage_init_default_calibration(&current_calibration);
    controller_storage_init_default_bindings(&current_bindings);
    controller_storage_init_default_preferences(&current_preferences);
    
    // Load settings from flash (will override defaults if found)
    ret = settings_load();
    if (ret) {
        LOG_WRN("Failed to load settings from flash: %d (using defaults)", ret);
        // Don't return error - using defaults is OK
    }
    
    storage_initialized = true;
    LOG_INF("Controller storage initialized successfully");
    
    return 0;
}

int controller_storage_save_calibration(const controller_calibration_t *cal)
{
    if (!storage_initialized) {
        LOG_ERR("Storage not initialized");
        return -ENODEV;
    }
    
    if (!cal) {
        return -EINVAL;
    }
    
    int ret = settings_save_one("controller/" STORAGE_KEY_CALIBRATION, cal, sizeof(*cal));
    if (ret == 0) {
        // Update cached copy
        memcpy(&current_calibration, cal, sizeof(current_calibration));
        LOG_INF("Calibration data saved to flash");
    } else {
        LOG_ERR("Failed to save calibration data: %d", ret);
    }
    
    return ret;
}

int controller_storage_load_calibration(controller_calibration_t *cal)
{
    if (!storage_initialized) {
        LOG_ERR("Storage not initialized");
        return -ENODEV;
    }
    
    if (!cal) {
        return -EINVAL;
    }
    
    // Return cached copy (already loaded during init)
    memcpy(cal, &current_calibration, sizeof(*cal));
    return 0;
}

int controller_storage_save_bindings(const controller_bindings_t *bindings)
{
    if (!storage_initialized) {
        LOG_ERR("Storage not initialized");
        return -ENODEV;
    }
    
    if (!bindings) {
        return -EINVAL;
    }
    
    int ret = settings_save_one("bindings/" STORAGE_KEY_BINDINGS, bindings, sizeof(*bindings));
    if (ret == 0) {
        // Update cached copy
        memcpy(&current_bindings, bindings, sizeof(current_bindings));
        LOG_INF("Bindings data saved to flash");
    } else {
        LOG_ERR("Failed to save bindings data: %d", ret);
    }
    
    return ret;
}

int controller_storage_load_bindings(controller_bindings_t *bindings)
{
    if (!storage_initialized) {
        LOG_ERR("Storage not initialized");
        return -ENODEV;
    }
    
    if (!bindings) {
        return -EINVAL;
    }
    
    // Return cached copy (already loaded during init)
    memcpy(bindings, &current_bindings, sizeof(*bindings));
    return 0;
}

int controller_storage_save_preferences(const controller_preferences_t *prefs)
{
    if (!storage_initialized) {
        LOG_ERR("Storage not initialized");
        return -ENODEV;
    }
    
    if (!prefs) {
        return -EINVAL;
    }
    
    int ret = settings_save_one("preferences/" STORAGE_KEY_PREFERENCES, prefs, sizeof(*prefs));
    if (ret == 0) {
        // Update cached copy
        memcpy(&current_preferences, prefs, sizeof(current_preferences));
        LOG_INF("Preferences data saved to flash");
    } else {
        LOG_ERR("Failed to save preferences data: %d", ret);
    }
    
    return ret;
}

int controller_storage_load_preferences(controller_preferences_t *prefs)
{
    if (!storage_initialized) {
        LOG_ERR("Storage not initialized");
        return -ENODEV;
    }
    
    if (!prefs) {
        return -EINVAL;
    }
    
    // Return cached copy (already loaded during init)
    memcpy(prefs, &current_preferences, sizeof(*prefs));
    return 0;
}

int controller_storage_factory_reset(void)
{
    if (!storage_initialized) {
        LOG_ERR("Storage not initialized");
        return -ENODEV;
    }
    
    LOG_WRN("Performing factory reset - all stored data will be lost!");
    
    int ret = 0;
    
    // Delete all settings
    ret |= settings_delete("controller/" STORAGE_KEY_CALIBRATION);
    ret |= settings_delete("bindings/" STORAGE_KEY_BINDINGS);
    ret |= settings_delete("preferences/" STORAGE_KEY_PREFERENCES);
    
    if (ret == 0) {
        // Reset cached copies to defaults
        controller_storage_init_default_calibration(&current_calibration);
        controller_storage_init_default_bindings(&current_bindings);
        controller_storage_init_default_preferences(&current_preferences);
        
        LOG_INF("Factory reset completed successfully");
    } else {
        LOG_ERR("Factory reset failed: %d", ret);
    }
    
    return ret;
}

int controller_storage_get_stats(size_t *used_bytes, size_t *free_bytes)
{
    // Note: Zephyr settings doesn't provide direct storage stats
    // This is a placeholder implementation
    if (used_bytes) {
        *used_bytes = sizeof(controller_calibration_t) + 
                     sizeof(controller_bindings_t) + 
                     sizeof(controller_preferences_t);
    }
    
    if (free_bytes) {
        *free_bytes = 4096; // Estimate - actual depends on flash partition
    }
    
    return 0;
}

void controller_storage_init_default_calibration(controller_calibration_t *cal)
{
    if (!cal) return;
    
    memset(cal, 0, sizeof(*cal));
    
    // Default analog stick calibration (assuming 12-bit ADC, center at 2048)
    cal->stick_center_x = 2048;
    cal->stick_center_y = 2048;
    cal->stick_min_x = 0;
    cal->stick_max_x = 4095;
    cal->stick_min_y = 0;
    cal->stick_max_y = 4095;
    cal->stick_deadzone = 200;  // ~5% deadzone
    
    // Default trigger calibration
    cal->trigger_min = 0;
    cal->trigger_max = 4095;
    
    // IMU offsets (start at zero - will be calibrated)
    cal->accel_offset_x = 0;
    cal->accel_offset_y = 0;
    cal->accel_offset_z = 0;
    cal->gyro_offset_x = 0;
    cal->gyro_offset_y = 0;
    cal->gyro_offset_z = 0;
    
    // Trackpad sensitivity
    cal->trackpad_sensitivity = 100; // Default sensitivity
    
    // All calibrations start as uncalibrated
    cal->stick_calibrated = false;
    cal->trigger_calibrated = false;
    cal->imu_calibrated = false;
    cal->trackpad_calibrated = false;
    
    LOG_DBG("Initialized default calibration");
}

void controller_storage_init_default_bindings(controller_bindings_t *bindings)
{
    if (!bindings) return;
    
    memset(bindings, 0, sizeof(*bindings));
    
    // Default button mapping (1:1 mapping)
    for (int i = 0; i < 16; i++) {
        bindings->button_map[i] = i;
    }
    
    // Default modes
    bindings->trigger_mode = 0;     // Analog
    bindings->stick_mode = 0;       // Gamepad
    bindings->trackpad_mode = 0;    // Mouse
    
    // Default orientations
    bindings->invert_stick_x = false;
    bindings->invert_stick_y = false;
    bindings->swap_sticks = false;
    
    LOG_DBG("Initialized default bindings");
}

void controller_storage_init_default_preferences(controller_preferences_t *prefs)
{
    if (!prefs) return;
    
    memset(prefs, 0, sizeof(*prefs));
    
    // Default display and LED settings
    prefs->display_brightness = 128;  // 50% brightness
    prefs->led_brightness = 64;       // 25% brightness
    
    // Default power settings
    prefs->sleep_enabled = true;
    prefs->sleep_timeout_ms = 300000; // 5 minutes
    
    // Default controller ID (will be overridden by compile-time setting)
    prefs->controller_id = 0;  // RIGHT
    
    // Default name
    strcpy(prefs->controller_name, "Controller");
    
    LOG_DBG("Initialized default preferences");
}
