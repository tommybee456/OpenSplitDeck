/**
 * @file trackpad_driver.c
 * @brief IQS7211E Trackpad Driver Implementation
 * 
 * This driver implements a clean interface for the IQS7211E trackpad sensor,
 * managing the complex Arduino-style state machine internally.
 */

#include "trackpad_driver.h"
#include "IQS7211E.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trackpad_driver, LOG_LEVEL_INF);

// Driver state and configuration
static bool driver_initialized = false;
static trackpad_config_t driver_config;
static iqs7211e_instance_t trackpad_device;
static struct gpio_callback trackpad_ready_cb_data;

// Driver statistics
static uint32_t total_updates = 0;
static uint32_t successful_reads = 0;
static uint32_t touch_events = 0;
static uint32_t gesture_events = 0;
static bool debug_logging = false;

// Forward declarations
static void trackpad_ready_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static trackpad_status_t trackpad_hardware_init(void);
static void trackpad_scale_coordinates(uint16_t raw_x, uint16_t raw_y, uint8_t *scaled_x, uint8_t *scaled_y);
static bool trackpad_validate_coordinates(uint16_t x, uint16_t y);

/**
 * @brief GPIO interrupt callback for trackpad ready pin
 */
static void trackpad_ready_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Call Arduino-exact interrupt handler
    iqs7211e_ready_interrupt();
    
    if (debug_logging) {
        LOG_DBG("Trackpad ready interrupt triggered");
    }
}

/**
 * @brief Initialize trackpad hardware (GPIO, I2C scanning, etc.)
 */
static trackpad_status_t trackpad_hardware_init(void)
{
    int ret;
    
    LOG_INF("Initializing trackpad hardware...");
    
    // Validate I2C device
    if (!device_is_ready(driver_config.i2c_dev)) {
        LOG_ERR("I2C device not ready for trackpad");
        return TRACKPAD_STATUS_ERROR;
    }
    
    // Hardware stabilization delay
    k_sleep(K_MSEC(50));
    
    // Configure ready pin
    ret = gpio_pin_configure_dt(driver_config.ready_pin, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure trackpad ready pin: %d", ret);
        return TRACKPAD_STATUS_ERROR;
    }
    
    // Configure RDY pin as input for polling (no interrupt for better ESB timing)
    // We still read the pin status synchronously during operation
    LOG_INF("RDY pin configured for polling - no interrupt for better performance");
    
    // Scan I2C bus to verify device presence
    bool device_found = false;
    uint8_t dummy_data;
    
    if (debug_logging) {
        LOG_INF("Scanning I2C bus for trackpad...");
        for (uint8_t addr = 0x50; addr <= 0x60; addr++) {
            int scan_ret = i2c_read(driver_config.i2c_dev, &dummy_data, 1, addr);
            if (scan_ret == 0) {
                LOG_INF("I2C device found at 0x%02X", addr);
                if (addr == driver_config.device_address) {
                    device_found = true;
                }
            }
        }
    } else {
        // Quick check for target device only
        int scan_ret = i2c_read(driver_config.i2c_dev, &dummy_data, 1, driver_config.device_address);
        device_found = (scan_ret == 0);
    }
    
    if (!device_found) {
        LOG_WRN("Trackpad device not found at I2C address 0x%02X", driver_config.device_address);
        return TRACKPAD_STATUS_DEVICE_NOT_FOUND;
    }
    
    LOG_INF("Trackpad hardware initialization complete");
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Scale raw coordinates to controller range
 */
static void trackpad_scale_coordinates(uint16_t raw_x, uint16_t raw_y, uint8_t *scaled_x, uint8_t *scaled_y)
{
    // Scale coordinates to controller range (0-controller_scale_max)
    uint32_t scaled_x_temp = ((uint32_t)raw_x * driver_config.controller_scale_max) / driver_config.coordinate_max_x;
    uint32_t scaled_y_temp = ((uint32_t)raw_y * driver_config.controller_scale_max) / driver_config.coordinate_max_y;
    
    // Clamp to valid range
    *scaled_x = (scaled_x_temp > driver_config.controller_scale_max) ? driver_config.controller_scale_max : (uint8_t)scaled_x_temp;
    *scaled_y = (scaled_y_temp > driver_config.controller_scale_max) ? driver_config.controller_scale_max : (uint8_t)scaled_y_temp;
}

/**
 * @brief Validate that coordinates are reasonable (not error values like 65535)
 */
static bool trackpad_validate_coordinates(uint16_t x, uint16_t y)
{
    // Reject clearly invalid values (65535, etc.)
    return (x < 32768 && y < 32768 && x > 0 && y > 0);
}

/**
 * @brief Initialize the trackpad driver
 */
trackpad_status_t trackpad_driver_init(const trackpad_config_t *config)
{
    if (!config) {
        LOG_ERR("Invalid trackpad configuration");
        return TRACKPAD_STATUS_INVALID_PARAM;
    }
    
    if (driver_initialized) {
        LOG_WRN("Trackpad driver already initialized");
        return TRACKPAD_STATUS_OK;
    }
    
    // Copy configuration
    driver_config = *config;
    debug_logging = config->debug_logging;
    
    LOG_INF("Initializing IQS7211E trackpad driver...");
    LOG_INF("  I2C Address: 0x%02X", config->device_address);
    LOG_INF("  Ready Pin: P%d.%02d", config->ready_pin->port == DEVICE_DT_GET(DT_NODELABEL(gpio0)) ? 0 : 1, 
            config->ready_pin->pin);
    LOG_INF("  Coordinate Range: %dx%d -> 0-%d", config->coordinate_max_x, config->coordinate_max_y, 
            config->controller_scale_max);
    
    // Initialize hardware
    trackpad_status_t status = trackpad_hardware_init();
    if (status != TRACKPAD_STATUS_OK) {
        LOG_ERR("Trackpad hardware initialization failed");
        return status;
    }
    
    // Initialize trackpad device structure
    trackpad_device.i2c_dev = driver_config.i2c_dev;
    
    // Arduino-style begin() initialization
    iqs7211e_begin(&trackpad_device, driver_config.device_address, driver_config.ready_pin->pin);
    
    // Initialize Arduino state machine
    bool init_success = false;
    int attempts = 0;
    const int max_attempts = 10;
    
    while (!init_success && attempts < max_attempts) {
        attempts++;
        
        // Call Arduino init function
        bool init_result = iqs7211e_init(&trackpad_device);
        
        // Run state machine multiple times to advance through initialization
        for (int i = 0; i < 10; i++) {
            iqs7211e_run(&trackpad_device);
            k_sleep(K_MSEC(100));  // Give time for ATI and other operations
            
            if (trackpad_device.new_data_available) {
                init_success = true;
                break;
            }
        }
        
        if (!init_success) {
            LOG_WRN("Trackpad initialization attempt %d failed, retrying...", attempts);
            k_sleep(K_MSEC(200));
        }
    }
    
    if (!init_success) {
        LOG_ERR("Trackpad initialization failed after %d attempts", max_attempts);
        return TRACKPAD_STATUS_TIMEOUT;
    }
    
    // Get device information
    uint16_t product_num = iqs7211e_getProductNum(&trackpad_device, false);
    uint8_t ver_major = iqs7211e_getmajorVersion(&trackpad_device, false);
    uint8_t ver_minor = iqs7211e_getminorVersion(&trackpad_device, false);
    
    LOG_INF("Trackpad initialized successfully!");
    LOG_INF("  Product: 0x%04X v%d.%d", product_num, ver_major, ver_minor);
    LOG_INF("  Initialization attempts: %d", attempts);
    
    // Reset statistics
    total_updates = 0;
    successful_reads = 0;
    touch_events = 0;
    gesture_events = 0;
    
    driver_initialized = true;
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Check if trackpad driver is initialized and device is ready
 */
bool trackpad_is_available(void)
{
    return driver_initialized && trackpad_device.new_data_available;
}

/**
 * @brief Update trackpad state and read new data if available
 */
trackpad_status_t trackpad_update(void)
{
    if (!driver_initialized) {
        return TRACKPAD_STATUS_NOT_INITIALIZED;
    }
    
    total_updates++;
    
    // Run Arduino state machine
    iqs7211e_run(&trackpad_device);
    
    // Debug logging every 250 updates
    if (debug_logging && (total_updates % 250 == 0)) {
        LOG_DBG("Trackpad state: main=%d, init=%d, new_data=%s, RDY=%s", 
                trackpad_device.iqs7211e_state.state,
                trackpad_device.iqs7211e_state.init_state,
                trackpad_device.new_data_available ? "YES" : "NO",
                iqs7211e_getRDYStatus(&trackpad_device) ? "YES" : "NO");
    }
    
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Get current trackpad data for controller integration
 */
trackpad_status_t trackpad_get_controller_data(trackpad_controller_data_t *data)
{
    if (!driver_initialized) {
        return TRACKPAD_STATUS_NOT_INITIALIZED;
    }
    
    if (!data) {
        return TRACKPAD_STATUS_INVALID_PARAM;
    }
    
    // Initialize data structure
    memset(data, 0, sizeof(trackpad_controller_data_t));
    data->device_ready = trackpad_device.new_data_available;
    data->new_data_available = trackpad_device.new_data_available;
    
    // Only read data if trackpad is ready and has new data
    if (trackpad_device.new_data_available && 
        trackpad_device.iqs7211e_state.state == IQS7211E_STATE_RUN) {
        
        successful_reads++;
        
        // Get number of fingers
        data->touch.num_fingers = iqs7211e_getFingerNumber(&trackpad_device);
        
        if (data->touch.num_fingers > 0) {
            // Get raw coordinates
            data->touch.finger1_x = iqs7211e_getXCoordinate(&trackpad_device, FINGER_1);
            data->touch.finger1_y = iqs7211e_getYCoordinate(&trackpad_device, FINGER_1);
            
            if (data->touch.num_fingers > 1) {
                data->touch.finger2_x = iqs7211e_getXCoordinate(&trackpad_device, FINGER_2);
                data->touch.finger2_y = iqs7211e_getYCoordinate(&trackpad_device, FINGER_2);
            }
            
            // Validate coordinates
            if (trackpad_validate_coordinates(data->touch.finger1_x, data->touch.finger1_y)) {
                // Scale coordinates for controller
                trackpad_scale_coordinates(data->touch.finger1_x, data->touch.finger1_y,
                                         &data->touch.scaled_x, &data->touch.scaled_y);
                data->touch.valid_touch = true;
                touch_events++;
                
                if (debug_logging) {
                    LOG_INF("TOUCH: X=%d, Y=%d, fingers=%d, scaled=(%d,%d)", 
                            data->touch.finger1_x, data->touch.finger1_y, 
                            data->touch.num_fingers,
                            data->touch.scaled_x, data->touch.scaled_y);
                }
            } else {
                if (debug_logging) {
                    LOG_WRN("Invalid coordinates: X=%d, Y=%d", data->touch.finger1_x, data->touch.finger1_y);
                }
                data->touch.valid_touch = false;
            }
        }
        
        // Read gestures
        iqs7211e_getGestures(&trackpad_device, false);
        data->gestures.single_tap = iqs7211e_isSingleTap(&trackpad_device);
        data->gestures.double_tap = iqs7211e_isDoubleTap(&trackpad_device);
        data->gestures.two_finger_tap = iqs7211e_isTwoFingerTap(&trackpad_device);
        data->gestures.press_and_hold = iqs7211e_isPressAndHold(&trackpad_device);
        data->gestures.palm_gesture = iqs7211e_isPalmGesture(&trackpad_device);
        data->gestures.swipe_up = iqs7211e_isSwipeUp(&trackpad_device);
        data->gestures.swipe_down = iqs7211e_isSwipeDown(&trackpad_device);
        data->gestures.swipe_left = iqs7211e_isSwipeLeft(&trackpad_device);
        data->gestures.swipe_right = iqs7211e_isSwipeRight(&trackpad_device);
        
        // Count gesture events
        if (data->gestures.single_tap || data->gestures.double_tap || 
            data->gestures.two_finger_tap || data->gestures.press_and_hold ||
            data->gestures.palm_gesture || data->gestures.swipe_up ||
            data->gestures.swipe_down || data->gestures.swipe_left ||
            data->gestures.swipe_right) {
            gesture_events++;
        }
        
        // Clear the new data flag after processing (Arduino pattern)
        trackpad_device.new_data_available = false;
    }
    
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Get only touch coordinates (simplified interface)
 */
trackpad_status_t trackpad_get_coordinates(uint8_t *scaled_x, uint8_t *scaled_y, uint8_t *num_fingers)
{
    trackpad_controller_data_t data;
    trackpad_status_t status = trackpad_get_controller_data(&data);
    
    if (status == TRACKPAD_STATUS_OK) {
        *scaled_x = data.touch.valid_touch ? data.touch.scaled_x : 0;
        *scaled_y = data.touch.valid_touch ? data.touch.scaled_y : 0;
        *num_fingers = data.touch.num_fingers;
    } else {
        *scaled_x = 0;
        *scaled_y = 0;
        *num_fingers = 0;
    }
    
    return status;
}

/**
 * @brief Get gesture detection results
 */
trackpad_status_t trackpad_get_gestures(trackpad_gestures_t *gestures)
{
    trackpad_controller_data_t data;
    trackpad_status_t status = trackpad_get_controller_data(&data);
    
    if (status == TRACKPAD_STATUS_OK && gestures) {
        *gestures = data.gestures;
    }
    
    return status;
}

/**
 * @brief Enter trackpad sleep mode for power saving
 */
trackpad_status_t trackpad_enter_sleep(void)
{
    if (!driver_initialized) {
        return TRACKPAD_STATUS_NOT_INITIALIZED;
    }
    
    LOG_INF("Trackpad entering sleep mode");
    
    // TODO: Implement IQS7211E sleep mode
    // This would typically involve writing to specific registers to put
    // the device in low power mode
    
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Wake up trackpad from sleep mode
 */
trackpad_status_t trackpad_wakeup(void)
{
    if (!driver_initialized) {
        return TRACKPAD_STATUS_NOT_INITIALIZED;
    }
    
    LOG_INF("Trackpad waking up from sleep");
    
    // TODO: Implement IQS7211E wake-up sequence
    // This would typically involve re-initializing the device
    
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Reset trackpad device
 */
trackpad_status_t trackpad_reset(void)
{
    if (!driver_initialized) {
        return TRACKPAD_STATUS_NOT_INITIALIZED;
    }
    
    LOG_INF("Resetting trackpad device");
    
    // Software reset
    iqs7211e_SW_Reset(&trackpad_device, true);
    
    // Reset state machine
    trackpad_device.iqs7211e_state.state = IQS7211E_STATE_START;
    trackpad_device.iqs7211e_state.init_state = IQS7211E_INIT_VERIFY_PRODUCT;
    
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Force recalibration of trackpad (re-ATI)
 */
trackpad_status_t trackpad_recalibrate(void)
{
    if (!driver_initialized) {
        return TRACKPAD_STATUS_NOT_INITIALIZED;
    }
    
    LOG_INF("Recalibrating trackpad (re-ATI)");
    
    // Force ATI recalibration
    iqs7211e_ReATI(&trackpad_device, false);
    
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Get trackpad device information
 */
trackpad_status_t trackpad_get_device_info(uint16_t *product_num, uint8_t *version_major, uint8_t *version_minor)
{
    if (!driver_initialized) {
        return TRACKPAD_STATUS_NOT_INITIALIZED;
    }
    
    if (!product_num || !version_major || !version_minor) {
        return TRACKPAD_STATUS_INVALID_PARAM;
    }
    
    *product_num = iqs7211e_getProductNum(&trackpad_device, false);
    *version_major = iqs7211e_getmajorVersion(&trackpad_device, false);
    *version_minor = iqs7211e_getminorVersion(&trackpad_device, false);
    
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Get trackpad driver statistics
 */
trackpad_status_t trackpad_get_statistics(uint32_t *total_updates_out, uint32_t *successful_reads_out, 
                                         uint32_t *touch_events_out, uint32_t *gesture_events_out)
{
    if (!total_updates_out || !successful_reads_out || !touch_events_out || !gesture_events_out) {
        return TRACKPAD_STATUS_INVALID_PARAM;
    }
    
    *total_updates_out = total_updates;
    *successful_reads_out = successful_reads;
    *touch_events_out = touch_events;
    *gesture_events_out = gesture_events;
    
    return TRACKPAD_STATUS_OK;
}

/**
 * @brief Enable or disable debug logging
 */
void trackpad_set_debug_logging(bool enabled)
{
    debug_logging = enabled;
    LOG_INF("Trackpad debug logging %s", enabled ? "enabled" : "disabled");
}

/**
 * @brief Get current trackpad state (for debugging)
 */
trackpad_status_t trackpad_get_state(uint8_t *main_state, uint8_t *init_state)
{
    if (!driver_initialized) {
        return TRACKPAD_STATUS_NOT_INITIALIZED;
    }
    
    if (!main_state || !init_state) {
        return TRACKPAD_STATUS_INVALID_PARAM;
    }
    
    *main_state = trackpad_device.iqs7211e_state.state;
    *init_state = trackpad_device.iqs7211e_state.init_state;
    
    return TRACKPAD_STATUS_OK;
}
