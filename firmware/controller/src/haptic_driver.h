/**
 * @file haptic_driver.h
 * @brief Haptic motor driver library for DRV2605
 * 
 * This library provides a clean interface for controlling the DRV2605 haptic motor driver
 * with external trigger mode and various feedback patterns.
 */

#ifndef HAPTIC_DRIVER_H
#define HAPTIC_DRIVER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>
#include <stdint.h>
#include "drv2605.h"

/**
 * @brief Haptic driver status enumeration
 */
typedef enum {
    HAPTIC_STATUS_NOT_READY = 0,
    HAPTIC_STATUS_READY,
    HAPTIC_STATUS_ERROR,
    HAPTIC_STATUS_NOT_DETECTED
} haptic_status_t;

/**
 * @brief Haptic feedback patterns
 */
typedef enum {
    HAPTIC_PATTERN_BUTTON_PRESS = 0,
    HAPTIC_PATTERN_STARTUP,
    HAPTIC_PATTERN_NOTIFICATION_LIGHT,
    HAPTIC_PATTERN_NOTIFICATION_MEDIUM,
    HAPTIC_PATTERN_NOTIFICATION_STRONG,
    HAPTIC_PATTERN_ERROR,
    HAPTIC_PATTERN_SUCCESS,
    HAPTIC_PATTERN_CUSTOM
} haptic_pattern_t;

/**
 * @brief Haptic driver context structure
 */
typedef struct {
    const struct device *i2c_dev;
    const struct gpio_dt_spec *trigger_pin;
    const struct gpio_dt_spec *enable_pin;
    haptic_status_t status;
    bool external_trigger_mode;
    uint8_t current_effect;
} haptic_context_t;

/**
 * @brief Initialize the haptic driver system
 * 
 * @param i2c_dev I2C device for communication
 * @param trigger_pin GPIO pin for external trigger
 * @param enable_pin GPIO pin for enable control
 * @return 0 on success, negative error code on failure
 */
int haptic_driver_init(const struct device *i2c_dev, 
                      const struct gpio_dt_spec *trigger_pin,
                      const struct gpio_dt_spec *enable_pin);

/**
 * @brief Get the current haptic driver status
 * 
 * @return Current haptic driver status
 */
haptic_status_t haptic_get_status(void);

/**
 * @brief Check if haptic driver is available
 * 
 * @return true if haptic is available, false otherwise
 */
bool haptic_is_available(void);

/**
 * @brief Setup external trigger mode
 * 
 * @param default_effect Default effect to use for external triggers
 * @return 0 on success, negative error code on failure
 */
int haptic_setup_external_trigger(drv2605_effect_t default_effect);

/**
 * @brief Send a pulse via external trigger
 * 
 * This triggers the currently loaded waveform via GPIO pulse
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_trigger_pulse(void);

/**
 * @brief Play a predefined haptic pattern
 * 
 * @param pattern Pattern to play
 * @return 0 on success, negative error code on failure
 */
int haptic_play_pattern(haptic_pattern_t pattern);

/**
 * @brief Play a specific DRV2605 effect via I2C
 * 
 * @param effect DRV2605 effect to play
 * @return 0 on success, negative error code on failure
 */
int haptic_play_effect(drv2605_effect_t effect);

/**
 * @brief Change the external trigger effect
 * 
 * @param new_effect New effect to load for external triggers
 * @return 0 on success, negative error code on failure
 */
int haptic_set_trigger_effect(drv2605_effect_t new_effect);

/**
 * @brief Enter standby mode (low power)
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_enter_standby(void);

/**
 * @brief Wake up from standby mode
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_wakeup(void);

/**
 * @brief Disable the haptic driver
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_disable(void);

/**
 * @brief Enable the haptic driver
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_enable(void);

/**
 * @brief Get debug information about the haptic driver
 * 
 * @param buffer Buffer to write debug info to
 * @param buffer_size Size of the buffer
 * @return Number of characters written
 */
int haptic_get_debug_info(char *buffer, size_t buffer_size);

/**
 * @brief Test the haptic driver with a simple pattern
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_test_driver(void);

/**
 * @brief Perform ERM motor auto-calibration
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_perform_erm_calibration(void);

/**
 * @brief Perform LRA motor auto-calibration
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_perform_lra_calibration(void);

/**
 * @brief Test stronger haptic effects for comparison
 * 
 * @return 0 on success, negative error code on failure
 */
int haptic_test_strong_effects(void);

#endif /* HAPTIC_DRIVER_H */
