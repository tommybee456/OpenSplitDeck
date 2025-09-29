/*
 * Power Management Driver - System Power and Sleep Management
 * 
 * This driver provides comprehensive power management functionality
 * including sleep modes, peripheral shutdown/wakeup, button combinations
 * for shutdown and factory reset, and system state persistence.
 */

#ifndef POWER_MGMT_DRIVER_H
#define POWER_MGMT_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

// Power management status codes
typedef enum {
    POWER_MGMT_STATUS_OK = 0,
    POWER_MGMT_STATUS_ERROR = -1,
    POWER_MGMT_STATUS_NOT_INITIALIZED = -2,
    POWER_MGMT_STATUS_INVALID_CONFIG = -3,
    POWER_MGMT_STATUS_GPIO_ERROR = -4,
    POWER_MGMT_STATUS_ALREADY_SLEEPING = -5
} power_mgmt_status_t;

// Power management states
typedef enum {
    POWER_STATE_ACTIVE,
    POWER_STATE_ENTERING_SLEEP,
    POWER_STATE_SLEEPING,
    POWER_STATE_WAKING_UP
} power_state_t;

// Button combo types
typedef enum {
    BUTTON_COMBO_SHUTDOWN,      // Normal shutdown combo
    BUTTON_COMBO_FACTORY_RESET  // Factory reset combo
} button_combo_type_t;

// Power management configuration
typedef struct {
    // Wake-up buttons (required)
    const struct gpio_dt_spec *wake_button1;
    const struct gpio_dt_spec *wake_button2;
    
    // Status LED (optional)
    const struct gpio_dt_spec *status_led;
    
    // Button combo timing
    uint32_t shutdown_combo_hold_ms;    // Default: 3000ms
    uint32_t factory_reset_combo_hold_ms; // Default: 5000ms
    
    // Sleep behavior
    bool auto_sleep_enabled;
    uint32_t auto_sleep_timeout_ms;     // Default: 300000ms (5 minutes)
    
    // Storage integration
    bool save_state_on_sleep;           // Save system state before sleep
    bool restore_state_on_wake;         // Restore system state after wake
} power_mgmt_config_t;

// Power management statistics
typedef struct {
    uint32_t sleep_count;
    uint32_t wake_count;
    uint32_t total_sleep_time_ms;
    uint32_t last_sleep_duration_ms;
    uint32_t factory_reset_count;
    uint32_t shutdown_combo_activations;
    bool last_wake_was_button;
} power_mgmt_stats_t;

// Callback function types
typedef void (*power_save_callback_t)(void);
typedef void (*power_restore_callback_t)(void);
typedef void (*power_peripheral_shutdown_callback_t)(void);
typedef void (*power_peripheral_wakeup_callback_t)(void);
typedef void (*power_factory_reset_callback_t)(void);

// Function prototypes

/**
 * Initialize power management driver
 * @param config Pointer to configuration structure
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_driver_init(const power_mgmt_config_t *config);

/**
 * Check if power management driver is initialized
 * @return true if initialized, false otherwise
 */
bool power_mgmt_driver_is_initialized(void);

/**
 * Get current power state
 * @return Current power state
 */
power_state_t power_mgmt_get_state(void);

/**
 * Check if system is currently sleeping
 * @return true if sleeping, false otherwise
 */
bool power_mgmt_is_sleeping(void);

/**
 * Enter sleep mode manually
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_enter_sleep(void);

/**
 * Wake up from sleep mode manually
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_wake_up(void);

/**
 * Check for button combinations (non-blocking)
 * @param combo_type Type of combo to check
 * @return true if combo is active and held long enough, false otherwise
 */
bool power_mgmt_check_button_combo(button_combo_type_t combo_type);

/**
 * Get power management statistics
 * @param stats Pointer to store statistics
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_get_stats(power_mgmt_stats_t *stats);

/**
 * Reset power management statistics
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_reset_stats(void);

/**
 * Set auto-sleep configuration
 * @param enabled true to enable auto-sleep, false to disable
 * @param timeout_ms timeout in milliseconds before auto-sleep
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_set_auto_sleep(bool enabled, uint32_t timeout_ms);

/**
 * Reset auto-sleep timer (call this on user activity)
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_reset_auto_sleep_timer(void);

// ============================================================================
// Callback Registration Functions
// ============================================================================

/**
 * Register callback for saving system state before sleep
 * @param callback Function to call for saving state
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_register_save_callback(power_save_callback_t callback);

/**
 * Register callback for restoring system state after wake
 * @param callback Function to call for restoring state
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_register_restore_callback(power_restore_callback_t callback);

/**
 * Register callback for shutting down peripherals before sleep
 * @param callback Function to call for peripheral shutdown
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_register_shutdown_callback(power_peripheral_shutdown_callback_t callback);

/**
 * Register callback for waking up peripherals after sleep
 * @param callback Function to call for peripheral wakeup
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_register_wakeup_callback(power_peripheral_wakeup_callback_t callback);

/**
 * Register callback for factory reset trigger
 * @param callback Function to call when factory reset is triggered
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_register_factory_reset_callback(power_factory_reset_callback_t callback);

// ============================================================================
// Advanced Features
// ============================================================================

/**
 * Force immediate peripheral shutdown (emergency)
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_emergency_shutdown(void);

/**
 * Validate system state after wake-up
 * @return POWER_MGMT_STATUS_OK if state is valid, error code otherwise
 */
power_mgmt_status_t power_mgmt_validate_state(void);

/**
 * Get time since last user activity
 * @return time in milliseconds since last activity
 */
uint32_t power_mgmt_get_idle_time(void);

/**
 * Set button combo hold times
 * @param shutdown_ms Hold time for shutdown combo
 * @param factory_reset_ms Hold time for factory reset combo
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_set_combo_timings(uint32_t shutdown_ms, uint32_t factory_reset_ms);

/**
 * Enable or disable specific button combos
 * @param combo_type Type of combo to configure
 * @param enabled true to enable, false to disable
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_enable_combo(button_combo_type_t combo_type, bool enabled);

/**
 * Process power management tasks (call from main loop)
 * This handles auto-sleep timing and other background tasks
 * @return POWER_MGMT_STATUS_OK on success, error code on failure
 */
power_mgmt_status_t power_mgmt_process(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_MGMT_DRIVER_H
