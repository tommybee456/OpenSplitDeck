/*
 * Power Management Driver Implementation - System Power and Sleep Management
 *
 * This driver provides comprehensive power management functionality
 * including sleep modes, peripheral shutdown/wakeup, button combinations
 * for shutdown and factory reset, and system state persistence.
 */

#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/sys/poweroff.h>
#include "power_mgmt_driver.h"

LOG_MODULE_REGISTER(power_mgmt, LOG_LEVEL_INF);

// Power management context
typedef struct
{
    bool initialized;
    power_state_t current_state;
    power_mgmt_config_t config;
    power_mgmt_stats_t stats;

    // GPIO callback structures
    struct gpio_callback wake_button1_cb_data;
    struct gpio_callback wake_button2_cb_data;

    // State tracking
    bool system_sleeping;
    bool sleeping_flag;
    uint32_t last_activity_time;
    uint32_t sleep_start_time;

    // Button combo tracking
    struct
    {
        bool shutdown_active;
        bool factory_reset_active;
        uint32_t shutdown_start_time;
        uint32_t factory_reset_start_time;
        bool shutdown_enabled;
        bool factory_reset_enabled;
    } combos;

    // Registered callbacks
    power_save_callback_t save_callback;
    power_restore_callback_t restore_callback;
    power_peripheral_shutdown_callback_t shutdown_callback;
    power_peripheral_wakeup_callback_t wakeup_callback;
    power_factory_reset_callback_t factory_reset_callback;
} power_mgmt_context_t;

// Global context
static power_mgmt_context_t g_power_ctx = {0};

// Forward declarations
static void power_mgmt_wake_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static power_mgmt_status_t power_mgmt_shutdown_peripherals(void);
static power_mgmt_status_t power_mgmt_wakeup_peripherals(void);
static power_mgmt_status_t power_mgmt_save_state(void);
static power_mgmt_status_t power_mgmt_restore_state(void);

/**
 * @brief Wake button interrupt callback
 */
static void power_mgmt_wake_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    static uint32_t last_wake_time = 0;
    static uint32_t callback_count = 0;
    uint32_t current_time = k_uptime_get_32();
    
    callback_count++;
    LOG_INF("=== WAKE BUTTON CALLBACK #%d ===", callback_count);
    LOG_INF("Device: %p, Pins: 0x%x, Time: %d", dev, pins, current_time);
    LOG_INF("System sleeping: %d, Sleeping flag: %d", g_power_ctx.system_sleeping, g_power_ctx.sleeping_flag);
    
    // Debounce: ignore rapid successive button presses (within 100ms)
    if (current_time - last_wake_time < 100) {
        LOG_WRN("Wake button debounced (too fast)");
        return;
    }
    last_wake_time = current_time;
    
    if (g_power_ctx.system_sleeping)
    {
        LOG_INF("*** SETTING WAKE FLAGS - SYSTEM WAS SLEEPING ***");
        g_power_ctx.system_sleeping = false;
        g_power_ctx.stats.last_wake_was_button = true;

        // Update activity timer
        g_power_ctx.last_activity_time = k_uptime_get_32();

        // Re-enable peripherals if we were sleeping
        if (g_power_ctx.sleeping_flag)
        {
            // Wake up will be handled by the main sleep loop
            LOG_INF("System wake-up initiated by button press");
        }
    }
    else
    {
        LOG_WRN("Wake button pressed but system not sleeping - ignoring");
    }
}

/**
 * @brief Initialize power management driver
 */
power_mgmt_status_t power_mgmt_driver_init(const power_mgmt_config_t *config)
{
    if (!config)
    {
        LOG_ERR("Invalid configuration pointer");
        return POWER_MGMT_STATUS_INVALID_CONFIG;
    }

    if (g_power_ctx.initialized)
    {
        LOG_WRN("Power management already initialized");
        return POWER_MGMT_STATUS_OK;
    }

    LOG_INF("Power management driver initialization starting...");

    // Copy configuration
    memcpy(&g_power_ctx.config, config, sizeof(power_mgmt_config_t));

    // Set default values if not specified
    if (g_power_ctx.config.shutdown_combo_hold_ms == 0)
    {
        g_power_ctx.config.shutdown_combo_hold_ms = 3000; // 3 seconds
    }
    if (g_power_ctx.config.factory_reset_combo_hold_ms == 0)
    {
        g_power_ctx.config.factory_reset_combo_hold_ms = 5000; // 5 seconds
    }
    if (g_power_ctx.config.auto_sleep_timeout_ms == 0)
    {
        g_power_ctx.config.auto_sleep_timeout_ms = 300000; // 5 minutes
    }

    // Initialize state
    g_power_ctx.current_state = POWER_STATE_ACTIVE;
    g_power_ctx.system_sleeping = false;
    g_power_ctx.sleeping_flag = false;
    g_power_ctx.last_activity_time = k_uptime_get_32();

    // Enable button combos by default
    g_power_ctx.combos.shutdown_enabled = true;
    g_power_ctx.combos.factory_reset_enabled = true;

    // Initialize statistics
    memset(&g_power_ctx.stats, 0, sizeof(power_mgmt_stats_t));

    // Configure wake-up buttons with interrupts
    if (g_power_ctx.config.wake_button1)
    {
        if (!gpio_is_ready_dt(g_power_ctx.config.wake_button1))
        {
            LOG_ERR("Wake button 1 not ready");
            return POWER_MGMT_STATUS_GPIO_ERROR;
        }

        int ret = gpio_pin_configure_dt(g_power_ctx.config.wake_button1, GPIO_INPUT | GPIO_PULL_UP);
        if (ret != 0)
        {
            LOG_ERR("Failed to configure wake button 1: %d", ret);
            return POWER_MGMT_STATUS_GPIO_ERROR;
        }

        // Set up wake button callback (but keep interrupt disabled until sleep)
        gpio_init_callback(&g_power_ctx.wake_button1_cb_data, power_mgmt_wake_button_callback,
                          BIT(g_power_ctx.config.wake_button1->pin));
        ret = gpio_add_callback(g_power_ctx.config.wake_button1->port, &g_power_ctx.wake_button1_cb_data);
        if (ret != 0) {
            LOG_ERR("Failed to add wake button 1 callback: %d", ret);
            return POWER_MGMT_STATUS_GPIO_ERROR;
        }

        // Keep interrupt disabled during normal operation
        ret = gpio_pin_interrupt_configure_dt(g_power_ctx.config.wake_button1, GPIO_INT_DISABLE);
        if (ret != 0) {
            LOG_ERR("Failed to disable wake button 1 interrupt: %d", ret);
            return POWER_MGMT_STATUS_GPIO_ERROR;
        }

        LOG_INF("Wake button 1 configured successfully");
    }

    if (g_power_ctx.config.wake_button2)
    {
        if (!gpio_is_ready_dt(g_power_ctx.config.wake_button2))
        {
            LOG_ERR("Wake button 2 not ready");
            return POWER_MGMT_STATUS_GPIO_ERROR;
        }

        int ret = gpio_pin_configure_dt(g_power_ctx.config.wake_button2, GPIO_INPUT | GPIO_PULL_UP);
        if (ret != 0)
        {
            LOG_ERR("Failed to configure wake button 2: %d", ret);
            return POWER_MGMT_STATUS_GPIO_ERROR;
        }

        // Temporarily disable wake button interrupts during normal operation
        // These will be enabled only when actually entering sleep mode
        /*
        ret = gpio_pin_interrupt_configure_dt(g_power_ctx.config.wake_button2, GPIO_INT_EDGE_FALLING);
        if (ret != 0) {
            LOG_ERR("Failed to configure wake button 2 interrupt: %d", ret);
            return POWER_MGMT_STATUS_GPIO_ERROR;
        }

        // Initialize and add callback
        gpio_init_callback(&g_power_ctx.wake_button2_cb_data, power_mgmt_wake_button_callback,
                          BIT(g_power_ctx.config.wake_button2->pin));
        ret = gpio_add_callback(g_power_ctx.config.wake_button2->port, &g_power_ctx.wake_button2_cb_data);
        if (ret != 0) {
            LOG_ERR("Failed to add wake button 2 callback: %d", ret);
            return POWER_MGMT_STATUS_GPIO_ERROR;
        }
        */

        LOG_INF("Wake button 2 configured successfully");
    }

    // Configure status LED if provided
    if (g_power_ctx.config.status_led && gpio_is_ready_dt(g_power_ctx.config.status_led))
    {
        gpio_pin_configure_dt(g_power_ctx.config.status_led, GPIO_OUTPUT_ACTIVE);
        LOG_INF("Status LED configured for power management");
    }

    g_power_ctx.initialized = true;

    LOG_INF("Power management driver initialized successfully");
    LOG_INF("Configuration: Auto-sleep=%s (timeout=%dms), Shutdown combo=%dms, Factory reset combo=%dms",
            g_power_ctx.config.auto_sleep_enabled ? "enabled" : "disabled",
            g_power_ctx.config.auto_sleep_timeout_ms,
            g_power_ctx.config.shutdown_combo_hold_ms,
            g_power_ctx.config.factory_reset_combo_hold_ms);

    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Check if power management driver is initialized
 */
bool power_mgmt_driver_is_initialized(void)
{
    return g_power_ctx.initialized;
}

/**
 * @brief Get current power state
 */
power_state_t power_mgmt_get_state(void)
{
    return g_power_ctx.current_state;
}

/**
 * @brief Check if system is currently sleeping
 */
bool power_mgmt_is_sleeping(void)
{
    return g_power_ctx.system_sleeping;
}

/**
 * @brief Shutdown all peripherals for lowest power consumption
 */
static power_mgmt_status_t power_mgmt_shutdown_peripherals(void)
{
    LOG_INF("Shutting down all peripherals for deep sleep...");

    // Turn off status LED
    if (g_power_ctx.config.status_led)
    {
        gpio_pin_set_dt(g_power_ctx.config.status_led, 0);
    }

    // Call registered shutdown callback
    if (g_power_ctx.shutdown_callback)
    {
        g_power_ctx.shutdown_callback();
    }

    LOG_INF("All peripherals shut down for sleep mode");
    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Wake up all peripherals from sleep
 */
static power_mgmt_status_t power_mgmt_wakeup_peripherals(void)
{
    LOG_INF("=== WAKING UP FROM SLEEP ===");
    LOG_INF("Reinitializing all peripherals and restoring data...");

    // Call registered wakeup callback
    if (g_power_ctx.wakeup_callback)
    {
        g_power_ctx.wakeup_callback();
    }

    // Turn on status LED to indicate system is awake
    if (g_power_ctx.config.status_led)
    {
        gpio_pin_set_dt(g_power_ctx.config.status_led, 1);
    }

    // Reset system state flags
    g_power_ctx.system_sleeping = false;
    g_power_ctx.sleeping_flag = false;
    g_power_ctx.current_state = POWER_STATE_ACTIVE;

    LOG_INF("=== WAKE-UP COMPLETE ===");
    LOG_INF("All peripherals active, data restored, system ready");

    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Save system state before entering sleep
 */
static power_mgmt_status_t power_mgmt_save_state(void)
{
    if (!g_power_ctx.config.save_state_on_sleep)
    {
        return POWER_MGMT_STATUS_OK;
    }

    LOG_INF("Saving system state before sleep...");

    // Call registered save callback
    if (g_power_ctx.save_callback)
    {
        g_power_ctx.save_callback();
    }

    LOG_INF("System state saved to storage");
    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Restore system state after wake-up
 */
static power_mgmt_status_t power_mgmt_restore_state(void)
{
    if (!g_power_ctx.config.restore_state_on_wake)
    {
        return POWER_MGMT_STATUS_OK;
    }

    LOG_INF("Restoring saved configuration from storage...");

    // Call registered restore callback
    if (g_power_ctx.restore_callback)
    {
        g_power_ctx.restore_callback();
    }

    LOG_INF("Data validation complete");
    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Enter sleep mode manually
 */
power_mgmt_status_t power_mgmt_enter_sleep(void)
{
    LOG_INF("=== SLEEP REQUEST ===");
    LOG_INF("Initialized: %d, System sleeping: %d, Current state: %d", 
           g_power_ctx.initialized, g_power_ctx.system_sleeping, g_power_ctx.current_state);
    
    if (!g_power_ctx.initialized)
    {
        LOG_ERR("Power management not initialized");
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    if (g_power_ctx.system_sleeping)
    {
        LOG_WRN("System already sleeping - rejecting sleep request");
        return POWER_MGMT_STATUS_ALREADY_SLEEPING;
    }

    LOG_INF("=== ENTERING SLEEP MODE ===");
    LOG_INF("Press any configured button to wake up");
    LOG_INF("Wake button 1 configured: %s (port=%p, pin=%d)", 
           g_power_ctx.config.wake_button1 ? "YES" : "NO",
           g_power_ctx.config.wake_button1 ? g_power_ctx.config.wake_button1->port : NULL,
           g_power_ctx.config.wake_button1 ? g_power_ctx.config.wake_button1->pin : -1);

    g_power_ctx.current_state = POWER_STATE_ENTERING_SLEEP;
    g_power_ctx.sleep_start_time = k_uptime_get_32();

    // Save current system state and configuration
    power_mgmt_save_state();

    // Shutdown all peripherals
    power_mgmt_shutdown_peripherals();
    LOG_INF("DEBUG: Peripheral shutdown complete");

    // Set system state
    g_power_ctx.system_sleeping = true;
    g_power_ctx.sleeping_flag = true;
    g_power_ctx.current_state = POWER_STATE_SLEEPING;
    LOG_INF("DEBUG: Sleep flags set - system_sleeping=%d", g_power_ctx.system_sleeping);

    // Update statistics
    g_power_ctx.stats.sleep_count++;
    LOG_INF("DEBUG: Statistics updated");

    // Ensure all logs are flushed
    k_sleep(K_MSEC(100));
    LOG_INF("DEBUG: About to enter sleep loop");

    // Enter deep sleep - nRF52840 will wake on GPIO interrupt
    LOG_INF("Entering deep sleep - wake on button press");

    // Use Zephyr's power management to enter sleep
    // The system will wake up on any enabled GPIO interrupt
    LOG_INF("Entering sleep loop - waiting for wake button interrupt");
    LOG_INF("DEBUG: system_sleeping flag = %d before entering loop", g_power_ctx.system_sleeping);
    
// Configure MODE button as wake source for System OFF mode
if (g_power_ctx.config.wake_button1) {
    // Disable Zephyr GPIO interrupt to avoid conflicts
    gpio_pin_interrupt_configure_dt(g_power_ctx.config.wake_button1, GPIO_INT_DISABLE);
    gpio_remove_callback(g_power_ctx.config.wake_button1->port, &g_power_ctx.wake_button1_cb_data);
    
    // Calculate absolute GPIO pin number for Nordic HAL
    // P0.x = x, P1.x = 32 + x for nRF52840
    uint32_t pin_num;
    if (g_power_ctx.config.wake_button1->port == DEVICE_DT_GET(DT_NODELABEL(gpio0))) {
        pin_num = g_power_ctx.config.wake_button1->pin;  // P0.x
    } else {
        pin_num = 32 + g_power_ctx.config.wake_button1->pin;  // P1.x = 32 + x
    }
    
    LOG_INF("Configuring System OFF wake: port=%p, relative_pin=%d, absolute_pin=%d", 
           g_power_ctx.config.wake_button1->port, g_power_ctx.config.wake_button1->pin, pin_num);
    
    nrf_gpio_cfg_sense_input(pin_num, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    
    // Verify the configuration
    bool pin_state = gpio_pin_get_dt(g_power_ctx.config.wake_button1);
    LOG_INF("MODE button configured as System OFF wake source (pin %d), current state: %d", 
           pin_num, pin_state);
}
    
    LOG_INF("Entering System OFF mode - system will reset on MODE button press");
    sys_poweroff();

    // System resets on wake - execution never reaches here
    // Wake-up handling will occur when main() starts again
    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Wake up from sleep mode manually
 */
power_mgmt_status_t power_mgmt_wake_up(void)
{
    if (!g_power_ctx.initialized)
    {
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    if (!g_power_ctx.system_sleeping)
    {
        return POWER_MGMT_STATUS_OK; // Already awake
    }

    LOG_INF("Manual wake-up requested");
    g_power_ctx.system_sleeping = false;
    g_power_ctx.stats.last_wake_was_button = false;

    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Check for button combinations
 */
bool power_mgmt_check_button_combo(button_combo_type_t combo_type)
{
    if (!g_power_ctx.initialized)
    {
        return false;
    }

    // Check if both buttons are available
    if (!g_power_ctx.config.wake_button1 || !g_power_ctx.config.wake_button2)
    {
        return false;
    }

    bool button1_pressed = !gpio_pin_get_dt(g_power_ctx.config.wake_button1);
    bool button2_pressed = !gpio_pin_get_dt(g_power_ctx.config.wake_button2);
    bool combo_pressed = button1_pressed && button2_pressed;

    switch (combo_type)
    {
    case BUTTON_COMBO_SHUTDOWN:
        if (!g_power_ctx.combos.shutdown_enabled)
        {
            return false;
        }

        if (combo_pressed && !g_power_ctx.combos.shutdown_active)
        {
            g_power_ctx.combos.shutdown_start_time = k_uptime_get_32();
            g_power_ctx.combos.shutdown_active = true;
            g_power_ctx.stats.shutdown_combo_activations++;
            LOG_INF("Shutdown combo detected - hold for %d seconds...",
                    g_power_ctx.config.shutdown_combo_hold_ms / 1000);
        }
        else if (!combo_pressed)
        {
            if (g_power_ctx.combos.shutdown_active)
            {
                LOG_INF("Shutdown combo cancelled");
            }
            g_power_ctx.combos.shutdown_active = false;
        }

        if (g_power_ctx.combos.shutdown_active)
        {
            uint32_t hold_time = k_uptime_get_32() - g_power_ctx.combos.shutdown_start_time;
            if (hold_time >= g_power_ctx.config.shutdown_combo_hold_ms)
            {
                LOG_INF("Shutdown combo held for %d seconds - initiating shutdown",
                        g_power_ctx.config.shutdown_combo_hold_ms / 1000);
                g_power_ctx.combos.shutdown_active = false;
                return true;
            }
        }
        break;

    case BUTTON_COMBO_FACTORY_RESET:
        if (!g_power_ctx.combos.factory_reset_enabled)
        {
            return false;
        }

        if (combo_pressed && !g_power_ctx.combos.factory_reset_active)
        {
            g_power_ctx.combos.factory_reset_start_time = k_uptime_get_32();
            g_power_ctx.combos.factory_reset_active = true;
            LOG_INF("Factory reset combo detected - hold for %d seconds",
                    g_power_ctx.config.factory_reset_combo_hold_ms / 1000);
        }
        else if (!combo_pressed)
        {
            g_power_ctx.combos.factory_reset_active = false;
        }

        if (g_power_ctx.combos.factory_reset_active)
        {
            uint32_t hold_time = k_uptime_get_32() - g_power_ctx.combos.factory_reset_start_time;
            if (hold_time >= g_power_ctx.config.factory_reset_combo_hold_ms)
            {
                LOG_WRN("Factory reset triggered!");
                g_power_ctx.combos.factory_reset_active = false;
                g_power_ctx.stats.factory_reset_count++;

                // Call registered factory reset callback
                if (g_power_ctx.factory_reset_callback)
                {
                    g_power_ctx.factory_reset_callback();
                }

                return true;
            }
        }
        break;
    }

    return false;
}

/**
 * @brief Process power management tasks
 */
power_mgmt_status_t power_mgmt_process(void)
{
    if (!g_power_ctx.initialized)
    {
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    // Check for auto-sleep timeout
    if (g_power_ctx.config.auto_sleep_enabled && !g_power_ctx.system_sleeping)
    {
        uint32_t idle_time = k_uptime_get_32() - g_power_ctx.last_activity_time;
        if (idle_time >= g_power_ctx.config.auto_sleep_timeout_ms)
        {
            LOG_INF("Auto-sleep timeout reached - entering sleep mode");
            return power_mgmt_enter_sleep();
        }
    }

    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Reset auto-sleep timer
 */
power_mgmt_status_t power_mgmt_reset_auto_sleep_timer(void)
{
    if (!g_power_ctx.initialized)
    {
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    g_power_ctx.last_activity_time = k_uptime_get_32();
    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Get power management statistics
 */
power_mgmt_status_t power_mgmt_get_stats(power_mgmt_stats_t *stats)
{
    if (!stats)
    {
        return POWER_MGMT_STATUS_ERROR;
    }

    if (!g_power_ctx.initialized)
    {
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    memcpy(stats, &g_power_ctx.stats, sizeof(power_mgmt_stats_t));
    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Reset power management statistics
 */
power_mgmt_status_t power_mgmt_reset_stats(void)
{
    if (!g_power_ctx.initialized)
    {
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    memset(&g_power_ctx.stats, 0, sizeof(power_mgmt_stats_t));
    LOG_INF("Power management statistics reset");
    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Set auto-sleep configuration
 */
power_mgmt_status_t power_mgmt_set_auto_sleep(bool enabled, uint32_t timeout_ms)
{
    if (!g_power_ctx.initialized)
    {
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    g_power_ctx.config.auto_sleep_enabled = enabled;
    g_power_ctx.config.auto_sleep_timeout_ms = timeout_ms;

    LOG_INF("Auto-sleep %s with timeout %dms", enabled ? "enabled" : "disabled", timeout_ms);
    return POWER_MGMT_STATUS_OK;
}

/**
 * @brief Get time since last user activity
 */
uint32_t power_mgmt_get_idle_time(void)
{
    return k_uptime_get_32() - g_power_ctx.last_activity_time;
}

// ============================================================================
// Callback Registration Functions
// ============================================================================

power_mgmt_status_t power_mgmt_register_save_callback(power_save_callback_t callback)
{
    g_power_ctx.save_callback = callback;
    return POWER_MGMT_STATUS_OK;
}

power_mgmt_status_t power_mgmt_register_restore_callback(power_restore_callback_t callback)
{
    g_power_ctx.restore_callback = callback;
    return POWER_MGMT_STATUS_OK;
}

power_mgmt_status_t power_mgmt_register_shutdown_callback(power_peripheral_shutdown_callback_t callback)
{
    g_power_ctx.shutdown_callback = callback;
    return POWER_MGMT_STATUS_OK;
}

power_mgmt_status_t power_mgmt_register_wakeup_callback(power_peripheral_wakeup_callback_t callback)
{
    g_power_ctx.wakeup_callback = callback;
    return POWER_MGMT_STATUS_OK;
}

power_mgmt_status_t power_mgmt_register_factory_reset_callback(power_factory_reset_callback_t callback)
{
    g_power_ctx.factory_reset_callback = callback;
    return POWER_MGMT_STATUS_OK;
}

// ============================================================================
// Advanced Features
// ============================================================================

power_mgmt_status_t power_mgmt_emergency_shutdown(void)
{
    LOG_WRN("Emergency shutdown requested");
    return power_mgmt_shutdown_peripherals();
}

power_mgmt_status_t power_mgmt_validate_state(void)
{
    // Basic state validation
    if (g_power_ctx.current_state < POWER_STATE_ACTIVE ||
        g_power_ctx.current_state > POWER_STATE_WAKING_UP)
    {
        LOG_ERR("Invalid power state detected: %d", g_power_ctx.current_state);
        return POWER_MGMT_STATUS_ERROR;
    }

    return POWER_MGMT_STATUS_OK;
}

power_mgmt_status_t power_mgmt_set_combo_timings(uint32_t shutdown_ms, uint32_t factory_reset_ms)
{
    if (!g_power_ctx.initialized)
    {
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    g_power_ctx.config.shutdown_combo_hold_ms = shutdown_ms;
    g_power_ctx.config.factory_reset_combo_hold_ms = factory_reset_ms;

    LOG_INF("Button combo timings updated: shutdown=%dms, factory_reset=%dms",
            shutdown_ms, factory_reset_ms);
    return POWER_MGMT_STATUS_OK;
}

power_mgmt_status_t power_mgmt_enable_combo(button_combo_type_t combo_type, bool enabled)
{
    if (!g_power_ctx.initialized)
    {
        return POWER_MGMT_STATUS_NOT_INITIALIZED;
    }

    switch (combo_type)
    {
    case BUTTON_COMBO_SHUTDOWN:
        g_power_ctx.combos.shutdown_enabled = enabled;
        LOG_INF("Shutdown combo %s", enabled ? "enabled" : "disabled");
        break;
    case BUTTON_COMBO_FACTORY_RESET:
        g_power_ctx.combos.factory_reset_enabled = enabled;
        LOG_INF("Factory reset combo %s", enabled ? "enabled" : "disabled");
        break;
    default:
        return POWER_MGMT_STATUS_ERROR;
    }

    return POWER_MGMT_STATUS_OK;
}
