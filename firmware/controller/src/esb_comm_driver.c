/*
 * ESB Communication Driver - Enhanced ShockBurst Communication Implementation
 *
 * This driver provides ESB (Enhanced ShockBurst) wireless communication
 * functionality for the controller system. It handles initialization,
 * data transmission with adaptive timing, and power management.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <esb.h>
#include "esb_comm_driver.h"

LOG_MODULE_REGISTER(esb_comm, LOG_LEVEL_INF);

// ESB communication context
typedef struct
{
    bool initialized;
    bool enabled;
    esb_comm_config_t config;
    esb_comm_stats_t stats;
    struct esb_payload tx_payload;
    uint32_t last_tx_attempt;
    bool last_tx_succeeded;
    // ACK payload timing support
    bool ack_timing_enabled; // Enable/disable ACK payload timing
    bool ack_timing_active;
    ack_timing_data_t last_ack_data; // Last received ACK payload data
    uint32_t next_tx_delay_ms;
    uint8_t current_rumble_left;  // 0-15 left motor intensity
    uint8_t current_rumble_right; // 0-15 right motor intensity
} esb_comm_context_t;

// Global context
static esb_comm_context_t g_esb_ctx = {0};

// Forward declarations
static int esb_comm_clocks_start(void);
static void esb_comm_event_handler(struct esb_evt const *event);

/**
 * @brief Start high frequency clocks required for ESB
 */
static int esb_comm_clocks_start(void)
{
    int err;
    int res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr)
    {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0)
    {
        LOG_ERR("Clock request failed: %d", err);
        return err;
    }

    do
    {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res)
        {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err);

    LOG_INF("HF clock started for ESB");
    return 0;
}

/**
 * @brief ESB event handler - processes transmission events
 */
static void esb_comm_event_handler(struct esb_evt const *event)
{
    switch (event->evt_id)
    {
    case ESB_EVENT_TX_SUCCESS:
        // Update statistics
        g_esb_ctx.stats.successful_transmissions++;
        g_esb_ctx.last_tx_succeeded = true;

        // Process ACK payload if present
        struct esb_payload ack_payload;
        int err = esb_read_rx_payload(&ack_payload);

        if (err == 0 && ack_payload.length >= sizeof(ack_timing_data_t))
        {
            // Valid ACK payload received - extract timing and rumble data
            memcpy(&g_esb_ctx.last_ack_data, ack_payload.data, sizeof(ack_timing_data_t));

            // Set next transmission delay from ACK payload (only if valid)
            if (g_esb_ctx.last_ack_data.next_delay_ms > 0 && g_esb_ctx.last_ack_data.next_delay_ms < 1000) {
                g_esb_ctx.next_tx_delay_ms = g_esb_ctx.last_ack_data.next_delay_ms;
                g_esb_ctx.ack_timing_active = true;
            } else {
                // Invalid timing data in ACK payload
                g_esb_ctx.ack_timing_active = false;
                g_esb_ctx.next_tx_delay_ms = g_esb_ctx.config.base_tx_interval_ms;
            }

            // Extract rumble data (upper 4 bits = left motor, lower 4 bits = right motor)
            g_esb_ctx.current_rumble_left = (g_esb_ctx.last_ack_data.rumble_data >> 4) & 0x0F;
            g_esb_ctx.current_rumble_right = g_esb_ctx.last_ack_data.rumble_data & 0x0F;

            // Log ACK payload data (occasionally, to avoid spam)
            static uint32_t last_ack_log = 0;
            uint32_t now = k_uptime_get_32();
            if ((now - last_ack_log) > 5000)
            { // Log every 5 seconds
                LOG_INF("ACK payload: delay=%dms, seq=%d, rumble=L%d/R%d, dongle_time=%u",
                        g_esb_ctx.last_ack_data.next_delay_ms,
                        g_esb_ctx.last_ack_data.sequence_num,
                        g_esb_ctx.current_rumble_left,
                        g_esb_ctx.current_rumble_right,
                        g_esb_ctx.last_ack_data.dongle_timestamp);
                last_ack_log = now;
            }
        }
        else
        {
            // No valid ACK payload - fall back to fixed timing
            g_esb_ctx.ack_timing_active = false;
            g_esb_ctx.next_tx_delay_ms = g_esb_ctx.config.base_tx_interval_ms;
            g_esb_ctx.current_rumble_left = 0;
            g_esb_ctx.current_rumble_right = 0;
        }

        // Turn off status LED (if configured)
        if (g_esb_ctx.config.status_led)
        {
            gpio_pin_set_dt(g_esb_ctx.config.status_led, 0);
        }

        // Clear buffers after successful transmission to prevent buildup
        esb_flush_tx();
        break;

    case ESB_EVENT_TX_FAILED:
        // Update statistics
        g_esb_ctx.stats.failed_transmissions++;
        g_esb_ctx.stats.retry_count++;
        g_esb_ctx.last_tx_succeeded = false;

        // On TX failure, disable ACK timing and use retry interval
        g_esb_ctx.ack_timing_active = false;
        g_esb_ctx.next_tx_delay_ms = g_esb_ctx.config.retry_interval_ms;
        g_esb_ctx.current_rumble_left = 0;
        g_esb_ctx.current_rumble_right = 0;

        // Turn off status LED on failure too
        if (g_esb_ctx.config.status_led)
        {
            gpio_pin_set_dt(g_esb_ctx.config.status_led, 0);
        }

        // Clear buffers after failed transmission to prevent radio jam
        esb_flush_tx();
        esb_flush_rx();
        break;

    case ESB_EVENT_RX_RECEIVED:
        // Controller is in PTX mode, shouldn't normally receive data
        // Clear RX buffer to prevent overflow
        esb_flush_rx();
        break;

    default:
        LOG_WRN("Unknown ESB event: %d", event->evt_id);
        // Clear all buffers on unknown events as safety measure
        esb_flush_tx();
        esb_flush_rx();
        break;
    }

    // Update total transmissions and success rate
    g_esb_ctx.stats.total_transmissions = g_esb_ctx.stats.successful_transmissions +
                                          g_esb_ctx.stats.failed_transmissions;

    if (g_esb_ctx.stats.total_transmissions > 0)
    {
        g_esb_ctx.stats.success_rate = (float)g_esb_ctx.stats.successful_transmissions /
                                       g_esb_ctx.stats.total_transmissions * 100.0f;
    }
}

/**
 * @brief Initialize ESB communication driver
 */
esb_comm_status_t esb_comm_driver_init(const esb_comm_config_t *config)
{
    int err;

    if (!config)
    {
        LOG_ERR("Invalid configuration pointer");
        return ESB_COMM_STATUS_ERROR;
    }

    if (g_esb_ctx.initialized)
    {
        LOG_WRN("ESB communication already initialized");
        return ESB_COMM_STATUS_OK;
    }

    LOG_INF("ESB communication driver initialization starting...");

    // Copy configuration
    memcpy(&g_esb_ctx.config, config, sizeof(esb_comm_config_t));

    // Set default values if not specified
    if (g_esb_ctx.config.base_tx_interval_ms == 0)
    {
        g_esb_ctx.config.base_tx_interval_ms = 5; // 5ms default (200Hz) - back to working original
    }
    if (g_esb_ctx.config.retry_interval_ms == 0)
    {
        g_esb_ctx.config.retry_interval_ms = 10; // 10ms retry interval - back to working original
    }
    if (g_esb_ctx.config.rf_channel == 0)
    {
        g_esb_ctx.config.rf_channel = 1; // Default RF channel
    }

    LOG_INF("Controller %d timing offset: +%dms (base: %dms, retry: %dms)",
            g_esb_ctx.config.controller_id, 0,
            g_esb_ctx.config.base_tx_interval_ms, g_esb_ctx.config.retry_interval_ms);

    // Initialize statistics
    memset(&g_esb_ctx.stats, 0, sizeof(esb_comm_stats_t));

    // Start clocks first (required for Nordic nRF52)
    err = esb_comm_clocks_start();
    if (err)
    {
        LOG_ERR("Clock start failed: %d", err);
        return ESB_COMM_STATUS_CLOCK_FAILED;
    }

    // ESB configuration - PTX mode for continuous transmission with ACK timing
    struct esb_config esb_cfg = ESB_DEFAULT_CONFIG;
    esb_cfg.protocol = ESB_PROTOCOL_ESB_DPL; // Dynamic payload length
    esb_cfg.mode = ESB_MODE_PTX;             // Controller transmits data continuously
    esb_cfg.retransmit_delay = 600;          // 600us delay between retransmissions
    esb_cfg.retransmit_count = 2;
    esb_cfg.tx_output_power = 8;             // Maximum TX power (8 dBm)
    esb_cfg.event_handler = esb_comm_event_handler;
    esb_cfg.bitrate = ESB_BITRATE_2MBPS;
    esb_cfg.selective_auto_ack = true; // Enable ACK for timing coordination
    esb_cfg.use_fast_ramp_up = false;  // Disable fast ramp up for stability during ADC activity
    esb_cfg.payload_length = 21;

    // Set up addresses - matching Nordic reference pattern
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7}; // RIGHT controller base (pipe 0)
    uint8_t base_addr_1[4] = {0xD4, 0xD4, 0xD4, 0xD4}; // LEFT controller base (pipe 1)
    uint8_t addr_prefix[8] = {0xE7, 0xD4, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

    err = esb_init(&esb_cfg);
    if (err)
    {
        LOG_ERR("ESB init failed: %d", err);
        return ESB_COMM_STATUS_CONFIG_FAILED;
    }

    err = esb_set_base_address_0(base_addr_0);
    if (err)
    {
        LOG_ERR("ESB set base address 0 failed: %d", err);
        return ESB_COMM_STATUS_CONFIG_FAILED;
    }

    err = esb_set_base_address_1(base_addr_1);
    if (err)
    {
        LOG_ERR("ESB set base address 1 failed: %d", err);
        return ESB_COMM_STATUS_CONFIG_FAILED;
    }

    err = esb_set_prefixes(addr_prefix, 8);
    if (err)
    {
        LOG_ERR("ESB set prefixes failed: %d", err);
        return ESB_COMM_STATUS_CONFIG_FAILED;
    }

    // Set RF channel
    err = esb_set_rf_channel(g_esb_ctx.config.rf_channel);
    if (err)
    {
        LOG_ERR("ESB set RF channel failed: %d", err);
        return ESB_COMM_STATUS_CONFIG_FAILED;
    }

    LOG_INF("Controller %d using ACK-based timing for interference avoidance",
            g_esb_ctx.config.controller_id);

    // Set radio TX power to 8 (8 dBm for nRF52840)
    err = esb_set_tx_power(ESB_TX_POWER_8DBM);
    if (err)
    {
        LOG_ERR("ESB set TX power failed: %d", err);
        return ESB_COMM_STATUS_CONFIG_FAILED;
    }

    // Configure status LED if provided
    if (g_esb_ctx.config.status_led && gpio_is_ready_dt(g_esb_ctx.config.status_led))
    {
        gpio_pin_configure_dt(g_esb_ctx.config.status_led, GPIO_OUTPUT_INACTIVE);
        LOG_INF("Status LED configured for ESB transmission feedback");
    }

    // Initialize state
    g_esb_ctx.initialized = true;
    g_esb_ctx.enabled = true;
    g_esb_ctx.last_tx_attempt = 0;
    g_esb_ctx.last_tx_succeeded = true;

    LOG_INF("ESB communication driver initialized successfully - ready for transmission");
    LOG_INF("Configuration: ID=%d, Base interval=%dms, Retry interval=%dms, RF channel=%d",
            g_esb_ctx.config.controller_id,
            g_esb_ctx.config.base_tx_interval_ms,
            g_esb_ctx.config.retry_interval_ms,
            g_esb_ctx.config.rf_channel);

    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Check if ESB communication driver is initialized
 */
bool esb_comm_driver_is_initialized(void)
{
    return g_esb_ctx.initialized;
}

/**
 * @brief Send controller data via ESB with adaptive timing (timed version)
 */
esb_comm_status_t esb_comm_send_data_timed(const esb_controller_data_t *data)
{
    if (!g_esb_ctx.initialized || !g_esb_ctx.enabled)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    if (!data)
    {
        return ESB_COMM_STATUS_ERROR;
    }

    uint32_t now = k_uptime_get_32();
    uint32_t tx_interval;

    // Use ACK payload timing if enabled and available, otherwise fall back to adaptive timing
    if (g_esb_ctx.ack_timing_enabled && g_esb_ctx.ack_timing_active && g_esb_ctx.next_tx_delay_ms > 0)
    {
        // Use timing from dongle's ACK payload
        tx_interval = g_esb_ctx.next_tx_delay_ms;
        
        // Debug logging for ACK timing usage (throttled)
        static uint32_t last_ack_timing_log = 0;
        uint32_t debug_now = k_uptime_get_32();
        if ((debug_now - last_ack_timing_log) > 3000) { // Log every 3 seconds
            LOG_INF("Using ACK timing: %dms (seq=%d)", tx_interval, g_esb_ctx.last_ack_data.sequence_num);
            last_ack_timing_log = debug_now;
        }
    }
    else
    {
        // Fall back to adaptive timing based on controller ID and last TX result
        if (!g_esb_ctx.last_tx_succeeded)
        {
            // If last TX failed (no ACK), use longer interval with controller-specific offset
            tx_interval = g_esb_ctx.config.retry_interval_ms;
        }
        else
        {
            // If last TX succeeded, use base interval with small offset
            tx_interval = g_esb_ctx.config.base_tx_interval_ms;
        }
        
        // Debug logging for fallback timing (throttled)
        static uint32_t last_fallback_log = 0;
        uint32_t debug_now = k_uptime_get_32();
        if ((debug_now - last_fallback_log) > 3000) { // Log every 3 seconds
            const char *reason = !g_esb_ctx.ack_timing_enabled ? "disabled" : 
                               !g_esb_ctx.ack_timing_active ? "inactive" :
                               g_esb_ctx.next_tx_delay_ms == 0 ? "zero_delay" : "unknown";
            LOG_INF("Using fallback timing: %dms (reason: %s)", tx_interval, reason);
            last_fallback_log = debug_now;
        }
    }

    // Check if enough time has passed since last attempt
    if ((now - g_esb_ctx.last_tx_attempt) < tx_interval)
    {
        return ESB_COMM_STATUS_OK; // Too soon to transmit, but not an error
    }

    // Proceed with transmission
    return esb_comm_send_immediate(data);
}

/**
 * @brief Send controller data via ESB immediately (force transmission)
 */
esb_comm_status_t esb_comm_send_immediate(const esb_controller_data_t *data)
{
    if (!g_esb_ctx.initialized || !g_esb_ctx.enabled)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    if (!data)
    {
        return ESB_COMM_STATUS_ERROR;
    }

    // Check if radio is ready for transmission (critical for preventing overload)
    if (esb_is_idle() != true)
    {
        // Radio is busy - don't queue another packet to prevent buffer overload
        g_esb_ctx.stats.failed_transmissions++;
        return ESB_COMM_STATUS_BUSY;
    }

    // Prepare payload
    g_esb_ctx.tx_payload.length = sizeof(esb_controller_data_t);
    g_esb_ctx.tx_payload.pipe = g_esb_ctx.config.controller_id; // LEFT=1, RIGHT=0
    memcpy(g_esb_ctx.tx_payload.data, data, sizeof(esb_controller_data_t));

    // Clear TX buffer first to prevent buffer overload
    esb_flush_tx();

    // Write payload to radio
    int err = esb_write_payload(&g_esb_ctx.tx_payload);

    // Update timestamp regardless of result
    g_esb_ctx.last_tx_attempt = k_uptime_get_32();
    g_esb_ctx.stats.last_tx_timestamp = g_esb_ctx.last_tx_attempt;

    if (err)
    {
        LOG_WRN("ESB write payload failed: %d (radio overload?)", err);
        g_esb_ctx.stats.failed_transmissions++;
        g_esb_ctx.last_tx_succeeded = false;

        // Additional buffer clearing on error to prevent jam
        esb_flush_tx();
        esb_flush_rx();

        return ESB_COMM_STATUS_TX_FAILED;
    }

    // Turn on status LED (if configured) - will be turned off by event handler on ACK
    if (g_esb_ctx.config.status_led)
    {
        gpio_pin_set_dt(g_esb_ctx.config.status_led, 1);
    }

    // No blocking delay - let event handler process TX events asynchronously

    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Send controller data via ESB (standard interface)
 */
esb_comm_status_t esb_comm_send_data(const esb_controller_data_t *data)
{
    return esb_comm_send_data_timed(data);
}

/**
 * @brief Get current transmission statistics
 */
esb_comm_status_t esb_comm_get_stats(esb_comm_stats_t *stats)
{
    if (!stats)
    {
        return ESB_COMM_STATUS_ERROR;
    }

    if (!g_esb_ctx.initialized)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    // Update current state
    g_esb_ctx.stats.last_tx_succeeded = g_esb_ctx.last_tx_succeeded;

    memcpy(stats, &g_esb_ctx.stats, sizeof(esb_comm_stats_t));
    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Reset transmission statistics
 */
esb_comm_status_t esb_comm_reset_stats(void)
{
    if (!g_esb_ctx.initialized)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    memset(&g_esb_ctx.stats, 0, sizeof(esb_comm_stats_t));
    LOG_INF("ESB communication statistics reset");
    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Set transmission timing parameters
 */
esb_comm_status_t esb_comm_set_timing(uint32_t base_interval_ms, uint32_t retry_interval_ms)
{
    if (!g_esb_ctx.initialized)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    if (base_interval_ms == 0 || retry_interval_ms == 0)
    {
        return ESB_COMM_STATUS_ERROR;
    }

    g_esb_ctx.config.base_tx_interval_ms = base_interval_ms;
    g_esb_ctx.config.retry_interval_ms = retry_interval_ms;

    LOG_INF("ESB timing updated: base=%dms, retry=%dms", base_interval_ms, retry_interval_ms);
    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Enable or disable ESB communication
 */
esb_comm_status_t esb_comm_enable(bool enable)
{
    if (!g_esb_ctx.initialized)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    if (enable && !g_esb_ctx.enabled)
    {
        // Re-enable ESB
        LOG_INF("Enabling ESB communication");
        g_esb_ctx.enabled = true;
    }
    else if (!enable && g_esb_ctx.enabled)
    {
        // Disable ESB
        LOG_INF("Disabling ESB communication");
        esb_disable();
        g_esb_ctx.enabled = false;
    }

    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Enter low power mode (disables ESB)
 */
esb_comm_status_t esb_comm_enter_sleep(void)
{
    if (!g_esb_ctx.initialized)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    LOG_INF("ESB entering sleep mode");
    esb_disable();
    g_esb_ctx.enabled = false;

    // Turn off status LED
    if (g_esb_ctx.config.status_led)
    {
        gpio_pin_set_dt(g_esb_ctx.config.status_led, 0);
    }

    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Wake up from low power mode (re-enables ESB)
 */
esb_comm_status_t esb_comm_wakeup(void)
{
    if (!g_esb_ctx.initialized)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    LOG_INF("ESB waking up from sleep mode");

    // Re-initialize ESB with current configuration
    esb_comm_status_t status = esb_comm_driver_init(&g_esb_ctx.config);
    if (status != ESB_COMM_STATUS_OK)
    {
        LOG_ERR("Failed to re-initialize ESB after wakeup: %d", status);
        return status;
    }

    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Get current ESB communication status
 */
bool esb_comm_is_ready(void)
{
    return g_esb_ctx.initialized && g_esb_ctx.enabled;
}

/**
 * @brief Get last transmission timestamp
 */
uint32_t esb_comm_get_last_tx_time(void)
{
    return g_esb_ctx.last_tx_attempt;
}

/**
 * @brief Check if last transmission was successful
 */
bool esb_comm_get_last_tx_success(void)
{
    return g_esb_ctx.last_tx_succeeded;
}

/**
 * @brief Enable ACK payload based timing
 */
esb_comm_status_t esb_comm_enable_ack_timing(bool enable)
{
    if (!g_esb_ctx.initialized)
    {
        return ESB_COMM_STATUS_NOT_INITIALIZED;
    }

    g_esb_ctx.ack_timing_enabled = enable;
    LOG_INF("ACK payload timing %s", enable ? "enabled" : "disabled");
    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Get current rumble data from last ACK payload
 */
esb_comm_status_t esb_comm_get_rumble_data(uint8_t *left_motor, uint8_t *right_motor)
{
    if (!left_motor || !right_motor)
    {
        return ESB_COMM_STATUS_ERROR;
    }

    *left_motor = g_esb_ctx.current_rumble_left;
    *right_motor = g_esb_ctx.current_rumble_right;

    return ESB_COMM_STATUS_OK;
}

/**
 * @brief Get dongle timestamp from last ACK payload
 */
uint32_t esb_comm_get_dongle_timestamp(void)
{
    return g_esb_ctx.last_ack_data.dongle_timestamp;
}

/**
 * @brief Get next transmission delay from last ACK payload
 */
uint16_t esb_comm_get_next_delay(void)
{
    return g_esb_ctx.last_ack_data.next_delay_ms;
}
