#include "controller_esb.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(controller_esb, LOG_LEVEL_INF);

// ESB data structures
static struct esb_payload rx_payload;
static simple_controller_state_t controller_state = {0};

// LED for debug feedback
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// Timing tracking for packet logging
static uint32_t last_packet_time = 0;

// ACK payload timing control variables
static uint8_t sequence_counter = 0;
static uint32_t last_tx_time[2] = {0, 0};  // Separate timing for each controller: [0]=right, [1]=left
static uint32_t last_any_rx_time = 0;      // Track most recent packet from ANY controller for collision detection
static const uint16_t BASE_INTERVAL_MS = 4;  // 4ms intervals for stable performance

// ESB event handler for ACK-based reception
static void simple_esb_event_handler(struct esb_evt const *event)
{
    // Always log events to debug communication
    // LOG_INF("DONGLE ESB EVENT: %d", event->evt_id);

    switch (event->evt_id)
    {
    case ESB_EVENT_RX_RECEIVED:
        // LOG_INF("RX RECEIVED - got controller data");
        
        uint32_t rx_start = k_uptime_get_32();
        static uint32_t last_rx_process_time = 0;
        
        // Get the received data
        if (esb_read_rx_payload(&rx_payload) == 0)
        {
            // Filter out spurious packets - we only want controller data
            if (rx_payload.length == sizeof(controller_data_t))
            {
                // LOG_INF("Valid controller data - length: %d, pipe: %d", rx_payload.length, rx_payload.pipe);
                // Parse the controller data
                controller_data_t *data = (controller_data_t *)rx_payload.data;

                // Calculate timing and determine controller half
                uint32_t current_time = k_uptime_get_32();
                uint32_t time_diff = current_time - last_packet_time;
                bool is_left = (data->flags & 0x80) != 0;
                uint8_t controller_id = is_left ? 1 : 0;

                // Calculate gap since ANY controller packet for collision detection (BEFORE updating timing)
                uint32_t gap_since_any = (last_any_rx_time == 0) ? 0 : (current_time - last_any_rx_time);

                // Log only problematic timing patterns to reduce logging overhead
                if (last_packet_time != 0 && time_diff < 2)
                {
                    LOG_WRN("COLLISION: %s controller - %dms gap (too close!)",
                            is_left ? "LEFT" : "RIGHT", time_diff);
                    
                    // Also log per-controller timing for collisions
                    uint32_t controller_gap = (last_tx_time[controller_id] == 0) ? 0 : 
                                             (current_time - last_tx_time[controller_id]);
                    LOG_WRN("    Controller gap: %dms since last from THIS controller", controller_gap);
                }

                // Update timing variables AFTER calculating gaps
                last_packet_time = current_time;
                last_any_rx_time = current_time;

                // Store the controller data
                controller_state.flags = data->flags;
                controller_state.trigger = data->trigger;
                controller_state.stickX = data->stickX;
                controller_state.stickY = data->stickY;
                controller_state.padX = data->padX;
                controller_state.padY = data->padY;
                controller_state.buttons = data->buttons;
                controller_state.accelX = data->accelX;
                controller_state.accelY = data->accelY;
                controller_state.accelZ = data->accelZ;
                controller_state.gyroX = data->gyroX;
                controller_state.gyroY = data->gyroY;
                controller_state.gyroZ = data->gyroZ;
                controller_state.data_received = true;
                controller_state.last_ping_time = current_time;

                // Track severe delays that indicate controller-side issues
                if (time_diff > 50) {
                    LOG_ERR("SEVERE DELAY: %dms gap from %s controller - likely controller freeze", 
                            time_diff, is_left ? "LEFT" : "RIGHT");
                }

                // Create ACK payload with timing control + rumble (lean 8-byte design)
                ack_timing_data_t ack_data = {
                    .next_delay_ms = 0,        // Will calculate below
                    .sequence_num = sequence_counter++,
                    .rumble_data = 0x00,       // No rumble for now: left=0, right=0 
                    .dongle_timestamp = current_time
                };
                
                // Staggered timing to prevent packet collisions
                // Both controllers use the same base interval
                uint32_t base_delay = BASE_INTERVAL_MS;  // 4ms for both
                
                // Check if we need collision avoidance padding
                // If this packet arrived ≤1ms after the last packet from ANY controller, add 1ms padding
                bool need_collision_padding = false;
                if (gap_since_any > 0 && gap_since_any <= 1) {
                    need_collision_padding = true;
                    LOG_WRN("COLLISION DETECTED: Adding 1ms padding to %s controller (gap was %dms)", 
                            is_left ? "LEFT" : "RIGHT", gap_since_any);
                }
                
                // Set the delay: base interval + collision padding if needed
                ack_data.next_delay_ms = base_delay + (need_collision_padding ? 1 : 0);
                
                // Log what timing we're actually sending to controllers - every 10th packet to reduce spam
                static uint32_t log_counter = 0;
                if (++log_counter % 10 == 0) {
                    uint32_t time_since_last = (last_tx_time[controller_id] == 0) ? 0 : (current_time - last_tx_time[controller_id]);
                    LOG_WRN("%s controller: base=%dms, since_last=%dms, sending_delay=%dms%s",
                            is_left ? "LEFT" : "RIGHT", base_delay, time_since_last, ack_data.next_delay_ms,
                            need_collision_padding ? " (PADDED)" : "");
                }
                
                // Update last transmission time tracking (per-controller)
                last_tx_time[controller_id] = current_time;
                
                // NOTE: last_any_rx_time already updated above after gap calculation
                
                // Queue ACK payload using Nordic's approach - this goes into TX FIFO
                // and will be attached to the ACK for the NEXT packet received on this pipe
                struct esb_payload ack_tx_payload = {0};
                ack_tx_payload.pipe = rx_payload.pipe;        // CRUCIAL - same pipe as RX
                ack_tx_payload.length = sizeof(ack_timing_data_t); // 8 bytes
                memcpy(ack_tx_payload.data, &ack_data, ack_tx_payload.length);
                
                // Queue it - this attaches to the next ACK on this pipe
                int result = esb_write_payload(&ack_tx_payload);
                if (result != 0) {
                    LOG_WRN("Failed to queue ACK payload: %d", result);
                } else {
                    // DEBUG: Let's verify we're actually sending ACK payloads
                    static uint32_t ack_counter = 0;
                    if (++ack_counter % 20 == 0) {
                        LOG_INF("ACK payload sent: delay=%dms, pipe=%d, seq=%d", 
                               ack_data.next_delay_ms, rx_payload.pipe, ack_data.sequence_num);
                    }
                }

                gpio_pin_set_dt(&led0, 1);
            }
            else
            {
                // LOG_DBG("Ignoring packet with wrong length: %d (expected %d)",
                //         rx_payload.length, sizeof(controller_data_t));
            }
        }
        else
        {
            LOG_WRN("Failed to read RX payload");
        }
        
        // Monitor RX processing time
        uint32_t rx_process_time = k_uptime_get_32() - rx_start;
        if (rx_process_time > 2) { // Warn if RX processing takes over 2ms
            LOG_WRN("Slow RX processing: %dms", rx_process_time);
        }
        
        // Track time between RX processing - only warn on major delays
        if (last_rx_process_time != 0) {
            uint32_t rx_interval = rx_start - last_rx_process_time;
            if (rx_interval > 20) { // Only warn if > 20ms (was 8ms)
                LOG_WRN("Long RX interval: %dms", rx_interval);
            }
        }
        last_rx_process_time = rx_start;
        
        break;

    case ESB_EVENT_TX_SUCCESS:
        // Every 50th ACK transmission, log success to verify ACK payloads are going out
        static uint32_t ack_tx_counter = 0;
        if (++ack_tx_counter % 50 == 0) {
            LOG_INF("ACK payloads being transmitted successfully (count: %d)", ack_tx_counter);
        }
        break;

    case ESB_EVENT_TX_FAILED:
        LOG_WRN("Failed to send ACK payload - flushing TX queue");
        // For PRX, this usually means the queued ACK payload couldn't be sent
        // Flush the TX FIFO to clear any stuck payloads
        esb_flush_tx();
        break;

    default:
        LOG_WRN("Unknown ESB event: %d", event->evt_id);
        break;
    }
}

// Clock initialization (based on Nordic reference)
int clocks_start(void)
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

    LOG_INF("HF clock started");
    return 0;
}

// Initialize ESB for ACK-based controller communication
int controller_esb_init(void)
{
    int err;

    LOG_INF("Dongle ESB initialization starting...");

    // Start clocks first (like Nordic reference)
    err = clocks_start();
    if (err)
    {
        LOG_ERR("Clock start failed: %d", err);
        return err;
    }

    // Initialize LED
    if (!gpio_is_ready_dt(&led0))
    {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

    // ESB configuration - PRX mode to receive controller data and send ACKs
    // Based on Nordic reference sample
    struct esb_config config = ESB_DEFAULT_CONFIG;
    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.mode = ESB_MODE_PRX; // Receiver mode to listen and send ACKs
    config.retransmit_delay = 1000;
    config.retransmit_count = 5;
    config.event_handler = simple_esb_event_handler;
    config.bitrate = ESB_BITRATE_2MBPS;
    config.selective_auto_ack = true; // Enable ACK for timing coordination
    config.use_fast_ramp_up = false;

    // Initialize ESB
    err = esb_init(&config);
    if (err)
    {
        LOG_ERR("ESB init failed: %d", err);
        return err;
    }

    // Set addresses - using same pattern as Nordic reference but matching controller
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7}; // RIGHT controller base (pipe 0)
    uint8_t base_addr_1[4] = {0xD4, 0xD4, 0xD4, 0xD4}; // LEFT controller base (pipe 1)
    uint8_t addr_prefix[8] = {0xE7, 0xD4, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

    err = esb_set_base_address_0(base_addr_0);
    if (err)
    {
        LOG_ERR("ESB set base address 0 failed: %d", err);
        return err;
    }

    err = esb_set_base_address_1(base_addr_1);
    if (err)
    {
        LOG_ERR("ESB set base address 1 failed: %d", err);
        return err;
    }

    err = esb_set_prefixes(addr_prefix, 8);
    if (err)
    {
        LOG_ERR("ESB set prefixes failed: %d", err);
        return err;
    }

    // Set RF channel
    err = esb_set_rf_channel(1);
    if (err)
    {
        LOG_ERR("ESB set RF channel failed: %d", err);
        return err;
    }

    // Set radio TX power to maximum
    err = esb_set_tx_power(ESB_TX_POWER_8DBM);
    if (err)
    {
        LOG_ERR("ESB set TX power failed: %d", err);
        return err;
    }

    LOG_INF("ESB configuration complete");

    // Write initial payload (like Nordic PRX reference does)
    struct esb_payload tx_payload = {0};
    tx_payload.length = 8;
    tx_payload.pipe = 0;
    tx_payload.data[0] = 0x10;
    tx_payload.data[1] = 0x11;

    err = esb_write_payload(&tx_payload);
    if (err)
    {
        LOG_ERR("Initial payload write failed: %d", err);
        return err;
    }

    LOG_INF("Setting up for packet reception");

    // Start receiving (like Nordic reference)
    err = esb_start_rx();
    if (err)
    {
        LOG_ERR("ESB start RX failed: %d", err);
        return err;
    }

    LOG_INF("ESB initialized for ACK-based communication - listening for controller data");
    return 0;
}

// Get current controller state
simple_controller_state_t *controller_esb_get_state(void)
{
    return &controller_state;
}

// Check if we have new data
bool controller_esb_has_new_data(void)
{
    // Consider data "new" if we received it within the last 100ms
    uint32_t now = k_uptime_get_32();
    return controller_state.data_received &&
           (now - controller_state.last_ping_time) < 100;
}
