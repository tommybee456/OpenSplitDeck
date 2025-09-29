/*
 * Dual Controller System - Zephyr ESB Implementation
 *
 * This code runs on a Seeed Xiao BLE nRF52840 Sense Plus
 * It can be configured as LEFT or RIGHT controller half
 * Each half has a unique ID and only responds to its specific ping
 *
 * Configuration: Set CONTROLLER_ID below
 * - 0 = RIGHT controller (responds to ping value 0)
 * - 1 = LEFT controller (responds to ping value 1)
 */

// ========================
// CONTROLLER CONFIGURATION
// ========================
#define CONTROLLER_ID 1 // Change to 0 for RIGHT controller, 1 for LEFT controller

// FREEZE DEBUGGING - Temporarily disable components to isolate freezing source
#define FREEZE_DEBUG_DISABLE_ADC 0 // Set to 1 to disable ADC reads
#define FREEZE_DEBUG_DISABLE_IMU 0 // Set to 1 to disable IMU reads

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_saadc.h>
#include <nrfx_saadc.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_ADC_NRFX_SAADC)
#include <nrfx_saadc.h>
#endif
#include <math.h>
#include <esb.h>
#include "IQS7211E.h"
#include "controller_storage.h"
#include "drv2605.h"
#include "display.h"
#include "haptic_driver.h"
#include "imu_driver.h"
#include "button_driver.h"
#include "analog_driver.h"
#include "esb_comm_driver.h"
#include "power_mgmt_driver.h"
#include "IQS7211E_init.h"
// #include "trackpad_driver.h"

LOG_MODULE_REGISTER(controller, LOG_LEVEL_INF);

// LED for status indication
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// Display State Varibles
static display_screen_type_t current_display_screen = DISPLAY_SCREEN_STATUS;
static display_analog_data_t display_analog_data = {0};
// REMOVED display_mutex - was causing priority inversion and 100-160ms thread delays
// Main thread (200Hz) calling display_set_screen competed with display thread (20Hz)

// I2C bus protection - REMOVED due to causing thread delays
// static K_MUTEX_DEFINE(i2c1_mutex);

// Thread health monitoring
static uint32_t trackpad_thread_heartbeat = 0;
static uint32_t display_thread_heartbeat = 0;

// Controller button GPIO definitions (used by button driver)
static const struct gpio_dt_spec stick_click = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 29,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec bumper = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 11,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec start = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 12,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec button_p4 = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 15,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec button_p5 = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 19,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec mode_button = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 1,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec dpad_down = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 13,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec dpad_left = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 14,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec dpad_right = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 15,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec dpad_up = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 3,
    .dt_flags = GPIO_ACTIVE_LOW};

static const struct gpio_dt_spec pad_click = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 7,
    .dt_flags = GPIO_ACTIVE_LOW};

// DRV2605 Haptic motor control pins (NFC pins - requires NFC disabled)
static const struct gpio_dt_spec haptic_trigger = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 9, // P0.09 (NFC1) - External trigger pin
    .dt_flags = GPIO_ACTIVE_HIGH};

static const struct gpio_dt_spec haptic_enable_pin = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 10, // P0.10 (NFC2) - Enable pin
    .dt_flags = GPIO_ACTIVE_HIGH};

// Battery voltage divider enable pin (XIAO nRF52840)
static const struct gpio_dt_spec vbat_enable = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 14,                    // P0.14 - Battery voltage divider enable (LOW=enabled, HIGH=disabled)
    .dt_flags = GPIO_ACTIVE_LOW}; // Active low means setting to 0 enables the divider

// Trackpad RDY pin for interrupt-based reading
static const struct gpio_dt_spec trackpad_rdy = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 5, // P1.05 - RDY pin from trackpad
    .dt_flags = GPIO_ACTIVE_LOW | GPIO_INT_EDGE_FALLING};

// Removed unused trackpad_ready GPIO spec

// I2C for peripherals
static const struct device *i2c_dev;

// IMU sensor - Now using imu_driver library

// ADC for analog inputs - Now using analog_driver library
static const struct device *adc_dev;

// Forward declarations for power management functions
void shutdown_all_peripherals(void);
void wakeup_all_peripherals(void);
void power_factory_reset_handler(void);

// Global flag to track if we need safe shutdown
// Sleep mode state

// Storage for configuration data
static controller_calibration_t controller_calibration;
static controller_bindings_t controller_bindings;
static controller_preferences_t controller_preferences;

// ESB payload structures - now using ESB communication driver
// Controller state
static esb_controller_data_t controller_data = {0};

// Function declarations
void buttons_init(void);                      // Initialize button driver
void adc_init(void);                          // Initialize analog driver
void esb_comm_init(void);                     // Initialize ESB communication driver
void read_button_inputs(void);                // Read buttons using driver
void read_analog_inputs(void);                // Read analog using driver
void print_analog_values(void);               // Debug: print all analog values
void calibrate_analog_inputs(void);           // Calibrate analog using driver
void read_trackpad_inputs(void);              // Read trackpad using driver
void read_imu_inputs(void);                   // Read IMU using driver
esb_comm_status_t send_controller_data(void); // Send data using ESB driver
void power_mgmt_init(void);                   // Initialize power management driver
void ui_init(void);                           // UI placeholder functions
void ui_create_main_screen(void);
void ui_create_menu_screen(void);
void ui_create_calibration_screen(void);
void ui_update_data(void);
void ui_handle_input(void);

// Initialize ESB communication using ESB driver library
void esb_comm_init(void)
{
        LOG_INF("Initializing ESB communication using driver library...");

        // Configure ESB communication
        // Configure timing with controller-specific offset to prevent collisions
        uint32_t base_interval = 5;   // 5ms base for RIGHT controller
        uint32_t retry_interval = 10; // 10ms retry for RIGHT controller

        // LEFT controller gets +2ms offset to stagger transmissions
        if (CONTROLLER_ID == 1)
        {                            // LEFT controller
                base_interval += 2;  // 7ms instead of 5ms
                retry_interval += 2; // 12ms instead of 10ms
        }

        esb_comm_config_t esb_config = {
            .controller_id = CONTROLLER_ID,
            .base_tx_interval_ms = base_interval,
            .retry_interval_ms = retry_interval,
            .rf_channel = 1,    // RF channel 1
            .status_led = &led0 // Use LED0 for status indication
        };

        LOG_INF("Controller %d ESB timing: base=%dms, retry=%dms (offset for collision avoidance)",
                CONTROLLER_ID, base_interval, retry_interval);
        LOG_INF("Note: Actual timing controlled by dongle ACK payload when available");

        esb_comm_status_t status = esb_comm_driver_init(&esb_config);
        if (status != ESB_COMM_STATUS_OK)
        {
                LOG_ERR("ESB communication driver initialization failed: %d", status);
                return;
        }

        // Enable ACK payload based timing
        status = esb_comm_enable_ack_timing(true);
        if (status == ESB_COMM_STATUS_OK)
        {
                LOG_INF("ACK payload timing enabled");
        }
        else
        {
                LOG_WRN("Failed to enable ACK payload timing: %d", status);
        }

        LOG_INF("ESB communication driver initialized successfully");
}

// Send controller data using ESB driver library with ACK payload timing
esb_comm_status_t send_controller_data(void)
{
        // Use the ESB communication driver for transmission
        esb_comm_status_t status = esb_comm_send_data(&controller_data);

        // Process ACK payload data if transmission was successful
        if (status == ESB_COMM_STATUS_OK)
        {
                // Check for rumble data from dongle
                uint8_t left_rumble, right_rumble;
                esb_comm_status_t rumble_status = esb_comm_get_rumble_data(&left_rumble, &right_rumble);

                if (rumble_status == ESB_COMM_STATUS_OK && (left_rumble > 0 || right_rumble > 0))
                {
                        // TODO: Implement rumble motor control when hardware is ready
                        static uint32_t last_rumble_log = 0;
                        uint32_t now = k_uptime_get_32();
                        if ((now - last_rumble_log) > 1000)
                        { // Log every 1 second to avoid spam
                                LOG_DBG("Rumble data received - Left: %d/15, Right: %d/15", left_rumble, right_rumble);
                                last_rumble_log = now;
                        }
                }
        }

        // Track radio busy conditions
        static uint32_t busy_count = 0;
        static uint32_t last_busy_log = 0;
        static uint32_t total_send_attempts = 0;

        total_send_attempts++;

        if (status == ESB_COMM_STATUS_BUSY)
        {
                busy_count++;
                uint32_t now = k_uptime_get_32();
                if ((now - last_busy_log) > 5000)
                { // Log every 5 seconds instead of 1
                        LOG_WRN("Radio busy: %u/%u attempts (%.1f%%) - potential overload",
                                busy_count, total_send_attempts,
                                (double)busy_count * 100.0 / total_send_attempts);
                        last_busy_log = now;
                        // Reset counters for next period
                        busy_count = 0;
                        total_send_attempts = 0;
                }
        }
        else if (status != ESB_COMM_STATUS_OK)
        {
                // Log other ESB errors to debug radio failures
                static uint32_t error_count = 0;
                static uint32_t last_error_log = 0;
                error_count++;

                uint32_t now = k_uptime_get_32();
                if ((now - last_error_log) > 2000) // Log every 2 seconds
                {
                        LOG_ERR("ESB transmission error: %d (count: %u/%u)",
                                status, error_count, total_send_attempts);
                        last_error_log = now;
                }
        }

        // Removed verbose ESB transmission logging
        static uint32_t tx_debug_counter = 0;
        if (status == ESB_COMM_STATUS_OK)
        {
                tx_debug_counter++;
                // Removed trackpad transmission debug logs
        }

        return status;
}

// Simulate different controller inputs
void update_controller_data(void)
{
        // Clear previous input state but preserve controller ID in flags
        controller_data.buttons = 0;
        controller_data.flags &= 0x80; // Keep only controller ID (bit 7), clear all other flags
        // controller_data.stickX = 0;
        // controller_data.stickY = 0;
        // controller_data.trigger = 0;
        controller_data.accelX = 0;
        controller_data.accelY = 0;
        controller_data.accelZ = 0;
        controller_data.gyroX = 0;
        controller_data.gyroY = 0;
        controller_data.gyroZ = 0;

        // Read real inputs using new driver libraries with timing diagnostics
        uint32_t start_cycles, end_cycles, duration_us;

        // Measure button reading time
        start_cycles = k_cycle_get_32();
        read_button_inputs(); // Use button driver library
        end_cycles = k_cycle_get_32();
        duration_us = k_cyc_to_us_floor32(end_cycles - start_cycles);
        if (duration_us > 500)
        { // >0.5ms for button reading is slow (lowered threshold)
                LOG_WRN("SLOW BUTTON READ: %dus", duration_us);
        }

        // Measure analog reading time with DETAILED diagnostics + ROUND-ROBIN READING
        // Only read ONE ADC channel per loop to prevent freezing from multi-channel blocking

        start_cycles = k_cycle_get_32();
#if FREEZE_DEBUG_DISABLE_ADC == 0
        // ADC reading handled by dedicated thread - just get the current data
        read_analog_inputs(); // Use existing analog driver function
        // If we didn't read this loop, analog values remain from last read

#else
        // ADC DISABLED FOR FREEZE DEBUG
        controller_data.stickX = 0;
        controller_data.stickY = 0;
        controller_data.trigger = 0;
#endif
        end_cycles = k_cycle_get_32();
        duration_us = k_cyc_to_us_floor32(end_cycles - start_cycles);

        // Only log timing if we actually read ADC this cycle
        if (duration_us > 1000)
        { // >1ms for analog reading is VERY slow
                LOG_ERR("*** ADC READ TOOK %dus (8ms interval) ***", duration_us);
        }
        else if (duration_us > 500)
        {
                LOG_WRN("ADC READ: %dus (8ms interval)", duration_us);
        }

        // Measure IMU reading time with DETAILED diagnostics
        start_cycles = k_cycle_get_32();
#if FREEZE_DEBUG_DISABLE_IMU == 0
        read_imu_inputs();
#else
        // IMU DISABLED FOR FREEZE DEBUG
        controller_data.accelX = 0;
        controller_data.accelY = 0;
        controller_data.accelZ = 0;
        controller_data.gyroX = 0;
        controller_data.gyroY = 0;
        controller_data.gyroZ = 0;
#endif
        end_cycles = k_cycle_get_32();
        duration_us = k_cyc_to_us_floor32(end_cycles - start_cycles);
        if (duration_us > 5000)
        { // >5ms for IMU reading is VERY slow
                LOG_ERR("*** FREEZE SUSPECT: IMU READ %dus ***", duration_us);
        }
        else if (duration_us > 2000)
        { // >2ms for IMU reading is slow (lowered threshold)
                LOG_WRN("SLOW IMU READ: %dus", duration_us);
        }
}

// Thread stacks
K_THREAD_STACK_DEFINE(trackpad_thread_stack, 1024);
K_THREAD_STACK_DEFINE(display_thread_stack, 1024);

// Thread control blocks
static struct k_thread trackpad_thread_data;
static struct k_thread display_thread_data;

// Semaphore for trackpad RDY interrupt
static K_SEM_DEFINE(trackpad_rdy_sem, 0, 1);

// Trackpad RDY interrupt callback
static struct gpio_callback trackpad_rdy_cb_data;

// Add this to your main.c - hybrid Arduino init + Zephyr reading
static iqs7211e_instance_t trackpad_instance;
static bool trackpad_arduino_initialized = false;

bool init_trackpad_arduino_hybrid(void)
{
        LOG_INF("=== ARDUINO HYBRID TRACKPAD INIT ===");

        // Initialize the Arduino-style IQS7211E library
        iqs7211e_begin(&trackpad_instance, 0x56, 5); // Address 0x56, RDY pin 5
        LOG_INF("Arduino IQS7211E begin() called");

        // Run the full Arduino initialization sequence
        uint32_t init_start = k_uptime_get_32();
        while (!trackpad_arduino_initialized && (k_uptime_get_32() - init_start) < 30000)
        { // 30 second timeout
                iqs7211e_run(&trackpad_instance);

                // Check if initialization is complete
                if (trackpad_instance.iqs7211e_state.init_state == IQS7211E_INIT_DONE)
                {
                        trackpad_arduino_initialized = true;
                        LOG_INF("Arduino IQS7211E initialization complete!");
                        break;
                }

                // Log progress every 2 seconds - reduced verbosity
                static uint32_t last_progress = 0;
                uint32_t now = k_uptime_get_32();
                if ((now - last_progress) > 5000) // Increased from 2 to 5 seconds
                {
                        // Reduced Arduino init progress logging
                        last_progress = now;
                }

                k_sleep(K_MSEC(100));
        }

        if (!trackpad_arduino_initialized)
        {
                LOG_ERR("Arduino IQS7211E initialization timeout!");
                return false;
        }

        // Now the trackpad is fully initialized with Arduino settings
        // We can switch to simple Zephyr I2C reads for coordinates
        LOG_INF("=== SWITCHING TO ZEPHYR I2C READS ===");

        // Test a simple coordinate read to make sure it works
        uint8_t coord_data[8];
        int ret = i2c_write_read(i2c_dev, 0x56, "\x10", 1, coord_data, 8);
        if (ret == 0)
        {
                // Variables used to verify read worked, but values not logged
                (void)(coord_data[0] | (coord_data[1] << 8)); // Suppress unused warning
                (void)(coord_data[2] | (coord_data[3] << 8)); // Suppress unused warning
                // Removed test coordinate logging
        }
        else
        {
                LOG_WRN("Test coordinate read failed: %d", ret);
        }

        LOG_INF("=== ARDUINO HYBRID TRACKPAD INIT COMPLETE ===");
        return true;
}

// Simple coordinate reading function with precise I2C timing diagnostics
bool read_trackpad_coordinates_simple(uint16_t *x, uint16_t *y)
{
        // Use cycle counter for microsecond precision timing
        uint32_t cycles_start = k_cycle_get_32();

        uint8_t coord_data[8];
        int ret = i2c_write_read(i2c_dev, 0x56, "\x10", 1, coord_data, 8);

        uint32_t cycles_end = k_cycle_get_32();
        uint32_t cycles_elapsed = cycles_end - cycles_start;

        // Convert cycles to microseconds (64MHz system clock)
        uint32_t microseconds = k_cyc_to_us_floor32(cycles_elapsed);

        // Log if I2C operation takes longer than expected (reduced spam)
        static uint32_t last_warning = 0;
        if (microseconds > 1000 && (k_uptime_get_32() - last_warning) > 5000)
        {
                LOG_WRN("I2C operation took %dus (%dms) - actual timing", microseconds, microseconds / 1000);
                last_warning = k_uptime_get_32();
        }

        if (ret == 0)
        {
                *x = coord_data[0] | (coord_data[1] << 8);
                *y = coord_data[2] | (coord_data[3] << 8);
                return true;
        }

        return false;
}

// Add this to your main.c - minimal direct trackpad init
bool init_trackpad_minimal(void)
{
        LOG_INF("=== MINIMAL TRACKPAD INIT ===");

        // Test I2C communication first
        uint8_t test_data;
        int ret = i2c_read(i2c_dev, &test_data, 1, 0x56);
        if (ret != 0)
        {
                LOG_ERR("Trackpad not responding at 0x56: %d", ret);
                return false;
        }
        LOG_INF("Trackpad responds to I2C");

        k_sleep(K_MSEC(100));

        // 1. Read product number to verify device
        uint8_t prod_data[2];
        ret = i2c_write_read(i2c_dev, 0x56, "\x00", 1, prod_data, 2); // Read from 0x00
        if (ret != 0)
        {
                LOG_ERR("Failed to read product number: %d", ret);
                return false;
        }

        uint16_t prod_num = prod_data[0] | (prod_data[1] << 8);
        LOG_INF("Product number: 0x%04X", prod_num);

        if (prod_num != 0x0458)
        { // IQS7211E product number
                LOG_ERR("Wrong product number, expected 0x0458");
                return false;
        }

        // 2. Software reset
        LOG_INF("Performing software reset...");
        uint8_t reset_cmd[] = {0x34, 0x02}; // Write 0x02 to address 0x34 (SW_RESET_BIT)
        ret = i2c_write(i2c_dev, reset_cmd, sizeof(reset_cmd), 0x56);
        if (ret != 0)
        {
                LOG_ERR("Software reset failed: %d", ret);
                return false;
        }

        k_sleep(K_MSEC(100)); // Wait for reset
        LOG_INF("Software reset complete");

        // 3. Write ALL settings directly (from IQS7211E_init.h)
        LOG_INF("Writing all settings directly...");

        // ALP Compensation (0x1F-0x20)
        uint8_t alp_comp[] = {0x1F, ALP_COMPENSATION_A_0, ALP_COMPENSATION_A_1,
                              ALP_COMPENSATION_B_0, ALP_COMPENSATION_B_1};
        ret = i2c_write(i2c_dev, alp_comp, sizeof(alp_comp), 0x56);
        if (ret != 0)
                LOG_WRN("ALP compensation write failed: %d", ret);

        // ATI Settings (0x21-0x2E) - Big block write
        uint8_t ati_settings[] = {
            0x21, // Start address
            TP_ATI_MULTIPLIERS_DIVIDERS_0, TP_ATI_MULTIPLIERS_DIVIDERS_1,
            TP_COMPENSATION_DIV, TP_REF_DRIFT_LIMIT,
            TP_ATI_TARGET_0, TP_ATI_TARGET_1,
            TP_MIN_COUNT_REATI_0, TP_MIN_COUNT_REATI_1,
            ALP_ATI_MULTIPLIERS_DIVIDERS_0, ALP_ATI_MULTIPLIERS_DIVIDERS_1,
            ALP_COMPENSATION_DIV, ALP_LTA_DRIFT_LIMIT,
            ALP_ATI_TARGET_0, ALP_ATI_TARGET_1};
        ret = i2c_write(i2c_dev, ati_settings, sizeof(ati_settings), 0x56);
        if (ret != 0)
                LOG_WRN("ATI settings write failed: %d", ret);

        k_sleep(K_MSEC(100));

        // Report Rates (0x28-0x32)
        uint8_t report_rates[] = {
            0x28, // Start address
            ACTIVE_MODE_REPORT_RATE_0, ACTIVE_MODE_REPORT_RATE_1,
            IDLE_TOUCH_MODE_REPORT_RATE_0, IDLE_TOUCH_MODE_REPORT_RATE_1,
            IDLE_MODE_REPORT_RATE_0, IDLE_MODE_REPORT_RATE_1,
            LP1_MODE_REPORT_RATE_0, LP1_MODE_REPORT_RATE_1,
            LP2_MODE_REPORT_RATE_0, LP2_MODE_REPORT_RATE_1,
            ACTIVE_MODE_TIMEOUT_0, ACTIVE_MODE_TIMEOUT_1,
            IDLE_TOUCH_MODE_TIMEOUT_0, IDLE_TOUCH_MODE_TIMEOUT_1,
            IDLE_MODE_TIMEOUT_0, IDLE_MODE_TIMEOUT_1,
            LP1_MODE_TIMEOUT_0, LP1_MODE_TIMEOUT_1,
            REATI_RETRY_TIME, REF_UPDATE_TIME,
            I2C_TIMEOUT_0, I2C_TIMEOUT_1};
        ret = i2c_write(i2c_dev, report_rates, sizeof(report_rates), 0x56);
        if (ret != 0)
                LOG_WRN("Report rates write failed: %d", ret);

        k_sleep(K_MSEC(100));
        // System Control (0x33-0x35) - THE CRITICAL ONE
        uint8_t sys_control[] = {
            0x33, // Start address
            SYSTEM_CONTROL_0, SYSTEM_CONTROL_1,
            CONFIG_SETTINGS0, CONFIG_SETTINGS1,
            OTHER_SETTINGS_0, OTHER_SETTINGS_1};
        ret = i2c_write(i2c_dev, sys_control, sizeof(sys_control), 0x56);
        if (ret != 0)
        {
                LOG_ERR("System control write failed: %d", ret);
                return false; // This is critical
        }
        LOG_INF("System control settings written successfully");

        k_sleep(K_MSEC(100));
        // Continue with other settings...
        // (Add more setting blocks as needed)

        // 4. Acknowledge reset
        LOG_INF("Acknowledging reset...");
        uint8_t ack_reset[] = {0x33, SYSTEM_CONTROL_0 | 0x80}; // Set ACK_RESET_BIT
        ret = i2c_write(i2c_dev, ack_reset, sizeof(ack_reset), 0x56);
        if (ret != 0)
        {
                LOG_ERR("Reset acknowledge failed: %d", ret);
                return false;
        }

        // 5. Start ATI
        LOG_INF("Starting ATI...");
        uint8_t start_ati[] = {0x33, SYSTEM_CONTROL_0 | 0x20}; // Set TP_RE_ATI_BIT
        ret = i2c_write(i2c_dev, start_ati, sizeof(start_ati), 0x56);
        if (ret != 0)
        {
                LOG_ERR("ATI start failed: %d", ret);
                return false;
        }

        // 6. Wait for ATI completion (with timeout)
        LOG_INF("Waiting for ATI completion...");
        uint32_t ati_start = k_uptime_get_32();
        bool ati_done = false;

        while ((k_uptime_get_32() - ati_start) < 5000)
        { // 5 second timeout
                uint8_t info_flags[2];
                ret = i2c_write_read(i2c_dev, 0x56, "\x0F", 1, info_flags, 2); // Read INFO_FLAGS
                if (ret == 0)
                {
                        if (info_flags[0] & 0x10)
                        { // IQS7211E_RE_ATI_OCCURRED_BIT
                                LOG_INF("ATI completed successfully!");
                                ati_done = true;
                                break;
                        }
                }
                k_sleep(K_MSEC(100));
        }

        if (!ati_done)
        {
                LOG_WRN("ATI timeout, but continuing anyway");
        }

        k_sleep(K_MSEC(100));

        // 7. Set event mode
        LOG_INF("Setting event mode...");
        uint8_t event_mode[] = {0x35, CONFIG_SETTINGS0, CONFIG_SETTINGS1 | 0x01}; // Set EVENT_MODE_BIT
        ret = i2c_write(i2c_dev, event_mode, sizeof(event_mode), 0x56);
        if (ret != 0)
        {
                LOG_WRN("Event mode set failed: %d", ret);
        }

        k_sleep(K_MSEC(100));

        LOG_INF("=== MINIMAL TRACKPAD INIT COMPLETE ===");
        return true;
}

// Replace your init_trackpad_enhanced() function with this C-compatible version:
bool init_trackpad_enhanced(void)
{
        LOG_INF("=== ARDUINO-STYLE TRACKPAD INIT ===");

        // Test I2C communication first
        uint8_t test_data;
        int ret = i2c_read(i2c_dev, &test_data, 1, 0x56);
        if (ret != 0)
        {
                LOG_ERR("Trackpad not responding at 0x56: %d", ret);
                return false;
        }
        LOG_INF("Trackpad responds to I2C");

        k_sleep(K_MSEC(100));

        // 1. Read product number to verify device
        uint8_t prod_data[2];
        ret = i2c_write_read(i2c_dev, 0x56, "\x00", 1, prod_data, 2);
        if (ret != 0)
        {
                LOG_ERR("Failed to read product number: %d", ret);
                return false;
        }

        uint16_t prod_num = prod_data[0] | (prod_data[1] << 8);
        LOG_INF("Product number: 0x%04X", prod_num);

        if (prod_num != 0x0458)
        {
                LOG_ERR("Wrong product number, expected 0x0458");
                return false;
        }

        // 2. Software reset - CORRECT ADDRESS
        LOG_INF("Performing software reset...");
        uint8_t reset_cmd[] = {0x51, 0x02}; // Write to SYSTEM_CONTROL_1, SW_RESET_BIT
        ret = i2c_write(i2c_dev, reset_cmd, sizeof(reset_cmd), 0x56);
        if (ret != 0)
        {
                LOG_ERR("Software reset failed: %d", ret);
                return false;
        }
        k_sleep(K_MSEC(250)); // Wait for reset
        LOG_INF("Software reset complete");

        // 3. Write settings using C macro (not C++ lambda)
        LOG_INF("Writing settings in Arduino order...");

// Helper macro for single writes with Arduino-like error handling
#define WRITE_SINGLE(addr, value, desc)                                                                  \
        do                                                                                               \
        {                                                                                                \
                uint8_t cmd[] = {addr, value};                                                           \
                int result = i2c_write(i2c_dev, cmd, 2, 0x56);                                           \
                if (result != 0)                                                                         \
                {                                                                                        \
                        LOG_ERR("Failed to write %s (0x%02X to 0x%02X): %d", desc, value, addr, result); \
                        return false;                                                                    \
                }                                                                                        \
                k_sleep(K_MSEC(10));                                                                     \
        } while (0)

        // WRITE IN EXACT ARDUINO ORDER

        // 1. ALP Compensation (0x1F-0x22)
        WRITE_SINGLE(0x1F, ALP_COMPENSATION_A_0, "ALP_COMP_A0");
        WRITE_SINGLE(0x20, ALP_COMPENSATION_A_1, "ALP_COMP_A1");
        WRITE_SINGLE(0x21, ALP_COMPENSATION_B_0, "ALP_COMP_B0");
        WRITE_SINGLE(0x22, ALP_COMPENSATION_B_1, "ALP_COMP_B1");
        k_sleep(K_MSEC(50));

        // 2. ATI Settings (0x23-0x30)
        WRITE_SINGLE(0x23, TP_ATI_MULTIPLIERS_DIVIDERS_0, "TP_ATI_MUL_DIV_0");
        WRITE_SINGLE(0x24, TP_ATI_MULTIPLIERS_DIVIDERS_1, "TP_ATI_MUL_DIV_1");
        WRITE_SINGLE(0x25, TP_COMPENSATION_DIV, "TP_COMP_DIV");
        WRITE_SINGLE(0x26, TP_REF_DRIFT_LIMIT, "TP_REF_DRIFT");
        WRITE_SINGLE(0x27, TP_ATI_TARGET_0, "TP_ATI_TARGET_0");
        WRITE_SINGLE(0x28, TP_ATI_TARGET_1, "TP_ATI_TARGET_1");
        WRITE_SINGLE(0x29, TP_MIN_COUNT_REATI_0, "TP_MIN_COUNT_0");
        WRITE_SINGLE(0x2A, TP_MIN_COUNT_REATI_1, "TP_MIN_COUNT_1");
        WRITE_SINGLE(0x2B, ALP_ATI_MULTIPLIERS_DIVIDERS_0, "ALP_ATI_MUL_0");
        WRITE_SINGLE(0x2C, ALP_ATI_MULTIPLIERS_DIVIDERS_1, "ALP_ATI_MUL_1");
        WRITE_SINGLE(0x2D, ALP_COMPENSATION_DIV, "ALP_COMP_DIV");
        WRITE_SINGLE(0x2E, ALP_LTA_DRIFT_LIMIT, "ALP_LTA_DRIFT");
        WRITE_SINGLE(0x2F, ALP_ATI_TARGET_0, "ALP_ATI_TARGET_0");
        WRITE_SINGLE(0x30, ALP_ATI_TARGET_1, "ALP_ATI_TARGET_1");
        k_sleep(K_MSEC(100));

        // 3. Report Rates (0x31-0x46)
        WRITE_SINGLE(0x31, ACTIVE_MODE_REPORT_RATE_0, "ACTIVE_RR_0");
        WRITE_SINGLE(0x32, ACTIVE_MODE_REPORT_RATE_1, "ACTIVE_RR_1");
        WRITE_SINGLE(0x33, IDLE_TOUCH_MODE_REPORT_RATE_0, "IDLE_TOUCH_RR_0");
        WRITE_SINGLE(0x34, IDLE_TOUCH_MODE_REPORT_RATE_1, "IDLE_TOUCH_RR_1");
        WRITE_SINGLE(0x35, IDLE_MODE_REPORT_RATE_0, "IDLE_RR_0");
        WRITE_SINGLE(0x36, IDLE_MODE_REPORT_RATE_1, "IDLE_RR_1");
        WRITE_SINGLE(0x37, LP1_MODE_REPORT_RATE_0, "LP1_RR_0");
        WRITE_SINGLE(0x38, LP1_MODE_REPORT_RATE_1, "LP1_RR_1");
        WRITE_SINGLE(0x39, LP2_MODE_REPORT_RATE_0, "LP2_RR_0");
        WRITE_SINGLE(0x3A, LP2_MODE_REPORT_RATE_1, "LP2_RR_1");
        WRITE_SINGLE(0x3B, ACTIVE_MODE_TIMEOUT_0, "ACTIVE_TO_0");
        WRITE_SINGLE(0x3C, ACTIVE_MODE_TIMEOUT_1, "ACTIVE_TO_1");
        WRITE_SINGLE(0x3D, IDLE_TOUCH_MODE_TIMEOUT_0, "IDLE_TOUCH_TO_0");
        WRITE_SINGLE(0x3E, IDLE_TOUCH_MODE_TIMEOUT_1, "IDLE_TOUCH_TO_1");
        WRITE_SINGLE(0x3F, IDLE_MODE_TIMEOUT_0, "IDLE_TO_0");
        WRITE_SINGLE(0x40, IDLE_MODE_TIMEOUT_1, "IDLE_TO_1");
        WRITE_SINGLE(0x41, LP1_MODE_TIMEOUT_0, "LP1_TO_0");
        WRITE_SINGLE(0x42, LP1_MODE_TIMEOUT_1, "LP1_TO_1");
        WRITE_SINGLE(0x43, REATI_RETRY_TIME, "REATI_RETRY");
        WRITE_SINGLE(0x44, REF_UPDATE_TIME, "REF_UPDATE");
        WRITE_SINGLE(0x45, I2C_TIMEOUT_0, "I2C_TO_0");
        WRITE_SINGLE(0x46, I2C_TIMEOUT_1, "I2C_TO_1");
        k_sleep(K_MSEC(100));

        // 4. System Control Settings (0x50-0x55) - THE CRITICAL ONES
        LOG_INF("Writing CRITICAL system control settings...");
        WRITE_SINGLE(0x50, SYSTEM_CONTROL_0, "SYS_CTRL_0");
        WRITE_SINGLE(0x51, SYSTEM_CONTROL_1, "SYS_CTRL_1");
        WRITE_SINGLE(0x52, CONFIG_SETTINGS0, "CONFIG_0");
        WRITE_SINGLE(0x53, CONFIG_SETTINGS1, "CONFIG_1");
        WRITE_SINGLE(0x54, OTHER_SETTINGS_0, "OTHER_0");
        WRITE_SINGLE(0x55, OTHER_SETTINGS_1, "OTHER_1");
        k_sleep(K_MSEC(100));

        // 6. Acknowledge reset (CORRECT address)
        LOG_INF("Acknowledging reset...");
        WRITE_SINGLE(0x50, SYSTEM_CONTROL_0 | 0x80, "ACK_RESET");
        k_sleep(K_MSEC(100));

        // 7. Start ATI (CORRECT address)
        LOG_INF("Starting ATI...");
        WRITE_SINGLE(0x50, SYSTEM_CONTROL_0 | 0x20, "START_ATI");
        k_sleep(K_MSEC(100));

        // 8. Wait for ATI with proper polling
        LOG_INF("Waiting for ATI completion...");
        uint32_t ati_start = k_uptime_get_32();
        bool ati_done = false;

        while ((k_uptime_get_32() - ati_start) < 10000)
        { // 10 second timeout
                uint8_t info_flags[2];
                ret = i2c_write_read(i2c_dev, 0x56, "\x0F", 1, info_flags, 2);
                if (ret == 0)
                {
                        if (info_flags[0] & 0x10)
                        { // ATI_OCCURRED_BIT
                                LOG_INF("ATI completed successfully!");
                                ati_done = true;
                                break;
                        }
                }
                k_sleep(K_MSEC(200));
        }

        if (!ati_done)
        {
                LOG_WRN("ATI timeout, but continuing anyway");
        }

        // 9. Set event mode (CORRECT address)
        LOG_INF("Setting event mode...");
        WRITE_SINGLE(0x53, CONFIG_SETTINGS1 | 0x01, "EVENT_MODE");
        k_sleep(K_MSEC(100));

#undef WRITE_SINGLE

        LOG_INF("=== ARDUINO-STYLE TRACKPAD INIT COMPLETE ===");
        return true;
}

// Global haptic pulse management variables (moved from function-local)
static uint32_t haptic_pulse_start = 0;
static bool haptic_pulse_active = false;

// Trackpad RDY interrupt handler
void trackpad_rdy_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
        ARG_UNUSED(dev);
        ARG_UNUSED(cb);
        ARG_UNUSED(pins);

        // Signal the trackpad thread that data is ready
        k_sem_give(&trackpad_rdy_sem);
}

// Add this function to trigger haptic via GPIO pin
void check_trackpad_haptic_feedback(uint16_t new_x, uint16_t new_y)
{
        static uint16_t last_x = 0;
        static uint16_t last_y = 0;
        static bool first_read = true;

        // Skip haptic on first read
        if (first_read)
        {
                last_x = new_x;
                last_y = new_y;
                first_read = false;
                return;
        }

        // Calculate distance moved
        int16_t x_diff = abs((int16_t)new_x - (int16_t)last_x);
        int16_t y_diff = abs((int16_t)new_y - (int16_t)last_y);

        // Check if movement exceeds threshold (60 units in either direction)
        if (x_diff >= 60 || y_diff >= 60)
        {
                if (!haptic_pulse_active)
                {
                        // Start haptic pulse
                        gpio_pin_set_dt(&haptic_trigger, 1);
                        haptic_pulse_start = k_uptime_get_32();
                        haptic_pulse_active = true;

                        // Update last position
                        last_x = new_x;
                        last_y = new_y;
                }
        }
}

// Separate function to handle haptic pulse completion - called every trackpad thread loop
void update_haptic_pulse_state(void)
{
        // Handle haptic pulse completion (called every trackpad thread iteration)
        if (haptic_pulse_active && (k_uptime_get_32() - haptic_pulse_start) >= 1) // 1ms pulse
        {
                gpio_pin_set_dt(&haptic_trigger, 0);
                haptic_pulse_active = false;
        }
}

// Update your trackpad thread:
void trackpad_thread_entry(void *p1, void *p2, void *p3)
{
        ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);

        // Wait for system to boot up and I2C to stabilize
        k_sleep(K_MSEC(3000));

        // Use Arduino library for initialization
        if (!init_trackpad_arduino_hybrid())
        {
                LOG_ERR("Arduino hybrid trackpad init failed");
                return;
        }

        // Configure RDY pin for interrupt-based reading
        if (!gpio_is_ready_dt(&trackpad_rdy))
        {
                LOG_ERR("Trackpad RDY pin not ready");
                // Fall back to polling mode
                LOG_INF("Falling back to polling mode...");
                goto polling_mode;
        }

        // Configure RDY pin as input with interrupt
        int ret = gpio_pin_configure_dt(&trackpad_rdy, GPIO_INPUT);
        if (ret != 0)
        {
                LOG_ERR("Failed to configure RDY pin: %d", ret);
                goto polling_mode;
        }

        // Setup interrupt callback
        gpio_init_callback(&trackpad_rdy_cb_data, trackpad_rdy_interrupt_handler, BIT(trackpad_rdy.pin));
        ret = gpio_add_callback(trackpad_rdy.port, &trackpad_rdy_cb_data);
        if (ret != 0)
        {
                LOG_ERR("Failed to add RDY callback: %d", ret);
                goto polling_mode;
        }

        // Enable interrupt - TRY FALLING EDGE instead
        LOG_INF("Trying FALLING EDGE interrupt (data ready when RDY goes LOW)...");
        ret = gpio_pin_interrupt_configure_dt(&trackpad_rdy, GPIO_INT_EDGE_FALLING);
        if (ret != 0)
        {
                LOG_ERR("Failed to configure RDY interrupt: %d", ret);
                goto polling_mode;
        }

        LOG_INF("RDY pin interrupt configured - using interrupt-based trackpad reading");

        // Interrupt-based reading loop
        while (1)
        {
                // Update thread heartbeat for monitoring
                trackpad_thread_heartbeat = k_uptime_get_32();

                // Update haptic pulse state every loop iteration (ensures pulse completion)
                update_haptic_pulse_state();

                // Wait for RDY interrupt (blocks until data is ready)
                if (k_sem_take(&trackpad_rdy_sem, K_MSEC(100)) == 0)
                {
                        k_usleep(50);
                        // RDY interrupt occurred - data is ready
                        uint16_t x, y;

                        if (read_trackpad_coordinates_simple(&x, &y))
                        {
                                // Check for valid coordinates
                                if (x != 0xFFFF && y != 0xFFFF && (x != 0 || y != 0))
                                {
                                        // Check for haptic feedback before updating controller data
                                        check_trackpad_haptic_feedback(x, y);

                                        controller_data.padX = x;
                                        controller_data.padY = y;
                                }
                                else
                                {
                                        controller_data.padX = 0;
                                        controller_data.padY = 0;
                                }
                        }
                        else
                        {
                                // I2C read failed - set coordinates to 0
                                controller_data.padX = 0;
                                controller_data.padY = 0;
                        }
                }
                else
                {
                        // Timeout - no interrupt in 100ms, consider no touch
                        controller_data.padX = 0;
                        controller_data.padY = 0;
                }
        }

polling_mode:
        LOG_INF("Starting polling mode coordinate reading...");

        // Fallback to polling mode if interrupt setup failed
        while (1)
        {
                // Update thread heartbeat for monitoring
                trackpad_thread_heartbeat = k_uptime_get_32();

                // Update haptic pulse state every loop iteration (ensures pulse completion)
                update_haptic_pulse_state();

                uint16_t x, y;

                if (read_trackpad_coordinates_simple(&x, &y))
                {
                        // Check for valid coordinates
                        if (x != 0xFFFF && y != 0xFFFF && (x != 0 || y != 0))
                        {
                                // Check for haptic feedback before updating controller data
                                check_trackpad_haptic_feedback(x, y);

                                controller_data.padX = x;
                                controller_data.padY = y;
                        }
                        else
                        {
                                controller_data.padX = 0;
                                controller_data.padY = 0;
                        }
                }
                else
                {
                        // I2C read failed - set coordinates to 0
                        controller_data.padX = 0;
                        controller_data.padY = 0;
                }

                k_sleep(K_MSEC(11)); // 60Hz polling - thread will be scheduled properly
        }
}

// Display thread function
void display_thread_entry(void *p1, void *p2, void *p3)
{
        ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);

        // Wait for system to boot up
        k_sleep(K_MSEC(500));

        while (1)
        {
                // Update thread heartbeat for monitoring
                display_thread_heartbeat = k_uptime_get_32();

                // No mutex needed - display operations are thread-safe without blocking

                // Handle different screen types
                switch (current_display_screen)
                {
                case DISPLAY_SCREEN_STATUS:
                        {
                                // Get current battery voltage for status display
                                uint16_t battery_mv = 0;
                                if (analog_driver_get_battery_voltage(&battery_mv) == ANALOG_STATUS_OK) {
                                        display_show_status_screen_with_battery(battery_mv);
                                } else {
                                        display_show_status_screen(); // Fallback without battery
                                }
                        }
                        break;

                case DISPLAY_SCREEN_ANALOG:
                        display_show_analog_screen(display_analog_data.stick_x,
                                                   display_analog_data.stick_y,
                                                   display_analog_data.trigger);
                        break;

                case DISPLAY_SCREEN_MENU:
                        // Add menu screen here
                        break;

                default:
                        {
                                // Get current battery voltage for default status display
                                uint16_t battery_mv = 0;
                                if (analog_driver_get_battery_voltage(&battery_mv) == ANALOG_STATUS_OK) {
                                        display_show_status_screen_with_battery(battery_mv);
                                } else {
                                        display_show_status_screen(); // Fallback without battery
                                }
                        }
                        break;
                }

                // Display operations completed - no mutex unlock needed

                // Display thread runs at 10Hz
                k_sleep(K_MSEC(100));
        }
}

// Function to safely change display screens (now lockless):
void display_set_screen(display_screen_type_t screen_type, void *data)
{
        // No mutex needed - simple variable updates are atomic
        current_display_screen = screen_type;

        if (screen_type == DISPLAY_SCREEN_ANALOG && data != NULL)
        {
                memcpy(&display_analog_data, data, sizeof(display_analog_data_t));
        }

        // No mutex unlock needed
}

// Initialize all controller buttons using button driver library
void buttons_init(void)
{
        LOG_INF("Initializing controller buttons using button driver library...");

        // Initialize button driver with all GPIO specifications
        button_status_t status = button_driver_init(
            &stick_click,
            &bumper,
            &start,
            &button_p4,
            &button_p5,
            &mode_button,
            &dpad_down,
            &dpad_left,
            &dpad_right,
            &dpad_up,
            &pad_click);

        if (status != BUTTON_STATUS_OK)
        {
                LOG_ERR("Button driver initialization failed: %d", status);
                return;
        }

        LOG_INF("Button driver initialized successfully with haptic feedback");
}

// Initialize ADC using analog driver library
void adc_init(void)
{
        LOG_INF("Initializing ADC using analog driver library...");

        // Get ADC device
        adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
        if (!device_is_ready(adc_dev))
        {
                LOG_ERR("ADC device not ready");
                adc_dev = NULL;
                return;
        }

        // Initialize analog driver
        analog_status_t status = analog_driver_init(adc_dev);
        if (status != ANALOG_STATUS_OK)
        {
                LOG_ERR("Analog driver initialization failed: %d", status);
                adc_dev = NULL;
                return;
        }

        LOG_INF("Analog driver initialized successfully - StickX(P0.02), StickY(P0.03), Trigger(P0.28) ready");

        // Start the ADC reading thread
        status = analog_driver_start_thread();
        if (status != ANALOG_STATUS_OK)
        {
                LOG_ERR("Failed to start ADC thread: %d", status);
        }
        else
        {
                LOG_INF("ADC thread started successfully");
        }
}

// Read physical button inputs using button driver library
void read_button_inputs(void)
{
        // Scan all buttons
        button_status_t status = button_driver_scan();
        if (status != BUTTON_STATUS_OK)
        {
                return; // Skip if scan failed
        }

        // Get button data in controller format
        button_data_t button_data;
        status = button_driver_get_data(&button_data);
        if (status == BUTTON_STATUS_OK)
        {
                // Check for trackpad click haptic feedback (using existing button data)
                static bool last_pad_click_state = false;
                bool current_pad_click = (button_data.buttons & 0x40) != 0; // Bit 6 = pad click

                // Trigger haptic on trackpad click (rising edge)
                if (current_pad_click && !last_pad_click_state && haptic_is_available())
                {
                        gpio_pin_set_dt(&haptic_trigger, 1);
                        k_sleep(K_USEC(100));
                        gpio_pin_set_dt(&haptic_trigger, 0);
                        LOG_DBG("Trackpad click haptic triggered");
                }

                last_pad_click_state = current_pad_click;
                // Update controller data structure
                controller_data.buttons = button_data.buttons;
                controller_data.flags |= (button_data.flags & 0x7F); // Merge flags (preserve other bits)
        }
}

// Read analog stick and trigger inputs using analog driver library (thread-based)
void read_analog_inputs(void)
{
        // Get controller-format analog data (thread reads continuously in background)
        analog_controller_data_t analog_data;
        analog_status_t status = analog_driver_get_controller_data(&analog_data);
        if (status == ANALOG_STATUS_OK)
        {
                // Update controller data structure directly
                controller_data.stickX = analog_data.stick_x;
                controller_data.stickY = analog_data.stick_y;
                controller_data.trigger = analog_data.trigger;
        }
}

// Debug function to print all analog values for calibration purposes
void print_analog_values(void)
{
        if (!analog_driver_is_initialized())
        {
                printk("Analog driver not initialized\n");
                return;
        }

        // Get the current raw values from the thread-based ADC
        int16_t stick_x_raw, stick_y_raw, trigger_raw;

        analog_driver_get_raw_value(ANALOG_CHANNEL_STICK_X, &stick_x_raw);
        analog_driver_get_raw_value(ANALOG_CHANNEL_STICK_Y, &stick_y_raw);
        analog_driver_get_raw_value(ANALOG_CHANNEL_TRIGGER, &trigger_raw);

        // Simple output - just the raw values that read_analog_inputs() sees
        static uint32_t print_counter = 0;
        print_counter++;

        // Print header every 20 readings
        if (print_counter % 20 == 1)
        {
                printk("\n=== RAW ANALOG VALUES (what read_analog_inputs sees) ===\n");
                printk("Count | StickX | StickY | Trigger |\n");
                printk("------+--------+--------+---------+\n");
        }

        printk("%5d | %6d | %6d | %7d |\n", print_counter, stick_x_raw, stick_y_raw, trigger_raw);

        // Summary every 50 readings
        if (print_counter % 50 == 0)
        {
                printk("\n--- MOVE YOUR CONTROLS TO SEE MIN/MAX RANGES ---\n");
                printk("StickX: %d (move stick left/right to see range)\n", stick_x_raw);
                printk("StickY: %d (move stick up/down to see range)\n", stick_y_raw);
                printk("Trigger: %d (pull trigger to see range)\n", trigger_raw);
                printk("--- USE THESE VALUES FOR YOUR DEFAULTS ---\n\n");
        }
}

// Calibrate analog inputs using analog driver library
void calibrate_analog_inputs(void)
{
        if (!analog_driver_is_initialized())
        {
                LOG_WRN("Analog driver not available for calibration");
                return;
        }

        LOG_INF("Starting analog input calibration using analog driver...");

        // Perform full calibration sequence
        analog_status_t status = analog_driver_full_calibration(2000); // 2 second delay between steps

        if (status == ANALOG_STATUS_OK)
        {
                LOG_INF("Analog calibration completed successfully");

                // Save calibration data to storage
                analog_calibration_t stick_x_cal, stick_y_cal, trigger_cal;

                if (analog_driver_get_calibration(ANALOG_CHANNEL_STICK_X, &stick_x_cal) == ANALOG_STATUS_OK)
                {
                        controller_calibration.stick_center_x = stick_x_cal.center_value;
                        controller_calibration.stick_deadzone = stick_x_cal.deadzone;
                }

                if (analog_driver_get_calibration(ANALOG_CHANNEL_STICK_Y, &stick_y_cal) == ANALOG_STATUS_OK)
                {
                        controller_calibration.stick_center_y = stick_y_cal.center_value;
                }

                if (analog_driver_get_calibration(ANALOG_CHANNEL_TRIGGER, &trigger_cal) == ANALOG_STATUS_OK)
                {
                        controller_calibration.trigger_min = trigger_cal.min_value;
                        controller_calibration.trigger_max = trigger_cal.max_value;
                }

                controller_calibration.stick_calibrated = true;
                controller_storage_save_calibration(&controller_calibration);
                LOG_INF("Calibration saved to flash storage");
        }
        else
        {
                LOG_ERR("Analog calibration failed: %d", status);
        }
}

// Read IMU sensor data using imu_driver library
void read_imu_inputs(void)
{
        // First read raw data to ensure filter is initialized
        imu_raw_data_t raw_data;
        int raw_ret = imu_read_raw_data(&raw_data);

        if (raw_ret != 0)
        {
                // Set to zero on read error
                controller_data.accelX = 0;
                controller_data.accelY = 0;
                controller_data.accelZ = 0;
                controller_data.gyroX = 0;
                controller_data.gyroY = 0;
                controller_data.gyroZ = 0;
                return;
        }

        // Now get the processed controller data
        imu_controller_data_t imu_data;
        int ret = imu_get_controller_data(&imu_data);

        if (ret == 0)
        {
                // Copy data to controller structure
                controller_data.accelX = imu_data.accel_x;
                controller_data.accelY = imu_data.accel_y;
                controller_data.accelZ = imu_data.accel_z;
                controller_data.gyroX = imu_data.gyro_x;
                controller_data.gyroY = imu_data.gyro_y;
                controller_data.gyroZ = imu_data.gyro_z;
        }
        else
        {
                // Set to zero on error
                controller_data.accelX = 0;
                controller_data.accelY = 0;
                controller_data.accelZ = 0;
                controller_data.gyroX = 0;
                controller_data.gyroY = 0;
                controller_data.gyroZ = 0;
        }
}

// Update your shutdown_all_peripherals function:
void shutdown_all_peripherals(void)
{
        LOG_INF("=== ENTERING SLEEP MODE ===");
        LOG_INF("Shutting down all peripherals for deep sleep...");

        // Turn off LED
        gpio_pin_set_dt(&led0, 0);

        // Shutdown display completely
        display_set_blanking(true);
        LOG_INF("Display blanked");

        // Manually power down SSD1306 via I2C commands
        const struct device *display_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
        if (device_is_ready(display_i2c))
        {
                // SSD1306 power off command sequence
                uint8_t power_off_cmd[] = {0x00, 0xAE};                   // Command mode, Display OFF
                int ret = i2c_write(display_i2c, power_off_cmd, 2, 0x3C); // SSD1306 address
                if (ret == 0)
                {
                        LOG_INF("Display powered down via I2C command");
                }
                else
                {
                        LOG_WRN("Failed to power down display via I2C: %d", ret);
                }

                // SSD1306 charge pump off command sequence
                uint8_t charge_pump_off_cmd[] = {0x00, 0x8D, 0x10};              // Command mode, Charge Pump OFF
                int ret2 = i2c_write(display_i2c, charge_pump_off_cmd, 3, 0x3C); // SSD1306 address
                if (ret2 == 0)
                {
                        LOG_INF("Display charge pump powered down via I2C command");
                }
                else
                {
                        LOG_WRN("Failed to power down display via I2C: %d", ret);
                }
        }
        else
        {
                LOG_WRN("I2C1 not ready for display power down");
        }

        k_thread_suspend(&trackpad_thread_data);
        k_thread_suspend(&display_thread_data);
        LOG_INF("Trackpad and display threads suspended");

        // Put DRV2605 in standby mode
        if (haptic_is_available())
        {
                haptic_enter_standby();
                LOG_INF("Haptic driver in standby");
        }
        LOG_INF("DEBUG: Haptic standby complete");

        // Put IMU in power-down mode
        if (imu_is_available())
        {
                LOG_INF("DEBUG: About to put IMU in standby");
                imu_enter_standby();
                LOG_INF("IMU powered down");
        }
        LOG_INF("DEBUG: IMU standby complete");

        // Stop trackpad thread (it will restart on wake)
        // The thread will automatically stop when system sleeps
        LOG_INF("DEBUG: About to stop ESB communication");

        // Stop ESB transmission to save power
        esb_comm_status_t ret = esb_comm_enter_sleep();
        LOG_INF("DEBUG: ESB enter_sleep returned: %d", ret);
        if (ret == ESB_COMM_STATUS_OK)
        {
                LOG_INF("ESB communication stopped for sleep");
        }

        // Power down ADC to save power
        const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
        if (device_is_ready(adc_dev))
        {
                int adc_ret = pm_device_action_run(adc_dev, PM_DEVICE_ACTION_SUSPEND);
                if (adc_ret == 0)
                {
                        LOG_INF("ADC powered down");
                }
                else
                {
                        LOG_WRN("Failed to power down ADC: %d", adc_ret);
                }
        }

        // Suspend I2C buses to save power
        const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
        const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

        if (device_is_ready(i2c0_dev))
        {
                int i2c_ret = pm_device_action_run(i2c0_dev, PM_DEVICE_ACTION_SUSPEND);
                if (i2c_ret == 0)
                {
                        LOG_INF("I2C0 bus suspended for sleep");
                }
                else
                {
                        LOG_WRN("Failed to suspend I2C0: %d", i2c_ret);
                }
        }

        if (device_is_ready(i2c1_dev))
        {
                int i2c_ret = pm_device_action_run(i2c1_dev, PM_DEVICE_ACTION_SUSPEND);
                if (i2c_ret == 0)
                {
                        LOG_INF("I2C1 bus suspended for sleep");
                }
                else
                {
                        LOG_WRN("Failed to suspend I2C1: %d", i2c_ret);
                }
        }

        LOG_INF("=== SLEEP MODE ACTIVE ===");
        LOG_INF("Press BUMPER button to wake up");
}

// Update your wakeup_all_peripherals function:
void wakeup_all_peripherals(void)
{
        LOG_INF("=== WAKING UP FROM SLEEP ===");
        LOG_INF("BUMPER button pressed - reinitializing all peripherals...");

        // Turn on status LED to indicate system is awake
        gpio_pin_set_dt(&led0, 1);

        // Resume I2C buses first to restore communication
        const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
        const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

        if (device_is_ready(i2c0_dev))
        {
                int i2c_ret = pm_device_action_run(i2c0_dev, PM_DEVICE_ACTION_RESUME);
                if (i2c_ret == 0)
                {
                        LOG_INF("I2C0 bus resumed");
                }
                else
                {
                        LOG_WRN("Failed to resume I2C0: %d", i2c_ret);
                }
        }

        if (device_is_ready(i2c1_dev))
        {
                int i2c_ret = pm_device_action_run(i2c1_dev, PM_DEVICE_ACTION_RESUME);
                if (i2c_ret == 0)
                {
                        LOG_INF("I2C1 bus resumed");
                }
                else
                {
                        LOG_WRN("Failed to resume I2C1: %d", i2c_ret);
                }
        }

        k_sleep(K_MSEC(100)); // Let I2C buses stabilize

        // Resume trackpad and display threads
        k_thread_resume(&trackpad_thread_data);
        k_thread_resume(&display_thread_data);
        LOG_INF("Trackpad and display threads resumed");

        // Re-enable ESB communication
        esb_comm_status_t ret = esb_comm_wakeup();
        if (ret != ESB_COMM_STATUS_OK)
        {
                LOG_WRN("Failed to re-initialize ESB: %d", ret);
        }
        else
        {
                LOG_INF("ESB communication restarted");
        }

        // Restore data from storage
        LOG_INF("Restoring saved configuration from storage...");
        controller_storage_load_calibration(&controller_calibration);
        controller_storage_load_bindings(&controller_bindings);
        controller_storage_load_preferences(&controller_preferences);

        // Re-enable display
        const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
        if (device_is_ready(display_dev))
        {
                int disp_ret = pm_device_action_run(display_dev, PM_DEVICE_ACTION_RESUME);
                if (disp_ret == 0)
                {
                        LOG_INF("Display powered up");
                }
                else
                {
                        LOG_WRN("Failed to power up display: %d", disp_ret);
                }
        }
        display_set_blanking(false);
        LOG_INF("Display reactivated");

        // Re-enable ADC
        const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
        if (device_is_ready(adc_dev))
        {
                int adc_ret = pm_device_action_run(adc_dev, PM_DEVICE_ACTION_RESUME);
                if (adc_ret == 0)
                {
                        LOG_INF("ADC powered up");
                }
                else
                {
                        LOG_WRN("Failed to power up ADC: %d", adc_ret);
                }
        }

        // Wake up DRV2605 haptic driver
        if (haptic_is_available())
        {
                haptic_wakeup();

                // Play a wake-up effect to confirm system is active
                k_sleep(K_MSEC(100)); // Small delay for haptic to wake up
                for (int i = 0; i < 2; i++)
                {
                        gpio_pin_set_dt(&haptic_trigger, 1);
                        k_sleep(K_MSEC(50));
                        gpio_pin_set_dt(&haptic_trigger, 0);
                        k_sleep(K_MSEC(50));
                }
                LOG_INF("Haptic driver awakened with wake-up feedback");
        }

        // Restart IMU
        if (imu_is_available())
        {
                imu_wakeup();
                LOG_INF("IMU reactivated");
        }

        // Trackpad and display threads will automatically restart

        LOG_INF("=== WAKE-UP COMPLETE ===");
        LOG_INF("All peripherals active, system ready");
}

// Simplified UI functions (LVGL disabled)
void ui_init(void)
{
        LOG_INF("UI initialization - using simple display patterns");
}

void ui_create_main_screen(void)
{
        LOG_DBG("UI create main screen - using simple patterns");
}

void ui_create_menu_screen(void)
{
        LOG_DBG("UI create menu screen - using simple patterns");
}

void ui_create_calibration_screen(void)
{
        LOG_DBG("UI create calibration screen - using simple patterns");
}

void ui_update_data(void)
{
        LOG_DBG("UI update data - using simple patterns");
}

void ui_handle_input(void)
{
        // Simple debug logging
        static uint32_t last_update = 0;
        uint32_t now = k_uptime_get_32();

        if ((now - last_update) > 1000)
        { // Update every 1 second
                LOG_INF("System running - uptime: %u ms", now);
                last_update = now;
        }
}

int main(void)
{
        int ret;

        LOG_INF("Zephyr ESB Controller Starting...");

        // Initialize LED
        if (!gpio_is_ready_dt(&led0))
        {
                LOG_ERR("LED device not ready");
                return -ENODEV;
        }
        gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

        // Initialize battery voltage divider (XIAO nRF52840 safe approach)
        if (!gpio_is_ready_dt(&vbat_enable))
        {
                LOG_ERR("VBAT enable GPIO not ready");
                return -ENODEV;
        }

        // Try different GPIO configuration approaches for P0.14
        LOG_INF("Attempting to configure P0.14 as current sink for voltage divider control...");

        // Method 1: Open-drain output (proper current sink)
        ret = gpio_pin_configure_dt(&vbat_enable, GPIO_OUTPUT_INACTIVE | GPIO_OPEN_DRAIN);
        if (ret != 0)
        {
                LOG_ERR("Failed to configure P0.14 as open-drain: %d", ret);
        }

        ret = gpio_pin_set_dt(&vbat_enable, 0); // Active LOW for current sink
        if (ret != 0)
        {
                LOG_ERR("Failed to set P0.14 to 0: %d", ret);
        }

        // Method 2: If open-drain not supported, try standard output with pull-down
        if (ret != 0)
        {
                ret = gpio_pin_configure_dt(&vbat_enable, GPIO_OUTPUT | GPIO_PULL_DOWN);
                if (ret == 0)
                {
                        ret = gpio_pin_set_dt(&vbat_enable, 0);
                        int pin_state = gpio_pin_get_dt(&vbat_enable);
                        LOG_INF("Method 2 (output+pulldown) result: set_result=%d, pin_state=%d", ret, pin_state);
                }
        }

        // Method 3: Raw GPIO register access for open-drain if needed
        if (ret != 0)
        {
                const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
                if (device_is_ready(gpio_dev))
                {
                        ret = gpio_pin_configure(gpio_dev, 14, GPIO_OUTPUT | GPIO_OPEN_DRAIN);
                        if (ret == 0)
                        {
                                ret = gpio_pin_set_raw(gpio_dev, 14, 0);
                                int raw_state = gpio_pin_get_raw(gpio_dev, 14);
                                LOG_INF("Method 3 (raw open-drain) result: set_result=%d, raw_pin_state=%d", ret, raw_state);
                        }
                }
        }

        // Final verification
        int final_pin_state = gpio_pin_get_dt(&vbat_enable);
        LOG_INF("Final P0.14 state: %d (configured as current sink)", final_pin_state);

        if (final_pin_state != 0)
        {
                LOG_WRN("P0.14 not sinking current properly - voltage divider may not work");
        }
        else
        {
                LOG_INF("P0.14 configured as current sink - should enable voltage divider");
        }

        // Initialize controller buttons
        buttons_init();

        // Initialize ADC for analog inputs
        adc_init();

        // Initialize ESB communication
        esb_comm_init();

        // Initialize I2C for trackpad (when connected) - MOVED TO LATER
        i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

        // Get I2C0 for IMU
        const struct device *imu_i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
        if (!device_is_ready(imu_i2c_dev))
        {
                LOG_ERR("I2C0 device not ready for IMU");
        }
        else
        {
                LOG_INF("I2C0 ready for IMU");
        }

        // Initialize SSD1306 display using display library
        LOG_INF("About to call display_library_init(CONTROLLER_ID=%d)", CONTROLLER_ID);
        ret = display_library_init(CONTROLLER_ID);
        LOG_INF("display_library_init returned: %d, status now: %d", ret, display_get_status());
        if (ret != 0)
        {
                LOG_WRN("Display initialization failed: %d", ret);
        }
        else
        {
                LOG_INF("Display library initialized successfully");
        }

        // Initialize haptic motor driver using haptic_driver library
        // Initialize haptic motor driver using haptic_driver library
        ret = haptic_driver_init(i2c_dev, &haptic_trigger, &haptic_enable_pin);
        if (ret != 0)
        {
                LOG_WRN("Haptic driver initialization failed: %d", ret);
        }
        else
        {
                // Small delay for DRV2605 to fully initialize
                k_sleep(K_MSEC(100));

                // Perform LRA auto-calibration BEFORE setting up external trigger
                LOG_INF("Starting LRA haptic auto-calibration...");
                ret = haptic_perform_lra_calibration();
                if (ret != 0)
                {
                        LOG_WRN("Haptic auto-calibration failed: %d", ret);
                }

                // Setup external trigger mode with LRA-optimized effect (after calibration)
                ret = haptic_setup_external_trigger(DRV2605_EFFECT_SHARP_TICK_2);
                if (ret != 0)
                {
                        LOG_WRN("Haptic external trigger setup failed: %d", ret);
                }
                else
                {
                        // Test haptic motor with a quick pulse to verify it's working
                        k_sleep(K_MSEC(50));
                        gpio_pin_set_dt(&haptic_trigger, 1);
                        k_sleep(K_MSEC(50));
                        gpio_pin_set_dt(&haptic_trigger, 0);
                        LOG_INF("Haptic motor test pulse sent");
                }
        }

        // Initialize power management (with combos disabled for now)
        power_mgmt_config_t power_config = {
            .wake_button1 = &mode_button,         // Use mode button as wake button
            .wake_button2 = NULL,                 // Use existing button as wake button 2
            .status_led = &led0,                  // Use system LED for status
            .shutdown_combo_hold_ms = 5000,       // 30 second hold for shutdown (very long to prevent accidental)
            .factory_reset_combo_hold_ms = 50000, // 50 second hold for factory reset (very long)
            .auto_sleep_enabled = false,          // Disable auto-sleep for now
            .auto_sleep_timeout_ms = 300000       // 5 minutes auto-sleep (when enabled)
        };

        power_mgmt_status_t pm_status = power_mgmt_driver_init(&power_config);
        if (pm_status != POWER_MGMT_STATUS_OK)
        {
                LOG_WRN("Power management initialization failed: %d", pm_status);
        }

        // Register power management callbacks for peripheral control
        power_mgmt_register_shutdown_callback(shutdown_all_peripherals);
        power_mgmt_register_wakeup_callback(wakeup_all_peripherals);

        // Disable button combos to prevent accidental system restart
        // These will only be enabled when intentionally entering sleep mode
        power_mgmt_enable_combo(BUTTON_COMBO_SHUTDOWN, false);
        power_mgmt_enable_combo(BUTTON_COMBO_FACTORY_RESET, false);
        LOG_INF("Power management button combos disabled for normal operation");

        // Initialize storage subsystem
        ret = controller_storage_init();
        if (ret != 0)
        {
                LOG_ERR("Failed to initialize storage: %d", ret);
                // Continue without storage - use defaults
        }
        else
        {
                // Load configuration from flash
                controller_storage_load_calibration(&controller_calibration);
                controller_storage_load_bindings(&controller_bindings);
                controller_storage_load_preferences(&controller_preferences);

                // Apply loaded analog calibration values to analog driver
                if (controller_calibration.stick_calibrated && analog_driver_is_initialized())
                {
                        analog_calibration_t stick_x_cal = {
                            .center_value = controller_calibration.stick_center_x,
                            .min_value = 0,
                            .max_value = 4095,
                            .deadzone = controller_calibration.stick_deadzone,
                            .is_calibrated = true};
                        analog_calibration_t stick_y_cal = {
                            .center_value = controller_calibration.stick_center_y,
                            .min_value = 0,
                            .max_value = 4095,
                            .deadzone = controller_calibration.stick_deadzone,
                            .is_calibrated = true};
                        analog_calibration_t trigger_cal = {
                            .center_value = controller_calibration.trigger_min,
                            .min_value = controller_calibration.trigger_min,
                            .max_value = controller_calibration.trigger_max,
                            .deadzone = 50,
                            .is_calibrated = true};

                        analog_driver_set_calibration(ANALOG_CHANNEL_STICK_X, &stick_x_cal);
                        analog_driver_set_calibration(ANALOG_CHANNEL_STICK_Y, &stick_y_cal);
                        analog_driver_set_calibration(ANALOG_CHANNEL_TRIGGER, &trigger_cal);

                        LOG_INF("Analog calibration values applied to driver");
                }
                else
                {
                        LOG_INF("No analog calibration found or driver not ready - using defaults");
                        analog_calibration_t stick_x_default = {
                            .center_value = 1850, // Assume center
                            .min_value = 650,     // Your measured min
                            .max_value = 3100,    // Your measured max
                            .deadzone = 75,       // Reasonable deadzone
                            .is_calibrated = false};

                        analog_calibration_t stick_y_default = {
                            .center_value = 1850,
                            .min_value = 650,
                            .max_value = 3100,
                            .deadzone = 75,
                            .is_calibrated = false};

                        analog_calibration_t trigger_default = {
                            .center_value = 1500, // Trigger rest position
                            .min_value = 1000,
                            .max_value = 1500,
                            .deadzone = 20,
                            .is_calibrated = false};

                        analog_driver_set_calibration(ANALOG_CHANNEL_STICK_X, &stick_x_default);
                        analog_driver_set_calibration(ANALOG_CHANNEL_STICK_Y, &stick_y_default);
                        analog_driver_set_calibration(ANALOG_CHANNEL_TRIGGER, &trigger_default);
                }

                LOG_INF("Configuration loaded from flash");
                LOG_INF("Stick calibrated: %s, IMU calibrated: %s",
                        controller_calibration.stick_calibrated ? "YES" : "NO",
                        controller_calibration.imu_calibrated ? "YES" : "NO");
        }

        // Initialize the IMU sensor using imu_driver library
        const struct device *imu_sensor_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6ds3tr_c));

        // Check if IMU sensor device is available first
        LOG_INF("Checking IMU sensor device availability...");

        ret = imu_driver_init(imu_sensor_dev, imu_i2c_dev, NULL);
        if (ret != 0)
        {
                LOG_WRN("IMU driver initialization failed: %d", ret);
                LOG_INF("This may be normal if IMU sensor is not present on this board variant");
        }
        else
        {
                LOG_INF("IMU driver initialized successfully");
        }

        // Initialize controller data with proper ID
#if CONTROLLER_ID == 1
        controller_data.flags = 0x80; // LEFT controller: set bit 7
        LOG_INF("Configured as LEFT controller (ID=1)");
#else
        controller_data.flags = 0x00; // RIGHT controller: clear bit 7
        LOG_INF("Configured as RIGHT controller (ID=0)");
#endif

        controller_data.trigger = 0;
        controller_data.stickX = 0;
        controller_data.stickY = 0;
        controller_data.padX = 0;
        controller_data.padY = 0;
        controller_data.buttons = 0;
        controller_data.accelX = 0;
        controller_data.accelY = 0;
        controller_data.accelZ = 0;
        controller_data.gyroX = 0;
        controller_data.gyroY = 0;
        controller_data.gyroZ = 0;

        LOG_INF("Waiting for radio to fully initialize...");
        k_sleep(K_MSEC(500));

        LOG_INF("Controller ready - starting continuous transmission with ACK timing");

        // Note: Trackpad will be initialized directly in read_trackpad_inputs() for simplicity
        LOG_INF("Trackpad will be initialized on first read attempt");

        // Report peripheral status after boot
        LOG_INF("=== PERIPHERAL STATUS REPORT ===");
        LOG_INF("Haptic motor: %s", haptic_is_available() ? "AVAILABLE" : "NOT AVAILABLE");
        LOG_INF("IMU sensor: %s", imu_is_available() ? "AVAILABLE" : "NOT AVAILABLE");
        LOG_INF("Button driver: %s", button_driver_is_initialized() ? "AVAILABLE" : "NOT AVAILABLE");
        LOG_INF("Analog driver: %s", analog_driver_is_initialized() ? "AVAILABLE" : "NOT AVAILABLE");
        LOG_INF("Display: %s", (display_get_status() == DISPLAY_STATUS_READY) ? "AVAILABLE" : "NOT AVAILABLE"); // Check actual status
        LOG_INF("Trackpad: Will be initialized on first touch");

        update_controller_data();

        uint32_t loop_counter = 0;

        // Create trackpad thread
        k_tid_t trackpad_tid = k_thread_create(&trackpad_thread_data, trackpad_thread_stack,
                                               K_THREAD_STACK_SIZEOF(trackpad_thread_stack),
                                               trackpad_thread_entry, NULL, NULL, NULL,
                                               5, 0, K_NO_WAIT); // Priority 5
        k_thread_name_set(trackpad_tid, "trackpad");

        // Create display thread
        k_tid_t display_tid = k_thread_create(&display_thread_data, display_thread_stack,
                                              K_THREAD_STACK_SIZEOF(display_thread_stack),
                                              display_thread_entry, NULL, NULL, NULL,
                                              6, 0, K_NO_WAIT); // Priority 6 (lower than trackpad)
        k_thread_name_set(display_tid, "display");

        LOG_INF("Trackpad, analog, and display threads created");

        while (true)
        {
                loop_counter++;

                // Main loop heartbeat - log every 30 seconds to track main loop health
                static uint32_t last_main_heartbeat = 0;
                uint32_t current_time = k_uptime_get_32();
                if ((current_time - last_main_heartbeat) > 30000)
                {
                        LOG_INF("Main loop heartbeat: iteration %u, uptime %ums", loop_counter, current_time);
                        last_main_heartbeat = current_time;
                }

                // Battery voltage monitoring - check every 10 seconds
                static uint32_t last_battery_read = 0;
                if ((current_time - last_battery_read) > 10000)
                {
                        uint16_t battery_mv = 0;
                        if (analog_driver_get_battery_voltage(&battery_mv) == ANALOG_STATUS_OK)
                        {
                                LOG_INF("Battery: %u mV", battery_mv);
                        }
                        else
                        {
                                LOG_WRN("Failed to read battery voltage");
                        }
                        
                        last_battery_read = current_time;
                }

                // Custom sleep combo detection: Start + Bumper for 5 seconds
                static uint32_t sleep_combo_start = 0;
                static bool sleep_combo_active = false;
                static uint32_t last_haptic_time = 0;
                static bool sleep_in_progress = false;

                bool start_pressed = gpio_pin_get_dt(&start);   // Start button
                bool bumper_pressed = gpio_pin_get_dt(&bumper); // Bumper button

                // Skip combo detection if sleep is already in progress
                if (sleep_in_progress)
                {
                        LOG_DBG("Sleep in progress - skipping combo detection");
                        k_sleep(K_MSEC(10));
                        continue;
                }

                // Debug button states occasionally - removed verbose logging
                static uint32_t last_button_debug = 0;
                // Reuse current_time from main heartbeat above
                if (current_time - last_button_debug > 5000)
                { // Every 5 seconds
                        // Removed button state debug logging
                        last_button_debug = current_time;
                }

                // Analog debug mode: Hold MODE button to print analog values
                static bool analog_debug_mode = false;
                static uint32_t last_analog_debug = 0;
                bool mode_pressed = gpio_pin_get_dt(&mode_button); // MODE button

                if (mode_pressed && !start_pressed && !bumper_pressed)
                {
                        if (!analog_debug_mode)
                        {
                                analog_debug_mode = true;
                                printk("\n*** ANALOG DEBUG MODE ACTIVATED ***\n");
                        }

                        // Update analog data for display (thread-safe)
                        analog_controller_data_t analog_data;
                        analog_driver_get_controller_data(&analog_data);

                        display_analog_data_t display_data = {
                            .stick_x = analog_data.stick_x,
                            .stick_y = analog_data.stick_y,
                            .trigger = analog_data.trigger};

                        display_set_screen(DISPLAY_SCREEN_ANALOG, &display_data);

                        // Print to console every 200ms
                        if (current_time - last_analog_debug > 200)
                        {
                                print_analog_values();
                                last_analog_debug = current_time;
                        }
                }
                else if (analog_debug_mode)
                {
                        analog_debug_mode = false;
                        printk("\n*** ANALOG DEBUG MODE DEACTIVATED ***\n");
                        // Return to status screen (thread-safe)
                        display_set_screen(DISPLAY_SCREEN_STATUS, NULL);
                }

                if (start_pressed && bumper_pressed && !sleep_combo_active)
                {
                        sleep_combo_start = k_uptime_get_32();
                        sleep_combo_active = true;
                        last_haptic_time = 0; // Reset haptic timing
                        LOG_INF("Sleep combo detected (Start+Bumper) - hold for 5 seconds...");

                        // Haptic feedback to indicate combo started
                        if (haptic_is_available())
                        {
                                gpio_pin_set_dt(&haptic_trigger, 1);
                                k_sleep(K_USEC(100));
                                gpio_pin_set_dt(&haptic_trigger, 0);
                        }
                }
                else if (!start_pressed || !bumper_pressed)
                {
                        if (sleep_combo_active)
                        {
                                LOG_INF("Sleep combo cancelled");
                        }
                        sleep_combo_active = false;
                        last_haptic_time = 0; // Reset haptic timing when cancelled
                }

                if (sleep_combo_active)
                {
                        uint32_t hold_time = k_uptime_get_32() - sleep_combo_start;

                        // Give haptic feedback every second during hold (non-blocking)
                        uint32_t seconds_held = hold_time / 1000;
                        uint32_t expected_haptic_time = seconds_held * 1000;

                        // Non-blocking haptic feedback every second during hold
                        static uint32_t sleep_combo_haptic_start = 0;
                        static bool sleep_combo_haptic_active = false;

                        if (seconds_held > 0 && expected_haptic_time > last_haptic_time && haptic_is_available())
                        {
                                if (!sleep_combo_haptic_active)
                                {
                                        gpio_pin_set_dt(&haptic_trigger, 1);
                                        sleep_combo_haptic_start = k_uptime_get_32();
                                        sleep_combo_haptic_active = true;
                                        last_haptic_time = expected_haptic_time;
                                        // Removed sleep combo haptic debug logging
                                }
                        }

                        // Non-blocking haptic pulse completion for sleep combo
                        if (sleep_combo_haptic_active && (k_uptime_get_32() - sleep_combo_haptic_start) >= 50)
                        {
                                gpio_pin_set_dt(&haptic_trigger, 0);
                                sleep_combo_haptic_active = false;
                        }

                        if (hold_time >= 5000)
                        { // 5 seconds
                                // Non-blocking triple vibrate at 5 seconds to confirm sleep
                                static uint32_t triple_vibe_start = 0;
                                static int triple_vibe_stage = 0; // 0=not started, 1-6=vibration stages
                                static bool triple_vibe_active = false;

                                if (haptic_is_available() && triple_vibe_stage == 0)
                                {
                                        // Start triple vibration sequence
                                        triple_vibe_start = k_uptime_get_32();
                                        triple_vibe_stage = 1;
                                        triple_vibe_active = false;
                                        // Removed sleep combo 5-second debug logging
                                }

                                // Handle triple vibration stages (non-blocking)
                                if (triple_vibe_stage > 0)
                                {
                                        uint32_t elapsed = k_uptime_get_32() - triple_vibe_start;

                                        switch (triple_vibe_stage)
                                        {
                                        case 1: // First vibration on
                                                if (!triple_vibe_active)
                                                {
                                                        gpio_pin_set_dt(&haptic_trigger, 1);
                                                        triple_vibe_active = true;
                                                }
                                                if (elapsed >= 100)
                                                        triple_vibe_stage = 2; // 100ms on
                                                break;
                                        case 2: // First vibration off
                                                if (triple_vibe_active)
                                                {
                                                        gpio_pin_set_dt(&haptic_trigger, 0);
                                                        triple_vibe_active = false;
                                                }
                                                if (elapsed >= 200)
                                                        triple_vibe_stage = 3; // 100ms off
                                                break;
                                        case 3: // Second vibration on
                                                if (!triple_vibe_active)
                                                {
                                                        gpio_pin_set_dt(&haptic_trigger, 1);
                                                        triple_vibe_active = true;
                                                }
                                                if (elapsed >= 300)
                                                        triple_vibe_stage = 4; // 100ms on
                                                break;
                                        case 4: // Second vibration off
                                                if (triple_vibe_active)
                                                {
                                                        gpio_pin_set_dt(&haptic_trigger, 0);
                                                        triple_vibe_active = false;
                                                }
                                                if (elapsed >= 400)
                                                        triple_vibe_stage = 5; // 100ms off
                                                break;
                                        case 5: // Third vibration on
                                                if (!triple_vibe_active)
                                                {
                                                        gpio_pin_set_dt(&haptic_trigger, 1);
                                                        triple_vibe_active = true;
                                                }
                                                if (elapsed >= 500)
                                                        triple_vibe_stage = 6; // 100ms on
                                                break;
                                        case 6: // Third vibration off - sequence complete
                                                if (triple_vibe_active)
                                                {
                                                        gpio_pin_set_dt(&haptic_trigger, 0);
                                                        triple_vibe_active = false;
                                                }
                                                // Proceed to sleep after triple vibration
                                                LOG_INF("Entering sleep mode!");
                                                sleep_combo_active = false;
                                                last_haptic_time = 0;
                                                triple_vibe_stage = 0; // Reset for next time

                                                // Set sleep in progress immediately
                                                sleep_in_progress = true;

                                                // Enter sleep mode directly
                                                power_mgmt_enter_sleep();

                                                // System will resume here after wake-up
                                                LOG_INF("Woke up from sleep mode!");
                                                sleep_in_progress = false;
                                                continue;
                                        }
                                }
                        }
                }

                // Monitor thread health and system resources (every 5 seconds)
                static uint32_t last_health_check = 0;
                uint32_t now = k_uptime_get_32();
                if ((now - last_health_check) > 5000)
                {
                        uint32_t trackpad_age = now - trackpad_thread_heartbeat;
                        uint32_t display_age = now - display_thread_heartbeat;

                        if (trackpad_age > 2000)
                        { // More than 2 seconds old
                                LOG_WRN("Trackpad thread appears frozen (heartbeat %ums old)", trackpad_age);
                        }
                        if (display_age > 2000)
                        {
                                LOG_WRN("Display thread appears frozen (heartbeat %ums old)", display_age);
                        }

                        // Add system resource monitoring
                        LOG_INF("System health: trackpad=%ums, display=%ums, uptime=%ums",
                                trackpad_age, display_age, now);

                        // Stack monitoring not available in this configuration
                        // Removed k_thread_stack_space_get calls due to linking issues

                        last_health_check = now;
                }

                // Measure ESB transmission time
                uint32_t tx_start_cycles = k_cycle_get_32();

                // Attempt to send controller data (with built-in timing control)
                esb_comm_status_t tx_status = send_controller_data();

                // Debug: Track transmission attempts every 5 seconds during potential issues
                static uint32_t tx_attempt_counter = 0;
                static uint32_t last_tx_debug = 0;
                tx_attempt_counter++;

                uint32_t current_uptime = k_uptime_get_32();
                if ((current_uptime - last_tx_debug) > 5000) // Every 5 seconds
                {
                        // Check if we're getting stuck in specific error states
                        static uint32_t consecutive_errors = 0;
                        if (tx_status != ESB_COMM_STATUS_OK)
                        {
                                consecutive_errors++;
                        }
                        else
                        {
                                consecutive_errors = 0;
                        }

                        // Get ESB statistics to see if radio is actually working
                        esb_comm_stats_t esb_stats = {0};
                        esb_comm_status_t stats_status = esb_comm_get_stats(&esb_stats);

                        if (stats_status == ESB_COMM_STATUS_OK)
                        {
                                // ESB Watchdog: Detect when stats stop incrementing despite "successful" sends
                                static uint32_t last_total_transmissions = 0;
                                static uint32_t watchdog_counter = 0;

                                if (esb_stats.total_transmissions == last_total_transmissions)
                                {
                                        watchdog_counter++;
                                        LOG_WRN("ESB Watchdog: Stats frozen for %u periods (total: %u)",
                                                watchdog_counter, esb_stats.total_transmissions);

                                        // Reset ESB if stats haven't changed for 3 periods (15 seconds)
                                        if (watchdog_counter >= 3)
                                        {
                                                LOG_ERR("ESB FROZEN - Attempting reset...");

                                                // Reinitialize ESB communication
                                                esb_comm_config_t esb_config = {
                                                    .controller_id = CONTROLLER_ID,
                                                    .base_tx_interval_ms = 5, // 5ms base interval (200Hz)
                                                    .retry_interval_ms = 10,  // 10ms retry interval
                                                    .rf_channel = 1,          // RF channel 1
                                                    .status_led = &led0       // Use LED0 for status indication
                                                };

                                                esb_comm_status_t reset_status = esb_comm_driver_init(&esb_config);
                                                if (reset_status == ESB_COMM_STATUS_OK)
                                                {
                                                        // Re-enable ACK timing after reset
                                                        esb_comm_enable_ack_timing(true);
                                                        LOG_INF("ESB reset successful");
                                                }
                                                else
                                                {
                                                        LOG_ERR("ESB reset failed: %d", reset_status);
                                                }

                                                watchdog_counter = 0;
                                                last_total_transmissions = 0; // Reset tracking
                                        }
                                }
                                else
                                {
                                        // Stats are incrementing normally
                                        watchdog_counter = 0;
                                        last_total_transmissions = esb_stats.total_transmissions;
                                }

                                LOG_INF("TX Debug: %u attempts, status: %d, trigger: %d, sticks: %d,%d, errors: %u",
                                        tx_attempt_counter, tx_status, controller_data.trigger,
                                        controller_data.stickX, controller_data.stickY, consecutive_errors);
                                LOG_INF("ESB Stats: total: %u, success: %u, failed: %u, rate: %.1f%%, last_ok: %s",
                                        esb_stats.total_transmissions, esb_stats.successful_transmissions,
                                        esb_stats.failed_transmissions, esb_stats.success_rate * 100.0f,
                                        esb_stats.last_tx_succeeded ? "yes" : "no");

                                // Check for data corruption - log full controller data structure
                                LOG_INF("Data Check: flags=0x%02X, buttons=0x%02X, padX=%d, padY=%d",
                                        controller_data.flags, controller_data.buttons,
                                        controller_data.padX, controller_data.padY);
                                LOG_INF("Data Check: accelX=%d, accelY=%d, accelZ=%d",
                                        controller_data.accelX, controller_data.accelY, controller_data.accelZ);

                                // Corruption detection: Check for impossible values
                                bool corruption_detected = false;
                                if (controller_data.stickX < -127 || controller_data.stickX > 127)
                                        corruption_detected = true;
                                if (controller_data.stickY < -127 || controller_data.stickY > 127)
                                        corruption_detected = true;
                                if (controller_data.trigger > 255)
                                        corruption_detected = true;
                                if (abs(controller_data.padX) > 32767 || abs(controller_data.padY) > 32767)
                                        corruption_detected = true;

                                if (corruption_detected)
                                {
                                        LOG_ERR("DATA CORRUPTION DETECTED! Stick/trigger values out of range");
                                }
                        }
                        else
                        {
                                LOG_ERR("Failed to get ESB stats: %d", stats_status);
                        }
                        last_tx_debug = current_uptime;
                        tx_attempt_counter = 0;
                }

                uint32_t tx_end_cycles = k_cycle_get_32();
                uint32_t tx_us = k_cyc_to_us_floor32(tx_end_cycles - tx_start_cycles);

                // Track longest transmission time
                static uint32_t max_tx_us = 0;
                if (tx_us > max_tx_us)
                {
                        max_tx_us = tx_us;
                }

                // Log if transmission takes too long (>5ms is suspicious)
                if (tx_us > 5000)
                {
                        LOG_WRN("SLOW ESB TRANSMISSION: %dus (max: %dus)", tx_us, max_tx_us);
                }

                // Debug logging every 1000 loops
                // if (loop_counter % 1000 == 1)
                // {
                //         LOG_INF("Main loop running - iteration %u", loop_counter);
                // }
                // haptic_test_strong_effects();

                // Timing diagnostics for latency spike detection
                uint32_t loop_start_cycles = k_cycle_get_32();
                static uint32_t last_loop_end_cycles = 0;
                static uint32_t max_gap_us = 0;
                static uint32_t max_loop_us = 0;

                // Calculate gap since last loop (should be close to sleep_delay)
                if (last_loop_end_cycles != 0)
                {
                        uint32_t gap_cycles = loop_start_cycles - last_loop_end_cycles;
                        uint32_t gap_us = k_cyc_to_us_floor32(gap_cycles);
                        if (gap_us > max_gap_us)
                        {
                                max_gap_us = gap_us;
                        }

                        // Log if gap is much larger than expected (>50ms = massive spike)
                        if (gap_us > 50000)
                        {
                                LOG_WRN("LATENCY SPIKE: %dus gap between loops!", gap_us);
                        }
                }

                // Dynamic loop delay based on ACK payload timing - split into two parts
                uint16_t next_delay = esb_comm_get_next_delay();
                uint16_t sleep_delay;

                if (next_delay > 0 && next_delay <= 100)
                {
                        // Use dongle-requested timing (limited to reasonable range)
                        sleep_delay = next_delay;
                }
                else
                {
                        // Fallback to default timing if no ACK payload or invalid timing
                        sleep_delay = 8;
                }

                // Add radio busy backoff - if transmission failed due to busy radio,
                // increase sleep delay to reduce pressure on the radio
                static uint32_t consecutive_busy_count = 0;
                if (tx_status == ESB_COMM_STATUS_BUSY)
                {
                        consecutive_busy_count++;
                        // Exponential backoff: 2ms extra delay per consecutive failure
                        uint16_t backoff_delay = consecutive_busy_count * 2;
                        if (backoff_delay > 10)
                                backoff_delay = 10; // Cap at 10ms extra
                        sleep_delay += backoff_delay;

                        if (consecutive_busy_count <= 3)
                        { // Don't spam logs
                                LOG_WRN("RADIO BUSY - adding %dms backoff (consecutive: %d)",
                                        backoff_delay, consecutive_busy_count);
                        }
                }
                else
                {
                        // Reset consecutive count on successful transmission
                        consecutive_busy_count = 0;
                }

                // Sleep for half the interval
                uint16_t half_delay = sleep_delay / 2;
                if (half_delay > 0)
                {
                        k_sleep(K_MSEC(half_delay));
                }

                // Measure controller data update time
                uint32_t update_start_cycles = k_cycle_get_32();

                // Update sensor data halfway through the transmission interval
                update_controller_data();

                uint32_t update_end_cycles = k_cycle_get_32();
                uint32_t update_us = k_cyc_to_us_floor32(update_end_cycles - update_start_cycles);

                // Track longest controller update time
                static uint32_t max_update_us = 0;
                if (update_us > max_update_us)
                {
                        max_update_us = update_us;
                }

                // Log if controller update takes too long (>10ms is suspicious)
                if (update_us > 10000)
                {
                        LOG_WRN("SLOW CONTROLLER UPDATE: %dus (max: %dus)", update_us, max_update_us);
                }

                // Sleep for the remaining time
                uint16_t remaining_delay = sleep_delay - half_delay;
                if (remaining_delay > 0)
                {
                        k_sleep(K_MSEC(remaining_delay));
                }

                // Complete loop timing measurement
                uint32_t loop_end_cycles = k_cycle_get_32();
                uint32_t total_loop_us = k_cyc_to_us_floor32(loop_end_cycles - loop_start_cycles);
                last_loop_end_cycles = loop_end_cycles;

                // Track longest loop time
                if (total_loop_us > max_loop_us)
                {
                        max_loop_us = total_loop_us;
                }

                // Log if total loop takes much longer than expected
                uint32_t expected_loop_us = sleep_delay * 1000; // Convert ms to us
                if (total_loop_us > (expected_loop_us + 10000))
                { // >10ms over expected
                        LOG_WRN("LONG LOOP: %dus (expected ~%dus, max: %dus)",
                                total_loop_us, expected_loop_us, max_loop_us);
                }

                // Periodic summary every 10 seconds
                static uint32_t last_summary = 0;
                uint32_t summary_now = k_uptime_get_32();
                if ((summary_now - last_summary) > 10000)
                {
                        LOG_INF("TIMING SUMMARY - TX:%dus, Update:%dus, Loop:%dus, Gap:%dus",
                                max_tx_us, max_update_us, max_loop_us, max_gap_us);
                        last_summary = summary_now;
                        // Reset maximums for next period
                        max_tx_us = 0;
                        max_update_us = 0;
                        max_loop_us = 0;
                        max_gap_us = 0;
                }

                // Freeze detection: track how long this loop iteration took (legacy)
                static uint32_t last_loop_time = 0;
                static uint32_t max_loop_time = 0;
                uint32_t loop_end_time = k_uptime_get_32();

                if (last_loop_time != 0)
                {
                        uint32_t loop_duration = loop_end_time - last_loop_time;
                        if (loop_duration > max_loop_time)
                        {
                                max_loop_time = loop_duration;
                        }

                        // Log if loop took unusually long (indicates freeze/blocking)
                        if (loop_duration > 100)
                        { // More than 100ms for one loop iteration
                                LOG_WRN("Long loop iteration: %ums (max so far: %ums)", loop_duration, max_loop_time);
                        }

                        // Reset max every minute for trending
                        static uint32_t last_max_reset = 0;
                        if ((loop_end_time - last_max_reset) > 60000)
                        {
                                LOG_INF("Loop timing: max iteration time in last minute: %ums", max_loop_time);
                                max_loop_time = 0;
                                last_max_reset = loop_end_time;
                        }
                }

                last_loop_time = loop_end_time;
        }

        return 0;
}
