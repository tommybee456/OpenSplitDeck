/**
 ******************************************************************************
 * @file    analog_driver.c
 * @brief   ADC Analog Input Driver Library Implementation
 * @author  Controller Team
 * @version V1.0
 * @date    2025
 ******************************************************************************
 */

#include "analog_driver.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_saadc.h>
#include <math.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(analog_driver, LOG_LEVEL_INF);

// Global context
static analog_driver_context_t g_analog_ctx = {0};

// Battery voltage divider enable pin (P0.14)
static const struct gpio_dt_spec battery_enable_pin = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 14,
    .dt_flags = GPIO_OUTPUT_ACTIVE
};

// Default configuration
static const analog_config_t default_config = {
    .resolution_bits = 12,
    .gain = ADC_GAIN_1_6,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40), // Increased for rapidly changing signals
    .filter_alpha = 0.8f // Lighter filtering for better responsiveness
};

// Channel names for debugging
static const char *channel_names[ANALOG_CHANNEL_COUNT] = {
    "STICK_X",
    "STICK_Y", 
    "TRIGGER",
    "BATTERY"
};

/**
 * @brief Initialize the analog driver
 */
analog_status_t analog_driver_init(const struct device *adc_device)
{
    LOG_INF("Initializing analog driver...");

    if (!adc_device || !device_is_ready(adc_device))
    {
        LOG_ERR("ADC device not ready");
        return ANALOG_STATUS_ERROR;
    }

    // Clear context
    memset(&g_analog_ctx, 0, sizeof(g_analog_ctx));

    // Store ADC device and configuration
    g_analog_ctx.adc_dev = adc_device;
    g_analog_ctx.config = default_config;

    // Initialize battery enable pin (P0.14)
    if (!gpio_is_ready_dt(&battery_enable_pin))
    {
        LOG_ERR("Battery enable pin not ready");
        return ANALOG_STATUS_ERROR;
    }
    
    int ret = gpio_pin_configure_dt(&battery_enable_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure battery enable pin: %d", ret);
        return ANALOG_STATUS_ERROR;
    }
    
    // Enable battery voltage divider
    gpio_pin_set_dt(&battery_enable_pin, 0); // Active low - enable divider
    LOG_INF("Battery voltage divider enabled");

    // Configure channel mappings (for nRF52840 SAADC)
    g_analog_ctx.channel_configs[ANALOG_CHANNEL_STICK_X] = (analog_channel_config_t){
        .adc_channel = 0,
        .adc_input = NRF_SAADC_INPUT_AIN0, // P0.02
        .name = "StickX"};

    g_analog_ctx.channel_configs[ANALOG_CHANNEL_STICK_Y] = (analog_channel_config_t){
        .adc_channel = 1,
        .adc_input = NRF_SAADC_INPUT_AIN1, // P0.03
        .name = "StickY"};

    g_analog_ctx.channel_configs[ANALOG_CHANNEL_TRIGGER] = (analog_channel_config_t){
        .adc_channel = 2,
        .adc_input = NRF_SAADC_INPUT_AIN4, // P0.28
        .name = "Trigger"};

    g_analog_ctx.channel_configs[ANALOG_CHANNEL_BATTERY] = (analog_channel_config_t){
        .adc_channel = 3,
        .adc_input = NRF_SAADC_INPUT_AIN7, // P0.31 - Battery via voltage divider
        .name = "Battery"};

    // Configure ADC channels
    for (int i = 0; i < ANALOG_CHANNEL_COUNT; i++)
    {
        struct adc_channel_cfg *cfg = &g_analog_ctx.adc_channel_configs[i];
        const analog_channel_config_t *ch_cfg = &g_analog_ctx.channel_configs[i];

        cfg->gain = g_analog_ctx.config.gain;
        cfg->reference = g_analog_ctx.config.reference;
        cfg->acquisition_time = g_analog_ctx.config.acquisition_time;
        cfg->channel_id = ch_cfg->adc_channel;
        cfg->differential = 0;
        cfg->input_positive = ch_cfg->adc_input;

        int ret = adc_channel_setup(g_analog_ctx.adc_dev, cfg);
        if (ret != 0)
        {
            LOG_ERR("Failed to setup ADC channel %d (%s): %d", i, ch_cfg->name, ret);
            return ANALOG_STATUS_ADC_ERROR;
        }

        LOG_INF("ADC channel %d (%s) configured successfully", i, ch_cfg->name);
    }

    // Initialize default calibration values (12-bit ADC)
    for (int i = 0; i < ANALOG_CHANNEL_COUNT; i++)
    {
        analog_calibration_t *cal = &g_analog_ctx.calibrations[i];

        if (i == ANALOG_CHANNEL_TRIGGER)
        {
            // Trigger: 0 to full scale
            cal->center_value = 0;
            cal->min_value = 0;
            cal->max_value = 4095;
            cal->deadzone = 50;
        }
        else
        {
            // Sticks: centered around middle
            cal->center_value = 2048; // 12-bit center
            cal->min_value = 0;
            cal->max_value = 4095;
            cal->deadzone = 100;
        }
        cal->is_calibrated = false;
    }

    g_analog_ctx.filter_initialized = false;
    g_analog_ctx.sample_count = 0;
    
    // Initialize thread synchronization
    k_mutex_init(&g_analog_ctx.data_mutex);
    g_analog_ctx.thread_running = false;
    g_analog_ctx.thread_stop_requested = false;

    // Perform SAADC offset calibration
    LOG_INF("Performing SAADC offset calibration...");
    nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_CALIBRATEDONE);
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_CALIBRATEOFFSET);
    
    // Wait for calibration to complete (timeout after 100ms)
    uint32_t timeout = 10000; // 100ms in 10us increments
    while (!nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_CALIBRATEDONE) && timeout-- > 0) {
        k_usleep(10);
    }
    
    if (timeout == 0) {
        LOG_WRN("SAADC offset calibration timeout");
    } else {
        nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_CALIBRATEDONE);
        LOG_INF("SAADC offset calibration completed");
    }

    g_analog_ctx.initialized = true;

    LOG_INF("Analog driver initialized with %d channels", ANALOG_CHANNEL_COUNT);
    return ANALOG_STATUS_OK;
}

/**
 * @brief Read all analog channels and update filtered values
 */
analog_status_t analog_driver_read_all(void)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    // Read each channel individually
    for (int i = 0; i < ANALOG_CHANNEL_COUNT; i++)
    {
        struct adc_sequence sequence = {
            .buffer = &g_analog_ctx.raw_buffer[i],
            .buffer_size = sizeof(int16_t),
            .resolution = g_analog_ctx.config.resolution_bits,
            .channels = BIT(g_analog_ctx.channel_configs[i].adc_channel),
        };

        int ret = adc_read(g_analog_ctx.adc_dev, &sequence);
        if (ret != 0)
        {
            LOG_WRN("ADC read failed for channel %d (%s): %d",
                    i, g_analog_ctx.channel_configs[i].name, ret);
            continue;
        }

        analog_data_t *data = &g_analog_ctx.channel_data[i];
        analog_calibration_t *cal = &g_analog_ctx.calibrations[i];

        // Store raw value
        data->raw_value = g_analog_ctx.raw_buffer[i];

        // Apply low-pass filtering
        if (!g_analog_ctx.filter_initialized)
        {
            // Initialize filter with first reading
            data->filtered_value = (float)data->raw_value;
            g_analog_ctx.filter_initialized = true;
        }
        else
        {
            // Exponential moving average filter
            float alpha = g_analog_ctx.config.filter_alpha;
            data->filtered_value = alpha * (float)data->raw_value +
                                   (1.0f - alpha) * data->filtered_value;
        }

        // Apply calibration and scaling
        int16_t calibrated_value = (int16_t)data->filtered_value;

        if (i == ANALOG_CHANNEL_TRIGGER)
        {
            // Simple trigger scaling: map raw value between min/max to 0-255
            // For your hardware: rest=1500, pressed=1000
            // We want: 1500→0, 1000→255
            // The trigger is INVERTED: higher raw value = less pressed

            int32_t rest_value = cal->max_value;   // 1500 (trigger released)
            int32_t pressed_value = cal->min_value; // 1000 (trigger pressed)

            // Don't clamp input - let the scaling handle out-of-range values
            int32_t current_value = calibrated_value;

            // Calculate range
            int32_t range = rest_value - pressed_value; // 1500 - 1000 = 500
            int32_t scaled;

            // Inverted scaling: rest_value (1500) → 0, pressed_value (1000) → 255
            scaled = ((rest_value - current_value) * 255) / range;

            // Clamp output to 0-255 range for uint8_t
            if (scaled < 0)
                scaled = 0;
            if (scaled > 255)
                scaled = 255;

            // Store as uint8_t in the trigger_value union member
            data->controller_value.trigger_value = (uint8_t)scaled;
            data->in_deadzone = false;
        }
        else
        {
            // Stick: -127 to +127 with deadzone around center
            int16_t offset_from_center = calibrated_value - cal->center_value;

            // Invert Y axis (channel 1)
            if (i == ANALOG_CHANNEL_STICK_Y) {
                offset_from_center = -offset_from_center;
            }

            if (abs(offset_from_center) <= cal->deadzone)
            {
                data->controller_value.stick_value = 0;
                data->in_deadzone = true;
            }
            else
            {
                int32_t scaled = 0;

                if (offset_from_center > 0)
                {
                    // Positive direction
                    int32_t range = cal->max_value - (cal->center_value + cal->deadzone);
                    int32_t offset_value = offset_from_center - cal->deadzone;

                    if (range > 0)
                    {
                        scaled = (offset_value * 127) / range;
                    }

                    // Clamp BEFORE casting
                    if (scaled > 127)
                        scaled = 127;
                    if (scaled < 0)
                        scaled = 0;
                }
                else
                {
                    // Negative direction
                    int32_t range = (cal->center_value - cal->deadzone) - cal->min_value;
                    int32_t offset_value = abs(offset_from_center) - cal->deadzone;

                    if (range > 0)
                    {
                        scaled = -((offset_value * 127) / range);
                    }

                    // Clamp BEFORE casting
                    if (scaled < -127)
                        scaled = -127;
                    if (scaled > 0)
                        scaled = 0;
                }

                // Now safe to cast
                data->controller_value.stick_value = (int8_t)scaled;
                data->in_deadzone = false;
            }
        }
    }

    g_analog_ctx.sample_count++;
    return ANALOG_STATUS_OK;
}

/**
 * @brief Read a single analog channel (for round-robin reading to prevent blocking)
 */
analog_status_t analog_driver_read_single_channel(analog_channel_id_t channel_id)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (channel_id >= ANALOG_CHANNEL_COUNT)
    {
        return ANALOG_STATUS_INVALID_CHANNEL;
    }

    // Read only the specified channel
    struct adc_sequence sequence = {
        .buffer = &g_analog_ctx.raw_buffer[channel_id],
        .buffer_size = sizeof(int16_t),
        .resolution = g_analog_ctx.config.resolution_bits,
        .channels = BIT(g_analog_ctx.channel_configs[channel_id].adc_channel),
    };

    int ret = adc_read(g_analog_ctx.adc_dev, &sequence);
    if (ret != 0)
    {
        LOG_WRN("ADC read failed for channel %d (%s): %d",
                channel_id, g_analog_ctx.channel_configs[channel_id].name, ret);
        return ANALOG_STATUS_ADC_ERROR;
    }

    analog_data_t *data = &g_analog_ctx.channel_data[channel_id];
    analog_calibration_t *cal = &g_analog_ctx.calibrations[channel_id];

    // Store raw value
    data->raw_value = g_analog_ctx.raw_buffer[channel_id];

    // Apply low-pass filtering (same logic as read_all)
    if (!g_analog_ctx.filter_initialized)
    {
        data->filtered_value = (float)data->raw_value;
        g_analog_ctx.filter_initialized = true;
    }
    else
    {
        data->filtered_value = (g_analog_ctx.config.filter_alpha * data->raw_value) + 
                              ((1.0f - g_analog_ctx.config.filter_alpha) * data->filtered_value);
    }

    // Apply calibration and scaling (same logic as read_all)
    int32_t calibrated_value = (int32_t)data->filtered_value;
    
    if (channel_id == ANALOG_CHANNEL_TRIGGER)
    {
        // Trigger: 0 to 255
        int32_t scaled = ((calibrated_value - cal->min_value) * 255) / 
                        (cal->max_value - cal->min_value);
        if (scaled < 0) scaled = 0;
        if (scaled > 255) scaled = 255;
        data->controller_value.trigger_value = (uint8_t)scaled;
        data->in_deadzone = false;
    }
    else
    {
        // Stick: -127 to +127 with deadzone
        int16_t offset_from_center = calibrated_value - cal->center_value;
        
        // Invert Y axis
        if (channel_id == ANALOG_CHANNEL_STICK_Y) {
            offset_from_center = -offset_from_center;
        }
        
        if (abs(offset_from_center) <= cal->deadzone)
        {
            data->controller_value.stick_value = 0;
            data->in_deadzone = true;
        }
        else
        {
            int32_t scaled = 0;
            if (offset_from_center > 0)
            {
                int32_t range = cal->max_value - (cal->center_value + cal->deadzone);
                int32_t offset_value = offset_from_center - cal->deadzone;
                if (range > 0) {
                    scaled = (offset_value * 127) / range;
                }
                if (scaled > 127) scaled = 127;
                if (scaled < 0) scaled = 0;
            }
            else
            {
                int32_t range = (cal->center_value - cal->deadzone) - cal->min_value;
                int32_t offset_value = abs(offset_from_center) - cal->deadzone;
                if (range > 0) {
                    scaled = -((offset_value * 127) / range);
                }
                if (scaled < -127) scaled = -127;
                if (scaled > 0) scaled = 0;
            }
            data->controller_value.stick_value = (int8_t)scaled;
            data->in_deadzone = false;
        }
    }

    return ANALOG_STATUS_OK;
}

/**
 * @brief Get controller-format analog data (thread-safe)
 */
analog_status_t analog_driver_get_controller_data(analog_controller_data_t *data)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (!data)
    {
        return ANALOG_STATUS_ERROR;
    }

    // Thread-safe data access
    k_mutex_lock(&g_analog_ctx.data_mutex, K_FOREVER);
    
    data->stick_x = g_analog_ctx.channel_data[ANALOG_CHANNEL_STICK_X].controller_value.stick_value;
    data->stick_y = g_analog_ctx.channel_data[ANALOG_CHANNEL_STICK_Y].controller_value.stick_value;
    data->trigger = g_analog_ctx.channel_data[ANALOG_CHANNEL_TRIGGER].controller_value.trigger_value;
    
    k_mutex_unlock(&g_analog_ctx.data_mutex);

    return ANALOG_STATUS_OK;
}

/**
 * @brief Get battery voltage in millivolts (thread-safe)
 */
analog_status_t analog_driver_get_battery_voltage(uint16_t *voltage_mv)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (!voltage_mv)
    {
        return ANALOG_STATUS_ERROR;
    }

    // Thread-safe data access
    k_mutex_lock(&g_analog_ctx.data_mutex, K_FOREVER);
    
    // Get raw ADC value for battery channel
    int16_t raw_value = g_analog_ctx.channel_data[ANALOG_CHANNEL_BATTERY].raw_value;
    
    k_mutex_unlock(&g_analog_ctx.data_mutex);

    // Convert to voltage (1/6 gain, 0.6V ref, 12-bit ADC)
    // Voltage at pin = (raw/4095) * 3.6V
    float pin_voltage = ((float)raw_value / 4095.0f) * 3.6f;
    
    // Account for XIAO voltage divider (1MΩ + 510kΩ)
    // Divider ratio = (R1 + R2) / R2 = (1000000 + 510000) / 510000 = 2.96
    float battery_voltage = pin_voltage * 2.96f;
    
    // Convert to millivolts
    *voltage_mv = (uint16_t)(battery_voltage * 1000.0f);

    return ANALOG_STATUS_OK;
}

// ADC thread stack
#define ADC_THREAD_STACK_SIZE 2048  // Increased from 1024
K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_THREAD_STACK_SIZE);

/**
 * @brief ADC reading thread function
 */
static void adc_thread_function(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    
    LOG_INF("ADC thread started");
    
    while (!g_analog_ctx.thread_stop_requested) {
        // Read each channel individually to prevent blocking
        for (int i = 0; i < ANALOG_CHANNEL_COUNT; i++) {
            if (g_analog_ctx.thread_stop_requested) {
                break;
            }
            
            struct adc_sequence sequence = {
                .buffer = &g_analog_ctx.raw_buffer[i],
                .buffer_size = sizeof(int16_t),
                .resolution = g_analog_ctx.config.resolution_bits,
                .channels = BIT(g_analog_ctx.channel_configs[i].adc_channel),
            };

            int ret = adc_read(g_analog_ctx.adc_dev, &sequence);
            if (ret != 0) {
                LOG_WRN("ADC read failed for channel %d (%s): %d",
                        i, g_analog_ctx.channel_configs[i].name, ret);
                continue;
            }
            
            // Process the reading with mutex protection (simple approach)
            k_mutex_lock(&g_analog_ctx.data_mutex, K_FOREVER);
            
            analog_data_t *data = &g_analog_ctx.channel_data[i];
            analog_calibration_t *cal = &g_analog_ctx.calibrations[i];
            
            // Store raw value
            data->raw_value = g_analog_ctx.raw_buffer[i];
            
            // Apply low-pass filtering
            if (!g_analog_ctx.filter_initialized) {
                data->filtered_value = (float)data->raw_value;
                g_analog_ctx.filter_initialized = true;
            } else {
                data->filtered_value = g_analog_ctx.config.filter_alpha * (float)data->raw_value +
                                     (1.0f - g_analog_ctx.config.filter_alpha) * data->filtered_value;
            }
            
            // Apply calibration and scaling
            int16_t calibrated_value = (int16_t)data->filtered_value;

            if (i == ANALOG_CHANNEL_TRIGGER) {
                // Trigger scaling (inverted)
                int32_t rest_value = cal->max_value;
                int32_t pressed_value = cal->min_value;
                int32_t current_value = calibrated_value;
                int32_t range = rest_value - pressed_value;
                int32_t scaled = ((rest_value - current_value) * 255) / range;
                
                if (scaled < 0) scaled = 0;
                if (scaled > 255) scaled = 255;
                
                data->controller_value.trigger_value = (uint8_t)scaled;
                data->in_deadzone = false;
            } else {
                // Stick scaling with deadzone
                int16_t offset_from_center = calibrated_value - cal->center_value;
                
                // Invert Y axis
                if (i == ANALOG_CHANNEL_STICK_Y) {
                    offset_from_center = -offset_from_center;
                }
                
                if (abs(offset_from_center) <= cal->deadzone) {
                    data->controller_value.stick_value = 0;
                    data->in_deadzone = true;
                } else {
                    int32_t scaled = 0;
                    
                    if (offset_from_center > 0) {
                        int32_t range = cal->max_value - (cal->center_value + cal->deadzone);
                        int32_t offset_value = offset_from_center - cal->deadzone;
                        if (range > 0) {
                            scaled = (offset_value * 127) / range;
                        }
                        if (scaled > 127) scaled = 127;
                        if (scaled < 0) scaled = 0;
                    } else {
                        int32_t range = (cal->center_value - cal->deadzone) - cal->min_value;
                        int32_t offset_value = abs(offset_from_center) - cal->deadzone;
                        if (range > 0) {
                            scaled = -((offset_value * 127) / range);
                        }
                        if (scaled < -127) scaled = -127;
                        if (scaled > 0) scaled = 0;
                    }
                    
                    data->controller_value.stick_value = (int8_t)scaled;
                    data->in_deadzone = false;
                }
            }
            
            k_mutex_unlock(&g_analog_ctx.data_mutex);
        }
        
        g_analog_ctx.sample_count++;
        
        // Debug logging every 500 samples (about every 2 seconds at 250Hz)
        static uint32_t debug_counter = 0;
        debug_counter++;
        if (debug_counter >= 500) {
            LOG_INF("ADC thread alive - %u samples processed", g_analog_ctx.sample_count);
            debug_counter = 0;
        }

        // Sleep for 2ms between complete readings (500Hz effective rate - stable and conservative)
        k_msleep(2);
    }
    
    LOG_INF("ADC thread stopped");
    g_analog_ctx.thread_running = false;
}

/**
 * @brief Start the ADC reading thread
 */
analog_status_t analog_driver_start_thread(void)
{
    if (!g_analog_ctx.initialized) {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }
    
    if (g_analog_ctx.thread_running) {
        LOG_WRN("ADC thread already running");
        return ANALOG_STATUS_OK;
    }
    
    g_analog_ctx.thread_stop_requested = false;
    g_analog_ctx.adc_thread_tid = k_thread_create(&g_analog_ctx.adc_thread_data,
                                                  adc_thread_stack,
                                                  K_THREAD_STACK_SIZEOF(adc_thread_stack),
                                                  adc_thread_function,
                                                  NULL, NULL, NULL,
                                                  K_PRIO_PREEMPT(10), // Lower priority to not interfere with radio
                                                  0, K_NO_WAIT);
    
    if (!g_analog_ctx.adc_thread_tid) {
        LOG_ERR("Failed to create ADC thread");
        return ANALOG_STATUS_ERROR;
    }
    
    g_analog_ctx.thread_running = true;
    LOG_INF("ADC thread started successfully");
    return ANALOG_STATUS_OK;
}

/**
 * @brief Stop the ADC reading thread
 */
analog_status_t analog_driver_stop_thread(void)
{
    if (!g_analog_ctx.thread_running) {
        return ANALOG_STATUS_OK;
    }
    
    LOG_INF("Stopping ADC thread...");
    g_analog_ctx.thread_stop_requested = true;
    
    // Wait for thread to finish (timeout after 1 second)
    uint32_t timeout = 100; // 1 second in 10ms increments
    while (g_analog_ctx.thread_running && timeout-- > 0) {
        k_msleep(10);
    }
    
    if (g_analog_ctx.thread_running) {
        LOG_WRN("ADC thread stop timeout");
        return ANALOG_STATUS_ERROR;
    }
    
    LOG_INF("ADC thread stopped successfully");
    return ANALOG_STATUS_OK;
}

/**
 * @brief Get raw ADC value for a specific channel
 */
analog_status_t analog_driver_get_raw_value(analog_channel_id_t channel_id, int16_t *raw_value)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (channel_id >= ANALOG_CHANNEL_COUNT || !raw_value)
    {
        return ANALOG_STATUS_INVALID_CHANNEL;
    }

    *raw_value = g_analog_ctx.channel_data[channel_id].raw_value;
    return ANALOG_STATUS_OK;
}

/**
 * @brief Get filtered value for a specific channel
 */
analog_status_t analog_driver_get_filtered_value(analog_channel_id_t channel_id, float *filtered_value)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (channel_id >= ANALOG_CHANNEL_COUNT || !filtered_value)
    {
        return ANALOG_STATUS_INVALID_CHANNEL;
    }

    *filtered_value = g_analog_ctx.channel_data[channel_id].filtered_value;
    return ANALOG_STATUS_OK;
}

/**
 * @brief Get controller-scaled value for a specific channel
 */
analog_status_t analog_driver_get_controller_value(analog_channel_id_t channel_id, int8_t *controller_value)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (channel_id >= ANALOG_CHANNEL_COUNT || !controller_value)
    {
        return ANALOG_STATUS_INVALID_CHANNEL;
    }

    *controller_value = g_analog_ctx.channel_data[channel_id].controller_value.stick_value;
    return ANALOG_STATUS_OK;
}

/**
 * @brief Calibrate a specific analog channel
 */
analog_status_t analog_driver_calibrate_channel(analog_channel_id_t channel_id,
                                                int16_t center_value,
                                                int16_t min_value,
                                                int16_t max_value,
                                                int16_t deadzone)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (channel_id >= ANALOG_CHANNEL_COUNT)
    {
        return ANALOG_STATUS_INVALID_CHANNEL;
    }

    analog_calibration_t *cal = &g_analog_ctx.calibrations[channel_id];

    cal->center_value = center_value;
    cal->min_value = min_value;
    cal->max_value = max_value;
    cal->deadzone = deadzone;
    cal->is_calibrated = true;

    LOG_INF("Channel %d (%s) calibrated: center=%d, min=%d, max=%d, deadzone=%d",
            channel_id, g_analog_ctx.channel_configs[channel_id].name,
            center_value, min_value, max_value, deadzone);

    return ANALOG_STATUS_OK;
}

/**
 * @brief Auto-calibrate analog stick channels (call with stick centered)
 */
analog_status_t analog_driver_auto_calibrate_sticks(uint16_t samples)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    LOG_INF("=== ANALOG STICK AUTO-CALIBRATION ===");
    LOG_INF("Make sure analog stick is centered and not being touched");
    LOG_INF("Calibration will start in 3 seconds...");

    k_sleep(K_MSEC(3000));

    LOG_INF("Taking %d calibration samples...", samples);

    // Accumulate samples for averaging
    float stick_x_sum = 0;
    float stick_y_sum = 0;

    for (int i = 0; i < samples; i++)
    {
        analog_status_t status = analog_driver_read_all();
        if (status != ANALOG_STATUS_OK)
        {
            LOG_ERR("Failed to read ADC during calibration: %d", status);
            return ANALOG_STATUS_CALIBRATION_FAILED;
        }

        stick_x_sum += g_analog_ctx.channel_data[ANALOG_CHANNEL_STICK_X].raw_value;
        stick_y_sum += g_analog_ctx.channel_data[ANALOG_CHANNEL_STICK_Y].raw_value;

        k_sleep(K_MSEC(20)); // 20ms between samples
    }

    // Calculate average center values
    int16_t stick_x_center = (int16_t)(stick_x_sum / samples);
    int16_t stick_y_center = (int16_t)(stick_y_sum / samples);

    // Set reasonable deadzone (about 2.5% of full range)
    int16_t deadzone = 100;

    // Apply calibration
    analog_driver_calibrate_channel(ANALOG_CHANNEL_STICK_X, stick_x_center, 0, 4095, deadzone);
    analog_driver_calibrate_channel(ANALOG_CHANNEL_STICK_Y, stick_y_center, 0, 4095, deadzone);

    LOG_INF("=== STICK CALIBRATION COMPLETE ===");
    LOG_INF("Stick X center: %d", stick_x_center);
    LOG_INF("Stick Y center: %d", stick_y_center);
    LOG_INF("Deadzone: %d", deadzone);

    return ANALOG_STATUS_OK;
}

/**
 * @brief Auto-calibrate trigger channel (call with trigger released)
 */
analog_status_t analog_driver_auto_calibrate_trigger(uint16_t samples)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    LOG_INF("=== TRIGGER AUTO-CALIBRATION ===");
    LOG_INF("Make sure trigger is fully released");
    LOG_INF("Calibration will start in 3 seconds...");

    k_sleep(K_MSEC(3000));

    LOG_INF("Taking %d calibration samples...", samples);

    float trigger_sum = 0;

    for (int i = 0; i < samples; i++)
    {
        analog_status_t status = analog_driver_read_all();
        if (status != ANALOG_STATUS_OK)
        {
            LOG_ERR("Failed to read ADC during trigger calibration: %d", status);
            return ANALOG_STATUS_CALIBRATION_FAILED;
        }

        trigger_sum += g_analog_ctx.channel_data[ANALOG_CHANNEL_TRIGGER].raw_value;
        k_sleep(K_MSEC(20));
    }

    int16_t trigger_min = (int16_t)(trigger_sum / samples);
    int16_t trigger_max = 4000; // Leave some headroom
    int16_t deadzone = 50;      // Small deadzone for trigger

    analog_driver_calibrate_channel(ANALOG_CHANNEL_TRIGGER, trigger_min, trigger_min, trigger_max, deadzone);

    LOG_INF("=== TRIGGER CALIBRATION COMPLETE ===");
    LOG_INF("Trigger min: %d", trigger_min);
    LOG_INF("Trigger max: %d", trigger_max);
    LOG_INF("Deadzone: %d", deadzone);

    return ANALOG_STATUS_OK;
}

/**
 * @brief Set filter coefficient for low-pass filtering
 */
analog_status_t analog_driver_set_filter_alpha(float alpha)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (alpha < 0.0f || alpha > 1.0f)
    {
        return ANALOG_STATUS_ERROR;
    }

    g_analog_ctx.config.filter_alpha = alpha;
    LOG_INF("Filter alpha set to %.2f", alpha);

    return ANALOG_STATUS_OK;
}

/**
 * @brief Check if a channel is in its deadzone
 */
bool analog_driver_is_in_deadzone(analog_channel_id_t channel_id)
{
    if (!g_analog_ctx.initialized || channel_id >= ANALOG_CHANNEL_COUNT)
    {
        return false;
    }

    return g_analog_ctx.channel_data[channel_id].in_deadzone;
}

/**
 * @brief Get calibration data for a channel
 */
analog_status_t analog_driver_get_calibration(analog_channel_id_t channel_id,
                                              analog_calibration_t *calibration)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (channel_id >= ANALOG_CHANNEL_COUNT || !calibration)
    {
        return ANALOG_STATUS_INVALID_CHANNEL;
    }

    *calibration = g_analog_ctx.calibrations[channel_id];
    return ANALOG_STATUS_OK;
}

/**
 * @brief Set calibration data for a channel
 */
analog_status_t analog_driver_set_calibration(analog_channel_id_t channel_id,
                                              const analog_calibration_t *calibration)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (channel_id >= ANALOG_CHANNEL_COUNT || !calibration)
    {
        return ANALOG_STATUS_INVALID_CHANNEL;
    }

    g_analog_ctx.calibrations[channel_id] = *calibration;
    return ANALOG_STATUS_OK;
}

/**
 * @brief Get driver statistics
 */
analog_status_t analog_driver_get_stats(uint32_t *total_samples, uint8_t *active_channels)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    if (total_samples)
    {
        *total_samples = g_analog_ctx.sample_count;
    }

    if (active_channels)
    {
        uint8_t count = 0;
        for (int i = 0; i < ANALOG_CHANNEL_COUNT; i++)
        {
            if (!g_analog_ctx.channel_data[i].in_deadzone)
            {
                count++;
            }
        }
        *active_channels = count;
    }

    return ANALOG_STATUS_OK;
}

/**
 * @brief Check if analog driver is properly initialized
 */
bool analog_driver_is_initialized(void)
{
    return g_analog_ctx.initialized;
}

/**
 * @brief Reset filter state
 */
analog_status_t analog_driver_reset_filters(void)
{
    if (!g_analog_ctx.initialized)
    {
        return ANALOG_STATUS_NOT_INITIALIZED;
    }

    g_analog_ctx.filter_initialized = false;
    LOG_INF("Analog filters reset");

    return ANALOG_STATUS_OK;
}

/**
 * @brief Get human-readable name for a channel
 */
const char *analog_driver_get_channel_name(analog_channel_id_t channel_id)
{
    if (channel_id >= ANALOG_CHANNEL_COUNT)
    {
        return "UNKNOWN";
    }

    return channel_names[channel_id];
}

/**
 * @brief Perform a complete calibration sequence for all channels
 */
analog_status_t analog_driver_full_calibration(uint32_t delay_ms)
{
    LOG_INF("=== FULL ANALOG CALIBRATION SEQUENCE ===");

    // Calibrate sticks first
    analog_status_t status = analog_driver_auto_calibrate_sticks(50);
    if (status != ANALOG_STATUS_OK)
    {
        LOG_ERR("Stick calibration failed: %d", status);
        return status;
    }

    k_sleep(K_MSEC(delay_ms));

    // Calibrate trigger
    status = analog_driver_auto_calibrate_trigger(50);
    if (status != ANALOG_STATUS_OK)
    {
        LOG_ERR("Trigger calibration failed: %d", status);
        return status;
    }

    LOG_INF("=== FULL CALIBRATION COMPLETE ===");
    return ANALOG_STATUS_OK;
}
