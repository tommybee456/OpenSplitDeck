/**
 ******************************************************************************
 * @file    analog_driver.h
 * @brief   ADC Analog Input Driver Library for Controller
 * @author  Controller Team
 * @version V1.0
 * @date    2025
 ******************************************************************************
 * @attention
 * 
 * This library provides a clean interface for managing ADC analog inputs
 * including analog sticks, triggers, and other analog sensors with
 * calibration, filtering, and deadzone support.
 * 
 ******************************************************************************
 */

#ifndef ANALOG_DRIVER_H
#define ANALOG_DRIVER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <stdint.h>
#include <stdbool.h>

// Analog status enumeration
typedef enum {
    ANALOG_STATUS_OK = 0,
    ANALOG_STATUS_ERROR = -1,
    ANALOG_STATUS_NOT_INITIALIZED = -2,
    ANALOG_STATUS_INVALID_CHANNEL = -3,
    ANALOG_STATUS_CALIBRATION_FAILED = -4,
    ANALOG_STATUS_ADC_ERROR = -5
} analog_status_t;

// Analog channel enumeration
typedef enum {
    ANALOG_CHANNEL_STICK_X = 0,
    ANALOG_CHANNEL_STICK_Y,
    ANALOG_CHANNEL_TRIGGER,
    ANALOG_CHANNEL_BATTERY,     // Battery voltage monitoring
    ANALOG_CHANNEL_COUNT  // Total number of analog channels
} analog_channel_id_t;

// Analog input configuration
typedef struct {
    uint8_t adc_channel;        // ADC channel number
    uint32_t adc_input;         // ADC input pin (e.g., NRF_SAADC_INPUT_AIN0)
    const char *name;           // Human-readable name
} analog_channel_config_t;

// Calibration data for an analog channel
typedef struct {
    int16_t center_value;       // Center/neutral position value
    int16_t min_value;          // Minimum expected value
    int16_t max_value;          // Maximum expected value
    int16_t deadzone;           // Deadzone around center
    bool is_calibrated;         // True if channel has been calibrated
} analog_calibration_t;

// Filtered analog data
typedef struct {
    int16_t raw_value;          // Raw ADC reading
    float filtered_value;       // Low-pass filtered value
    union {
        int8_t stick_value;     // Stick value (-127 to +127)
        uint8_t trigger_value;  // Trigger value (0 to 255)
    } controller_value;         // Scaled value for controller
    bool in_deadzone;           // True if value is within deadzone
} analog_data_t;

// Analog driver configuration
typedef struct {
    uint16_t resolution_bits;   // ADC resolution (e.g., 12)
    uint32_t gain;              // ADC gain setting
    uint32_t reference;         // ADC reference voltage
    uint32_t acquisition_time;  // ADC acquisition time
    float filter_alpha;         // Low-pass filter coefficient (0.0-1.0)
} analog_config_t;

// Analog driver context
typedef struct {
    bool initialized;
    const struct device *adc_dev;
    analog_config_t config;
    analog_channel_config_t channel_configs[ANALOG_CHANNEL_COUNT];
    struct adc_channel_cfg adc_channel_configs[ANALOG_CHANNEL_COUNT];
    analog_calibration_t calibrations[ANALOG_CHANNEL_COUNT];
    analog_data_t channel_data[ANALOG_CHANNEL_COUNT];
    int16_t raw_buffer[ANALOG_CHANNEL_COUNT];
    bool filter_initialized;
    uint32_t sample_count;
    
    // Thread-based ADC reading
    struct k_thread adc_thread_data;
    k_tid_t adc_thread_tid;
    struct k_mutex data_mutex;  // Protect channel_data access
    bool thread_running;
    bool thread_stop_requested;
} analog_driver_context_t;

// Controller analog data structure (matches main.c format)
typedef struct {
    int8_t stick_x;     // Analog stick X (-127 to +127)
    int8_t stick_y;     // Analog stick Y (-127 to +127)
    uint8_t trigger;    // Trigger value (0 to 255)
} analog_controller_data_t;

// Function prototypes

/**
 * @brief Initialize the analog driver with SAADC offset calibration
 * @param adc_device Pointer to ADC device
 * @return analog_status_t Status of initialization
 */
analog_status_t analog_driver_init(const struct device *adc_device);

/**
 * @brief Start the ADC reading thread
 * @return analog_status_t Status of thread start operation
 */
analog_status_t analog_driver_start_thread(void);

/**
 * @brief Stop the ADC reading thread
 * @return analog_status_t Status of thread stop operation
 */
analog_status_t analog_driver_stop_thread(void);

/**
 * @brief Get controller-format analog data (thread-safe)
 * @param data Pointer to analog_controller_data_t structure to fill
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_get_controller_data(analog_controller_data_t *data);

/**
 * @brief Get battery voltage in millivolts (thread-safe)
 * @param voltage_mv Pointer to store battery voltage in mV
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_get_battery_voltage(uint16_t *voltage_mv);

/**
 * @brief Get raw ADC value for a specific channel
 * @param channel_id Channel to read
 * @param raw_value Pointer to store raw ADC value
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_get_raw_value(analog_channel_id_t channel_id, int16_t *raw_value);

/**
 * @brief Get filtered value for a specific channel
 * @param channel_id Channel to read
 * @param filtered_value Pointer to store filtered value
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_get_filtered_value(analog_channel_id_t channel_id, float *filtered_value);

/**
 * @brief Get controller-scaled value for a specific channel
 * @param channel_id Channel to read
 * @param controller_value Pointer to store controller value
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_get_controller_value(analog_channel_id_t channel_id, int8_t *controller_value);

/**
 * @brief Calibrate a specific analog channel
 * @param channel_id Channel to calibrate
 * @param center_value Center/neutral position value
 * @param min_value Minimum expected value
 * @param max_value Maximum expected value
 * @param deadzone Deadzone around center
 * @return analog_status_t Status of calibration
 */
analog_status_t analog_driver_calibrate_channel(analog_channel_id_t channel_id, 
                                               int16_t center_value, 
                                               int16_t min_value, 
                                               int16_t max_value, 
                                               int16_t deadzone);

/**
 * @brief Auto-calibrate analog stick channels (call with stick centered)
 * @param samples Number of samples to average for calibration
 * @return analog_status_t Status of auto-calibration
 */
analog_status_t analog_driver_auto_calibrate_sticks(uint16_t samples);

/**
 * @brief Auto-calibrate trigger channel (call with trigger released)
 * @param samples Number of samples to average for calibration
 * @return analog_status_t Status of auto-calibration
 */
analog_status_t analog_driver_auto_calibrate_trigger(uint16_t samples);

/**
 * @brief Set filter coefficient for low-pass filtering
 * @param alpha Filter coefficient (0.0 = heavy filtering, 1.0 = no filtering)
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_set_filter_alpha(float alpha);

/**
 * @brief Check if a channel is in its deadzone
 * @param channel_id Channel to check
 * @return bool True if in deadzone, false otherwise
 */
bool analog_driver_is_in_deadzone(analog_channel_id_t channel_id);

/**
 * @brief Get calibration data for a channel
 * @param channel_id Channel to get calibration for
 * @param calibration Pointer to store calibration data
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_get_calibration(analog_channel_id_t channel_id, 
                                             analog_calibration_t *calibration);

/**
 * @brief Set calibration data for a channel
 * @param channel_id Channel to set calibration for
 * @param calibration Pointer to calibration data
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_set_calibration(analog_channel_id_t channel_id, 
                                             const analog_calibration_t *calibration);

/**
 * @brief Get driver statistics
 * @param total_samples Pointer to store total samples taken
 * @param active_channels Pointer to store number of active channels
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_get_stats(uint32_t *total_samples, uint8_t *active_channels);

/**
 * @brief Check if analog driver is properly initialized
 * @return bool True if initialized, false otherwise
 */
bool analog_driver_is_initialized(void);

/**
 * @brief Reset filter state (call when stick positions change rapidly)
 * @return analog_status_t Status of operation
 */
analog_status_t analog_driver_reset_filters(void);

/**
 * @brief Get human-readable name for a channel
 * @param channel_id Channel ID
 * @return const char* Channel name string
 */
const char* analog_driver_get_channel_name(analog_channel_id_t channel_id);

/**
 * @brief Perform a complete calibration sequence for all channels
 * @param delay_ms Delay between calibration steps in milliseconds
 * @return analog_status_t Status of complete calibration
 */
analog_status_t analog_driver_full_calibration(uint32_t delay_ms);

#endif // ANALOG_DRIVER_H
