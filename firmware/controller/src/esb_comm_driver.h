/*
 * ESB Communication Driver - Enhanced ShockBurst Communication
 * 
 * This driver provides ESB (Enhanced ShockBurst) wireless communication
 * functionality for the controller system. It handles initialization,
 * data transmission with adaptive timing, and power management.
 */

#ifndef ESB_COMM_DRIVER_H
#define ESB_COMM_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

// ESB communication status codes
typedef enum {
    ESB_COMM_STATUS_OK = 0,
    ESB_COMM_STATUS_ERROR = -1,
    ESB_COMM_STATUS_NOT_INITIALIZED = -2,
    ESB_COMM_STATUS_CLOCK_FAILED = -3,
    ESB_COMM_STATUS_CONFIG_FAILED = -4,
    ESB_COMM_STATUS_TX_FAILED = -5,
    ESB_COMM_STATUS_BUSY = -6
} esb_comm_status_t;

// ACK timing data structure (received from dongle in ACK payload)
typedef struct {
    uint16_t next_delay_ms;      // 2 bytes: 0-65535ms timing control
    uint8_t sequence_num;        // 1 byte: debugging/sync tracking
    uint8_t rumble_data;         // 1 byte: rumble control (upper 4 bits=left motor, lower 4 bits=right motor)
    uint32_t dongle_timestamp;   // 4 bytes: dongle time sync
} __packed ack_timing_data_t;   // Total: 8 bytes

// Controller data structure for transmission
typedef struct
{
    uint8_t flags;   // Mode bits, controller ID, mouse buttons, trigger buttons
    uint8_t trigger;  // Analog trigger (L2/R2)
    int8_t stickX;   // Analog stick X
    int8_t stickY;   // Analog stick Y
    int16_t padX;     // Trackpad X
    int16_t padY;     // Trackpad Y
    uint8_t buttons; // Gamepad buttons
    int16_t accelX;  // IMU accelerometer X (-32768 to 32767)
    int16_t accelY;  // IMU accelerometer Y (-32768 to 32767)
    int16_t accelZ;  // IMU accelerometer Z (-32768 to 32767)
    int16_t gyroX;   // IMU gyroscope X (-32768 to 32767)
    int16_t gyroY;   // IMU gyroscope Y (-32768 to 32767)
    int16_t gyroZ;   // IMU gyroscope Z (-32768 to 32767)
} __packed esb_controller_data_t;

// ESB communication configuration
typedef struct {
    uint8_t controller_id;           // 0=RIGHT, 1=LEFT
    uint32_t base_tx_interval_ms;    // Base transmission interval (default 8ms)
    uint32_t retry_interval_ms;      // Retry interval on failed TX (default 16ms)
    uint8_t rf_channel;              // RF channel (default 1)
    const struct gpio_dt_spec *status_led; // Optional status LED
} esb_comm_config_t;

// ESB communication statistics
typedef struct {
    uint32_t total_transmissions;
    uint32_t successful_transmissions;
    uint32_t failed_transmissions;
    uint32_t retry_count;
    float success_rate;
    uint32_t last_tx_timestamp;
    bool last_tx_succeeded;
} esb_comm_stats_t;

// Function prototypes

/**
 * Initialize ESB communication driver
 * @param config Pointer to configuration structure
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_driver_init(const esb_comm_config_t *config);

/**
 * Check if ESB communication driver is initialized
 * @return true if initialized, false otherwise
 */
bool esb_comm_driver_is_initialized(void);

/**
 * Send controller data via ESB with adaptive timing
 * This function implements adaptive timing based on previous TX success/failure
 * @param data Pointer to controller data to transmit
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_send_data(const esb_controller_data_t *data);

/**
 * Send controller data via ESB (non-blocking)
 * This function only transmits if enough time has passed since last attempt
 * @param data Pointer to controller data to transmit
 * @return ESB_COMM_STATUS_OK if transmitted, other codes if skipped or failed
 */
esb_comm_status_t esb_comm_send_data_timed(const esb_controller_data_t *data);

/**
 * Get current transmission statistics
 * @param stats Pointer to store statistics
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_get_stats(esb_comm_stats_t *stats);

/**
 * Reset transmission statistics
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_reset_stats(void);

/**
 * Set transmission timing parameters
 * @param base_interval_ms Base transmission interval in milliseconds
 * @param retry_interval_ms Retry interval for failed transmissions in milliseconds
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_set_timing(uint32_t base_interval_ms, uint32_t retry_interval_ms);

/**
 * Enable or disable ESB communication (for power management)
 * @param enable true to enable, false to disable
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_enable(bool enable);

/**
 * Enter low power mode (disables ESB)
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_enter_sleep(void);

/**
 * Wake up from low power mode (re-enables ESB)
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_wakeup(void);

/**
 * Get current ESB communication status
 * @return true if enabled and ready for transmission, false otherwise
 */
bool esb_comm_is_ready(void);

/**
 * Force immediate transmission (bypass timing)
 * Used for critical updates that can't wait for timing intervals
 * @param data Pointer to controller data to transmit
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_send_immediate(const esb_controller_data_t *data);

/**
 * Get last transmission timestamp
 * @return timestamp of last transmission attempt in milliseconds
 */
uint32_t esb_comm_get_last_tx_time(void);

/**
 * Check if last transmission was successful
 * @return true if last transmission received ACK, false otherwise
 */
bool esb_comm_get_last_tx_success(void);

/**
 * Get current rumble motor values (from last ACK payload)
 * @param left_motor Pointer to store left motor intensity (0-15)
 * @param right_motor Pointer to store right motor intensity (0-15)
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_get_rumble_data(uint8_t *left_motor, uint8_t *right_motor);

/**
 * Get the last received ACK timing data
 * @param timing_data Pointer to store the timing data
 * @return ESB_COMM_STATUS_OK if valid timing data available, error code otherwise
 */
esb_comm_status_t esb_comm_get_ack_timing(ack_timing_data_t *timing_data);

/**
 * Check if ACK payload based timing is active
 * @return true if using ACK payload timing, false if using fixed timing
 */
bool esb_comm_is_ack_timing_active(void);

/**
 * Enable or disable ACK payload based timing control
 * @param enable true to enable ACK timing, false to use fixed timing
 * @return ESB_COMM_STATUS_OK on success, error code on failure
 */
esb_comm_status_t esb_comm_enable_ack_timing(bool enable);

/**
 * Get the next transmission delay from ACK payload timing
 * @return next delay in milliseconds, 0 if no timing data available
 */
uint16_t esb_comm_get_next_delay(void);

#ifdef __cplusplus
}
#endif

#endif // ESB_COMM_DRIVER_H
