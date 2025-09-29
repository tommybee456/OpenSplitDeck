/**
 * @file imu_driver.h
 * @brief IMU sensor driver for LSM6DS3TR-C
 * 
 * This library provides a clean interface for managing the LSM6DS3TR-C IMU sensor
 * including initialization, data reading, filtering, and power management.
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IMU driver status enumeration
 */
typedef enum {
    IMU_STATUS_NOT_READY = 0,        /**< IMU not initialized */
    IMU_STATUS_READY = 1,            /**< IMU ready and operational */
    IMU_STATUS_ERROR = 2,            /**< IMU error state */
    IMU_STATUS_NOT_DETECTED = 3,     /**< IMU not detected on I2C bus */
    IMU_STATUS_CALIBRATING = 4       /**< IMU in calibration mode */
} imu_status_t;

/**
 * @brief IMU configuration structure
 */
typedef struct {
    uint16_t accel_odr;              /**< Accelerometer output data rate (Hz) */
    uint16_t gyro_odr;               /**< Gyroscope output data rate (Hz) */
    float filter_alpha;              /**< Low-pass filter coefficient (0.0-1.0) */
    bool auto_calibrate;             /**< Enable automatic calibration on startup */
    float accel_scale_factor;        /**< Accelerometer scaling factor */
    float gyro_scale_factor;         /**< Gyroscope scaling factor */
} imu_config_t;

/**
 * @brief Raw IMU sensor data structure
 */
typedef struct {
    double accel_x;                  /**< Raw accelerometer X (m/s²) */
    double accel_y;                  /**< Raw accelerometer Y (m/s²) */
    double accel_z;                  /**< Raw accelerometer Z (m/s²) */
    double gyro_x;                   /**< Raw gyroscope X (rad/s) */
    double gyro_y;                   /**< Raw gyroscope Y (rad/s) */
    double gyro_z;                   /**< Raw gyroscope Z (rad/s) */
    uint32_t timestamp;              /**< Data timestamp (ms) */
} imu_raw_data_t;

/**
 * @brief Filtered IMU data structure
 */
typedef struct {
    double accel_x;                  /**< Filtered accelerometer X */
    double accel_y;                  /**< Filtered accelerometer Y */
    double accel_z;                  /**< Filtered accelerometer Z */
    double gyro_x;                   /**< Filtered gyroscope X */
    double gyro_y;                   /**< Filtered gyroscope Y */
    double gyro_z;                   /**< Filtered gyroscope Z */
    bool filter_initialized;         /**< Filter initialization status */
} imu_filtered_data_t;

/**
 * @brief Scaled IMU data for controller output
 */
typedef struct {
    int16_t accel_x;                 /**< Scaled accelerometer X (-32768 to 32767) */
    int16_t accel_y;                 /**< Scaled accelerometer Y (-32768 to 32767) */
    int16_t accel_z;                 /**< Scaled accelerometer Z (-32768 to 32767) */
    int16_t gyro_x;                  /**< Scaled gyroscope X (-32768 to 32767) */
    int16_t gyro_y;                  /**< Scaled gyroscope Y (-32768 to 32767) */
    int16_t gyro_z;                  /**< Scaled gyroscope Z (-32768 to 32767) */
} imu_controller_data_t;

/**
 * @brief IMU calibration data structure
 */
typedef struct {
    double accel_offset_x;           /**< Accelerometer X offset */
    double accel_offset_y;           /**< Accelerometer Y offset */
    double accel_offset_z;           /**< Accelerometer Z offset */
    double gyro_offset_x;            /**< Gyroscope X offset */
    double gyro_offset_y;            /**< Gyroscope Y offset */
    double gyro_offset_z;            /**< Gyroscope Z offset */
    bool calibrated;                 /**< Calibration completion flag */
} imu_calibration_t;

/**
 * @brief IMU driver context structure
 */
typedef struct {
    const struct device *sensor_dev; /**< Zephyr sensor device */
    const struct device *i2c_dev;   /**< I2C device for scanning */
    imu_status_t status;             /**< Current IMU status */
    imu_config_t config;             /**< IMU configuration */
    imu_raw_data_t raw_data;         /**< Latest raw sensor data */
    imu_filtered_data_t filtered_data; /**< Filtered sensor data */
    imu_calibration_t calibration;   /**< Calibration data */
    uint32_t read_count;             /**< Total number of successful reads */
    uint32_t error_count;            /**< Total number of read errors */
} imu_context_t;

// ============================================================================
// Core IMU Functions
// ============================================================================

/**
 * @brief Initialize the IMU driver system
 * @param sensor_dev Zephyr sensor device pointer
 * @param i2c_dev I2C device pointer for bus scanning
 * @param config IMU configuration (NULL for defaults)
 * @return 0 on success, negative error code on failure
 */
int imu_driver_init(const struct device *sensor_dev, 
                   const struct device *i2c_dev,
                   const imu_config_t *config);

/**
 * @brief Get the current IMU driver status
 * @return Current IMU status
 */
imu_status_t imu_get_status(void);

/**
 * @brief Check if IMU driver is available and ready
 * @return true if IMU is ready, false otherwise
 */
bool imu_is_available(void);

/**
 * @brief Read raw sensor data from IMU
 * @param raw_data Pointer to store raw data (optional, can be NULL)
 * @return 0 on success, negative error code on failure
 */
int imu_read_raw_data(imu_raw_data_t *raw_data);

/**
 * @brief Get the latest filtered IMU data
 * @param filtered_data Pointer to store filtered data
 * @return 0 on success, negative error code on failure
 */
int imu_get_filtered_data(imu_filtered_data_t *filtered_data);

/**
 * @brief Get scaled IMU data for controller output
 * @param controller_data Pointer to store scaled data
 * @return 0 on success, negative error code on failure
 */
int imu_get_controller_data(imu_controller_data_t *controller_data);

// ============================================================================
// Configuration Functions
// ============================================================================

/**
 * @brief Set IMU output data rates
 * @param accel_odr Accelerometer ODR in Hz (0 to power down)
 * @param gyro_odr Gyroscope ODR in Hz (0 to power down)
 * @return 0 on success, negative error code on failure
 */
int imu_set_data_rates(uint16_t accel_odr, uint16_t gyro_odr);

/**
 * @brief Set low-pass filter coefficient
 * @param alpha Filter coefficient (0.0 = max filtering, 1.0 = no filtering)
 * @return 0 on success, negative error code on failure
 */
int imu_set_filter_alpha(float alpha);

/**
 * @brief Set scaling factors for controller output
 * @param accel_scale Accelerometer scaling factor
 * @param gyro_scale Gyroscope scaling factor
 * @return 0 on success, negative error code on failure
 */
int imu_set_scale_factors(float accel_scale, float gyro_scale);

// ============================================================================
// Calibration Functions
// ============================================================================

/**
 * @brief Start IMU calibration process
 * @param duration_ms Calibration duration in milliseconds
 * @return 0 on success, negative error code on failure
 */
int imu_start_calibration(uint32_t duration_ms);

/**
 * @brief Check if calibration is in progress
 * @return true if calibrating, false otherwise
 */
bool imu_is_calibrating(void);

/**
 * @brief Get current calibration data
 * @param calibration Pointer to store calibration data
 * @return 0 on success, negative error code on failure
 */
int imu_get_calibration(imu_calibration_t *calibration);

/**
 * @brief Set calibration data (from storage)
 * @param calibration Pointer to calibration data
 * @return 0 on success, negative error code on failure
 */
int imu_set_calibration(const imu_calibration_t *calibration);

/**
 * @brief Reset calibration to defaults
 * @return 0 on success, negative error code on failure
 */
int imu_reset_calibration(void);

// ============================================================================
// Power Management Functions
// ============================================================================

/**
 * @brief Put IMU into power-down mode
 * @return 0 on success, negative error code on failure
 */
int imu_power_down(void);

/**
 * @brief Wake up IMU from power-down mode
 * @return 0 on success, negative error code on failure
 */
int imu_power_up(void);

/**
 * @brief Set IMU to standby mode (low power)
 * @return 0 on success, negative error code on failure
 */
int imu_enter_standby(void);

/**
 * @brief Wake up IMU from standby mode
 * @return 0 on success, negative error code on failure
 */
int imu_wakeup(void);

// ============================================================================
// Debug and Diagnostics Functions
// ============================================================================

/**
 * @brief Get IMU driver statistics
 * @param read_count Pointer to store read count (optional)
 * @param error_count Pointer to store error count (optional)
 * @return 0 on success, negative error code on failure
 */
int imu_get_statistics(uint32_t *read_count, uint32_t *error_count);

/**
 * @brief Get debug information about the IMU driver
 * @param buffer Buffer to store debug info
 * @param buffer_size Size of the buffer
 * @return Number of characters written, negative on error
 */
int imu_get_debug_info(char *buffer, size_t buffer_size);

/**
 * @brief Test the IMU driver functionality
 * @return 0 on success, negative error code on failure
 */
int imu_test_driver(void);

/**
 * @brief Scan I2C bus for IMU device
 * @param i2c_dev I2C device to scan
 * @return 0 if device found, negative error code otherwise
 */
int imu_scan_bus(const struct device *i2c_dev);

/**
 * @brief Reset IMU driver statistics
 */
void imu_reset_statistics(void);

// ============================================================================
// Default Configuration
// ============================================================================

/**
 * @brief Get default IMU configuration
 * @return Default configuration structure
 */
imu_config_t imu_get_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_DRIVER_H */
