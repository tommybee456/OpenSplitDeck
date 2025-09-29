/**
 * @file imu_driver.c
 * @brief IMU sensor driver implementation for LSM6DS3TR-C
 */

#include "imu_driver.h"

LOG_MODULE_REGISTER(imu_driver, LOG_LEVEL_INF);

// Global IMU context
static imu_context_t imu_ctx = {
    .sensor_dev = NULL,
    .i2c_dev = NULL,
    .status = IMU_STATUS_NOT_READY,
    .config = {0},
    .raw_data = {0},
    .filtered_data = {0},
    .calibration = {0},
    .read_count = 0,
    .error_count = 0
};

// Default configuration
static const imu_config_t DEFAULT_CONFIG = {
    .accel_odr = 104,               // 104 Hz
    .gyro_odr = 104,                // 104 Hz  
    .filter_alpha = 0.4f,           // Moderate filtering
    .auto_calibrate = false,        // Manual calibration
    .accel_scale_factor = 800.0f,   // Scale to controller range
    .gyro_scale_factor = 1200.0f    // Scale to controller range
};

/**
 * @brief Initialize the IMU driver system
 */
int imu_driver_init(const struct device *sensor_dev, 
                   const struct device *i2c_dev,
                   const imu_config_t *config)
{
    LOG_INF("=== IMU DRIVER INITIALIZATION ===");
    
    // Clear the context
    memset(&imu_ctx, 0, sizeof(imu_ctx));
    imu_ctx.status = IMU_STATUS_NOT_READY;
    
    // Store device references
    imu_ctx.sensor_dev = sensor_dev;
    imu_ctx.i2c_dev = i2c_dev;
    
    // Use provided config or defaults
    if (config) {
        imu_ctx.config = *config;
    } else {
        imu_ctx.config = DEFAULT_CONFIG;
    }
    
    // Check if sensor device is available
    if (!sensor_dev) {
        LOG_ERR("IMU sensor device not provided");
        imu_ctx.status = IMU_STATUS_ERROR;
        return -ENODEV;
    }
    
    // Check if sensor device is ready (be more permissive like the original code)
    if (!device_is_ready(sensor_dev)) {
        LOG_WRN("IMU sensor device not ready - attempting initialization anyway");
        // Don't return error immediately, try to proceed like the original code
    } else {
        LOG_INF("LSM6DS3TR-C IMU sensor device ready");
    }
    
    // Configure accelerometer output data rate
    struct sensor_value odr_val = {
        .val1 = imu_ctx.config.accel_odr,
        .val2 = 0
    };
    
    int ret = sensor_attr_set(sensor_dev, SENSOR_CHAN_ACCEL_XYZ, 
                             SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_val);
    if (ret == 0) {
        LOG_INF("Accelerometer ODR set to %d Hz", imu_ctx.config.accel_odr);
    } else {
        LOG_WRN("Failed to set accelerometer ODR: %d", ret);
    }
    
    // Configure gyroscope output data rate
    ret = sensor_attr_set(sensor_dev, SENSOR_CHAN_GYRO_XYZ, 
                         SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_val);
    if (ret == 0) {
        LOG_INF("Gyroscope ODR set to %d Hz", imu_ctx.config.gyro_odr);
    } else {
        LOG_WRN("Failed to set gyroscope ODR: %d", ret);
    }
    
    // Initialize calibration to neutral values
    imu_reset_calibration();
    
    // Test initial sensor read (be more permissive like original code)
    ret = imu_read_raw_data(NULL);
    if (ret == 0) {
        LOG_INF("Initial IMU sensor read successful");
        imu_ctx.status = IMU_STATUS_READY;
    } else {
        LOG_WRN("Initial IMU sensor read failed: %d", ret);
        LOG_INF("IMU may not be present on this board variant");
        imu_ctx.status = IMU_STATUS_NOT_DETECTED;
        return -ENODEV;  // Return error like original code would
    }
    
    LOG_INF("=== IMU DRIVER INIT COMPLETE ===");
    return 0;
}

/**
 * @brief Get the current IMU driver status
 */
imu_status_t imu_get_status(void)
{
    return imu_ctx.status;
}

/**
 * @brief Check if IMU driver is available and ready
 */
bool imu_is_available(void)
{
    return (imu_ctx.status == IMU_STATUS_READY);
}

/**
 * @brief Read raw sensor data from IMU
 */
int imu_read_raw_data(imu_raw_data_t *raw_data)
{
    // Don't check if available during initialization - be more permissive like original
    if (imu_ctx.status == IMU_STATUS_ERROR) {
        imu_ctx.error_count++;
        return -ENODEV;
    }

    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    
    // Fetch all sensor data at once for coherency
    int ret = sensor_sample_fetch(imu_ctx.sensor_dev);
    if (ret != 0) {
        LOG_WRN("IMU sensor_sample_fetch failed: %d", ret);
        imu_ctx.error_count++;
        return ret;
    }
    
    // Read accelerometer data
    ret = sensor_channel_get(imu_ctx.sensor_dev, SENSOR_CHAN_ACCEL_X, &accel[0]);
    ret |= sensor_channel_get(imu_ctx.sensor_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]);
    ret |= sensor_channel_get(imu_ctx.sensor_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]);
    
    if (ret != 0) {
        LOG_WRN("Failed to read accelerometer data: %d", ret);
        imu_ctx.error_count++;
        return ret;
    }
    
    // Read gyroscope data
    ret = sensor_channel_get(imu_ctx.sensor_dev, SENSOR_CHAN_GYRO_X, &gyro[0]);
    ret |= sensor_channel_get(imu_ctx.sensor_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]);
    ret |= sensor_channel_get(imu_ctx.sensor_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]);
    
    if (ret != 0) {
        LOG_WRN("Failed to read gyroscope data: %d", ret);
        imu_ctx.error_count++;
        return ret;
    }
    
    // Convert to double values and store in context
    imu_ctx.raw_data.accel_x = sensor_value_to_double(&accel[0]);
    imu_ctx.raw_data.accel_y = sensor_value_to_double(&accel[1]);
    imu_ctx.raw_data.accel_z = sensor_value_to_double(&accel[2]);
    imu_ctx.raw_data.gyro_x = sensor_value_to_double(&gyro[0]);
    imu_ctx.raw_data.gyro_y = sensor_value_to_double(&gyro[1]);
    imu_ctx.raw_data.gyro_z = sensor_value_to_double(&gyro[2]);
    imu_ctx.raw_data.timestamp = k_uptime_get_32();
    
    // Apply calibration offsets
    imu_ctx.raw_data.accel_x -= imu_ctx.calibration.accel_offset_x;
    imu_ctx.raw_data.accel_y -= imu_ctx.calibration.accel_offset_y;
    imu_ctx.raw_data.accel_z -= imu_ctx.calibration.accel_offset_z;
    imu_ctx.raw_data.gyro_x -= imu_ctx.calibration.gyro_offset_x;
    imu_ctx.raw_data.gyro_y -= imu_ctx.calibration.gyro_offset_y;
    imu_ctx.raw_data.gyro_z -= imu_ctx.calibration.gyro_offset_z;
    
    // Apply low-pass filter
    if (!imu_ctx.filtered_data.filter_initialized) {
        // Initialize filter with first reading
        imu_ctx.filtered_data.accel_x = imu_ctx.raw_data.accel_x;
        imu_ctx.filtered_data.accel_y = imu_ctx.raw_data.accel_y;
        imu_ctx.filtered_data.accel_z = imu_ctx.raw_data.accel_z;
        imu_ctx.filtered_data.gyro_x = imu_ctx.raw_data.gyro_x;
        imu_ctx.filtered_data.gyro_y = imu_ctx.raw_data.gyro_y;
        imu_ctx.filtered_data.gyro_z = imu_ctx.raw_data.gyro_z;
        imu_ctx.filtered_data.filter_initialized = true;
    } else {
        // Exponential moving average filter
        float alpha = imu_ctx.config.filter_alpha;
        float beta = 1.0f - alpha;
        
        imu_ctx.filtered_data.accel_x = alpha * imu_ctx.raw_data.accel_x + beta * imu_ctx.filtered_data.accel_x;
        imu_ctx.filtered_data.accel_y = alpha * imu_ctx.raw_data.accel_y + beta * imu_ctx.filtered_data.accel_y;
        imu_ctx.filtered_data.accel_z = alpha * imu_ctx.raw_data.accel_z + beta * imu_ctx.filtered_data.accel_z;
        imu_ctx.filtered_data.gyro_x = alpha * imu_ctx.raw_data.gyro_x + beta * imu_ctx.filtered_data.gyro_x;
        imu_ctx.filtered_data.gyro_y = alpha * imu_ctx.raw_data.gyro_y + beta * imu_ctx.filtered_data.gyro_y;
        imu_ctx.filtered_data.gyro_z = alpha * imu_ctx.raw_data.gyro_z + beta * imu_ctx.filtered_data.gyro_z;
    }
    
    // Copy data to output if requested
    if (raw_data) {
        *raw_data = imu_ctx.raw_data;
    }
    
    imu_ctx.read_count++;
    return 0;
}

/**
 * @brief Get the latest filtered IMU data
 */
int imu_get_filtered_data(imu_filtered_data_t *filtered_data)
{
    if (!filtered_data) {
        return -EINVAL;
    }
    
    if (!imu_is_available()) {
        return -ENODEV;
    }
    
    *filtered_data = imu_ctx.filtered_data;
    return 0;
}

/**
 * @brief Get scaled IMU data for controller output
 */
int imu_get_controller_data(imu_controller_data_t *controller_data)
{
    if (!controller_data) {
        return -EINVAL;
    }
    
    if (!imu_is_available() || !imu_ctx.filtered_data.filter_initialized) {
        return -ENODEV;
    }
    
    // Scale and clamp to int16_t range (-32768 to 32767)
    // Note: Coordinate system mapping for controller compatibility
    double temp_ax = imu_ctx.filtered_data.accel_y * imu_ctx.config.accel_scale_factor;
    double temp_ay = imu_ctx.filtered_data.accel_z * imu_ctx.config.accel_scale_factor;
    double temp_az = imu_ctx.filtered_data.accel_x * imu_ctx.config.accel_scale_factor;
    
    double temp_gx = imu_ctx.filtered_data.gyro_y * imu_ctx.config.gyro_scale_factor;
    double temp_gy = imu_ctx.filtered_data.gyro_z * imu_ctx.config.gyro_scale_factor;
    double temp_gz = imu_ctx.filtered_data.gyro_x * imu_ctx.config.gyro_scale_factor;
    
    // Clamp to int16_t range
    controller_data->accel_x = (int16_t)((temp_ax < -32768) ? -32768 : ((temp_ax > 32767) ? 32767 : temp_ax));
    controller_data->accel_y = (int16_t)((temp_ay < -32768) ? -32768 : ((temp_ay > 32767) ? 32767 : temp_ay));
    controller_data->accel_z = (int16_t)((temp_az < -32768) ? -32768 : ((temp_az > 32767) ? 32767 : temp_az));
    controller_data->gyro_x = (int16_t)((temp_gx < -32768) ? -32768 : ((temp_gx > 32767) ? 32767 : temp_gx));
    controller_data->gyro_y = (int16_t)((temp_gy < -32768) ? -32768 : ((temp_gy > 32767) ? 32767 : temp_gy));
    controller_data->gyro_z = (int16_t)((temp_gz < -32768) ? -32768 : ((temp_gz > 32767) ? 32767 : temp_gz));
    
    return 0;
}

/**
 * @brief Set IMU output data rates
 */
int imu_set_data_rates(uint16_t accel_odr, uint16_t gyro_odr)
{
    if (!imu_is_available()) {
        return -ENODEV;
    }
    
    struct sensor_value odr_val;
    
    // Set accelerometer ODR
    odr_val.val1 = accel_odr;
    odr_val.val2 = 0;
    int ret = sensor_attr_set(imu_ctx.sensor_dev, SENSOR_CHAN_ACCEL_XYZ, 
                             SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_val);
    if (ret != 0) {
        LOG_ERR("Failed to set accelerometer ODR to %d Hz: %d", accel_odr, ret);
        return ret;
    }
    
    // Set gyroscope ODR
    odr_val.val1 = gyro_odr;
    odr_val.val2 = 0;
    ret = sensor_attr_set(imu_ctx.sensor_dev, SENSOR_CHAN_GYRO_XYZ, 
                         SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_val);
    if (ret != 0) {
        LOG_ERR("Failed to set gyroscope ODR to %d Hz: %d", gyro_odr, ret);
        return ret;
    }
    
    // Update configuration
    imu_ctx.config.accel_odr = accel_odr;
    imu_ctx.config.gyro_odr = gyro_odr;
    
    LOG_INF("IMU ODR updated: Accel=%d Hz, Gyro=%d Hz", accel_odr, gyro_odr);
    return 0;
}

/**
 * @brief Set low-pass filter coefficient
 */
int imu_set_filter_alpha(float alpha)
{
    if (alpha < 0.0f || alpha > 1.0f) {
        return -EINVAL;
    }
    
    imu_ctx.config.filter_alpha = alpha;
    LOG_INF("IMU filter alpha set to %.3f", alpha);
    return 0;
}

/**
 * @brief Set scaling factors for controller output
 */
int imu_set_scale_factors(float accel_scale, float gyro_scale)
{
    if (accel_scale <= 0.0f || gyro_scale <= 0.0f) {
        return -EINVAL;
    }
    
    imu_ctx.config.accel_scale_factor = accel_scale;
    imu_ctx.config.gyro_scale_factor = gyro_scale;
    
    LOG_INF("IMU scale factors updated: Accel=%.1f, Gyro=%.1f", accel_scale, gyro_scale);
    return 0;
}

/**
 * @brief Start IMU calibration process
 */
int imu_start_calibration(uint32_t duration_ms)
{
    if (!imu_is_available()) {
        return -ENODEV;
    }
    
    LOG_INF("=== IMU CALIBRATION START ===");
    LOG_INF("Keep IMU stationary for %u ms", duration_ms);
    
    imu_ctx.status = IMU_STATUS_CALIBRATING;
    
    // Reset calibration
    imu_reset_calibration();
    
    uint32_t samples = 0;
    double accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    double gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    
    uint32_t start_time = k_uptime_get_32();
    uint32_t sample_interval = 10; // Sample every 10ms
    
    while ((k_uptime_get_32() - start_time) < duration_ms) {
        imu_raw_data_t raw_data;
        if (imu_read_raw_data(&raw_data) == 0) {
            accel_sum_x += raw_data.accel_x;
            accel_sum_y += raw_data.accel_y;
            accel_sum_z += raw_data.accel_z;
            gyro_sum_x += raw_data.gyro_x;
            gyro_sum_y += raw_data.gyro_y;
            gyro_sum_z += raw_data.gyro_z;
            samples++;
        }
        k_sleep(K_MSEC(sample_interval));
    }
    
    if (samples > 0) {
        // Calculate average offsets
        imu_ctx.calibration.accel_offset_x = accel_sum_x / samples;
        imu_ctx.calibration.accel_offset_y = accel_sum_y / samples;
        imu_ctx.calibration.accel_offset_z = (accel_sum_z / samples) - 9.81; // Account for gravity
        imu_ctx.calibration.gyro_offset_x = gyro_sum_x / samples;
        imu_ctx.calibration.gyro_offset_y = gyro_sum_y / samples;
        imu_ctx.calibration.gyro_offset_z = gyro_sum_z / samples;
        imu_ctx.calibration.calibrated = true;
        
        LOG_INF("=== CALIBRATION COMPLETE ===");
        LOG_INF("Samples: %u", samples);
        LOG_INF("Accel offsets: X=%.3f, Y=%.3f, Z=%.3f", 
                imu_ctx.calibration.accel_offset_x,
                imu_ctx.calibration.accel_offset_y,
                imu_ctx.calibration.accel_offset_z);
        LOG_INF("Gyro offsets: X=%.3f, Y=%.3f, Z=%.3f",
                imu_ctx.calibration.gyro_offset_x,
                imu_ctx.calibration.gyro_offset_y,
                imu_ctx.calibration.gyro_offset_z);
    } else {
        LOG_ERR("Calibration failed - no samples collected");
        imu_ctx.status = IMU_STATUS_ERROR;
        return -EIO;
    }
    
    imu_ctx.status = IMU_STATUS_READY;
    return 0;
}

/**
 * @brief Check if calibration is in progress
 */
bool imu_is_calibrating(void)
{
    return (imu_ctx.status == IMU_STATUS_CALIBRATING);
}

/**
 * @brief Get current calibration data
 */
int imu_get_calibration(imu_calibration_t *calibration)
{
    if (!calibration) {
        return -EINVAL;
    }
    
    *calibration = imu_ctx.calibration;
    return 0;
}

/**
 * @brief Set calibration data (from storage)
 */
int imu_set_calibration(const imu_calibration_t *calibration)
{
    if (!calibration) {
        return -EINVAL;
    }
    
    imu_ctx.calibration = *calibration;
    LOG_INF("IMU calibration data loaded");
    return 0;
}

/**
 * @brief Reset calibration to defaults
 */
int imu_reset_calibration(void)
{
    memset(&imu_ctx.calibration, 0, sizeof(imu_ctx.calibration));
    LOG_DBG("IMU calibration reset to defaults");
    return 0;
}

/**
 * @brief Put IMU into power-down mode
 */
int imu_power_down(void)
{
    if (!imu_is_available()) {
        return -ENODEV;
    }
    
    LOG_INF("Putting IMU into power-down mode");
    return imu_set_data_rates(0, 0); // 0 Hz = power down
}

/**
 * @brief Wake up IMU from power-down mode
 */
int imu_power_up(void)
{
    if (imu_ctx.status == IMU_STATUS_NOT_READY) {
        return -ENODEV;
    }
    
    LOG_INF("Waking up IMU from power-down mode");
    return imu_set_data_rates(imu_ctx.config.accel_odr, imu_ctx.config.gyro_odr);
}

/**
 * @brief Set IMU to standby mode (low power)
 */
int imu_enter_standby(void)
{
    LOG_INF("IMU entering standby mode");
    return imu_power_down();
}

/**
 * @brief Wake up IMU from standby mode
 */
int imu_wakeup(void)
{
    LOG_INF("IMU waking up from standby");
    return imu_power_up();
}

/**
 * @brief Get IMU driver statistics
 */
int imu_get_statistics(uint32_t *read_count, uint32_t *error_count)
{
    if (read_count) {
        *read_count = imu_ctx.read_count;
    }
    if (error_count) {
        *error_count = imu_ctx.error_count;
    }
    return 0;
}

/**
 * @brief Get debug information about the IMU driver
 */
int imu_get_debug_info(char *buffer, size_t buffer_size)
{
    if (!buffer || buffer_size == 0) {
        return -EINVAL;
    }
    
    return snprintf(buffer, buffer_size,
        "IMU Driver Debug Info:\n"
        "Status: %d\n"
        "Sensor Dev: %p\n"
        "I2C Dev: %p\n"
        "Read Count: %u\n"
        "Error Count: %u\n"
        "Filter Alpha: %.3f\n"
        "Accel ODR: %u Hz\n"
        "Gyro ODR: %u Hz\n"
        "Calibrated: %s\n",
        imu_ctx.status,
        imu_ctx.sensor_dev,
        imu_ctx.i2c_dev,
        imu_ctx.read_count,
        imu_ctx.error_count,
        imu_ctx.config.filter_alpha,
        imu_ctx.config.accel_odr,
        imu_ctx.config.gyro_odr,
        imu_ctx.calibration.calibrated ? "YES" : "NO");
}

/**
 * @brief Test the IMU driver functionality
 */
int imu_test_driver(void)
{
    if (!imu_is_available()) {
        LOG_ERR("IMU driver not available for testing");
        return -ENODEV;
    }
    
    LOG_INF("=== IMU DRIVER TEST ===");
    
    // Test 1: Read raw data
    LOG_INF("Test 1: Reading raw IMU data");
    imu_raw_data_t raw_data;
    int ret = imu_read_raw_data(&raw_data);
    if (ret != 0) {
        LOG_ERR("Test 1 failed: %d", ret);
        return ret;
    }
    LOG_INF("Raw data: A(%.2f,%.2f,%.2f) G(%.2f,%.2f,%.2f)",
            raw_data.accel_x, raw_data.accel_y, raw_data.accel_z,
            raw_data.gyro_x, raw_data.gyro_y, raw_data.gyro_z);
    
    // Test 2: Get controller data
    LOG_INF("Test 2: Getting controller data");
    imu_controller_data_t controller_data;
    ret = imu_get_controller_data(&controller_data);
    if (ret != 0) {
        LOG_ERR("Test 2 failed: %d", ret);
        return ret;
    }
    LOG_INF("Controller data: A(%d,%d,%d) G(%d,%d,%d)",
            controller_data.accel_x, controller_data.accel_y, controller_data.accel_z,
            controller_data.gyro_x, controller_data.gyro_y, controller_data.gyro_z);
    
    LOG_INF("=== IMU DRIVER TEST COMPLETE ===");
    return 0;
}

/**
 * @brief Scan I2C bus for IMU device
 */
int imu_scan_bus(const struct device *i2c_dev)
{
    if (!i2c_dev || !device_is_ready(i2c_dev)) {
        return -ENODEV;
    }
    
    LOG_INF("Scanning I2C bus for IMU devices...");
    int found_devices = 0;
    
    // LSM6DS3TR-C typical addresses: 0x6A, 0x6B
    uint8_t test_addresses[] = {0x6A, 0x6B};
    
    for (int i = 0; i < sizeof(test_addresses); i++) {
        uint8_t addr = test_addresses[i];
        uint8_t dummy_data;
        int ret = i2c_read(i2c_dev, &dummy_data, 1, addr);
        if (ret == 0) {
            LOG_INF("Found I2C device at address 0x%02X", addr);
            found_devices++;
        }
        k_sleep(K_MSEC(5)); // Small delay between reads
    }
    
    // Also scan general range
    for (uint8_t addr = 0x10; addr <= 0x77; addr++) {
        uint8_t dummy_data;
        int ret = i2c_read(i2c_dev, &dummy_data, 1, addr);
        if (ret == 0) {
            bool already_found = false;
            for (int i = 0; i < sizeof(test_addresses); i++) {
                if (addr == test_addresses[i]) {
                    already_found = true;
                    break;
                }
            }
            if (!already_found) {
                LOG_INF("Found unknown I2C device at address 0x%02X", addr);
                found_devices++;
            }
        }
        k_sleep(K_MSEC(2)); // Small delay
    }
    
    LOG_INF("I2C scan complete - found %d devices", found_devices);
    return (found_devices > 0) ? 0 : -ENODEV;
}

/**
 * @brief Reset IMU driver statistics
 */
void imu_reset_statistics(void)
{
    imu_ctx.read_count = 0;
    imu_ctx.error_count = 0;
    LOG_INF("IMU driver statistics reset");
}

/**
 * @brief Get default IMU configuration
 */
imu_config_t imu_get_default_config(void)
{
    return DEFAULT_CONFIG;
}
