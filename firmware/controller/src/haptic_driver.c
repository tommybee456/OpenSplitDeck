/**
 * @file haptic_driver.c
 * @brief Haptic motor driver implementation for DRV2605
 */

#include "haptic_driver.h"
#include "drv2605.h"

LOG_MODULE_REGISTER(haptic_driver, LOG_LEVEL_INF);

// Global haptic context
static haptic_context_t haptic_ctx = {
    .i2c_dev = NULL,
    .trigger_pin = NULL,
    .enable_pin = NULL,
    .status = HAPTIC_STATUS_NOT_READY,
    .external_trigger_mode = false,
    .current_effect = 0
};

// DRV2605 I2C address
#define DRV2605_I2C_ADDR 0x5A

/**
 * @brief Initialize the haptic driver system
 */
int haptic_driver_init(const struct device *i2c_dev, 
                      const struct gpio_dt_spec *trigger_pin,
                      const struct gpio_dt_spec *enable_pin)
{
    LOG_INF("=== HAPTIC DRIVER INITIALIZATION ===");
    
    // Clear the context
    memset(&haptic_ctx, 0, sizeof(haptic_ctx));
    haptic_ctx.status = HAPTIC_STATUS_NOT_READY;
    
    // Store device references
    haptic_ctx.i2c_dev = i2c_dev;
    haptic_ctx.trigger_pin = trigger_pin;
    haptic_ctx.enable_pin = enable_pin;
    
    if (!i2c_dev) {
        LOG_ERR("I2C device not available for haptic driver");
        haptic_ctx.status = HAPTIC_STATUS_ERROR;
        return -ENODEV;
    }
    LOG_INF("I2C device available at %p", i2c_dev);
    
    // Check if I2C device is ready
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        haptic_ctx.status = HAPTIC_STATUS_ERROR;
        return -ENODEV;
    }
    LOG_INF("I2C device is ready");
    
    // Configure control pins
    if (trigger_pin) {
        int ret = gpio_pin_configure_dt(trigger_pin, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Failed to configure trigger pin: %d", ret);
            haptic_ctx.status = HAPTIC_STATUS_ERROR;
            return ret;
        }
        LOG_INF("Trigger pin (P%d.%02d) configured", trigger_pin->port->name[4] - '0', trigger_pin->pin);
    }
    
    if (enable_pin) {
        int ret = gpio_pin_configure_dt(enable_pin, GPIO_OUTPUT_ACTIVE);
        if (ret != 0) {
            LOG_ERR("Failed to configure enable pin: %d", ret);
            haptic_ctx.status = HAPTIC_STATUS_ERROR;
            return ret;
        }
        LOG_INF("Enable pin (P%d.%02d) configured", enable_pin->port->name[4] - '0', enable_pin->pin);
    }
    
    // Small delay for DRV2605 to stabilize after enable
    k_sleep(K_MSEC(10));
    
    // Try to detect DRV2605
    uint8_t test_data = 0;
    int scan_ret = i2c_reg_read_byte(i2c_dev, DRV2605_I2C_ADDR, 0x00, &test_data);
    LOG_INF("DRV2605 detection: ret=%d, status=0x%02X", scan_ret, test_data);
    
    if (scan_ret != 0) {
        LOG_ERR("DRV2605 not detected at address 0x%02X", DRV2605_I2C_ADDR);
        haptic_ctx.status = HAPTIC_STATUS_NOT_DETECTED;
        return -ENODEV;
    }
    
    LOG_INF("DRV2605 detected successfully!");
    
    // Skip auto-calibration during init to avoid log overflow
    // Will be triggered later after system boot completes
    printk("*** HAPTIC: DRV2605 ready - auto-calibration will run after boot\n");
    
    haptic_ctx.status = HAPTIC_STATUS_READY;
    
    LOG_INF("=== HAPTIC DRIVER INIT COMPLETE ===");
    return 0;
}

/**
 * @brief Get the current haptic driver status
 */
haptic_status_t haptic_get_status(void)
{
    return haptic_ctx.status;
}

/**
 * @brief Check if haptic driver is available
 */
bool haptic_is_available(void)
{
    return (haptic_ctx.status == HAPTIC_STATUS_READY);
}

/**
 * @brief Setup external trigger mode
 */
int haptic_setup_external_trigger(drv2605_effect_t default_effect)
{
    if (!haptic_is_available()) {
        return -ENODEV;
    }
    
    LOG_INF("Setting up external trigger mode with effect %d", default_effect);
    
    // Step 1: Exit standby mode
    LOG_DBG("Step 1 - Exit standby mode");
    int ret1 = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x01, 0x00);
    k_sleep(K_MSEC(5));
    
    // Step 2: Set motor library (detect if LRA or ERM mode was used)
    LOG_DBG("Step 2 - Set motor library");
    uint8_t feedback_reg = 0;
    int lib_ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x1A, &feedback_reg);
    int ret2 = 0;
    
    if (lib_ret == 0 && (feedback_reg & 0x80)) {
        // LRA mode detected (bit 7 set)
        LOG_INF("LRA mode detected - using LRA library");
        ret2 = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x03, 0x06); // LRA Library
        k_sleep(K_MSEC(5));
    } else {
        // ERM mode (bit 7 clear or read failed)
        LOG_INF("ERM mode detected - using ERM library");
        ret2 = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x03, 0x01); // ERM Library
        k_sleep(K_MSEC(5));
    }
    
    // Step 2.5: Motor type already configured during calibration
    LOG_DBG("Step 2.5 - Motor type configured during calibration");
    k_sleep(K_MSEC(5));
    
    // Verify calibration values are still present after motor setup
    uint8_t comp_check = 0, bemf_check = 0;
    if (i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x18, &comp_check) == 0 &&
        i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x19, &bemf_check) == 0) {
        LOG_INF("Calibration verification: Comp=0x%02X, BackEMF=0x%02X", comp_check, bemf_check);
    }
    
    // Step 3: Set default waveform
    LOG_DBG("Step 3 - Set default waveform");
    int ret3 = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x04, default_effect);
    k_sleep(K_MSEC(2));
    
    // Step 4: End sequence
    LOG_DBG("Step 4 - End sequence");
    int ret4 = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x05, 0x00);
    k_sleep(K_MSEC(2));
    
    // Step 5: Set external trigger mode
    LOG_DBG("Step 5 - Set external trigger mode");
    int ret5 = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x01, 0x01);
    k_sleep(K_MSEC(5));
    
    // Step 6: Boost motor strength with enhanced control register settings
    LOG_DBG("Step 6 - Apply strength boost settings");
    
    // Control1: Enhanced startup boost and drive strength
    int ret6a = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x1B, 0xB3); // Higher startup boost
    if (ret6a != 0) {
        LOG_WRN("Failed to set enhanced Control1: %d", ret6a);
    }
    
    // Control2: Bidirectional input with optimized sample timing  
    int ret6b = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x1C, 0xF5); // Bidirectional + fast sample
    if (ret6b != 0) {
        LOG_WRN("Failed to set Control2: %d", ret6b);
    }
    
    // Control3: ERM open loop with supply compensation and stronger output
    int ret6c = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x1D, 0xE0); // Max output + supply comp
    if (ret6c != 0) {
        LOG_WRN("Failed to set enhanced Control3: %d", ret6c);
    }
    
    LOG_INF("Enhanced strength settings applied for stronger haptic feedback");
    
    LOG_INF("External trigger setup results: %d, %d, %d, %d, %d", ret1, ret2, ret3, ret4, ret5);
    
    if (ret1 == 0 && ret2 == 0 && ret3 == 0 && ret4 == 0 && ret5 == 0) {
        haptic_ctx.external_trigger_mode = true;
        haptic_ctx.current_effect = default_effect;
        LOG_INF("External trigger mode with enhanced strength configured successfully!");
        return 0;
    } else {
        LOG_WRN("Some register writes failed during external trigger setup");
        return -EIO;
    }
}

/**
 * @brief Send a pulse via external trigger
 */
int haptic_trigger_pulse(void)
{
    if (!haptic_is_available() || !haptic_ctx.external_trigger_mode || !haptic_ctx.trigger_pin) {
        return -ENODEV;
    }
    
    // Send pulse on trigger pin
    gpio_pin_set_dt(haptic_ctx.trigger_pin, 1);  // Rising edge
    k_sleep(K_USEC(100));                        // Short pulse width (100µs)
    gpio_pin_set_dt(haptic_ctx.trigger_pin, 0);  // Falling edge
    
    LOG_DBG("External trigger pulse sent");
    return 0;
}

/**
 * @brief Play a predefined haptic pattern
 */
int haptic_play_pattern(haptic_pattern_t pattern)
{
    if (!haptic_is_available()) {
        return -ENODEV;
    }
    
    LOG_DBG("Playing haptic pattern: %d", pattern);
    
    switch (pattern) {
        case HAPTIC_PATTERN_BUTTON_PRESS:
            if (haptic_ctx.external_trigger_mode) {
                return haptic_trigger_pulse();
            } else {
                return haptic_play_effect(DRV2605_EFFECT_SHARP_CLICK_100);
            }
            
        case HAPTIC_PATTERN_STARTUP:
            // Double pulse for startup
            if (haptic_ctx.external_trigger_mode) {
                haptic_trigger_pulse();
                k_sleep(K_MSEC(100));
                return haptic_trigger_pulse();
            } else {
                haptic_play_effect(DRV2605_EFFECT_SOFT_BUMP_30);
                k_sleep(K_MSEC(200));
                return haptic_play_effect(DRV2605_EFFECT_SOFT_BUMP_30);
            }
            
        case HAPTIC_PATTERN_NOTIFICATION_LIGHT:
            return haptic_play_effect(DRV2605_EFFECT_SOFT_BUMP_30);
            
        case HAPTIC_PATTERN_NOTIFICATION_MEDIUM:
            return haptic_play_effect(DRV2605_EFFECT_SHARP_CLICK_60);
            
        case HAPTIC_PATTERN_NOTIFICATION_STRONG:
            return haptic_play_effect(DRV2605_EFFECT_SHARP_CLICK_100);
            
        case HAPTIC_PATTERN_ERROR:
            // Triple pulse for error
            haptic_play_effect(DRV2605_EFFECT_SHARP_CLICK_100);
            k_sleep(K_MSEC(100));
            haptic_play_effect(DRV2605_EFFECT_SHARP_CLICK_100);
            k_sleep(K_MSEC(100));
            return haptic_play_effect(DRV2605_EFFECT_SHARP_CLICK_100);
            
        case HAPTIC_PATTERN_SUCCESS:
            return haptic_play_effect(DRV2605_EFFECT_TRANSITION_RAMP_UP_LONG_SMOOTH_1);
            
        default:
            LOG_WRN("Unknown haptic pattern: %d", pattern);
            return -EINVAL;
    }
}

/**
 * @brief Play a specific DRV2605 effect via I2C
 */
int haptic_play_effect(drv2605_effect_t effect)
{
    if (!haptic_is_available()) {
        return -ENODEV;
    }
    
    LOG_DBG("Playing DRV2605 effect: %d", effect);
    
    // Exit standby mode
    int ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x01, 0x00);
    if (ret != 0) return ret;
    k_sleep(K_MSEC(1));
    
    // Set waveform
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x04, effect);
    if (ret != 0) return ret;
    
    // End sequence
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x05, 0x00);
    if (ret != 0) return ret;
    
    // Trigger playback
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x0C, 0x01);
    if (ret != 0) return ret;
    
    return 0;
}

/**
 * @brief Change the external trigger effect
 */
int haptic_set_trigger_effect(drv2605_effect_t new_effect)
{
    if (!haptic_is_available() || !haptic_ctx.external_trigger_mode) {
        return -ENODEV;
    }
    
    LOG_INF("Changing trigger effect from %d to %d", haptic_ctx.current_effect, new_effect);
    
    // Update the waveform in slot 0
    int ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x04, new_effect);
    if (ret == 0) {
        haptic_ctx.current_effect = new_effect;
        LOG_INF("Trigger effect updated successfully");
    } else {
        LOG_WRN("Failed to update trigger effect: %d", ret);
    }
    
    return ret;
}

/**
 * @brief Enter standby mode (low power)
 */
int haptic_enter_standby(void)
{
    if (!haptic_is_available()) {
        LOG_WRN("Haptic not available for standby");
        return -ENODEV;
    }
    
    LOG_INF("Entering haptic standby mode using enable pin");
    
    // Use the hardware enable pin to put the DRV2605 into standby
    // This is much safer than I2C communication during system shutdown
    return haptic_disable();
}

/**
 * @brief Wake up from standby mode
 */
int haptic_wakeup(void)
{
    if (haptic_ctx.status == HAPTIC_STATUS_NOT_READY) {
        return -ENODEV;
    }
    
    LOG_INF("Waking up haptic driver using enable pin");
    
    // Use the hardware enable pin to wake up the DRV2605
    int ret = haptic_enable();
    
    if (ret == 0) {
        // Restore external trigger mode if it was enabled
        if (haptic_ctx.external_trigger_mode) {
            ret = haptic_setup_external_trigger(haptic_ctx.current_effect);
        }
    }
    
    return ret;
}

/**
 * @brief Disable the haptic driver
 */
int haptic_disable(void)
{
    if (haptic_ctx.enable_pin) {
        LOG_INF("Disabling haptic driver via enable pin");
        return gpio_pin_set_dt(haptic_ctx.enable_pin, 0);
    }
    return -ENODEV;
}

/**
 * @brief Enable the haptic driver
 */
int haptic_enable(void)
{
    if (haptic_ctx.enable_pin) {
        LOG_INF("Enabling haptic driver via enable pin");
        int ret = gpio_pin_set_dt(haptic_ctx.enable_pin, 1);
        if (ret == 0) {
            k_sleep(K_MSEC(10)); // Stabilization delay
        }
        return ret;
    }
    return -ENODEV;
}

/**
 * @brief Get debug information about the haptic driver
 */
int haptic_get_debug_info(char *buffer, size_t buffer_size)
{
    if (!buffer || buffer_size == 0) {
        return -EINVAL;
    }
    
    return snprintf(buffer, buffer_size,
        "Haptic Driver Debug Info:\n"
        "Status: %d\n"
        "I2C Dev: %p\n"
        "External Trigger: %s\n"
        "Current Effect: %d\n"
        "Trigger Pin: P%s.%02d\n"
        "Enable Pin: P%s.%02d\n",
        haptic_ctx.status,
        haptic_ctx.i2c_dev,
        haptic_ctx.external_trigger_mode ? "YES" : "NO",
        haptic_ctx.current_effect,
        haptic_ctx.trigger_pin ? haptic_ctx.trigger_pin->port->name : "NULL",
        haptic_ctx.trigger_pin ? haptic_ctx.trigger_pin->pin : 0,
        haptic_ctx.enable_pin ? haptic_ctx.enable_pin->port->name : "NULL",
        haptic_ctx.enable_pin ? haptic_ctx.enable_pin->pin : 0);
}

/**
 * @brief Test the haptic driver with a simple pattern
 */
int haptic_test_driver(void)
{
    if (!haptic_is_available()) {
        LOG_ERR("Haptic driver not available for testing");
        return -ENODEV;
    }
    
    LOG_INF("=== HAPTIC DRIVER TEST ===");
    
    // Test 1: Simple effect
    LOG_INF("Test 1: Playing simple click effect");
    int ret = haptic_play_effect(DRV2605_EFFECT_SHARP_CLICK_100);
    if (ret != 0) {
        LOG_ERR("Test 1 failed: %d", ret);
        return ret;
    }
    k_sleep(K_MSEC(500));
    
    // Test 2: External trigger (if enabled)
    if (haptic_ctx.external_trigger_mode) {
        LOG_INF("Test 2: Testing external trigger");
        ret = haptic_trigger_pulse();
        if (ret != 0) {
            LOG_ERR("Test 2 failed: %d", ret);
            return ret;
        }
        k_sleep(K_MSEC(500));
    }
    
    // Test 3: Pattern test
    LOG_INF("Test 3: Testing startup pattern");
    ret = haptic_play_pattern(HAPTIC_PATTERN_STARTUP);
    if (ret != 0) {
        LOG_ERR("Test 3 failed: %d", ret);
        return ret;
    }
    
    LOG_INF("=== HAPTIC DRIVER TEST COMPLETE ===");
    return 0;
}

/**
 * @brief Test stronger haptic effects for comparison
 */
int haptic_test_strong_effects(void)
{
    if (!haptic_is_available()) {
        return -ENODEV;
    }
    
    LOG_INF("Testing various strong haptic effects...");
    
    // Test sequence of increasingly strong effects
    drv2605_effect_t strong_effects[] = {
        DRV2605_EFFECT_STRONG_CLICK_100,    // Effect 1 - Strongest basic click
        DRV2605_EFFECT_STRONG_CLICK_1,      // Effect 17 - Alternative strong
        DRV2605_EFFECT_STRONG_CLICK_2,      // Effect 17 - Alternative strong
        DRV2605_EFFECT_STRONG_CLICK_3,      // Effect 17 - Alternative strong
        DRV2605_EFFECT_STRONG_CLICK_4,      // Effect 17 - Alternative strong
        DRV2605_EFFECT_PULSING_STRONG_1,    // Effect 38 - Pulsing strong
        DRV2605_EFFECT_STRONG_BUZZ_100      // Effect 14 - Strong buzz
    };
    
    for (int i = 0; i < 7; i++) {
        LOG_INF("Testing effect %d: %d", i+1, strong_effects[i]);
        int ret = haptic_play_effect(strong_effects[i]);
        if (ret != 0) {
            LOG_WRN("Failed to play test effect %d: %d", strong_effects[i], ret);
        }
        k_sleep(K_MSEC(1000)); // 1 second between tests
    }
    
    LOG_INF("Strong effects test complete");
    return 0;
}

/**
 * @brief Perform ERM motor auto-calibration
 */
int haptic_perform_erm_calibration(void)
{
    if (!haptic_is_available()) {
        return -ENODEV;
    }
    
    printk("*** HAPTIC: Performing ERM auto-calibration...\n");
    
    // Step 1: Exit standby mode
    printk("*** HAPTIC: Setting auto-calibration mode\n");
    int ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x01, 0x07); // Auto-calibration mode
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set auto-cal mode: %d\n", ret);
        return ret;
    }
    k_sleep(K_MSEC(10));
    
    // Step 2: Configure feedback control for ERM
    printk("*** HAPTIC: Configuring ERM feedback control\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x1A, 0x36); // ERM mode, BEMF gain=1x
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set ERM mode: %d\n", ret);
        return ret;
    }
    
    // Step 3: Set rated voltage for stronger ERM motor (~3.5V)
    printk("*** HAPTIC: Setting rated voltage\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x16, 0x4A); // ~3.5V rated voltage (higher)
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set rated voltage: %d\n", ret);
        return ret;
    }
    
    // Step 4: Set overdrive clamp voltage (~4.5V)
    printk("*** HAPTIC: Setting overdrive clamp\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x17, 0x9A); // ~4.5V overdrive clamp (higher)
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set overdrive clamp: %d\n", ret);
        return ret;
    }
    
    // Step 5: Start auto-calibration
    printk("*** HAPTIC: Starting calibration sequence\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x0C, 0x01); // GO bit
    if (ret != 0) {
        printk("*** HAPTIC: Failed to start calibration: %d\n", ret);
        return ret;
    }
    
    printk("*** HAPTIC: Waiting for calibration to complete...\n");
    
    // Step 6: Wait for calibration to complete (up to 3 seconds)
    uint32_t start_time = k_uptime_get_32();
    uint8_t go_reg = 0;
    
    while ((k_uptime_get_32() - start_time) < 3000) { // 3 second timeout
        ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x0C, &go_reg);
        if (ret != 0) {
            printk("*** HAPTIC: Failed to read GO register: %d\n", ret);
            return ret;
        }
        
        printk("*** HAPTIC: GO register value: 0x%02X\n", go_reg);
        
        // Check if GO bit is cleared (calibration complete)
        if ((go_reg & 0x01) == 0) {
            uint32_t duration = k_uptime_get_32() - start_time;
            printk("*** HAPTIC: Auto-calibration completed in %d ms\n", duration);
            break;
        }
        
        k_sleep(K_MSEC(50));
    }
    
    // Check if calibration succeeded
    if (go_reg & 0x01) {
        printk("*** HAPTIC: Auto-calibration timeout\n");
        return -ETIMEDOUT;
    }
    
    // Read calibration results from status register
    uint8_t status_reg = 0;
    ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x00, &status_reg);
    if (ret == 0) {
        printk("*** HAPTIC: Status register: 0x%02X\n", status_reg);
        if (status_reg & 0x08) {
            printk("*** HAPTIC: Auto-calibration failed - DIAG bit set\n");
        } else {
            printk("*** HAPTIC: Auto-calibration successful - motor optimized\n");
        }
        
        // Read compensation result and back-EMF
        uint8_t comp_result = 0, back_emf = 0;
        ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x18, &comp_result);
        if (ret == 0) {
            printk("*** HAPTIC: Compensation result: 0x%02X\n", comp_result);
        }
        ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x19, &back_emf);
        if (ret == 0) {
            printk("*** HAPTIC: Back-EMF result: 0x%02X\n", back_emf);
        }
        
        // Store calibration results for potential restoration
        if (comp_result != 0 || back_emf != 0) {
            printk("*** HAPTIC: Calibration values stored: Comp=0x%02X, BackEMF=0x%02X\n", comp_result, back_emf);
            // Note: DRV2605 should automatically preserve these values,
            // but we're recording them for verification
        }
    }
    
    // Return to standby mode - let haptic_setup_external_trigger handle final config
    printk("*** HAPTIC: Returning to standby mode\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x01, 0x40); // Standby mode
    
    printk("*** HAPTIC: ERM auto-calibration process complete - ready for external trigger setup\n");
    return 0;
}

/**
 * @brief Perform LRA motor auto-calibration
 */
int haptic_perform_lra_calibration(void)
{
    if (!haptic_is_available()) {
        return -ENODEV;
    }

    printk("*** HAPTIC: Performing LRA auto-calibration...\n");

    // Step 1: Exit standby mode and set auto-calibration mode
    printk("*** HAPTIC: Setting auto-calibration mode\n");
    int ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x01, 0x07); // Auto-calibration mode
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set auto-cal mode: %d\n", ret);
        return ret;
    }
    k_sleep(K_MSEC(10));

    // Step 2: Configure feedback control for LRA
    printk("*** HAPTIC: Configuring LRA feedback control\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x1A, 0xB6); // LRA mode, BEMF gain=1x
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set LRA mode: %d\n", ret);
        return ret;
    }

    // Step 3: Set LRA library 
    printk("*** HAPTIC: Setting LRA library\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x03, 0x06); // LRA Library
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set LRA library: %d\n", ret);
        return ret;
    }

    // Step 4: Set rated voltage for LRA motor (~2.0V typical for LRA)
    printk("*** HAPTIC: Setting LRA rated voltage\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x16, 0x3E); // ~2.0V rated voltage
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set rated voltage: %d\n", ret);
        return ret;
    }

    // Step 5: Set overdrive clamp voltage (~2.5V for LRA)
    printk("*** HAPTIC: Setting LRA overdrive clamp\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x17, 0x6C); // ~2.5V overdrive clamp
    if (ret != 0) {
        printk("*** HAPTIC: Failed to set overdrive clamp: %d\n", ret);
        return ret;
    }

    // Step 6: Start auto-calibration
    printk("*** HAPTIC: Starting LRA calibration sequence\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x0C, 0x01); // GO bit
    if (ret != 0) {
        printk("*** HAPTIC: Failed to start calibration: %d\n", ret);
        return ret;
    }

    // Step 7: Wait for calibration to complete (up to 3 seconds for LRA)
    printk("*** HAPTIC: Waiting for LRA calibration to complete...\n");
    bool calibration_complete = false;
    uint32_t start_time = k_uptime_get_32();
    uint32_t timeout_ms = 3000; // 3 second timeout for LRA calibration

    while ((k_uptime_get_32() - start_time) < timeout_ms) {
        uint8_t go_bit = 0;
        ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x0C, &go_bit);
        if (ret == 0 && (go_bit & 0x01) == 0) {
            calibration_complete = true;
            printk("*** HAPTIC: LRA calibration completed in %ums\n", 
                   (uint32_t)(k_uptime_get_32() - start_time));
            break;
        }
        k_sleep(K_MSEC(50)); // Check every 50ms
    }

    if (!calibration_complete) {
        printk("*** HAPTIC: LRA calibration timeout - may still be in progress\n");
        // Don't return error - some motors take longer
    }

    // Step 8: Check calibration results
    uint8_t diag_result = 0;
    ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x00, &diag_result);
    if (ret == 0) {
        if (diag_result & 0x08) { // DIAG_RESULT bit
            printk("*** HAPTIC: LRA auto-calibration successful - motor optimized\n");
        } else {
            printk("*** HAPTIC: LRA calibration completed with warnings - check motor connection\n");
        }

        // Read LRA resonance frequency and impedance results
        uint8_t lra_period = 0, lra_impedance = 0;
        ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x20, &lra_period);
        if (ret == 0) {
            printk("*** HAPTIC: LRA Period result: 0x%02X\n", lra_period);
        }
        ret = i2c_reg_read_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x21, &lra_impedance);
        if (ret == 0) {
            printk("*** HAPTIC: LRA Impedance result: 0x%02X\n", lra_impedance);
        }

        // Store calibration results for verification
        if (lra_period != 0 || lra_impedance != 0) {
            printk("*** HAPTIC: LRA Calibration values stored: Period=0x%02X, Impedance=0x%02X\n", 
                   lra_period, lra_impedance);
        }
    }

    // Return to standby mode - let haptic_setup_external_trigger handle final config
    printk("*** HAPTIC: Returning to standby mode\n");
    ret = i2c_reg_write_byte(haptic_ctx.i2c_dev, DRV2605_I2C_ADDR, 0x01, 0x40); // Standby mode

    printk("*** HAPTIC: LRA auto-calibration process complete - ready for external trigger setup\n");
    return 0;
}
