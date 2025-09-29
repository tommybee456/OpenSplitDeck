/*
 * DRV2605 Haptic Motor Driver Implementation
 * 
 * This module provides control for the DRV2605 haptic motor driver
 * including waveform playback, real-time control, and effect sequencing.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "drv2605.h"

LOG_MODULE_REGISTER(drv2605, LOG_LEVEL_INF);

// Private function prototypes
static int drv2605_write_reg(const drv2605_device_t *dev, uint8_t reg, uint8_t value);
static int drv2605_read_reg(const drv2605_device_t *dev, uint8_t reg, uint8_t *value);
static int drv2605_wait_for_ready(const drv2605_device_t *dev, uint32_t timeout_ms);

int drv2605_init(drv2605_device_t *dev, const drv2605_config_t *config)
{
    if (!dev || !config || !config->i2c_dev) {
        return -EINVAL;
    }
    
    LOG_INF("Initializing DRV2605 haptic driver...");
    
    dev->config = config;
    dev->initialized = false;
    dev->calibrated = false;
    dev->last_effect = 0;
    
    // Check if I2C device is ready
    if (!device_is_ready(config->i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    // Test communication by reading status register
    uint8_t status;
    int ret = drv2605_read_reg(dev, DRV2605_REG_STATUS, &status);
    if (ret != 0) {
        LOG_ERR("Failed to communicate with DRV2605: %d", ret);
        return ret;
    }
    
    LOG_INF("DRV2605 status: 0x%02X", status);
    
    // Reset to standby mode
    ret = drv2605_write_reg(dev, DRV2605_REG_MODE, DRV2605_MODE_STANDBY);
    if (ret != 0) {
        LOG_ERR("Failed to set standby mode: %d", ret);
        return ret;
    }
    
    // Clear real-time playback input
    ret = drv2605_write_reg(dev, DRV2605_REG_RTP_INPUT, 0x00);
    if (ret != 0) {
        LOG_ERR("Failed to clear RTP input: %d", ret);
        return ret;
    }
    
    // Set default waveform (strong click) and end sequence
    ret = drv2605_write_reg(dev, DRV2605_REG_WAVESEQ1, 1); // Strong click
    if (ret != 0) {
        LOG_ERR("Failed to set default waveform: %d", ret);
        return ret;
    }
    
    ret = drv2605_write_reg(dev, DRV2605_REG_WAVESEQ2, 0); // End sequence
    if (ret != 0) {
        LOG_ERR("Failed to set sequence end: %d", ret);
        return ret;
    }
    
    // Clear timing registers (use defaults)
    ret = drv2605_write_reg(dev, DRV2605_REG_OVERDRIVE, 0);
    if (ret != 0) {
        LOG_ERR("Failed to clear overdrive: %d", ret);
        return ret;
    }
    
    ret = drv2605_write_reg(dev, DRV2605_REG_SUSTAIN_POS, 0);
    if (ret != 0) {
        LOG_ERR("Failed to clear sustain pos: %d", ret);
        return ret;
    }
    
    ret = drv2605_write_reg(dev, DRV2605_REG_SUSTAIN_NEG, 0);
    if (ret != 0) {
        LOG_ERR("Failed to clear sustain neg: %d", ret);
        return ret;
    }
    
    ret = drv2605_write_reg(dev, DRV2605_REG_BREAK, 0);
    if (ret != 0) {
        LOG_ERR("Failed to clear brake: %d", ret);
        return ret;
    }
    
    // Set audio-to-vibe max input level (if using audio mode)
    ret = drv2605_write_reg(dev, DRV2605_REG_AUDIOMAX, 0x64);
    if (ret != 0) {
        LOG_ERR("Failed to set audio max: %d", ret);
        return ret;
    }

    // Set motor type in feedback control register
    if (config->motor_type == DRV2605_MOTOR_LRA) {
        ret = drv2605_use_lra(dev);
    } else {
        ret = drv2605_use_erm(dev);
    }
    if (ret != 0) {
        LOG_ERR("Failed to set motor type: %d", ret);
        return ret;
    }
    
    // Set waveform library
    ret = drv2605_set_library(dev, config->library);
    if (ret != 0) {
        LOG_ERR("Failed to set library: %d", ret);
        return ret;
    }
    
    // Set rated voltage if specified
    if (config->rated_voltage > 0) {
        ret = drv2605_write_reg(dev, DRV2605_REG_RATEDV, config->rated_voltage);
        if (ret != 0) {
            LOG_ERR("Failed to set rated voltage: %d", ret);
            return ret;
        }
    }
    
    // Set overdrive clamp if specified
    if (config->overdrive_clamp > 0) {
        ret = drv2605_write_reg(dev, DRV2605_REG_CLAMPV, config->overdrive_clamp);
        if (ret != 0) {
            LOG_ERR("Failed to set overdrive clamp: %d", ret);
            return ret;
        }
    }
    
    // Configure control registers for optimal performance
    ret = drv2605_write_reg(dev, DRV2605_REG_CONTROL1, 0x93); // Default settings
    if (ret != 0) {
        LOG_ERR("Failed to set control1: %d", ret);
        return ret;
    }
    
    ret = drv2605_write_reg(dev, DRV2605_REG_CONTROL2, 0xF5); // Default settings
    if (ret != 0) {
        LOG_ERR("Failed to set control2: %d", ret);
        return ret;
    }
    
    ret = drv2605_write_reg(dev, DRV2605_REG_CONTROL3, 0x80); // Default settings
    if (ret != 0) {
        LOG_ERR("Failed to set control3: %d", ret);
        return ret;
    }
    
    // Perform auto-calibration if requested
    if (config->auto_calibration) {
        LOG_INF("Performing auto-calibration...");
        ret = drv2605_auto_calibrate(dev);
        if (ret != 0) {
            LOG_WRN("Auto-calibration failed: %d (continuing anyway)", ret);
        } else {
            dev->calibrated = true;
        }
    }
    
    // Set to internal trigger mode
    ret = drv2605_set_mode(dev, DRV2605_MODE_INTERNAL);
    if (ret != 0) {
        LOG_ERR("Failed to set internal mode: %d", ret);
        return ret;
    }
    
    dev->initialized = true;
    LOG_INF("DRV2605 initialized successfully (Motor: %s, Library: %d, Calibrated: %s)",
            (config->motor_type == DRV2605_MOTOR_LRA) ? "LRA" : "ERM",
            config->library,
            dev->calibrated ? "YES" : "NO");
    
    return 0;
}

int drv2605_play_effect(drv2605_device_t *dev, drv2605_effect_t effect)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    if (effect == 0 || effect > 123) {
        LOG_WRN("Invalid effect ID: %d", effect);
        return -EINVAL;
    }
    
    LOG_DBG("Playing effect: %d", effect);
    
    // Clear sequence registers and set single effect
    int ret = drv2605_write_reg(dev, DRV2605_REG_WAVESEQ1, effect);
    if (ret != 0) return ret;
    
    ret = drv2605_write_reg(dev, DRV2605_REG_WAVESEQ2, 0); // End sequence
    if (ret != 0) return ret;
    
    // Trigger playback
    ret = drv2605_write_reg(dev, DRV2605_REG_GO, 0x01);
    if (ret != 0) return ret;
    
    dev->last_effect = effect;
    return 0;
}

int drv2605_play_sequence(drv2605_device_t *dev, const uint8_t *effects, uint8_t count)
{
    if (!dev || !dev->initialized || !effects) {
        return -EINVAL;
    }
    
    if (count > 8) {
        LOG_WRN("Sequence too long, truncating to 8 effects");
        count = 8;
    }
    
    LOG_DBG("Playing sequence of %d effects", count);
    
    // Program sequence registers
    uint8_t seq_regs[] = {
        DRV2605_REG_WAVESEQ1, DRV2605_REG_WAVESEQ2, DRV2605_REG_WAVESEQ3, DRV2605_REG_WAVESEQ4,
        DRV2605_REG_WAVESEQ5, DRV2605_REG_WAVESEQ6, DRV2605_REG_WAVESEQ7, DRV2605_REG_WAVESEQ8
    };
    
    int ret;
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t value = (i < count) ? effects[i] : 0;
        ret = drv2605_write_reg(dev, seq_regs[i], value);
        if (ret != 0) return ret;
    }
    
    // Trigger playback
    ret = drv2605_write_reg(dev, DRV2605_REG_GO, 0x01);
    if (ret != 0) return ret;
    
    return 0;
}

int drv2605_set_rtp(drv2605_device_t *dev, uint8_t amplitude)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    // Set RTP mode
    int ret = drv2605_set_mode(dev, DRV2605_MODE_RTP);
    if (ret != 0) return ret;
    
    // Set amplitude (bit 7 = 0 for unsigned, bits 6:0 = amplitude)
    uint8_t rtp_value = amplitude & 0x7F;
    ret = drv2605_write_reg(dev, DRV2605_REG_RTP_INPUT, rtp_value);
    if (ret != 0) return ret;
    
    LOG_DBG("Set RTP amplitude: %d", amplitude);
    return 0;
}

int drv2605_stop(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_DBG("Stopping haptic playback");
    
    // Stop playback by writing 0 to GO register
    int ret = drv2605_write_reg(dev, DRV2605_REG_GO, 0x00);
    if (ret != 0) return ret;
    
    // If in RTP mode, set amplitude to 0
    uint8_t mode;
    ret = drv2605_read_reg(dev, DRV2605_REG_MODE, &mode);
    if (ret == 0 && (mode & 0x07) == DRV2605_MODE_RTP) {
        ret = drv2605_write_reg(dev, DRV2605_REG_RTP_INPUT, 0);
    }
    
    return ret;
}

int drv2605_set_mode(drv2605_device_t *dev, drv2605_mode_t mode)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_DBG("Setting mode: %d", mode);
    return drv2605_write_reg(dev, DRV2605_REG_MODE, mode);
}

int drv2605_set_library(drv2605_device_t *dev, drv2605_library_t library)
{
    if (!dev) {
        return -EINVAL;
    }
    
    if (library > DRV2605_LIB_TS2200_F) {
        return -EINVAL;
    }
    
    LOG_DBG("Setting library: %d", library);
    return drv2605_write_reg(dev, DRV2605_REG_LIBRARY, library);
}

int drv2605_is_busy(drv2605_device_t *dev, bool *busy)
{
    if (!dev || !dev->initialized || !busy) {
        return -EINVAL;
    }
    
    uint8_t go_reg;
    int ret = drv2605_read_reg(dev, DRV2605_REG_GO, &go_reg);
    if (ret != 0) return ret;
    
    *busy = (go_reg & 0x01) != 0;
    return 0;
}

int drv2605_auto_calibrate(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_INF("Starting auto-calibration...");
    
    // Set calibration mode
    int ret = drv2605_set_mode(dev, DRV2605_MODE_AUTOCAL);
    if (ret != 0) return ret;
    
    // Start calibration
    ret = drv2605_write_reg(dev, DRV2605_REG_GO, 0x01);
    if (ret != 0) return ret;
    
    // Wait for calibration to complete (up to 2 seconds)
    ret = drv2605_wait_for_ready(dev, 2000);
    if (ret != 0) {
        LOG_ERR("Auto-calibration timeout");
        return ret;
    }
    
    // Check calibration results
    uint8_t status;
    ret = drv2605_get_status(dev, &status);
    if (ret != 0) return ret;
    
    if (status & 0x08) {
        LOG_ERR("Auto-calibration failed (diag bit set)");
        return -EIO;
    }
    
    // Read compensation results
    uint8_t comp_result, back_emf;
    ret = drv2605_read_reg(dev, DRV2605_REG_AUTOCALCOMP, &comp_result);
    if (ret != 0) return ret;
    
    ret = drv2605_read_reg(dev, DRV2605_REG_AUTOCALEMP, &back_emf);
    if (ret != 0) return ret;
    
    LOG_INF("Auto-calibration completed: Comp=0x%02X, BackEMF=0x%02X", comp_result, back_emf);
    
    // Return to internal trigger mode
    ret = drv2605_set_mode(dev, DRV2605_MODE_INTERNAL);
    if (ret != 0) return ret;
    
    dev->calibrated = true;
    return 0;
}

int drv2605_standby(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_DBG("Entering standby mode");
    return drv2605_set_mode(dev, DRV2605_MODE_STANDBY);
}

int drv2605_wakeup(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_DBG("Waking from standby");
    return drv2605_set_mode(dev, DRV2605_MODE_INTERNAL);
}

int drv2605_get_status(drv2605_device_t *dev, uint8_t *status)
{
    if (!dev || !dev->initialized || !status) {
        return -EINVAL;
    }
    
    return drv2605_read_reg(dev, DRV2605_REG_STATUS, status);
}

int drv2605_get_supply_voltage(drv2605_device_t *dev, uint16_t *voltage_mv)
{
    if (!dev || !dev->initialized || !voltage_mv) {
        return -EINVAL;
    }
    
    uint8_t vbat_reg;
    int ret = drv2605_read_reg(dev, DRV2605_REG_VBAT, &vbat_reg);
    if (ret != 0) return ret;
    
    // Convert to millivolts: VBAT = (vbat_reg * 5.44V) / 255
    *voltage_mv = (uint16_t)((vbat_reg * 5440UL) / 255);
    
    return 0;
}

int drv2605_use_erm(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_DBG("Setting motor type to ERM");
    
    // Clear bit 7 in feedback register for ERM mode
    uint8_t feedback_reg;
    int ret = drv2605_read_reg(dev, DRV2605_REG_FEEDBACK, &feedback_reg);
    if (ret != 0) return ret;
    
    feedback_reg &= 0x7F; // Clear N_ERM_LRA bit (bit 7)
    ret = drv2605_write_reg(dev, DRV2605_REG_FEEDBACK, feedback_reg);
    if (ret != 0) return ret;
    
    // Set ERM open loop mode in Control3 register
    uint8_t control3_reg;
    ret = drv2605_read_reg(dev, DRV2605_REG_CONTROL3, &control3_reg);
    if (ret != 0) return ret;
    
    control3_reg |= 0x20; // Set NG_THRESH to 2 for ERM open loop
    return drv2605_write_reg(dev, DRV2605_REG_CONTROL3, control3_reg);
}

int drv2605_use_lra(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_DBG("Setting motor type to LRA");
    
    // Set bit 7 in feedback register for LRA mode
    uint8_t feedback_reg;
    int ret = drv2605_read_reg(dev, DRV2605_REG_FEEDBACK, &feedback_reg);
    if (ret != 0) return ret;
    
    feedback_reg |= 0x80; // Set N_ERM_LRA bit (bit 7)
    return drv2605_write_reg(dev, DRV2605_REG_FEEDBACK, feedback_reg);
}

int drv2605_set_waveform(drv2605_device_t *dev, uint8_t slot, uint8_t waveform)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    if (slot > 7) {
        LOG_WRN("Invalid waveform slot: %d (max 7)", slot);
        return -EINVAL;
    }
    
    LOG_DBG("Setting waveform slot %d to effect %d", slot, waveform);
    
    return drv2605_write_reg(dev, DRV2605_REG_WAVESEQ1 + slot, waveform);
}

int drv2605_go(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_DBG("Starting waveform playback");
    
    return drv2605_write_reg(dev, DRV2605_REG_GO, 0x01);
}

// Convenience functions

int drv2605_button_click(drv2605_device_t *dev, uint8_t intensity)
{
    drv2605_effect_t effect;
    
    switch (intensity) {
    case 0:
        effect = DRV2605_EFFECT_SOFT_BUMP_30;
        break;
    case 1:
        effect = DRV2605_EFFECT_SHARP_CLICK_60;
        break;
    case 2:
    default:
        effect = DRV2605_EFFECT_STRONG_CLICK_100;
        break;
    }
    
    return drv2605_play_effect(dev, effect);
}

int drv2605_trigger_rumble(drv2605_device_t *dev, uint16_t duration_ms, uint8_t intensity)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    // Use RTP mode for variable duration rumble
    int ret = drv2605_set_rtp(dev, intensity);
    if (ret != 0) return ret;
    
    // Sleep for duration, then stop
    k_sleep(K_MSEC(duration_ms));
    
    return drv2605_stop(dev);
}

int drv2605_notification_buzz(drv2605_device_t *dev, uint8_t pattern)
{
    drv2605_effect_t effect;
    
    switch (pattern) {
    case 0:
        effect = DRV2605_EFFECT_STRONG_BUZZ_100;
        break;
    case 1:
        effect = DRV2605_EFFECT_DOUBLE_CLICK_100;
        break;
    case 2:
    default:
        effect = DRV2605_EFFECT_TRIPLE_CLICK_100;
        break;
    }
    
    return drv2605_play_effect(dev, effect);
}

int drv2605_enable_external_trigger(drv2605_device_t *dev, bool edge_trigger)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    drv2605_mode_t mode = edge_trigger ? DRV2605_MODE_EXTERNAL : DRV2605_MODE_EXTERNAL_LVL;
    
    LOG_INF("Enabling external trigger mode: %s", edge_trigger ? "EDGE" : "LEVEL");
    
    return drv2605_set_mode(dev, mode);
}

int drv2605_setup_digital_trigger(drv2605_device_t *dev, drv2605_effect_t trigger_effect)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_INF("Setting up digital trigger mode with effect %d", trigger_effect);
    
    // Set the effect to play when trigger is activated
    int ret = drv2605_set_waveform(dev, 0, trigger_effect);
    if (ret != 0) return ret;
    
    // End the sequence
    ret = drv2605_set_waveform(dev, 1, 0);
    if (ret != 0) return ret;
    
    // Enable external edge trigger mode
    ret = drv2605_enable_external_trigger(dev, true); // Edge trigger
    if (ret != 0) return ret;
    
    LOG_INF("Digital trigger configured - effect will play on trigger pull");
    return 0;
}

int drv2605_enable_analog_trigger(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_INF("Enabling PWM/analog trigger mode for proportional haptic feedback");
    
    return drv2605_set_mode(dev, DRV2605_MODE_PWM);
}

int drv2605_enable_internal_trigger(drv2605_device_t *dev)
{
    if (!dev || !dev->initialized) {
        return -EINVAL;
    }
    
    LOG_INF("Returning to internal (software) trigger mode");
    
    return drv2605_set_mode(dev, DRV2605_MODE_INTERNAL);
}

// Private functions

static int drv2605_write_reg(const drv2605_device_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    
    int ret = i2c_write(dev->config->i2c_dev, buf, sizeof(buf), dev->config->i2c_addr);
    if (ret != 0) {
        LOG_ERR("I2C write failed: reg=0x%02X, value=0x%02X, error=%d", reg, value, ret);
    }
    
    return ret;
}

static int drv2605_read_reg(const drv2605_device_t *dev, uint8_t reg, uint8_t *value)
{
    int ret = i2c_write_read(dev->config->i2c_dev, dev->config->i2c_addr,
                            &reg, 1, value, 1);
    if (ret != 0) {
        LOG_ERR("I2C read failed: reg=0x%02X, error=%d", reg, ret);
    }
    
    return ret;
}

static int drv2605_wait_for_ready(const drv2605_device_t *dev, uint32_t timeout_ms)
{
    uint32_t start_time = k_uptime_get_32();
    bool busy = true;
    
    while (busy && (k_uptime_get_32() - start_time) < timeout_ms) {
        int ret = drv2605_is_busy(dev, &busy);
        if (ret != 0) return ret;
        
        if (busy) {
            k_sleep(K_MSEC(10));
        }
    }
    
    return busy ? -ETIMEDOUT : 0;
}
