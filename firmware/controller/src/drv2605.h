/*
 * DRV2605 Haptic Motor Driver
 * 
 * This module provides control for the DRV2605 haptic motor driver
 * including waveform playback, real-time control, and effect sequencing.
 */

#ifndef DRV2605_H
#define DRV2605_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

// DRV2605 I2C Address
#define DRV2605_I2C_ADDR        0x5A

// DRV2605 Register Addresses
#define DRV2605_REG_STATUS      0x00
#define DRV2605_REG_MODE        0x01
#define DRV2605_REG_RTP_INPUT   0x02
#define DRV2605_REG_LIBRARY     0x03
#define DRV2605_REG_WAVESEQ1    0x04
#define DRV2605_REG_WAVESEQ2    0x05
#define DRV2605_REG_WAVESEQ3    0x06
#define DRV2605_REG_WAVESEQ4    0x07
#define DRV2605_REG_WAVESEQ5    0x08
#define DRV2605_REG_WAVESEQ6    0x09
#define DRV2605_REG_WAVESEQ7    0x0A
#define DRV2605_REG_WAVESEQ8    0x0B
#define DRV2605_REG_GO          0x0C
#define DRV2605_REG_OVERDRIVE   0x0D
#define DRV2605_REG_SUSTAIN_POS 0x0E
#define DRV2605_REG_SUSTAIN_NEG 0x0F
#define DRV2605_REG_BREAK       0x10
#define DRV2605_REG_AUDIOCTRL   0x11
#define DRV2605_REG_AUDIOLVL    0x12
#define DRV2605_REG_AUDIOMAX    0x13
#define DRV2605_REG_RATEDV      0x16
#define DRV2605_REG_CLAMPV      0x17
#define DRV2605_REG_AUTOCALCOMP 0x18
#define DRV2605_REG_AUTOCALEMP  0x19
#define DRV2605_REG_FEEDBACK    0x1A
#define DRV2605_REG_CONTROL1    0x1B
#define DRV2605_REG_CONTROL2    0x1C
#define DRV2605_REG_CONTROL3    0x1D
#define DRV2605_REG_CONTROL4    0x1E
#define DRV2605_REG_VBAT        0x21
#define DRV2605_REG_LRARESON    0x22

// Operating Modes
typedef enum {
    DRV2605_MODE_STANDBY      = 0x00,
    DRV2605_MODE_INTERNAL     = 0x00,  // Internal trigger mode
    DRV2605_MODE_EXTERNAL     = 0x01,  // External edge trigger
    DRV2605_MODE_EXTERNAL_LVL = 0x02,  // External level trigger
    DRV2605_MODE_PWM          = 0x03,  // PWM/analog input
    DRV2605_MODE_AUDIO        = 0x04,  // Audio-to-haptic
    DRV2605_MODE_RTP          = 0x05,  // Real-time playback
    DRV2605_MODE_DIAGNOSE     = 0x06,  // Diagnostics
    DRV2605_MODE_AUTOCAL      = 0x07   // Auto-calibration
} drv2605_mode_t;

// Waveform Libraries
typedef enum {
    DRV2605_LIB_EMPTY         = 0,     // Empty
    DRV2605_LIB_TS2200_A      = 1,     // TS2200 Library A
    DRV2605_LIB_TS2200_B      = 2,     // TS2200 Library B
    DRV2605_LIB_TS2200_C      = 3,     // TS2200 Library C
    DRV2605_LIB_TS2200_D      = 4,     // TS2200 Library D
    DRV2605_LIB_TS2200_E      = 5,     // TS2200 Library E
    DRV2605_LIB_LRA           = 6,     // LRA Library
    DRV2605_LIB_TS2200_F      = 7      // TS2200 Library F
} drv2605_library_t;

// Common Waveform Effects (for TS2200 Library A)
typedef enum {
    DRV2605_EFFECT_STRONG_CLICK_100    = 1,
    DRV2605_EFFECT_STRONG_CLICK_60     = 2,
    DRV2605_EFFECT_STRONG_CLICK_30     = 3,
    DRV2605_EFFECT_SHARP_CLICK_100     = 4,
    DRV2605_EFFECT_SHARP_CLICK_60      = 5,
    DRV2605_EFFECT_SHARP_CLICK_30      = 6,
    DRV2605_EFFECT_SOFT_BUMP_100       = 7,
    DRV2605_EFFECT_SOFT_BUMP_60        = 8,
    DRV2605_EFFECT_SOFT_BUMP_30        = 9,
    DRV2605_EFFECT_DOUBLE_CLICK_100    = 10,
    DRV2605_EFFECT_DOUBLE_CLICK_60     = 11,
    DRV2605_EFFECT_TRIPLE_CLICK_100    = 12,
    DRV2605_EFFECT_SOFT_FUZZ_60        = 13,
    DRV2605_EFFECT_STRONG_BUZZ_100     = 14,
    DRV2605_EFFECT_ALERT_750MS         = 15,
    DRV2605_EFFECT_ALERT_1000MS        = 16,
    DRV2605_EFFECT_STRONG_CLICK_1      = 17,
    DRV2605_EFFECT_STRONG_CLICK_2      = 18,
    DRV2605_EFFECT_STRONG_CLICK_3      = 19,
    DRV2605_EFFECT_STRONG_CLICK_4      = 20,
    DRV2605_EFFECT_MEDIUM_CLICK_1      = 21,
    DRV2605_EFFECT_MEDIUM_CLICK_2      = 22,
    DRV2605_EFFECT_MEDIUM_CLICK_3      = 23,
    DRV2605_EFFECT_SHARP_TICK_1        = 24,
    DRV2605_EFFECT_SHARP_TICK_2        = 25,
    DRV2605_EFFECT_SHARP_TICK_3        = 26,
    DRV2605_EFFECT_SHORT_DOUBLE_SHARP_TICK_1 = 27,
    DRV2605_EFFECT_SHORT_DOUBLE_SHARP_TICK_2 = 28,
    DRV2605_EFFECT_SHORT_DOUBLE_SHARP_TICK_3 = 29,
    DRV2605_EFFECT_LONG_DOUBLE_SHARP_TICK_1  = 30,
    DRV2605_EFFECT_LONG_DOUBLE_SHARP_TICK_2  = 31,
    DRV2605_EFFECT_LONG_DOUBLE_SHARP_TICK_3  = 32,
    DRV2605_EFFECT_BUZZ_1              = 33,
    DRV2605_EFFECT_BUZZ_2              = 34,
    DRV2605_EFFECT_BUZZ_3              = 35,
    DRV2605_EFFECT_BUZZ_4              = 36,
    DRV2605_EFFECT_BUZZ_5              = 37,
    DRV2605_EFFECT_PULSING_STRONG_1    = 38,
    DRV2605_EFFECT_PULSING_STRONG_2    = 39,
    DRV2605_EFFECT_PULSING_STRONG_3    = 40,
    DRV2605_EFFECT_PULSING_SHARP_1     = 41,
    DRV2605_EFFECT_PULSING_SHARP_2     = 42,
    DRV2605_EFFECT_TRANSITION_CLICK_1  = 43,
    DRV2605_EFFECT_TRANSITION_CLICK_2  = 44,
    DRV2605_EFFECT_TRANSITION_CLICK_3  = 45,
    DRV2605_EFFECT_TRANSITION_CLICK_4  = 46,
    DRV2605_EFFECT_TRANSITION_CLICK_5  = 47,
    DRV2605_EFFECT_TRANSITION_CLICK_6  = 48,
    DRV2605_EFFECT_TRANSITION_HUM_1    = 49,
    DRV2605_EFFECT_TRANSITION_HUM_2    = 50,
    DRV2605_EFFECT_TRANSITION_HUM_3    = 51,
    DRV2605_EFFECT_TRANSITION_HUM_4    = 52,
    DRV2605_EFFECT_TRANSITION_HUM_5    = 53,
    DRV2605_EFFECT_TRANSITION_HUM_6    = 54,
    DRV2605_EFFECT_TRANSITION_RAMP_DOWN_LONG_SMOOTH_1 = 55,
    DRV2605_EFFECT_TRANSITION_RAMP_DOWN_LONG_SMOOTH_2 = 56,
    DRV2605_EFFECT_TRANSITION_RAMP_DOWN_MED_SMOOTH_1  = 57,
    DRV2605_EFFECT_TRANSITION_RAMP_DOWN_MED_SMOOTH_2  = 58,
    DRV2605_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SMOOTH_1 = 59,
    DRV2605_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SMOOTH_2 = 60,
    DRV2605_EFFECT_TRANSITION_RAMP_UP_LONG_SMOOTH_1   = 61,
    DRV2605_EFFECT_TRANSITION_RAMP_UP_LONG_SMOOTH_2   = 62,
    DRV2605_EFFECT_TRANSITION_RAMP_UP_MED_SMOOTH_1    = 63,
    DRV2605_EFFECT_TRANSITION_RAMP_UP_MED_SMOOTH_2    = 64,
    DRV2605_EFFECT_TRANSITION_RAMP_UP_SHORT_SMOOTH_1  = 65,
    DRV2605_EFFECT_TRANSITION_RAMP_UP_SHORT_SMOOTH_2  = 66,
    DRV2605_EFFECT_LONG_BUZZ_FOR_PROGRAMMATIC_STOPPING = 67,
    DRV2605_EFFECT_SMOOTH_HUM_1        = 68,
    DRV2605_EFFECT_SMOOTH_HUM_2        = 69,
    DRV2605_EFFECT_SMOOTH_HUM_3        = 70,
    DRV2605_EFFECT_SMOOTH_HUM_4        = 71,
    DRV2605_EFFECT_SMOOTH_HUM_5        = 72,
    DRV2605_EFFECT_CLICK_TRAIN_10_100  = 73,
    DRV2605_EFFECT_CLICK_TRAIN_10_60   = 74,
    DRV2605_EFFECT_CLICK_TRAIN_10_30   = 75,
    DRV2605_EFFECT_CLICK_TRAIN_5_100   = 76,
    DRV2605_EFFECT_CLICK_TRAIN_5_60    = 77,
    DRV2605_EFFECT_CLICK_TRAIN_5_30    = 78,
    DRV2605_EFFECT_CLICK_TRAIN_1_100   = 79,
    DRV2605_EFFECT_CLICK_TRAIN_1_60    = 80,
    DRV2605_EFFECT_CLICK_TRAIN_1_30    = 81
} drv2605_effect_t;

// Motor Type
typedef enum {
    DRV2605_MOTOR_ERM         = 0,     // Eccentric Rotating Mass
    DRV2605_MOTOR_LRA         = 1      // Linear Resonant Actuator
} drv2605_motor_type_t;

// Device configuration structure
typedef struct {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
    drv2605_motor_type_t motor_type;
    drv2605_library_t library;
    uint8_t rated_voltage;      // Motor rated voltage (V_RMS * 255 / 5.44V)
    uint8_t overdrive_clamp;    // Overdrive clamp voltage
    bool auto_calibration;      // Enable auto-calibration
} drv2605_config_t;

// Device instance structure
typedef struct {
    const drv2605_config_t *config;
    bool initialized;
    bool calibrated;
    uint8_t last_effect;
} drv2605_device_t;

// Function prototypes

/**
 * Initialize the DRV2605 haptic driver
 * @param dev Pointer to device instance
 * @param config Pointer to configuration
 * @return 0 on success, negative on error
 */
int drv2605_init(drv2605_device_t *dev, const drv2605_config_t *config);

/**
 * Play a single haptic effect
 * @param dev Pointer to device instance
 * @param effect Effect ID to play
 * @return 0 on success, negative on error
 */
int drv2605_play_effect(drv2605_device_t *dev, drv2605_effect_t effect);

/**
 * Play a sequence of haptic effects
 * @param dev Pointer to device instance
 * @param effects Array of effect IDs (max 8, 0 = end of sequence)
 * @param count Number of effects in sequence
 * @return 0 on success, negative on error
 */
int drv2605_play_sequence(drv2605_device_t *dev, const uint8_t *effects, uint8_t count);

/**
 * Set real-time playback value
 * @param dev Pointer to device instance
 * @param amplitude Amplitude value (0-127)
 * @return 0 on success, negative on error
 */
int drv2605_set_rtp(drv2605_device_t *dev, uint8_t amplitude);

/**
 * Stop current haptic playback
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_stop(drv2605_device_t *dev);

/**
 * Set operating mode
 * @param dev Pointer to device instance
 * @param mode Operating mode
 * @return 0 on success, negative on error
 */
int drv2605_set_mode(drv2605_device_t *dev, drv2605_mode_t mode);

/**
 * Set waveform library
 * @param dev Pointer to device instance
 * @param library Library selection
 * @return 0 on success, negative on error
 */
int drv2605_set_library(drv2605_device_t *dev, drv2605_library_t library);

/**
 * Check if device is busy playing an effect
 * @param dev Pointer to device instance
 * @param busy Pointer to store busy status
 * @return 0 on success, negative on error
 */
int drv2605_is_busy(drv2605_device_t *dev, bool *busy);

/**
 * Perform auto-calibration
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_auto_calibrate(drv2605_device_t *dev);

/**
 * Enter standby mode (low power)
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_standby(drv2605_device_t *dev);

/**
 * Wake from standby mode
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_wakeup(drv2605_device_t *dev);

/**
 * Read device status register
 * @param dev Pointer to device instance
 * @param status Pointer to store status value
 * @return 0 on success, negative on error
 */
int drv2605_get_status(drv2605_device_t *dev, uint8_t *status);

/**
 * Read supply voltage
 * @param dev Pointer to device instance
 * @param voltage_mv Pointer to store voltage in millivolts
 * @return 0 on success, negative on error
 */
int drv2605_get_supply_voltage(drv2605_device_t *dev, uint16_t *voltage_mv);

/**
 * Use ERM (Eccentric Rotating Mass) motor mode
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_use_erm(drv2605_device_t *dev);

/**
 * Use LRA (Linear Resonant Actuator) motor mode
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_use_lra(drv2605_device_t *dev);

/**
 * Set a specific waveform in sequence slot
 * @param dev Pointer to device instance
 * @param slot Sequence slot (0-7)
 * @param waveform Waveform ID (0 = end sequence)
 * @return 0 on success, negative on error
 */
int drv2605_set_waveform(drv2605_device_t *dev, uint8_t slot, uint8_t waveform);

/**
 * Start waveform playback
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_go(drv2605_device_t *dev);

// Convenience functions for common controller haptics

/**
 * Play button click feedback
 * @param dev Pointer to device instance
 * @param intensity 0=light, 1=medium, 2=strong
 * @return 0 on success, negative on error
 */
int drv2605_button_click(drv2605_device_t *dev, uint8_t intensity);

/**
 * Play trigger rumble
 * @param dev Pointer to device instance
 * @param duration_ms Duration in milliseconds
 * @param intensity Intensity 0-127
 * @return 0 on success, negative on error
 */
int drv2605_trigger_rumble(drv2605_device_t *dev, uint16_t duration_ms, uint8_t intensity);

/**
 * Play notification buzz
 * @param dev Pointer to device instance
 * @param pattern 0=single, 1=double, 2=triple
 * @return 0 on success, negative on error
 */
int drv2605_notification_buzz(drv2605_device_t *dev, uint8_t pattern);

/**
 * Enable external trigger mode for hardware-controlled haptics
 * @param dev Pointer to device instance
 * @param edge_trigger true=edge trigger, false=level trigger
 * @return 0 on success, negative on error
 */
int drv2605_enable_external_trigger(drv2605_device_t *dev, bool edge_trigger);

/**
 * Configure digital trigger mode with specific effect
 * @param dev Pointer to device instance
 * @param trigger_effect Effect to play when trigger is activated
 * @return 0 on success, negative on error
 */
int drv2605_setup_digital_trigger(drv2605_device_t *dev, drv2605_effect_t trigger_effect);

/**
 * Enable PWM/analog trigger mode for proportional haptic feedback
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_enable_analog_trigger(drv2605_device_t *dev);

/**
 * Return to internal (software) trigger mode
 * @param dev Pointer to device instance
 * @return 0 on success, negative on error
 */
int drv2605_enable_internal_trigger(drv2605_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* DRV2605_H */
