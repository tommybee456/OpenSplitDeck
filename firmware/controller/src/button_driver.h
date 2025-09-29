/**
 ******************************************************************************
 * @file    button_driver.h
 * @brief   GPIO Button Driver Library for Controller
 * @author  Controller Team
 * @version V1.0
 * @date    2025
 ******************************************************************************
 * @attention
 * 
 * This library provides a clean interface for managing all controller GPIO
 * buttons including digital buttons, D-pad, and other input pins.
 * 
 ******************************************************************************
 */

#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>
#include <stdbool.h>

// Button status enumeration
typedef enum {
    BUTTON_STATUS_OK = 0,
    BUTTON_STATUS_ERROR = -1,
    BUTTON_STATUS_NOT_INITIALIZED = -2,
    BUTTON_STATUS_INVALID_BUTTON = -3
} button_status_t;

// Button ID enumeration for easy reference
typedef enum {
    BUTTON_STICK_CLICK = 0,
    BUTTON_BUMPER,
    BUTTON_START,
    BUTTON_P4,
    BUTTON_P5,
    BUTTON_MODE,
    BUTTON_DPAD_DOWN,
    BUTTON_DPAD_LEFT,
    BUTTON_DPAD_RIGHT,
    BUTTON_DPAD_UP,
    BUTTON_PAD_CLICK,
    BUTTON_COUNT  // Total number of buttons
} button_id_t;

// Button state structure
typedef struct {
    bool current_state;     // Current button state (true = pressed)
    bool previous_state;    // Previous button state for edge detection
    uint32_t press_time;    // Time when button was first pressed
    uint32_t release_time;  // Time when button was released
    bool is_pressed;        // True if button is currently pressed
    bool just_pressed;      // True if button was just pressed this cycle
    bool just_released;     // True if button was just released this cycle
} button_state_t;

// Button configuration structure
typedef struct {
    const struct gpio_dt_spec *gpio_spec;  // GPIO specification
    bool active_low;                       // True if button is active low
    bool pull_up;                         // True if internal pull-up should be enabled
} button_config_t;

// Controller button data structure (matches main.c format)
typedef struct {
    uint8_t buttons;    // Main button byte (bits 0-7)
    uint8_t flags;      // Additional flags for extra buttons
} button_data_t;

// Button driver context structure
typedef struct {
    bool initialized;
    button_state_t button_states[BUTTON_COUNT];
    button_config_t button_configs[BUTTON_COUNT];
    button_data_t current_data;
    uint32_t scan_count;
    bool haptic_feedback_enabled;
} button_driver_context_t;

// Function prototypes

/**
 * @brief Initialize the button driver with GPIO specifications
 * @param stick_click GPIO spec for analog stick click button
 * @param bumper GPIO spec for bumper button (L1)
 * @param start GPIO spec for start button
 * @param button_p4 GPIO spec for P4 button
 * @param button_p5 GPIO spec for P5 button
 * @param mode_button GPIO spec for mode button
 * @param dpad_down GPIO spec for D-pad down
 * @param dpad_left GPIO spec for D-pad left
 * @param dpad_right GPIO spec for D-pad right
 * @param dpad_up GPIO spec for D-pad up
 * @param pad_click GPIO spec for trackpad click
 * @return button_status_t Status of initialization
 */
button_status_t button_driver_init(
    const struct gpio_dt_spec *stick_click,
    const struct gpio_dt_spec *bumper,
    const struct gpio_dt_spec *start,
    const struct gpio_dt_spec *button_p4,
    const struct gpio_dt_spec *button_p5,
    const struct gpio_dt_spec *mode_button,
    const struct gpio_dt_spec *dpad_down,
    const struct gpio_dt_spec *dpad_left,
    const struct gpio_dt_spec *dpad_right,
    const struct gpio_dt_spec *dpad_up,
    const struct gpio_dt_spec *pad_click
);

/**
 * @brief Scan all buttons and update their states
 * @return button_status_t Status of scan operation
 */
button_status_t button_driver_scan(void);

/**
 * @brief Get current button data in controller format
 * @param data Pointer to button_data_t structure to fill
 * @return button_status_t Status of operation
 */
button_status_t button_driver_get_data(button_data_t *data);

/**
 * @brief Check if a specific button is currently pressed
 * @param button_id ID of button to check
 * @return bool True if button is pressed, false otherwise
 */
bool button_driver_is_pressed(button_id_t button_id);

/**
 * @brief Check if a specific button was just pressed (edge detection)
 * @param button_id ID of button to check
 * @return bool True if button was just pressed this cycle
 */
bool button_driver_just_pressed(button_id_t button_id);

/**
 * @brief Check if a specific button was just released (edge detection)
 * @param button_id ID of button to check
 * @return bool True if button was just released this cycle
 */
bool button_driver_just_released(button_id_t button_id);

/**
 * @brief Get the duration a button has been pressed
 * @param button_id ID of button to check
 * @return uint32_t Duration in milliseconds (0 if not pressed)
 */
uint32_t button_driver_get_press_duration(button_id_t button_id);

/**
 * @brief Enable or disable haptic feedback for button presses
 * @param enable True to enable haptic feedback, false to disable
 * @return button_status_t Status of operation
 */
button_status_t button_driver_set_haptic_feedback(bool enable);

/**
 * @brief Get current button statistics
 * @param total_scans Pointer to store total number of scans performed
 * @param active_buttons Pointer to store number of currently active buttons
 * @return button_status_t Status of operation
 */
button_status_t button_driver_get_stats(uint32_t *total_scans, uint8_t *active_buttons);

/**
 * @brief Check if button driver is properly initialized
 * @return bool True if initialized, false otherwise
 */
bool button_driver_is_initialized(void);

/**
 * @brief Reset all button states and statistics
 * @return button_status_t Status of operation
 */
button_status_t button_driver_reset(void);

/**
 * @brief Get human-readable name for a button ID
 * @param button_id ID of button
 * @return const char* Button name string
 */
const char* button_driver_get_button_name(button_id_t button_id);

#endif // BUTTON_DRIVER_H
