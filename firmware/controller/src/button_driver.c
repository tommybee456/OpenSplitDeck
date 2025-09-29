/**
 ******************************************************************************
 * @file    button_driver.c
 * @brief   GPIO Button Driver Library Implementation
 * @author  Controller Team
 * @version V1.0
 * @date    2025
 ******************************************************************************
 */

#include "button_driver.h"
#include "haptic_driver.h" // For haptic feedback
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(button_driver, LOG_LEVEL_INF);

// Global context
static button_driver_context_t g_button_ctx = {0};

// Button name strings for debugging
static const char *button_names[BUTTON_COUNT] = {
    "STICK_CLICK",
    "BUMPER",
    "START",
    "P4",
    "P5",
    "MODE",
    "DPAD_DOWN",
    "DPAD_LEFT",
    "DPAD_RIGHT",
    "DPAD_UP",
    "PAD_CLICK"};

/**
 * @brief Initialize the button driver with GPIO specifications
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
    const struct gpio_dt_spec *pad_click)
{
    LOG_INF("Initializing button driver...");

    // Clear context
    memset(&g_button_ctx, 0, sizeof(g_button_ctx));

    // Store GPIO specifications
    const struct gpio_dt_spec *gpio_specs[BUTTON_COUNT] = {
        stick_click, // BUTTON_STICK_CLICK
        bumper,      // BUTTON_BUMPER
        start,       // BUTTON_START
        button_p4,   // BUTTON_P4
        button_p5,   // BUTTON_P5
        mode_button, // BUTTON_MODE
        dpad_down,   // BUTTON_DPAD_DOWN
        dpad_left,   // BUTTON_DPAD_LEFT
        dpad_right,  // BUTTON_DPAD_RIGHT
        dpad_up,     // BUTTON_DPAD_UP
        pad_click    // BUTTON_PAD_CLICK
    };

    // Configure each button
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        if (!gpio_specs[i])
        {
            LOG_ERR("Button %d (%s) GPIO spec is NULL", i, button_names[i]);
            return BUTTON_STATUS_ERROR;
        }

        // Store configuration
        g_button_ctx.button_configs[i].gpio_spec = gpio_specs[i];
        g_button_ctx.button_configs[i].active_low = (gpio_specs[i]->dt_flags & GPIO_ACTIVE_LOW) != 0;
        g_button_ctx.button_configs[i].pull_up = true; // All buttons use pull-up

        // Configure GPIO
        int ret = gpio_pin_configure_dt(gpio_specs[i], GPIO_INPUT | GPIO_PULL_UP);
        if (ret != 0)
        {
            LOG_ERR("Failed to configure button %d (%s): %d", i, button_names[i], ret);
            return BUTTON_STATUS_ERROR;
        }

        // Initialize button state
        g_button_ctx.button_states[i].current_state = false;
        g_button_ctx.button_states[i].previous_state = false;
        g_button_ctx.button_states[i].press_time = 0;
        g_button_ctx.button_states[i].release_time = 0;
        g_button_ctx.button_states[i].is_pressed = false;
        g_button_ctx.button_states[i].just_pressed = false;
        g_button_ctx.button_states[i].just_released = false;

        LOG_INF("Button %d (%s) configured successfully", i, button_names[i]);
    }

    // Enable haptic feedback by default
    g_button_ctx.haptic_feedback_enabled = true;
    g_button_ctx.initialized = true;
    g_button_ctx.scan_count = 0;

    LOG_INF("Button driver initialized with %d buttons", BUTTON_COUNT);
    return BUTTON_STATUS_OK;
}

/**
 * @brief Scan all buttons and update their states
 */
button_status_t button_driver_scan(void)
{
    if (!g_button_ctx.initialized)
    {
        return BUTTON_STATUS_NOT_INITIALIZED;
    }

    uint32_t current_time = k_uptime_get_32();
    bool any_button_just_pressed = false;

    // Scan each button
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        button_state_t *state = &g_button_ctx.button_states[i];
        const button_config_t *config = &g_button_ctx.button_configs[i];

        // Store previous state
        state->previous_state = state->current_state;

        // Read current GPIO state
        int gpio_value = gpio_pin_get_dt(config->gpio_spec);
        if (gpio_value < 0)
        {
            LOG_WRN("Failed to read button %d (%s): %d", i, button_names[i], gpio_value);
            continue;
        }

        // Convert GPIO value to button state (considering active_low)
        if (config->active_low)
        {
            state->current_state = !(gpio_value == 0); // Active low: 0 = pressed
        }
        else
        {
            state->current_state = !(gpio_value == 1); // Active high: 1 = pressed
        }

        // Update edge detection flags
        state->just_pressed = (!state->previous_state && state->current_state);
        state->just_released = (state->previous_state && !state->current_state);
        state->is_pressed = state->current_state;

        // Update timing
        if (state->just_pressed)
        {
            state->press_time = current_time;
            any_button_just_pressed = true;
        }
        else if (state->just_released)
        {
            state->release_time = current_time;
        }
    }

    // Provide haptic feedback for button presses
    // if (any_button_just_pressed && g_button_ctx.haptic_feedback_enabled)
    // {
    //     if (haptic_is_available())
    //     {
    //         haptic_play_pattern(HAPTIC_PATTERN_BUTTON_PRESS);
    //     }
    // }

    g_button_ctx.scan_count++;
    return BUTTON_STATUS_OK;
}

/**
 * @brief Get current button data in controller format
 */
button_status_t button_driver_get_data(button_data_t *data)
{
    if (!g_button_ctx.initialized)
    {
        return BUTTON_STATUS_NOT_INITIALIZED;
    }

    if (!data)
    {
        return BUTTON_STATUS_ERROR;
    }

    // Clear data structure
    data->buttons = 0;
    data->flags = 0;

    // Map button states to controller data format
    // Buttons format: Start/Select(0x80), Trackpad Click(0x40), Stick Click(0x20), Bumper(0x10), A/Down(0x08), B/Right(0x04), X/Left(0x02), Y/Up(0x01)
    if (g_button_ctx.button_states[BUTTON_START].is_pressed) {
        data->buttons |= 0x80; // Bit 7: Start/Select
    }
    if (g_button_ctx.button_states[BUTTON_PAD_CLICK].is_pressed) {
        data->buttons |= 0x40; // Bit 6: Trackpad Click
    }
    if (g_button_ctx.button_states[BUTTON_STICK_CLICK].is_pressed) {
        data->buttons |= 0x20; // Bit 5: Stick click
    }
    if (g_button_ctx.button_states[BUTTON_BUMPER].is_pressed) {
        data->buttons |= 0x10; // Bit 4: Bumper (L1)
    }
    if (g_button_ctx.button_states[BUTTON_DPAD_DOWN].is_pressed) {
        data->buttons |= 0x08; // Bit 3: A/Down
    }
    if (g_button_ctx.button_states[BUTTON_DPAD_RIGHT].is_pressed) {
        data->buttons |= 0x04; // Bit 2: B/Right
    }
    if (g_button_ctx.button_states[BUTTON_DPAD_LEFT].is_pressed) {
        data->buttons |= 0x02; // Bit 1: X/Left
    }
    if (g_button_ctx.button_states[BUTTON_DPAD_UP].is_pressed) {
        data->buttons |= 0x01; // Bit 0: Y/Up
    }

    // Flags format: ID(0x80), Mode1(0x40), Mode2(0x20), TBD(0x10), TBD(0x08), TrackpadTap(0x04), P4(0x02), P5(0x01)
    // Note: You might want to set ID bit if this is a specific controller
    // data->flags |= 0x80; // ID bit - uncomment if needed
    
    if (g_button_ctx.button_states[BUTTON_MODE].is_pressed) {
        data->flags |= 0x40; // Bit 6: Mode1 button
    }
    // If you have a second mode button, map it to:
    // data->flags |= 0x20; // Bit 5: Mode2 button
    
    // Bit 4 and 3 are marked as TBD (To Be Determined)
    
    // Note: TrackpadTap might be different from TrackpadClick - you may need to distinguish these
    // For now, I'll leave this bit available for a different trackpad gesture
    // data->flags |= 0x04; // Bit 2: TrackpadTap (different from click?)
    
    if (g_button_ctx.button_states[BUTTON_P4].is_pressed) {
        data->flags |= 0x02; // Bit 1: P4 button
    }
    if (g_button_ctx.button_states[BUTTON_P5].is_pressed) {
        data->flags |= 0x01; // Bit 0: P5 button
    }

    // LOG_INF("Buttons: %c%c%c%c%c%c%c%c (0x%02X) Flags: %c%c%c%c%c%c%c%c (0x%02X)",
    //         (data->buttons & 0x80) ? '1' : '0',
    //         (data->buttons & 0x40) ? '1' : '0',
    //         (data->buttons & 0x20) ? '1' : '0',
    //         (data->buttons & 0x10) ? '1' : '0',
    //         (data->buttons & 0x08) ? '1' : '0',
    //         (data->buttons & 0x04) ? '1' : '0',
    //         (data->buttons & 0x02) ? '1' : '0',
    //         (data->buttons & 0x01) ? '1' : '0',
    //         data->buttons,
    //         (data->flags & 0x80) ? '1' : '0',
    //         (data->flags & 0x40) ? '1' : '0',
    //         (data->flags & 0x20) ? '1' : '0',
    //         (data->flags & 0x10) ? '1' : '0',
    //         (data->flags & 0x08) ? '1' : '0',
    //         (data->flags & 0x04) ? '1' : '0',
    //         (data->flags & 0x02) ? '1' : '0',
    //         (data->flags & 0x01) ? '1' : '0',
    //         data->flags);

    // Store current data in context
    g_button_ctx.current_data = *data;

    return BUTTON_STATUS_OK;
}

/**
 * @brief Check if a specific button is currently pressed
 */
bool button_driver_is_pressed(button_id_t button_id)
{
    if (!g_button_ctx.initialized || button_id >= BUTTON_COUNT)
    {
        return false;
    }

    return g_button_ctx.button_states[button_id].is_pressed;
}

/**
 * @brief Check if a specific button was just pressed (edge detection)
 */
bool button_driver_just_pressed(button_id_t button_id)
{
    if (!g_button_ctx.initialized || button_id >= BUTTON_COUNT)
    {
        return false;
    }

    return g_button_ctx.button_states[button_id].just_pressed;
}

/**
 * @brief Check if a specific button was just released (edge detection)
 */
bool button_driver_just_released(button_id_t button_id)
{
    if (!g_button_ctx.initialized || button_id >= BUTTON_COUNT)
    {
        return false;
    }

    return g_button_ctx.button_states[button_id].just_released;
}

/**
 * @brief Get the duration a button has been pressed
 */
uint32_t button_driver_get_press_duration(button_id_t button_id)
{
    if (!g_button_ctx.initialized || button_id >= BUTTON_COUNT)
    {
        return 0;
    }

    const button_state_t *state = &g_button_ctx.button_states[button_id];

    if (!state->is_pressed)
    {
        return 0; // Button not currently pressed
    }

    return k_uptime_get_32() - state->press_time;
}

/**
 * @brief Enable or disable haptic feedback for button presses
 */
button_status_t button_driver_set_haptic_feedback(bool enable)
{
    if (!g_button_ctx.initialized)
    {
        return BUTTON_STATUS_NOT_INITIALIZED;
    }

    g_button_ctx.haptic_feedback_enabled = enable;
    LOG_INF("Button haptic feedback %s", enable ? "enabled" : "disabled");

    return BUTTON_STATUS_OK;
}

/**
 * @brief Get current button statistics
 */
button_status_t button_driver_get_stats(uint32_t *total_scans, uint8_t *active_buttons)
{
    if (!g_button_ctx.initialized)
    {
        return BUTTON_STATUS_NOT_INITIALIZED;
    }

    if (total_scans)
    {
        *total_scans = g_button_ctx.scan_count;
    }

    if (active_buttons)
    {
        uint8_t count = 0;
        for (int i = 0; i < BUTTON_COUNT; i++)
        {
            if (g_button_ctx.button_states[i].is_pressed)
            {
                count++;
            }
        }
        *active_buttons = count;
    }

    return BUTTON_STATUS_OK;
}

/**
 * @brief Check if button driver is properly initialized
 */
bool button_driver_is_initialized(void)
{
    return g_button_ctx.initialized;
}

/**
 * @brief Reset all button states and statistics
 */
button_status_t button_driver_reset(void)
{
    if (!g_button_ctx.initialized)
    {
        return BUTTON_STATUS_NOT_INITIALIZED;
    }

    // Reset all button states
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        button_state_t *state = &g_button_ctx.button_states[i];
        state->current_state = false;
        state->previous_state = false;
        state->press_time = 0;
        state->release_time = 0;
        state->is_pressed = false;
        state->just_pressed = false;
        state->just_released = false;
    }

    // Reset statistics
    g_button_ctx.scan_count = 0;

    // Clear current data
    g_button_ctx.current_data.buttons = 0;
    g_button_ctx.current_data.flags = 0;

    LOG_INF("Button driver state reset");
    return BUTTON_STATUS_OK;
}

/**
 * @brief Get human-readable name for a button ID
 */
const char *button_driver_get_button_name(button_id_t button_id)
{
    if (button_id >= BUTTON_COUNT)
    {
        return "UNKNOWN";
    }

    return button_names[button_id];
}
