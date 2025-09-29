/**
 * @file display.h
 * @brief Display library for SSD1306 OLED display
 * 
 * This library provides a clean interface for controlling the SSD1306 display
 * with simple text and graphics functions.
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Display configuration constants
 */
#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 32
#define DISPLAY_BUFFER_SIZE (DISPLAY_WIDTH * (DISPLAY_HEIGHT / 8))

/**
 * @brief Display status enumeration
 */
typedef enum {
    DISPLAY_STATUS_NOT_READY = 0,
    DISPLAY_STATUS_READY,
    DISPLAY_STATUS_ERROR
} display_status_t;

/**
 * @brief Display context structure
 */
typedef struct {
    const struct device *dev;
    display_status_t status;
    uint8_t buffer[DISPLAY_BUFFER_SIZE];
    uint32_t frame_counter;
    bool needs_update;
    uint8_t controller_id;  // 0=RIGHT, 1=LEFT
} display_context_t;

/**
 * @brief Initialize the display system
 * 
 * @param controller_id Controller ID (0=RIGHT, 1=LEFT)
 * @return 0 on success, negative error code on failure
 */
int display_library_init(uint8_t controller_id);

/**
 * @brief Get the current display status
 * 
 * @return Current display status
 */
display_status_t display_get_status(void);

/**
 * @brief Clear the display buffer
 */
void display_clear(void);

/**
 * @brief Set a pixel in the display buffer
 * 
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-31)
 * @param on true to turn pixel on, false to turn off
 */
void display_set_pixel(int16_t x, int16_t y, bool on);

/**
 * @brief Draw a simple letter at specified position
 * 
 * @param letter Character to draw ('R', 'L', etc.)
 * @param x X position for letter
 * @param y Y position for letter
 */
void display_draw_letter(char letter, int16_t x, int16_t y);

/**
 * @brief Draw controller identification (R for RIGHT, L for LEFT)
 * 
 * @param controller_id 0 for RIGHT, 1 for LEFT
 */
void display_draw_controller_id(uint8_t controller_id);

/**
 * @brief Add status indicators to the display
 * 
 * @param show_blink true to show blinking status dot
 */
void display_draw_status_indicators(bool show_blink);

/**
 * @brief Update the physical display with buffer contents
 * 
 * @return 0 on success, negative error code on failure
 */
int display_refresh(void);

/**
 * @brief Write display buffer to screen (call only when content changes)
 */
void display_refresh_screen(void);

/**
 * @brief Main display update function - call this regularly
 * 
 * This handles automatic refresh timing and frame counting
 */
void display_update(void);

/**
 * @brief Enable/disable display blanking
 * 
 * @param blank true to blank display, false to unblank
 * @return 0 on success, negative error code on failure
 */
int display_set_blanking(bool blank);

/**
 * @brief Get current frame counter
 * 
 * @return Current frame counter value
 */
uint32_t display_get_frame_counter(void);

void display_draw_hline(int16_t x, int16_t y, int16_t width);

/**
 * @brief Draw a vertical line  
 */
void display_draw_vline(int16_t x, int16_t y, int16_t height);

/**
 * @brief Draw a rectangle
 */
void display_draw_rect(int16_t x, int16_t y, int16_t width, int16_t height, bool filled);

/**
 * @brief Draw a simple number (0-9)
 */
void display_draw_number(int16_t x, int16_t y, uint8_t number);

/**
 * @brief Draw a bitmap image from a byte array
 * 
 * @param x X position for top-left corner
 * @param y Y position for top-left corner
 * @param width Width of the bitmap in pixels
 * @param height Height of the bitmap in pixels
 * @param bitmap Pointer to bitmap data (1 bit per pixel, MSB first)
 */
void display_draw_bitmap(int16_t x, int16_t y, int16_t width, int16_t height, const uint8_t* bitmap);

/**
 * @brief Draw a bitmap with transparency
 * 
 * @param x X position for top-left corner
 * @param y Y position for top-left corner  
 * @param width Width of the bitmap in pixels
 * @param height Height of the bitmap in pixels
 * @param bitmap Pointer to bitmap data (2 bits per pixel: 00=transparent, 01=off, 10/11=on)
 */
void display_draw_bitmap_transparent(int16_t x, int16_t y, int16_t width, int16_t height, const uint8_t* bitmap);

/**
 * @brief Draw a simple text string using numbers
 */
void display_draw_simple_text(int16_t x, int16_t y, const char* text);

/**
 * @brief Draw battery level indicator
 * 
 * @param x X position for battery
 * @param y Y position for battery  
 * @param voltage_mv Battery voltage in millivolts
 */
void display_draw_battery_level(int16_t x, int16_t y, uint16_t voltage_mv);

/**
 * @brief Display controller status screen
 */
void display_show_status_screen(void);

/**
 * @brief Display controller status screen with battery level
 * 
 * @param battery_mv Battery voltage in millivolts
 */
void display_show_status_screen_with_battery(uint16_t battery_mv);

/**
 * @brief Display analog values screen
 */
void display_show_analog_screen(int16_t stick_x, int16_t stick_y, uint8_t trigger);

/**
 * @brief Display menu screen
 */
void display_show_menu(uint8_t selected_item);

/**
 * @brief Display screen types
 */
typedef enum {
    DISPLAY_SCREEN_STATUS = 0,
    DISPLAY_SCREEN_ANALOG,
    DISPLAY_SCREEN_MENU,
    DISPLAY_SCREEN_CUSTOM
} display_screen_type_t;

/**
 * @brief Display data structure for analog screen
 */
typedef struct {
    int16_t stick_x;
    int16_t stick_y;
    uint8_t trigger;
} display_analog_data_t;

/**
 * @brief Set the current display screen (thread-safe)
 * 
 * @param screen_type Type of screen to display
 * @param data Optional data for the screen (can be NULL)
 */
void display_set_screen(display_screen_type_t screen_type, void* data);

/**
 * @brief Get current display screen type
 */
display_screen_type_t display_get_current_screen(void);

#endif /* DISPLAY_H */
