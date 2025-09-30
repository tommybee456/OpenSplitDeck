#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "controller_esb.h"
#include "usb_hid_composite.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

bool left_trackpad_pressed = false;
bool right_trackpad_pressed = false;

bool left_mode_pressed = false;
bool right_mode_pressed = false;

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
// Helper function to map a value from one range to another
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Convert controller data to HID reports (using separated controller states)
static void process_controller_data(const struct device *hid_dev)
{
    uint32_t func_start = k_uptime_get_32();
    
    // Get SEPARATE controller states - no more shared state corruption!
    simple_controller_state_t *left_controller = controller_esb_get_left_state();
    simple_controller_state_t *right_controller = controller_esb_get_right_state();

    // Static variables to hold complete state
    static uint8_t dpad = 8;
    static uint8_t buttons1 = 0, buttons2 = 0;
    static uint8_t left_x = 128, left_y = 128;
    static uint8_t right_x = 128, right_y = 128;
    static uint8_t left_trigger = 0, right_trigger = 0;
    static bool touch1_active = false;
    static bool touch2_active = false;
    static uint16_t touch1_x = 0, touch1_y = 0;
    static uint16_t touch2_x = 0, touch2_y = 0;
    static int16_t accel_x = 0, accel_y = 0, accel_z = 0;
    static int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;

    static uint16_t raw_touch2_x = 0;
    static uint16_t raw_touch2_y = 0;

    static uint16_t last_touch2_x = 0, last_touch2_y = 0;
    static bool last_touch2_active = false;
    static float interp_factor = 0.7f; // Adjust for smoothness

    static uint16_t raw_touch_x = 0;
    static uint16_t raw_touch_y = 0;

    // Static variables for interpolation
    static uint16_t last_touch_x = 0, last_touch_y = 0;
    static bool last_touch_active = false;

    // Process LEFT controller data independently
    if (left_controller->data_received)
    {
        // Left controller data
        left_x = (uint8_t)(left_controller->stickX + 128);
        left_y = (uint8_t)(left_controller->stickY + 128);
        left_trigger = left_controller->trigger;

        // Touchpad from left controller
        touch2_active = (left_controller->padX != 0 || left_controller->padY != 0);

        // Raw trackpad values after mapping
        raw_touch2_x = touch2_active ? (map(left_controller->padY, 0, 1023, 959, 0)) : 0;
        raw_touch2_y = touch2_active ? (map(left_controller->padX, 0, 1023, 0, 942)) : 0;

        // Left controller Dpad processing
        if ((left_controller->buttons & 0x08) && !(left_controller->buttons & 0x04) && !(left_controller->buttons & 0x02) && !(left_controller->buttons & 0x01)) // just down
        {
            dpad = 4; // Down
        }
        if (!(left_controller->buttons & 0x08) && (left_controller->buttons & 0x04) && !(left_controller->buttons & 0x02) && !(left_controller->buttons & 0x01)) // just right
        {
            dpad = 2; // Right
        }
        if (!(left_controller->buttons & 0x08) && !(left_controller->buttons & 0x04) && (left_controller->buttons & 0x02) && !(left_controller->buttons & 0x01)) // just left
        {
            dpad = 6; // Left
        }
        if (!(left_controller->buttons & 0x08) && !(left_controller->buttons & 0x04) && !(left_controller->buttons & 0x02) && (left_controller->buttons & 0x01)) // just up
        {
            dpad = 0; // Up
        }
        if ((left_controller->buttons & 0x08) && (left_controller->buttons & 0x04) && !(left_controller->buttons & 0x02) && !(left_controller->buttons & 0x01)) // down right
        {
            dpad = 3; // Down Right
        }
        if (!(left_controller->buttons & 0x08) && (left_controller->buttons & 0x04) && !(left_controller->buttons & 0x02) && (left_controller->buttons & 0x01)) // right up
        {
            dpad = 1; // Right Up
        }
        if ((left_controller->buttons & 0x08) && !(left_controller->buttons & 0x04) && (left_controller->buttons & 0x02) && !(left_controller->buttons & 0x01)) // down left
        {
            dpad = 5; // Left Down
        }
        if (!(left_controller->buttons & 0x08) && !(left_controller->buttons & 0x04) && (left_controller->buttons & 0x02) && (left_controller->buttons & 0x01)) // left up
        {
            dpad = 7; // Up Left
        }
        if (!(left_controller->buttons & 0x08) && !(left_controller->buttons & 0x04) && !(left_controller->buttons & 0x02) && !(left_controller->buttons & 0x01)) // neutral
        {
            dpad = 8; // neutral
        }

        // Left controller buttons
        if (left_controller->buttons & 0x10 || left_controller->flags & 0x01) // 0x01 is P5 just hard coding it for now
        {
            buttons1 |= (1 << 4); // Bumper
        }
        else
        {
            buttons1 &= ~(1 << 4); // Bumper
        }

        if (left_controller->buttons & 0x40)
        {
            buttons2 |= (1 << 5); // Trackpad Click
            left_trackpad_pressed = true;
        }
        else
        {
            left_trackpad_pressed = false;
        }

        if (left_controller->buttons & 0x20 || left_controller->flags & 0x02) // 0x02 is P4 just hard coding it for now
        {
            buttons2 |= (1 << 2); // Stick Click
        }
        else
        {
            buttons2 &= ~(1 << 2); // Stick Click
        }

        if (left_controller->buttons & 0x80)
        {
            buttons2 |= (1 << 0); // Select
        }
        else
        {
            buttons2 &= ~(1 << 0); // Select
        }

        if (left_controller->flags & 0x40)
        {
            buttons2 |= (1 << 4); // PS button
            left_mode_pressed = true;
        }
        else
        {
            left_mode_pressed = false;
        }
    }

    // Process RIGHT controller data independently
    if (right_controller->data_received)
    {
        // Right controller data
        right_x = (uint8_t)(right_controller->stickX + 128);
        right_y = (uint8_t)(right_controller->stickY + 128);
        right_trigger = right_controller->trigger;

        // IMU from right controller only
        accel_x = right_controller->accelX;
        accel_y = right_controller->accelY;
        accel_z = right_controller->accelZ;
        gyro_x = right_controller->gyroX;
        gyro_y = right_controller->gyroY;
        gyro_z = right_controller->gyroZ;

        gyro_y = -gyro_y;
        accel_y = -accel_y;

        // Touchpad from right controller
        touch1_active = (right_controller->padX != 0 || right_controller->padY != 0);
        
        // Raw trackpad values after mapping
        raw_touch_x = touch1_active ? (map(right_controller->padY, 0, 1023, 0, 959) + 959) : 0;
        raw_touch_y = touch1_active ? (map(right_controller->padX, 0, 1023, 942, 0)) : 0;

        // Right controller buttons
        if (right_controller->buttons & 0x01)
        {
            buttons1 |= (1 << 3); // Y
        }
        else
        {
            buttons1 &= ~(1 << 3); // Y
        }

        if (right_controller->buttons & 0x02)
        {
            buttons1 |= (1 << 0); // X
        }
        else
        {
            buttons1 &= ~(1 << 0); // X
        }

        if (right_controller->buttons & 0x04)
        {
            buttons1 |= (1 << 2); // B
        }
        else
        {
            buttons1 &= ~(1 << 2); // B
        }

        if (right_controller->buttons & 0x08 || right_controller->flags & 0x01) // 0x01 is P5 just hard coding it for now
        {
            buttons1 |= (1 << 1); // A
        }
        else
        {
            buttons1 &= ~(1 << 1); // A
        }

        if (right_controller->buttons & 0x10)
        {
            buttons1 |= (1 << 5); // Bumper
        }
        else
        {
            buttons1 &= ~(1 << 5); // Bumper
        }

        if (right_controller->buttons & 0x20 || right_controller->flags & 0x02) // 0x02 is P4 just hard coding it for now
        {
            buttons2 |= (1 << 3); // Stick Click
        }
        else
        {
            buttons2 &= ~(1 << 3); // Stick Click
        }

        if (right_controller->buttons & 0x40)
        {
            buttons2 |= (1 << 5); // Trackpad Click
            right_trackpad_pressed = true;
        }
        else
        {
            right_trackpad_pressed = false;
        }

        if (right_controller->buttons & 0x80)
        {
            buttons2 |= (1 << 1); // Start
        }
        else
        {
            buttons2 &= ~(1 << 1); // Start
        }

        if (right_controller->flags & 0x40)
        {
            buttons2 |= (1 << 4); // Guide button
            buttons1 |= (1 << 1); // A
            right_mode_pressed = true;
        }
        else
        {
            right_mode_pressed = false;
        }
    }

    // Handle trackpad and mode button combinations
    if (!left_trackpad_pressed && !right_trackpad_pressed)
    {
        buttons2 &= ~(1 << 5);
    }

    if (!left_mode_pressed && !right_mode_pressed)
    {
        buttons2 &= ~(1 << 4); // Clear PS/Guide button
    }

    if (touch1_active && last_touch_active)
    {
        // Interpolate between last and current position
        touch1_x = (uint16_t)(last_touch_x * interp_factor + raw_touch_x * (1.0f - interp_factor));
        touch1_y = (uint16_t)(last_touch_y * interp_factor + raw_touch_y * (1.0f - interp_factor));
    }
    else
    {
        // First touch or touch just started
        touch1_x = raw_touch_x;
        touch1_y = raw_touch_y;
    }

    // Store for next frame
    last_touch_x = touch1_x;
    last_touch_y = touch1_y;
    last_touch_active = touch1_active;

    if (touch2_active && last_touch2_active)
    {
        // Interpolate between last and current position
        touch2_x = (uint16_t)(last_touch2_x * interp_factor + raw_touch2_x * (1.0f - interp_factor));
        touch2_y = (uint16_t)(last_touch2_y * interp_factor + raw_touch2_y * (1.0f - interp_factor));
    }
    else
    {
        // First touch or touch just started
        touch2_x = raw_touch2_x;
        touch2_y = raw_touch2_y;
    }

    // Store for next frame
    last_touch2_x = touch2_x;
    last_touch2_y = touch2_y;
    last_touch2_active = touch2_active;
    // With interp(end)

    // Send the report (same as before)
    usb_hid_send_ds4_report_with_touchpad_and_imu(hid_dev, dpad, buttons1, buttons2,
                                                  left_x, left_y, right_x, right_y,
                                                  left_trigger, right_trigger,
                                                  touch1_active, touch1_x, touch1_y,
                                                  touch2_active, touch2_x, touch2_y,
                                                  accel_x, accel_y, accel_z,
                                                  gyro_x, gyro_y, gyro_z);
                                                  
    // Log function timing if it's slow
    uint32_t func_time = k_uptime_get_32() - func_start;
    if (func_time > 5) { // Only warn if function takes over 5ms (was 1ms)
        LOG_WRN("Slow process_controller_data: %dms", func_time);
    }
}

int main(void)
{
    const struct device *hid_dev;
    int ret;

    if (!gpio_is_ready_dt(&led0))
    {
        // LOG_ERR("LED device %s is not ready", led0.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
    if (ret < 0)
    {
        // LOG_ERR("Failed to configure the LED pin, error: %d", ret);
        return 0;
    }

    // Initialize USB HID composite device
    ret = usb_hid_composite_init();
    if (ret != 0)
    {
        // LOG_ERR("Failed to initialize USB HID");
        return 0;
    }

    // Get the HID device handle
    hid_dev = usb_hid_composite_get_device();
    if (hid_dev == NULL)
    {
        // LOG_ERR("Failed to get USB HID device");
        return 0;
    }

    // Initialize ESB
    ret = controller_esb_init();
    if (ret != 0)
    {
        // LOG_ERR("Failed to initialize ESB");
        return 0;
    }

    // LOG_INF("Waiting for radio to fully initialize...");
    k_sleep(K_MSEC(500));
    // LOG_INF("Starting ESB ping loop");

    // Main loop - poll controllers and process responses
    uint32_t heartbeat_timer = 0;
    uint32_t last_report_time = 0;
    uint32_t last_ping_time = 0;

    while (true)
    {
        uint32_t now = k_uptime_get_32();
        uint32_t loop_start = now;

        if (now - last_report_time >= 4)
        { // 250Hz (4ms)
            last_report_time = now;
        //     uint32_t process_start = k_uptime_get_32();
            process_controller_data(hid_dev);
        //     uint32_t process_end = k_uptime_get_32();
            
            // Log if processing takes too long
            // uint32_t process_time = process_end - process_start;
            // if (process_time > 5) { // Only warn if processing takes over 5ms (was 2ms)
            //     LOG_WRN("Long processing time: %dms", process_time);
            // }
        }

        // Log if entire loop iteration takes too long
        uint32_t loop_end = k_uptime_get_32();
        uint32_t loop_time = loop_end - loop_start;
        if (loop_time > 10) { // Only warn if loop takes over 10ms (was 3ms)
            LOG_WRN("Long loop time: %dms", loop_time);
        }

        // Small delay to prevent overwhelming the system
        k_sleep(K_USEC(250));
    }
    return 0;
}
