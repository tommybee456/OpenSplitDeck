#ifndef CONTROLLER_ESB_H
#define CONTROLLER_ESB_H

#include <zephyr/kernel.h>
#include <esb.h>

// Controller data structure - must match controller side
typedef struct
{
    uint8_t flags;   // Mode bits, controller ID, mouse buttons, trigger buttons
    uint8_t trigger;  // Analog trigger (L2/R2)
    int8_t stickX;   // Analog stick X
    int8_t stickY;   // Analog stick Y
    int16_t padX;    // Trackpad X (-32767 to 32767)
    int16_t padY;    // Trackpad Y (-32767 to 32767)
    uint8_t buttons; // Digital button states
    int16_t accelX;  // Accelerometer X
    int16_t accelY;  // Accelerometer Y
    int16_t accelZ;  // Accelerometer Z
    int16_t gyroX;   // Gyroscope X
    int16_t gyroY;   // Gyroscope Y
    int16_t gyroZ;   // Gyroscope Z
} __packed controller_data_t;

// ACK payload structure for timing control + rumble (lean design - 8 bytes total)
typedef struct
{
    uint16_t next_delay_ms;      // How long controller should wait before next transmission (0-65535ms)
    uint8_t sequence_num;        // Sequence tracking for debugging/sync
    uint8_t rumble_data;         // Rumble intensity: bits 7-4=left motor, bits 3-0=right motor (0-15 each)
    uint32_t dongle_timestamp;   // Dongle's current time for sync
} __packed ack_timing_data_t;

// Simple controller state for dongle
typedef struct
{
    uint8_t flags;
    uint8_t trigger;
    int8_t stickX;
    int8_t stickY;
    int16_t padX;
    int16_t padY;
    uint8_t buttons;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    bool data_received;
    uint32_t last_ping_time;
} simple_controller_state_t;

// Function declarations
int controller_esb_init(void);
simple_controller_state_t *controller_esb_get_state(void);
bool controller_esb_has_new_data(void);

#endif // CONTROLLER_ESB_H
