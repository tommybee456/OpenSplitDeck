#ifndef I2C_ARDUINO_H
#define I2C_ARDUINO_H

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>
#define I2C_BUFFER_MAX 32  // adjust if needed

typedef struct {
    const struct device *i2c_dev;
    uint16_t address;
    uint8_t buffer[I2C_BUFFER_MAX];
    size_t length;
} ArduinoI2C;

void wire_begin(ArduinoI2C *wire, const struct device *i2c_dev);
void wire_beginTransmission(ArduinoI2C *wire, uint16_t address);
int  wire_write(ArduinoI2C *wire, uint8_t byte);
int  wire_write_buffer(ArduinoI2C *wire, const uint8_t *data, size_t len);
int  wire_endTransmission(ArduinoI2C *wire);
int  wire_endTransmission_stop(ArduinoI2C *wire, bool stop);
int  wire_requestFrom(ArduinoI2C *wire, uint16_t address, size_t length);
int  wire_available(ArduinoI2C *wire);
uint8_t wire_read(ArduinoI2C *wire);

#endif
