#include "i2c_arduino.h"
#include <string.h>

void wire_begin(ArduinoI2C *wire, const struct device *i2c_dev) {
    wire->i2c_dev = i2c_dev;
    wire->length = 0;
    wire->address = 0;
}

void wire_beginTransmission(ArduinoI2C *wire, uint16_t address) {
    wire->address = address;
    wire->length = 0;
}

int wire_write(ArduinoI2C *wire, uint8_t byte) {
    if (wire->length >= I2C_BUFFER_MAX) return -ENOMEM;
    wire->buffer[wire->length++] = byte;
    return 0;
}

int wire_write_buffer(ArduinoI2C *wire, const uint8_t *data, size_t len) {
    if ((wire->length + len) > I2C_BUFFER_MAX) return -ENOMEM;
    memcpy(&wire->buffer[wire->length], data, len);
    wire->length += len;
    return 0;
}

int wire_endTransmission(ArduinoI2C *wire) {
    int ret = i2c_write(wire->i2c_dev, wire->buffer, wire->length, wire->address);
    wire->length = 0;
    return ret;
}

int wire_endTransmission_stop(ArduinoI2C *wire, bool stop) {
    // In Zephyr I2C, we always do a complete transaction
    // The stop parameter doesn't affect the actual I2C transaction in our implementation
    // but we maintain compatibility with Arduino Wire library
    int ret = i2c_write(wire->i2c_dev, wire->buffer, wire->length, wire->address);
    wire->length = 0;
    return ret;
}

int wire_requestFrom(ArduinoI2C *wire, uint16_t address, size_t length) {
    if (length > I2C_BUFFER_MAX) return -ENOMEM;
    wire->address = address;
    int ret = i2c_read(wire->i2c_dev, wire->buffer, length, address);
    if (ret == 0) {
        wire->length = length;
    } else {
        wire->length = 0;
    }
    return ret;
}

int wire_available(ArduinoI2C *wire) {
    return wire->length;
}

uint8_t wire_read(ArduinoI2C *wire) {
    if (wire->length > 0) {
        uint8_t data = wire->buffer[0];
        // Shift buffer contents
        for (size_t i = 0; i < wire->length - 1; i++) {
            wire->buffer[i] = wire->buffer[i + 1];
        }
        wire->length--;
        return data;
    }
    return 0;
}
