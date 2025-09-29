cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(controller)

target_sources(app PRIVATE
    src/main.c
    src/IQS7211E.c
    src/i2c_arduino.c
    src/controller_storage.c
    src/drv2605.c
    src/display.c
    src/haptic_driver.c
    src/imu_driver.c
    src/button_driver.c
    src/analog_driver.c
    src/esb_comm_driver.c
    src/power_mgmt_driver.c
    # src/trackpad_driver.c  # Temporarily disabled while fixing IQS7211E
)
