/*
* This file contains all the necessary settings for the IQS7211E and this file can
* be changed from the GUI or edited here
* File:   IQS7211E_init.h
* Author: Azoteq
*/ 

#ifndef IQS7211E_INIT_H
#define IQS7211E_INIT_H

/* Change the ALP ATI Compensation */
/* Memory Map Position 0x1F - 0x20 */
#define ALP_COMPENSATION_A_0                     0x9C
#define ALP_COMPENSATION_A_1                     0x00
#define ALP_COMPENSATION_B_0                     0xA4
#define ALP_COMPENSATION_B_1                     0x00

/* Change the ATI Settings */
/* Memory Map Position 0x21 - 0x27 */
#define TP_ATI_MULTIPLIERS_DIVIDERS_0            0xE1
#define TP_ATI_MULTIPLIERS_DIVIDERS_1            0x0D
#define TP_COMPENSATION_DIV                      0x08
#define TP_REF_DRIFT_LIMIT                       0x32
#define TP_ATI_TARGET_0                          0x2C
#define TP_ATI_TARGET_1                          0x01
#define TP_MIN_COUNT_REATI_0                     0x32
#define TP_MIN_COUNT_REATI_1                     0x00
#define ALP_ATI_MULTIPLIERS_DIVIDERS_0           0x23
#define ALP_ATI_MULTIPLIERS_DIVIDERS_1           0x02
#define ALP_COMPENSATION_DIV                     0x02
#define ALP_LTA_DRIFT_LIMIT                      0x14
#define ALP_ATI_TARGET_0                         0x00
#define ALP_ATI_TARGET_1                         0x00

/* Change the Report Rates and Timing */
/* Memory Map Position 0x28 - 0x32 */
#define ACTIVE_MODE_REPORT_RATE_0                0x0A
#define ACTIVE_MODE_REPORT_RATE_1                0x00
#define IDLE_TOUCH_MODE_REPORT_RATE_0            0x32
#define IDLE_TOUCH_MODE_REPORT_RATE_1            0x00
#define IDLE_MODE_REPORT_RATE_0                  0x14
#define IDLE_MODE_REPORT_RATE_1                  0x00
#define LP1_MODE_REPORT_RATE_0                   0x50
#define LP1_MODE_REPORT_RATE_1                   0x00
#define LP2_MODE_REPORT_RATE_0                   0xA0
#define LP2_MODE_REPORT_RATE_1                   0x00
#define ACTIVE_MODE_TIMEOUT_0                    0x0A
#define ACTIVE_MODE_TIMEOUT_1                    0x00
#define IDLE_TOUCH_MODE_TIMEOUT_0                0x3C
#define IDLE_TOUCH_MODE_TIMEOUT_1                0x00
#define IDLE_MODE_TIMEOUT_0                      0x0A
#define IDLE_MODE_TIMEOUT_1                      0x00
#define LP1_MODE_TIMEOUT_0                       0x0A
#define LP1_MODE_TIMEOUT_1                       0x00
#define REATI_RETRY_TIME                         0x05
#define REF_UPDATE_TIME                          0x08
#define I2C_TIMEOUT_0                            0x64
#define I2C_TIMEOUT_1                            0x00

/* Change the System Settings */
/* Memory Map Position 0x33 - 0x35 */
#define SYSTEM_CONTROL_0                         0x00
#define SYSTEM_CONTROL_1                         0x00
#define CONFIG_SETTINGS0                         0xAC
#define CONFIG_SETTINGS1                         0x06
#define OTHER_SETTINGS_0                         0x00
#define OTHER_SETTINGS_1                         0x00

/* Change the ALP Settings */
/* Memory Map Position 0x36 - 0x37 */
#define ALP_SETUP_0                              0x77
#define ALP_SETUP_1                              0x03
#define ALP_TX_ENABLE_0                          0x08
#define ALP_TX_ENABLE_1                          0x05

/* Change the Thresholds and Debounce Settings */
/* Memory Map Position 0x38 - 0x3A */
#define TRACKPAD_TOUCH_SET_THRESHOLD             0x32
#define TRACKPAD_TOUCH_CLEAR_THRESHOLD           0x14
#define ALP_THRESHOLD_0                          0x08
#define ALP_THRESHOLD_1                          0x00
#define ALP_SET_DEBOUNCE                         0x04
#define ALP_CLEAR_DEBOUNCE                       0x04

/* Change the Button and ALP count and LTA betas */
/* Memory Map Position 0x3B - 0x3C */
#define ALP_COUNT_BETA_LP1                       0xDC
#define ALP_LTA_BETA_LP1                         0x08
#define ALP_COUNT_BETA_LP2                       0xF0
#define ALP_LTA_BETA_LP2                         0x10

/* Change the Hardware Settings */
/* Memory Map Position 0x3D - 0x40 */
#define TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH   0x02
#define TP_CONVERSION_FREQUENCY_FRACTION_VALUE   0x1A
#define ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH  0x02
#define ALP_CONVERSION_FREQUENCY_FRACTION_VALUE  0x1A
#define TRACKPAD_HARDWARE_SETTINGS_0             0x03
#define TRACKPAD_HARDWARE_SETTINGS_1             0x8C
#define ALP_HARDWARE_SETTINGS_0                  0x67
#define ALP_HARDWARE_SETTINGS_1                  0x9C

/* Change the Trackpad Settings */
/* Memory Map Position 0x41 - 0x49 */
#define TRACKPAD_SETTINGS_0_0                    0x29
#define TRACKPAD_SETTINGS_0_1                    0x06
#define TRACKPAD_SETTINGS_1_0                    0x05
#define TRACKPAD_SETTINGS_1_1                    0x02
#define X_RESOLUTION_0                           0xE8
#define X_RESOLUTION_1                           0x03
#define Y_RESOLUTION_0                           0xE8
#define Y_RESOLUTION_1                           0x03
#define XY_DYNAMIC_FILTER_BOTTOM_SPEED_0         0x06
#define XY_DYNAMIC_FILTER_BOTTOM_SPEED_1         0x00
#define XY_DYNAMIC_FILTER_TOP_SPEED_0            0x7C
#define XY_DYNAMIC_FILTER_TOP_SPEED_1            0x00
#define XY_DYNAMIC_FILTER_BOTTOM_BETA            0x07
#define XY_DYNAMIC_FILTER_STATIC_FILTER_BETA     0x80
#define STATIONARY_TOUCH_MOV_THRESHOLD           0x14
#define FINGER_SPLIT_FACTOR                      0x03
#define X_TRIM_VALUE                             0x14
#define Y_TRIM_VALUE                             0x14

/* Change the Settings Version Numbers */
/* Memory Map Position 0x4A - 0x4A */
#define MINOR_VERSION                            0x00
#define MAJOR_VERSION                            0x00

/* Change the Gesture Settings */
/* Memory Map Position 0x4B - 0x55 */
#define GESTURE_ENABLE_0                         0x1F
#define GESTURE_ENABLE_1                         0xFF
#define TAP_TOUCH_TIME_0                         0x96
#define TAP_TOUCH_TIME_1                         0x00
#define TAP_WAIT_TIME_0                          0x96
#define TAP_WAIT_TIME_1                          0x00
#define TAP_DISTANCE_0                           0x32
#define TAP_DISTANCE_1                           0x00
#define HOLD_TIME_0                              0x2C
#define HOLD_TIME_1                              0x01
#define SWIPE_TIME_0                             0x96
#define SWIPE_TIME_1                             0x00
#define SWIPE_X_DISTANCE_0                       0xC8
#define SWIPE_X_DISTANCE_1                       0x00
#define SWIPE_Y_DISTANCE_0                       0xC8
#define SWIPE_Y_DISTANCE_1                       0x00
#define SWIPE_X_CONS_DIST_0                      0x64
#define SWIPE_X_CONS_DIST_1                      0x00
#define SWIPE_Y_CONS_DIST_0                      0x64
#define SWIPE_Y_CONS_DIST_1                      0x00
#define SWIPE_ANGLE                              0x17
#define PALM_THRESHOLD                           0x1E

/* Change the RxTx Mapping */
/* Memory Map Position 0x56 - 0x5C */
#define RX_TX_MAP_0                              0x03
#define RX_TX_MAP_1                              0x04
#define RX_TX_MAP_2                              0x05
#define RX_TX_MAP_3                              0x00
#define RX_TX_MAP_4                              0x02
#define RX_TX_MAP_5                              0x01
#define RX_TX_MAP_6                              0x06
#define RX_TX_MAP_7                              0x07
#define RX_TX_MAP_8                              0x08
#define RX_TX_MAP_9                              0x09
#define RX_TX_MAP_10                             0x0A
#define RX_TX_MAP_11                             0x0B
#define RX_TX_MAP_12                             0x0C
#define RX_TX_MAP_FILLER                         0x00

/* Change the Allocation of channels into cycles 0-9 */
/* Memory Map Position 0x5D - 0x6B */
#define PLACEHOLDER_0                            0x05
#define CH_1_CYCLE_0                             0x00
#define CH_2_CYCLE_0                             0x01
#define PLACEHOLDER_1                            0x05
#define CH_1_CYCLE_1                             0x06
#define CH_2_CYCLE_1                             0x07
#define PLACEHOLDER_2                            0x05
#define CH_1_CYCLE_2                             0x0C
#define CH_2_CYCLE_2                             0x0D
#define PLACEHOLDER_3                            0x05
#define CH_1_CYCLE_3                             0x12
#define CH_2_CYCLE_3                             0x13
#define PLACEHOLDER_4                            0x05
#define CH_1_CYCLE_4                             0x18
#define CH_2_CYCLE_4                             0x19
#define PLACEHOLDER_5                            0x05
#define CH_1_CYCLE_5                             0x03
#define CH_2_CYCLE_5                             0x02
#define PLACEHOLDER_6                            0x05
#define CH_1_CYCLE_6                             0x09
#define CH_2_CYCLE_6                             0x08
#define PLACEHOLDER_7                            0x05
#define CH_1_CYCLE_7                             0x0F
#define CH_2_CYCLE_7                             0x0E
#define PLACEHOLDER_8                            0x05
#define CH_1_CYCLE_8                             0x15
#define CH_2_CYCLE_8                             0x14
#define PLACEHOLDER_9                            0x05
#define CH_1_CYCLE_9                             0x1B
#define CH_2_CYCLE_9                             0x1A

/* Change the Allocation of channels into cycles 10-19 */
/* Memory Map Position 0x6C - 0x7A */
#define PLACEHOLDER_10                           0x05
#define CH_1_CYCLE_10                            0x04
#define CH_2_CYCLE_10                            0xFF
#define PLACEHOLDER_11                           0x05
#define CH_1_CYCLE_11                            0x0A
#define CH_2_CYCLE_11                            0xFF
#define PLACEHOLDER_12                           0x05
#define CH_1_CYCLE_12                            0x10
#define CH_2_CYCLE_12                            0xFF
#define PLACEHOLDER_13                           0x05
#define CH_1_CYCLE_13                            0x16
#define CH_2_CYCLE_13                            0xFF
#define PLACEHOLDER_14                           0x05
#define CH_1_CYCLE_14                            0x1C
#define CH_2_CYCLE_14                            0xFF
#define PLACEHOLDER_15                           0x05
#define CH_1_CYCLE_15                            0x05
#define CH_2_CYCLE_15                            0xFF
#define PLACEHOLDER_16                           0x05
#define CH_1_CYCLE_16                            0x0B
#define CH_2_CYCLE_16                            0xFF
#define PLACEHOLDER_17                           0x05
#define CH_1_CYCLE_17                            0x11
#define CH_2_CYCLE_17                            0xFF
#define PLACEHOLDER_18                           0x05
#define CH_1_CYCLE_18                            0x17
#define CH_2_CYCLE_18                            0xFF
#define PLACEHOLDER_19                           0x05
#define CH_1_CYCLE_19                            0x1D
#define CH_2_CYCLE_19                            0xFE

/* Change the Allocation of channels into cycles 20 */
/* Memory Map Position 0x7B - 0x7C */
#define PLACEHOLDER_20                           0x05
#define CH_1_CYCLE_20                            0xFF
#define CH_2_CYCLE_20                            0xFF

#endif	/* IQS7211E_INIT_H */