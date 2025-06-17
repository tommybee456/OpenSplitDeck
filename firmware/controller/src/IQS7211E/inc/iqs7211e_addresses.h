/******************************************************************************
 *                                                                            *
 *                                                                            *
 *                                Copyright by                                *
 *                                                                            *
 *                              Azoteq (Pty) Ltd                              *
 *                          Republic of South Africa                          *
 *                                                                            *
 *                           Tel: +27(0)21 863 0033                           *
 *                           E-mail: info@azoteq.com                          *
 *                                                                            *
 * ========================================================================== *
 * Refer to IQS7211E datasheet for more information, available here:          *
 * - https://www.azoteq.com/design/datasheets/                                *
 * ========================================================================== *
 *                       IQS7211E - Registers & Memory Map                    *
*******************************************************************************/
#ifndef __IQS7211E_ADDRESSES_H
#define __IQS7211E_ADDRESSES_H

/* Device Information - Read Only */

/* All Banks: 0x00 - 0x09 */
#define IQS7211E_MM_PROD_NUM                0x00
#define IQS7211E_MM_MAJOR_VERSION_NUM       0x01
#define IQS7211E_MM_MINOR_VERSION_NUM       0x02

/* GESTURE_DATA: 0x0A - 0x17 */
#define IQS7211E_MM_RELATIVE_X	            0x0A
#define IQS7211E_MM_RELATIVE_Y              0x0B
#define IQS7211E_MM_GESTURE_X               0x0C
#define IQS7211E_MM_GESTURE_Y               0x0D
#define IQS7211E_MM_GESTURES                0x0E
#define IQS7211E_MM_INFO_FLAGS              0x0F
#define IQS7211E_MM_FINGER_1_X	            0x10
#define IQS7211E_MM_FINGER_1_Y              0x11
#define IQS7211E_MM_FINGER_1_TOUCH_STRENGTH 0x12
#define IQS7211E_MM_FINGER_1_AREA           0x13
#define IQS7211E_MM_FINGER_2_X              0x14
#define IQS7211E_MM_FINGER_2_Y              0x15
#define IQS7211E_MM_FINGER_2_TOUCH_STRENGTH 0x16
#define IQS7211E_MM_FINGER_2_AREA           0x17

/* CHANNEL STATES & COUNTS: 0x18 - 0x1E */
#define IQS7211E_MM_TOUCH_STATE_0           0x18
#define IQS7211E_MM_TOUCH_STATE_1           0x19
#define IQS7211E_MM_TOUCH_STATE_2           0x1A
#define IQS7211E_MM_ALP_CHANNEL_COUNT       0x1B
#define IQS7211E_MM_ALP_CHANNEL_LTA         0x1C
#define IQS7211E_MM_ALP_CHANNEL_COUNT_A     0x1D
#define IQS7211E_MM_ALP_CHANNEL_COUNT_B     0x1E


/* ALP & TP ATI SETTINGS: 0x1F - 0x27 */
#define IQS7211E_MM_ALP_ATI_COMP_A         	0x1F
#define IQS7211E_MM_ALP_ATI_COMP_B         	0x20
#define IQS7211E_MM_TP_GLOBAL_MIRRORS       0x21
#define IQS7211E_MM_TP_REF_DRIFT          	0x22
#define IQS7211E_MM_TP_TARGET              	0x23
#define IQS7211E_MM_TP_REATI_COUNTS        	0x24
#define IQS7211E_MM_ALP_MIRRORS           	0x25
#define IQS7211E_MM_ALP_REF_DRIFT         	0x26
#define IQS7211E_MM_ALP_TARGET             	0x27


/* REPORT RATES AND TIMINGS: 0x28 - 0x32 */
#define IQS7211E_MM_ACTIVE_MODE_RR         	0x28
#define IQS7211E_MM_IDLE_TOUCH_MODE_RR      0x29
#define IQS7211E_MM_IDLE_MODE_RR            0x2A
#define IQS7211E_MM_LP1_MODE_RR         	  0x2B
#define IQS7211E_MM_LP2_MODE_RR             0x2C
#define IQS7211E_MM_ACTIVE_MODE_TIMEOUT     0x2D
#define IQS7211E_MM_IDLE_TOUCH_MODE_TIMEOUT 0x2E
#define IQS7211E_MM_IDLE_MODE_TIMEOUT       0x2F
#define IQS7211E_MM_LP1_MODE_TIMEOUT        0x30
#define IQS7211E_MM_REF_UPDATE_REATI_TIME   0x31
#define IQS7211E_MM_I2C_TIMEOUT             0x32

/* SYSTEM AND ALP SETUP SETTINGS: 0x33 - 0x37 */
#define IQS7211E_MM_SYS_CONTROL            	0x33
#define IQS7211E_MM_CONFIG_SETTINGS         0x34
#define IQS7211E_MM_OTHER_SETTINGS          0x35
#define IQS7211E_MM_ALP_SETUP            	  0x36
#define IQS7211E_MM_ALP_TX_ENABLE           0x37

/* TRACKPAD AND ALP THRESHOLDS: 0x38 - 0x3A */
#define IQS7211E_MM_TP_TOUCH_SET_CLEAR_THR  0x38
#define IQS7211E_MM_ALP_THRESHOLD           0x39
#define IQS7211E_MM_ALP_SET_CLEAR_DEBOUNCE  0x3A

/* ALP CHANNEL SETUP: 0x3B - 0x3C */
#define IQS7211E_MM_LP1_FILTERS         	  0x3B
#define IQS7211E_MM_LP2_FILTERS             0x3C

/* CHANNEL SETUP: 0x3D - 0x40 */

#define IQS7211E_MM_TP_CONV_FREQ            0x3D
#define IQS7211E_MM_ALP_CONV_FREQ           0x3E
#define IQS7211E_MM_TP_HARDWARE             0x3F
#define IQS7211E_MM_ALP_HARDWARE            0x40

/* TP SETUP: 0x41 - 0x49 */
#define IQS7211E_MM_TP_RX_SETTINGS         	0x41
#define IQS7211E_MM_MAX_TOUCHES_TX          0x41
#define IQS7211E_MM_X_RESOLUTION            0x43
#define IQS7211E_MM_Y_RESOLUTION         	  0x44
#define IQS7211E_MM_XY_FILTER_BOTTOM_SPEED  0x45
#define IQS7211E_MM_XY_FILTER_TOPSPEED      0x46
#define IQS7211E_MM_STATIC_FILTER           0x47
#define IQS7211E_MM_FINGER_SPLIT_MOVEMENT	  0x48
#define IQS7211E_MM_TRIM_VALUES             0x49

/*SETTINGS VERSIONS: 0x4A */

#define IQS7211E_MM_SETTINGS_VERSION        0x4A

/* GESTURE SETTINGS: 0x4B - 0x55 */
#define IQS7211E_MM_GESTURE_ENABLE       	  0x4B
#define IQS7211E_MM_TAP_TIME                0x4C
#define IQS7211E_MM_AIR_TIME                0x4D
#define IQS7211E_MM_TAP_DISTANCE            0x4E
#define IQS7211E_MM_HOLD_TIME               0x4F
#define IQS7211E_MM_SWIPE_TIME              0x50
#define IQS7211E_MM_X_INITIAL_DISTANCE      0x51
#define IQS7211E_MM_Y_INITIAL_DISTANCE      0x52
#define IQS7211E_MM_X_CONSECUTIVE_DISTANCE  0x53
#define IQS7211E_MM_Y_CONSECUTIVE_DISTANCE  0x54
#define IQS7211E_MM_THRESHOLD_ANGLE         0x55

/* GESTURE SETTINGS: 0x56 - 0x5C */
#define IQS7211E_MM_RX_TX_MAPPING_0_1       0x56
#define IQS7211E_MM_RX_TX_MAPPING_2_3       0x57
#define IQS7211E_MM_RX_TX_MAPPING_4_5	      0x58
#define IQS7211E_MM_RX_TX_MAPPING_6_7       0x59
#define IQS7211E_MM_RX_TX_MAPPING_8_9       0x5A
#define IQS7211E_MM_RX_TX_MAPPING_10_11     0x5B
#define IQS7211E_MM_RX_TX_MAPPING_12        0x5C

/* CYCLE SETTINGS: 0x5D - 0x6B */
#define IQS7211E_MM_PROXA_CYCLE0            0x5D
#define IQS7211E_MM_PROXB_CYCLE0            0x5E
#define IQS7211E_MM_CYCLE1  	              0x5F
#define IQS7211E_MM_PROXA_CYCLE2            0x60
#define IQS7211E_MM_PROXB_CYCLE2            0x61
#define IQS7211E_MM_CYCLE3                  0x62
#define IQS7211E_MM_PROXA_CYCLE4            0x63
#define IQS7211E_MM_PROXB_CYCLE4            0x64
#define IQS7211E_MM_CYCLE5                  0x65
#define IQS7211E_MM_PROXA_CYCLE6            0x66
#define IQS7211E_MM_PROXB_CYCLE6            0x67
#define IQS7211E_MM_CYCLE7	                0x68
#define IQS7211E_MM_PROXA_CYCLE8            0x69
#define IQS7211E_MM_PROXB_CYCLE8            0x6A
#define IQS7211E_MM_CYCLE9                  0x6B

/* CYCLE SETTINGS2: 0x6C - 0x7C */
#define IQS7211E_MM_PROXA_CYCLE10            0x6C
#define IQS7211E_MM_PROXB_CYCLE10            0x6D
#define IQS7211E_MM_CYCLE11  	               0x6E
#define IQS7211E_MM_PROXA_CYCLE12            0x6F
#define IQS7211E_MM_PROXB_CYCLE12            0x70
#define IQS7211E_MM_CYCLE13                  0x71
#define IQS7211E_MM_PROXA_CYCLE14            0x72
#define IQS7211E_MM_PROXB_CYCLE14            0x73
#define IQS7211E_MM_CYCLE15                  0x74
#define IQS7211E_MM_PROXA_CYCLE16            0x75
#define IQS7211E_MM_PROXB_CYCLE16            0x76
#define IQS7211E_MM_CYCLE17	                 0x77
#define IQS7211E_MM_PROXA_CYCLE18            0x78
#define IQS7211E_MM_PROXB_CYCLE18            0x79
#define IQS7211E_MM_CYCLE19                  0x7A
#define IQS7211E_MM_PROXA_CYCLE20            0x7B
#define IQS7211E_MM_PROXb_CYCLE20            0x7C

#endif /* __IQS7211E_ADDRESSES_H */