/**
 ******************************************************************************
 * @file    IQS7211E.h
 * @brief   This file contains the header information for an IQS7211E Arduino 
 *          library.
 *          The goal of the library is to provide easy functionality for
 *          initializing and using the Azoteq IQS7211E capacitive touch device.
 * @author  JN. Lochner - Azoteq PTY Ltd
 * @version V1.1
 * @date    2023
 ******************************************************************************
 ******************************************************************************
 * @attention  Makes use of the following standard Arduino libraries:
 *       - Arduino.h   -> included in IQS7211E.h, comes standard with Arduino
 *       - Wire.h      -> Included in IQS7211E.h, comes standard with Arduino
 *****************************************************************************/

#ifndef IQS7211E_h
#define IQS7211E_h

/* Include Files */
#include "Arduino.h"
#include "Wire.h"
#include "./inc/iqs7211e_addresses.h"

/* Device Firmware version select */
#define IQS7211E_v1_0 1

/* Public Global Definitions */
/* For use with Wire.h library. True argument with some functions closes the 
I2C communication window. */
#define STOP true     
/* For use with Wire.h library. False argument with some functions keeps the 
I2C communication window open. */
#define RESTART false 

/* Device Info */
#define IQS7211E_PRODUCT_NUM                            0x0458

// Info Flags Byte Bits.
#define IQS7211E_CHARGING_MODE_BIT_0                    0
#define IQS7211E_CHARGING_MODE_BIT_1                    1
#define IQS7211E_CHARGING_MODE_BIT_2                    2
#define IQS7211E_ACTIVE_BITS     	                0b000
#define IQS7211E_IDLE_TOUCH_BITS	                0b001
#define IQS7211E_IDLE_BITS		                0b010
#define IQS7211E_LP1_BITS		                0b011
#define IQS7211E_LP2_BITS		                0b100
#define IQS7211E_ATI_ERROR_BIT	                        3
#define IQS7211E_RE_ATI_OCCURRED_BIT	                4
#define IQS7211E_ALP_ATI_ERROR_BIT	                5
#define IQS7211E_ALP_RE_ATI_OCCURRED_BIT	        4
#define IQS7211E_SHOW_RESET_BIT	                        7
#define IQS7211E_NUM_FINGERS_BIT_0                      0       // 8
#define IQS7211E_NUM_FINGERS_BIT_1                      1       // 9
#define IQS7211E_NO_FINGERS_BITS     	                0b00
#define IQS7211E_1_FINGER_ACTIVE_BITS	                0b01
#define IQS7211E_2_FINGER_ACTIVE_BITS		        0b10
#define IQS7211E_TP_MOVEMENT_BIT	                2       // 10
#define IQS7211E_TOO_MANY_FINGERS_BIT	                4       // 12
#define IQS7211E_ALP_OUTPUT_BIT	                        6       // 14

// System Control Bits
#define IQS7211E_MODE_SELECT_BIT_0	                0
#define IQS7211E_MODE_SELECT_BIT_1                      1
#define IQS7211E_MODE_SELECT_BIT_2                      2
#define IQS7211E_TP_RESEED_BIT		                3
#define IQS7211E_ALP_RESEED_BIT		                4
#define IQS7211E_TP_RE_ATI_BIT		                5
#define IQS7211E_ALP_RE_ATI_BIT		                6
#define IQS7211E_ACK_RESET_BIT		                7
#define IQS7211E_SW_RESET_BIT		                1       // 9
#define IQS7211E_SUSPEND_BIT                            3       // 11

// Config Settings Bits
#define IQS7211E_EVENT_MODE_BIT	                        0       // 8

// Gesture Bits
#define IQS7211E_GESTURE_SINGLE_TAP_BIT                 0
#define IQS7211E_GESTURE_DOUBLE_TAP_BIT                 1
#define IQS7211E_GESTURE_TRIPLE_TAP_BIT                 2
#define IQS7211E_GESTURE_PRESS_HOLD_BIT                 3
#define IQS7211E_GESTURE_PALM_GESTURE_BIT               4        
#define IQS7211E_GESTURE_SWIPE_X_POSITIVE_BIT           0 // 8
#define IQS7211E_GESTURE_SWIPE_X_NEGATIVE_BIT           1 // 9       
#define IQS7211E_GESTURE_SWIPE_Y_POSITIVE_BIT           2 // 10
#define IQS7211E_GESTURE_SWIPE_Y_NEGATIVE_BIT           3 // 11
#define IQS7211E_GESTURE_SWIPE_HOLD_X_POSITIVE_BIT      4 // 12
#define IQS7211E_GESTURE_SWIPE_HOLD_X_NEGATIVE_BIT      5 // 13
#define IQS7211E_GESTURE_SWIPE_HOLD_Y_POSITIVE_BIT      6 // 14
#define IQS7211E_GESTURE_SWIPE_HOLD_Y_NEGATIVE_BIT      7 // 15

#define FINGER_1                                        1
#define FINGER_2                                        2

/* Defines and structs for IQS7211E states */
/**
 * @brief  iqs7211e Init Enumeration.
 */
typedef enum
{
        IQS7211E_INIT_NONE = (uint8_t) 0x00,
        IQS7211E_INIT_VERIFY_PRODUCT,
        IQS7211E_INIT_READ_RESET,
	IQS7211E_INIT_CHIP_RESET,
	IQS7211E_INIT_UPDATE_SETTINGS,
	IQS7211E_INIT_CHECK_RESET,
	IQS7211E_INIT_ACK_RESET,
	IQS7211E_INIT_ATI,
        IQS7211E_INIT_WAIT_FOR_ATI,
        IQS7211E_INIT_READ_DATA,
	IQS7211E_INIT_ACTIVATE_EVENT_MODE,
        IQS7211E_INIT_ACTIVATE_STREAM_MODE,
	IQS7211E_INIT_DONE
} iqs7211e_init_e;

typedef enum {
        IQS7211E_STATE_NONE = (uint8_t) 0x00,
        IQS7211E_STATE_START,
        IQS7211E_STATE_INIT,
        IQS7211E_STATE_SW_RESET,
        IQS7211E_STATE_CHECK_RESET,
	IQS7211E_STATE_RUN,
} iqs7211e_state_e;

typedef enum
{
        IQS7211E_CH_NONE = (uint8_t) 0x00,
        IQS7211E_CH_PROX,
        IQS7211E_CH_TOUCH,
        IQS7211E_CH_UNKNOWN,
} iqs7211e_ch_states;
typedef enum
{
        IQS7211E_ACTIVE = (uint8_t) 0x00,
        IQS7211E_IDLE_TOUCH,
        IQS7211E_IDLE,
        IQS7211E_LP1,
        IQS7211E_LP2,
        IQS7211E_POWER_UNKNOWN
} iqs7211e_power_modes;

typedef enum {
        IQS7211E_GESTURE_SINGLE_TAP = (uint8_t) 0x00,
        IQS7211E_GESTURE_DOUBLE_TAP,
        IQS7211E_GESTURE_TRIPLE_TAP,
        IQS7211E_GESTURE_PRESS_HOLD,
        IQS7211E_GESTURE_PALM_GESTURE,
        IQS7211E_GESTURE_SWIPE_X_POSITIVE,
        IQS7211E_GESTURE_SWIPE_X_NEGATIVE,
        IQS7211E_GESTURE_SWIPE_Y_POSITIVE,
        IQS7211E_GESTURE_SWIPE_Y_NEGATIVE,
        IQS7211E_GESTURE_SWIPE_HOLD_X_POSITIVE,
        IQS7211E_GESTURE_SWIPE_HOLD_X_NEGATIVE,
        IQS7211E_GESTURE_SWIPE_HOLD_Y_POSITIVE,
        IQS7211E_GESTURE_SWIPE_HOLD_Y_NEGATIVE,
        IQS7211E_GESTURE_NONE,
} iqs7211e_gestures_e;

/* IQS7211E Memory map data variables, only save the data that might be used 
during program runtime */
#pragma pack(1)
typedef struct
{
	/* READ ONLY */			//  I2C Addresses:
	uint8_t VERSION_DETAILS[20]; 	// 	0x00 -> 0x09
        uint8_t GESTURES[2]; 		// 	0x0E
	uint8_t INFO_FLAGS[2];          // 	0x0F
        uint8_t FINGER_1_X[2];          // 	0x10
        uint8_t FINGER_1_Y[2];          // 	0x11
        uint8_t FINGER_2_X[2];          // 	0x14
        uint8_t FINGER_2_Y[2];          // 	0x15

	/* READ WRITE */		//  I2C Addresses:
	uint8_t SYSTEM_CONTROL[2]; 	// 	0x33
} IQS7211E_MEMORY_MAP;
#pragma pack(4)

#pragma pack(1)
typedef struct {
        iqs7211e_state_e        state;
        iqs7211e_init_e         init_state;
}iqs7211e_s;
#pragma pack(4)

/* Class Prototype */
class IQS7211E
{
public:
        /* Public Constructors */
        IQS7211E();

        /* Public Device States */
        iqs7211e_s iqs7211e_state;

        /* Public Variables */
        IQS7211E_MEMORY_MAP IQSMemoryMap;
        bool new_data_available;

        /* Public Methods */
        void begin(uint8_t deviceAddressIn, uint8_t readyPinIn);
        bool init(void);
        void run(void);
        void queueValueUpdates(void);
        bool readATIactive(void);
        uint16_t getProductNum(bool stopOrRestart);
        uint8_t getmajorVersion(bool stopOrRestart);
        uint8_t getminorVersion(bool stopOrRestart);
        void acknowledgeReset(bool stopOrRestart);
        void ReATI(bool stopOrRestart);
        void SW_Reset(bool stopOrRestart);
        void writeMM(bool stopOrRestart);
        void clearRDY(void);
        bool getRDYStatus(void);

        void setStreamMode(bool stopOrRestart);
        void setEventMode(bool stopOrRestart);

        void updateInfoFlags(bool stopOrRestart);
        iqs7211e_power_modes getPowerMode(void);
        bool checkReset(void);
        void updateAbsCoordinates(bool stopOrRestart, uint8_t fingerNum);
        uint16_t getAbsYCoordinate(uint8_t fingerNum);
        uint16_t getAbsXCoordinate(uint8_t fingerNum);
        bool touchpad_event_occurred(void);
        iqs7211e_gestures_e get_touchpad_event(void);
        uint8_t getNumFingers(void);

        void force_I2C_communication(void);

private:
        /* Private Variables */
        uint8_t _deviceAddress;

        /* Private Methods */
        void readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);
        void writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);
        void writeRandomBytes16(uint16_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);
        bool getBit(uint8_t data, uint8_t bit_number);
        uint8_t setBit(uint8_t data, uint8_t bit_number);
        uint8_t clearBit(uint8_t data, uint8_t bit_number);
};
#endif /* IQS7211E_h */
