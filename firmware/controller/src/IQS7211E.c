/******************************************************************************
 * @file     IQS7211E.cpp
 * @brief    This file contains the constructors and methods which allow ease 
 *           of use of an IQS7211E capacitive touch controller. The IQS7211E is 
 *           a capacitive touch Integrated Circuit (IC) which provides multiple 
 *           channel functionality. This class provides an easy means of 
 *           initializing and interacting with the IQS7211E device from an 
 *           Arduino.
 * @author   JN. Lochner - Azoteq PTY Ltd
 * @version  V1.1
 * @date     2023
 ******************************************************************************
 * @attention  Makes use of the following standard Arduino libraries:
 * - Arduino.h -> Included in IQS7211E.h, comes standard with Arduino
 * - Wire.h    -> Included in IQS7211E.h, comes standard with Arduino
 *****************************************************************************/

/* Include Files */
#include "IQS7211E_init.h"
#include "IQS7211E.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(iqs7211e, LOG_LEVEL_INF);

/* Private Global Variables */
static bool iqs7211e_deviceRDY = false;
static uint8_t iqs7211e_ready_pin;
static struct gpio_dt_spec ready_pin_spec = {0}; // For polling RDY pin (no interrupt)

// GPIO interrupt callback for RDY pin (disabled)
static struct gpio_callback ready_pin_cb_data;

/* Private Functions */
static void iqs7211e_ready_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/*****************************************************************************/
/*                            PUBLIC METHODS                                 */
/*****************************************************************************/

/**
 * @name   iqs7211e_begin
 * @brief  A method to initialize the IQS7211E device with the device address 
 *         and ready pin specified by the user.
 * @param  instance        -> Pointer to the IQS7211E instance
 * @param  deviceAddressIn -> The address of the IQS7211E device.
 * @param  readyPinIn      -> The Arduino pin which is connected to the 
 *                            RDY/MCLR pin of the IQS7211E device.
 * @retval None.
 * @note   - Receiving a true return value does not mean that initialization was 
 *           successful.
 *         - Receiving a true return value only means that the IQS device 
 *           responded to the request for communication.
 *         - Receiving a false return value means that initialization did not 
 *           take place at all.
 *         - If communication is successfully established then it is unlikely 
 *           that initialization will fail.
 */
void iqs7211e_begin(iqs7211e_instance_t *instance, uint8_t deviceAddressIn, uint8_t readyPinIn)
{
  /* Initialize I2C communication here, since this library can't function 
    without it. */
  // Get I2C device - this should be passed from main.c
  const struct device *i2c_device = DEVICE_DT_GET(DT_NODELABEL(i2c1));
  wire_begin(&instance->wire, i2c_device);

  instance->_deviceAddress = deviceAddressIn;
  iqs7211e_ready_pin = readyPinIn;
  
  // Set up GPIO spec for ready pin polling in Zephyr (no interrupt for better performance)
  ready_pin_spec.port = DEVICE_DT_GET(DT_NODELABEL(gpio1));
  ready_pin_spec.pin = 5;  // P1.05 - Trackpad ready pin  
  ready_pin_spec.dt_flags = GPIO_ACTIVE_LOW;
  
  // Configure pin as input only (no interrupt)
  int ret = gpio_pin_configure_dt(&ready_pin_spec, GPIO_INPUT);
  if (ret != 0) {
    LOG_ERR("Failed to configure RDY pin as input: %d", ret);
    return;
  }
  
  // Configure RDY pin as input for polling (no interrupt for better ESB timing)
  // We still need to read the pin status during initialization and operation
  LOG_INF("RDY pin configured for polling - no interrupt for better performance");

  /* Initialize "running" and "init" state machine variables. */
  instance->iqs7211e_state.state      = IQS7211E_STATE_START;
  instance->iqs7211e_state.init_state = IQS7211E_INIT_VERIFY_PRODUCT;
}

/**
 * @name   iqs7211e_init
 * @brief  A method that runs through a normal start-up routine to set up the 
 *         IQS7211E with the desired settings from the IQS7211E_init.h file.
 * @param  instance -> Pointer to the IQS7211E instance
 * @retval Returns true if the full start-up routine has been completed, 
 *         returns false if not.
 * @note   - No false return will be given, the program will thus be stuck when 
 *           one of the cases is not able to finish.
 *         - See serial communication to find the ERROR case.
 *         - Note that the LOG commands can be commented out to save 
 *           on dynamic memory.
 */
bool iqs7211e_init(iqs7211e_instance_t *instance)
{
  uint16_t prod_num;
  uint8_t ver_maj, ver_min;

  switch (instance->iqs7211e_state.init_state)
  {
    /* Verifies product number to determine if the correct device is connected  
    for this example */
    case IQS7211E_INIT_VERIFY_PRODUCT:
      // Check RDY status first
      iqs7211e_getRDYStatus(instance);  // This will update iqs7211e_deviceRDY and print debug info
      
      if(iqs7211e_deviceRDY)
      {
        LOG_INF("\tIQS7211E_INIT_VERIFY_PRODUCT");
        prod_num  = iqs7211e_getProductNum(instance, RESTART);
        ver_maj   = iqs7211e_getmajorVersion(instance, RESTART);
        ver_min   = iqs7211e_getminorVersion(instance, STOP);
        LOG_INF("\t\tProduct number is: %d v%d.%d", prod_num, ver_maj, ver_min);
        if(prod_num == IQS7211E_PRODUCT_NUM)
        {
          LOG_INF("\t\tIQS7211E Release UI Confirmed!");
          instance->iqs7211e_state.init_state = IQS7211E_INIT_READ_RESET;
        }
        else
        {
          LOG_ERR("\t\tDevice is not a IQS7211E!");
          instance->iqs7211e_state.init_state = IQS7211E_INIT_NONE;
        }
      }
      else
      {
        static uint32_t last_rdy_debug = 0;
        uint32_t now = k_uptime_get_32();
        if ((now - last_rdy_debug) > 500) {  // Debug every 500ms
          LOG_WRN("\tWaiting for RDY pin - device not ready yet");
          last_rdy_debug = now;
        }
      }
    break;

    /* Verify if a reset has occurred */
    case IQS7211E_INIT_READ_RESET:
      // Force settings write during initial configuration - don't wait for RDY
      if(iqs7211e_deviceRDY || instance->iqs7211e_state.state == IQS7211E_STATE_INIT)
      {
        LOG_INF("\tIQS7211E_INIT_READ_RESET (forcing for initial config)");
        iqs7211e_updateInfoFlags(instance, RESTART);
        if (iqs7211e_checkReset(instance, STOP))
        {
          LOG_INF("\t\tReset event occurred.");
          instance->iqs7211e_state.init_state = IQS7211E_INIT_UPDATE_SETTINGS;
        }
        else
        {
          LOG_INF("\t\t No Reset Event Detected - Request SW Reset");
          instance->iqs7211e_state.init_state = IQS7211E_INIT_CHIP_RESET;
        }
      }
    break;

    /* Perform SW Reset */
    case IQS7211E_INIT_CHIP_RESET:
      // Force chip reset during initial configuration - don't wait for RDY
      if(iqs7211e_deviceRDY || instance->iqs7211e_state.state == IQS7211E_STATE_INIT)
      {
        LOG_INF("\tIQS7211E_INIT_CHIP_RESET (forcing for initial config)");

        //Perform SW Reset
        iqs7211e_SW_Reset(instance, STOP);
        LOG_INF("\t\tSoftware Reset Bit Set.");
        k_sleep(K_MSEC(100));
        instance->iqs7211e_state.init_state = IQS7211E_INIT_READ_RESET;
      }
    break;

    /* Write all settings to IQS7211E from .h file */
    case IQS7211E_INIT_UPDATE_SETTINGS:
      // Force settings write during initial configuration - don't wait for RDY
      if(iqs7211e_deviceRDY || instance->iqs7211e_state.state == IQS7211E_STATE_INIT)
      {
        LOG_INF("\tIQS7211E_INIT_UPDATE_SETTINGS (forcing for initial config)");
        iqs7211e_writeMM(instance, RESTART);
        instance->iqs7211e_state.init_state = IQS7211E_INIT_ACK_RESET;
      }
    break;

    /* Acknowledge that the device went through a reset */
    case IQS7211E_INIT_ACK_RESET:
      // Force reset acknowledgment during initial configuration - don't wait for RDY
      if(iqs7211e_deviceRDY || instance->iqs7211e_state.state == IQS7211E_STATE_INIT)
      {
        LOG_INF("\tIQS7211E_INIT_ACK_RESET (forcing for initial config)");
        iqs7211e_acknowledgeReset(instance, STOP);
        instance->iqs7211e_state.init_state = IQS7211E_INIT_ATI;
      }
      break;

    /* Run the ATI algorithm to recalibrate the device with the newly added 
    settings */
    case IQS7211E_INIT_ATI:
      // Force ATI during initial configuration - don't wait for RDY
      if(iqs7211e_deviceRDY || instance->iqs7211e_state.state == IQS7211E_STATE_INIT)
      {
        LOG_INF("\tIQS7211E_INIT_ATI (forcing for initial config)");
        iqs7211e_ReATI(instance, STOP);
        instance->iqs7211e_state.init_state = IQS7211E_INIT_WAIT_FOR_ATI;
        LOG_INF("\tIQS7211E_INIT_WAIT_FOR_ATI");
      }
    break;

    /* Read the ATI Active bit to see if the rest of the program can continue */
    case IQS7211E_INIT_WAIT_FOR_ATI:
      // Exact Arduino implementation with optional timeout for raw data reading
      static uint32_t ati_start_time = 0;
      
      // Initialize timeout on first entry
      if (ati_start_time == 0) {
        ati_start_time = k_uptime_get_32();
      }
      
      if(iqs7211e_deviceRDY)
      {
        if(!iqs7211e_readATIactive(instance))
        {
          LOG_INF("\t\tATI DONE - Initialization Complete!");
          instance->iqs7211e_state.init_state = IQS7211E_INIT_READ_DATA;
        }
      }
      
      // Timeout after 20 seconds for raw data reading (less aggressive than before)
      uint32_t elapsed = k_uptime_get_32() - ati_start_time;
      if (elapsed > 2000) {
        LOG_WRN("\t\tATI TIMEOUT (2s) - Proceeding with raw data reading");
        LOG_WRN("\t\tNote: ATI may still be running, but device should be functional");
        instance->iqs7211e_state.init_state = IQS7211E_INIT_READ_DATA;
      }
    break;

    /* Read the latest data from the iqs7211e */
    case IQS7211E_INIT_READ_DATA:
      // Force initial data read to complete initialization - don't wait for RDY yet
      LOG_INF("\tIQS7211E_INIT_READ_DATA (forcing for initial config)");
      iqs7211e_queueValueUpdates(instance);
      instance->iqs7211e_state.init_state = IQS7211E_INIT_ACTIVATE_EVENT_MODE;
    break;

    /* Turn on I2C Event mode */
    case IQS7211E_INIT_ACTIVATE_EVENT_MODE:
      // Force event mode activation to complete initialization - don't wait for RDY yet
      LOG_INF("\tIQS7211E_INIT_ACTIVATE_EVENT_MODE (forcing for initial config)");
      iqs7211e_setEventMode(instance, STOP);
      instance->iqs7211e_state.init_state = IQS7211E_INIT_DONE;
    break;

    /* Turn on I2C Stream mode */
    case IQS7211E_INIT_ACTIVATE_STREAM_MODE:
      if(iqs7211e_deviceRDY)
      {
        LOG_INF("\tIQS7211E_INIT_ACTIVATE_STREAM_MODE");
        iqs7211e_setStreamMode(instance, STOP);
        instance->iqs7211e_state.init_state = IQS7211E_INIT_DONE;
      }
    break;

    /* If all operations have been completed correctly, the RDY pin can be set
     * up as an interrupt to indicate when new data is available */
    case IQS7211E_INIT_DONE:
      LOG_INF("\tIQS7211E_INIT_DONE");
      instance->new_data_available = true;
      return true;
    break;

    default:
      break;
  }
  return false;
}

/**
 * @name    iqs7211e_run
 * @brief   Run the device ready check and read the necessary bytes when the 
 *          IQS7211E has pulled the RDY line low.
 *          The new_data_available flag will be set when a ready low is received 
 *          from the IQS7211E.
 * @param   instance -> Pointer to the IQS7211E instance
 * @retval  None.
 * @note    queueValueUpdates can be edited by the user if other data should be 
 *          read every time a RDY window is received.
 */
void iqs7211e_run(iqs7211e_instance_t *instance)
{
  switch (instance->iqs7211e_state.state)
  {
    /* After a hardware reset, this is the starting position of the running state 
    machine */
    case IQS7211E_STATE_START:
      LOG_INF("IQS7211E Initialization:");
      instance->iqs7211e_state.state = IQS7211E_STATE_INIT;
    break;

    /* Perform the initialization routine on the IQS7211E */
    case IQS7211E_STATE_INIT:
      if(iqs7211e_init(instance))
      {
        LOG_INF("IQS7211E Initialization complete!\n");
        instance->iqs7211e_state.state = IQS7211E_STATE_RUN;
      }
    break;

    /* Send an I2C software reset in the next RDY window */
    case IQS7211E_STATE_SW_RESET:
      if(iqs7211e_deviceRDY)
      {
        iqs7211e_SW_Reset(instance, STOP);
        instance->iqs7211e_state.state = IQS7211E_STATE_RUN;
      }
    break;

    /* Continuous reset monitoring state, ensure no reset event has occurred 
    for data to be valid */
    case IQS7211E_STATE_CHECK_RESET:
      if(iqs7211e_checkReset(instance, STOP))
      {
        LOG_WRN("Reset Occurred!\n");
        // instance->new_data_available = false;
        // instance->iqs7211e_state.state = IQS7211E_STATE_START;
        // instance->iqs7211e_state.init_state = IQS7211E_INIT_VERIFY_PRODUCT;
      }
      /* A reset did not occur, move to the run state and wait for a new RDY 
      window */
      else
      {
        instance->new_data_available = true; /* No reset, thus data is valid */
        instance->iqs7211e_state.state = IQS7211E_STATE_RUN;
      }
    break;

    /* If a RDY Window is open, read the latest values from the IQS7211E */
    case IQS7211E_STATE_RUN:
      if(iqs7211e_deviceRDY)
      {
        iqs7211e_queueValueUpdates(instance);
        iqs7211e_deviceRDY = false;
        instance->new_data_available = false;
        instance->iqs7211e_state.state = IQS7211E_STATE_CHECK_RESET;
      }
    break;
    
    case IQS7211E_STATE_NONE:
    default:
      /* Initialize to start state if in undefined state */
      instance->iqs7211e_state.state = IQS7211E_STATE_START;
    break;
  }
}

/**
 * @name   iqs7211e_ready_interrupt
 * @brief  A method used as an interrupt function. Only activated when a 
 *         High -> Low (Falling edge) and LOW -> High (Rising edge) interrupt 
 *         is seen on the correct GPIO interrupt pin.
 * @param  dev  -> GPIO device
 * @param  cb   -> GPIO callback 
 * @param  pins -> Pin mask
 * @retval None.
 * @note   Keep this function as simple as possible to prevent stuck states 
 *         and slow operations.
 */
static void iqs7211e_ready_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Read the actual GPIO pin state (RDY is active LOW)
    const struct gpio_dt_spec ready_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
        .pin = 5,  // P1.05 - Trackpad ready pin
        .dt_flags = GPIO_ACTIVE_LOW
    };
    
    // Read pin state - if LOW (0), device is ready
    int pin_state = gpio_pin_get_dt(&ready_pin);
    iqs7211e_deviceRDY = (pin_state == 0);  // Active LOW means 0 = ready
    
    // Simple debug (minimal logging in ISR)
    // LOG_INF("RDY interrupt: pin=%d, ready=%s", pin_state, iqs7211e_deviceRDY ? "YES" : "NO");
}

/**
  * @name   iqs7211e_clearRDY
  * @brief  A method used to clear the ready interrupt bit.
  * @param  instance -> Pointer to the IQS7211E instance
  * @retval None.
  */
void iqs7211e_clearRDY(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  iqs7211e_deviceRDY = false;
}

/**
/**
  * @name   iqs7211e_getRDYStatus
  * @brief  A method used to retrieve the device RDY status.
  * @param  instance -> Pointer to the IQS7211E instance
  * @retval Returns the boolean IQS7211E RDY state.
  *         - True when RDY line is LOW
  *         - False when RDY line is HIGH
  */
bool iqs7211e_getRDYStatus(iqs7211e_instance_t *instance)
{
  // Read the GPIO pin state directly (polling mode - no interrupt)
  if (ready_pin_spec.port != NULL) {
    int pin_state = gpio_pin_get_dt(&ready_pin_spec);
    if (pin_state >= 0) {
      // Update the global state variable based on actual pin reading
      iqs7211e_deviceRDY = (pin_state == 0);  // Active LOW means 0 = ready
      return iqs7211e_deviceRDY;
    } else {
      LOG_WRN("Failed to read RDY pin state: %d", pin_state);
      return false;
    }
  }
  
  // Fallback: return current state if GPIO not initialized
  return iqs7211e_deviceRDY;
}

/**
 * @name    iqs7211e_queueValueUpdates
 * @brief   All I2C read operations in the queueValueUpdates method will be 
 *          performed each time the IQS7211E opens a RDY window.
 * @param   instance -> Pointer to the IQS7211E instance
 * @retval  None.
 * @note    Any Address in the memory map can be read from here. This is where 
 *          data read from the chip gets updated.
 */
void iqs7211e_queueValueUpdates(iqs7211e_instance_t *instance)
{
  uint8_t transferBytes[10]; // The array which will hold the bytes to be transferred.

  /* Read the gesture and info flags. */
  iqs7211e_readRandomBytes(instance, IQS7211E_MM_GESTURES, 8, transferBytes, RESTART);

  /* Assign the gesture flags to the gesture flags */
  instance->IQSMemoryMap.GESTURES[0]    = transferBytes[0];
  instance->IQSMemoryMap.GESTURES[1]    = transferBytes[1];

  /* Assign the info flags to the info flags */
  instance->IQSMemoryMap.INFO_FLAGS[0]  = transferBytes[2];
  instance->IQSMemoryMap.INFO_FLAGS[1]  = transferBytes[3];

  /* Read Finger 1 x and y coordinate. */
  instance->IQSMemoryMap.FINGER_1_X[0]  = transferBytes[4];
  instance->IQSMemoryMap.FINGER_1_X[1]  = transferBytes[5];
  instance->IQSMemoryMap.FINGER_1_Y[0]  = transferBytes[6];
  instance->IQSMemoryMap.FINGER_1_Y[1]  = transferBytes[7];

  /* Read Finger 2 x and y coordinate. */
  iqs7211e_readRandomBytes(instance, IQS7211E_MM_FINGER_2_X, 4, transferBytes, STOP);

  instance->IQSMemoryMap.FINGER_2_X[0]  = transferBytes[0];
  instance->IQSMemoryMap.FINGER_2_X[1]  = transferBytes[1];
  instance->IQSMemoryMap.FINGER_2_Y[0]  = transferBytes[2];
  instance->IQSMemoryMap.FINGER_2_Y[1]  = transferBytes[3];
}

/**
  * @name	  iqs7211e_readATIactive
  * @brief  A method that checks if the ATI routine is still active
  * @param  instance -> Pointer to the IQS7211E instance
  * @retval Returns true if the ATI_ACTIVE_BIT is cleared, false if the 
  *         ATI_ACTIVE_BIT is set.
  * @note   If the ATI routine is active the channel states (NONE, PROX, TOUCH) 
  *         might exhibit unwanted behaviour. Thus it is advised to wait for the 
  *         routine to complete before continuing.
  */
bool iqs7211e_readATIactive(iqs7211e_instance_t *instance)
{
  // Use the direct INFO_FLAGS read like Arduino does
  iqs7211e_updateInfoFlags(instance, STOP);

  // Check bit 4 in INFO_FLAGS[0] (same as Arduino)
  if (iqs7211e_getBit(instance->IQSMemoryMap.INFO_FLAGS[0], IQS7211E_RE_ATI_OCCURRED_BIT)) {
    return true; // ATI done
  }

  return false; // ATI still active
}

/**
 * @name	 iqs7211e_checkReset
 * @brief  A method that checks if the device has reset and returns the reset status.
 * @param  instance -> Pointer to the IQS7211E instance
 * @param  stopOrRestart -> Specifies whether the communications window must be 
 *                          kept open or must be closed after this action.
 * @retval Returns true if a reset has occurred, false if no reset has occurred.
 * @note   If a reset has occurred the device settings should be reloaded using 
 *         the begin function. After new device settings have been reloaded the 
 *         acknowledge reset function can be used to clear the reset flag.
 */
bool iqs7211e_checkReset(iqs7211e_instance_t *instance, bool stopOrRestart)
{
	/* Perform a bitwise AND operation inside the getBit function with the  
  SHOW_RESET_BIT to return the reset status */
  return iqs7211e_getBit(instance->IQSMemoryMap.INFO_FLAGS[0], IQS7211E_SHOW_RESET_BIT);
}

/**
 * @name	 getProductNum
 * @brief  A method that  gets the device product number and returns the result
 * @param  stopOrRestart -> Specifies whether the communications window must be 
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval Returns product number as a 16-bit unsigned integer value
 * @note   If the product is not correctly identified an appropriate messages 
 *         should be displayed.
 */
uint16_t iqs7211e_getProductNum(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  uint8_t transferBytes[2];	      // A temporary array to hold the byte to be transferred.
  uint8_t prodNumLow = 0;         // Temporary storage for the low byte.
  uint8_t prodNumHigh = 0;        // Temporary storage for the high byte.
  uint16_t prodNumReturn = 0;     // The 16-bit return value.

	/* Read the Device info from the IQS7211E. */
	iqs7211e_readRandomBytes(instance, IQS7211E_MM_PROD_NUM, 2, transferBytes, stopOrRestart);

  /* Construct the 16-bit return value. */
  prodNumLow = transferBytes[0];
  prodNumHigh = transferBytes[1];
  prodNumReturn = (uint16_t)(prodNumLow);
  prodNumReturn |= (uint16_t)(prodNumHigh<<8);
  /* Return the counts value. */
  return prodNumReturn;
}

/**
 * @name	 getmajorVersion
 * @brief  A method that gets the device firmware major version number
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval Returns major version number as an 8-bit unsigned integer value.
 */
uint8_t iqs7211e_getmajorVersion(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  /* A temporary array to hold the byte to be transferred. */
  uint8_t transferBytes[2]; 
  /* Temporary storage for the firmware version major number. */
  uint8_t ver_maj = 0;      

  /* Read the info from the IQS7211E. */
  iqs7211e_readRandomBytes(instance, IQS7211E_MM_MAJOR_VERSION_NUM, 2, transferBytes, stopOrRestart);

  /* Extract major value from correct byte */
  ver_maj = transferBytes[0];
  /* Return the major firmware version number value. */
  return ver_maj;
}

/**
 * @name	getminorVersion
 * @brief  A method that checks the device firmware minor version number
 * @param  stopOrRestart -> Specifies whether the communications window must be 
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval Returns minor version number as an 8-bit unsigned integer value.
 */
uint8_t iqs7211e_getminorVersion(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  /* A temporary array to hold the byte to be transferred. */
  uint8_t transferBytes[2]; 
  /* Temporary storage for the firmware version minor number. */
  uint8_t ver_min = 0;      

  /* Read the info from the IQS7211E. */
  iqs7211e_readRandomBytes(instance, IQS7211E_MM_MINOR_VERSION_NUM, 2, transferBytes, stopOrRestart);
  /* Extract minor value from correct byte */
  ver_min = transferBytes[0];
  /* Return the minor firmware version number value. */
  return ver_min;
}

/**
 * @name	  acknowledgeReset
 * @brief   A method that clears the Show Reset bit by setting the Ack Reset bit
 * @param   stopOrRestart -> Specifies whether the communications window must be 
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval  None.
 * @note    If a reset has occurred the device settings should be reloaded using 
 *          the begin function. After new device settings have been reloaded this
 *          method should be used to clear the reset bit.
 */
void iqs7211e_acknowledgeReset(iqs7211e_instance_t *instance, bool stopOrRestart)
{
	uint8_t transferByte[2];	// A temporary array to hold the bytes to be transferred.
	/* Read the System Flags from the IQS7211E, these must be read first in order not to change any settings. */
	iqs7211e_readRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 2, transferByte, RESTART);
	/* Write the Ack Reset bit to bit 1 to clear the Reset Event Flag. */
	transferByte[0] = iqs7211e_setBit(transferByte[0], IQS7211E_ACK_RESET_BIT);
	/* Write the new byte to the System Flags address. */
	iqs7211e_writeRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 2, transferByte, stopOrRestart);
}

/**
 * @name   ReATI
 * @brief  A method that sets the ReATI bit to force the IQS7211E device
 *         to run the Automatic Tuning Implementation (ATI) routine on the
 *         enabled channels.
 * @param  stopOrRestart -> Specifies whether the communications window must 
 *                          be kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   To force ATI, IQS7211E_TP_RE_ATI_BIT in the System Control register 
 *         is set.
 */
void iqs7211e_ReATI(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  uint8_t transferByte[2]; // Array to store the bytes transferred.

  iqs7211e_readRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 2, transferByte, RESTART);
  /* Set the TP_RE_ATI_BIT in the SYS_Control register */
  transferByte[0] = iqs7211e_setBit(transferByte[0], IQS7211E_TP_RE_ATI_BIT); 
  /* Write the new byte to the required device. */
  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 2, transferByte, stopOrRestart);
}

/**
 * @name   SW_Reset
 * @brief  A method that sets the SW RESET bit to force the IQS7211E 
 *         device to do a SW reset.
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   To perform SW Reset, SW_RESET_BIT in SYSTEM_CONTROL is set.
 */
void iqs7211e_SW_Reset(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  uint8_t transferByte[2]; // Array to store the bytes transferred.

  iqs7211e_readRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 2, transferByte, RESTART);
  /* Set the SW_RESET_BIT in the SYS_Control register */
  transferByte[1] = iqs7211e_setBit(transferByte[1], IQS7211E_SW_RESET_BIT); 
  /* Write the new byte to the required device. */
  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 2, transferByte, stopOrRestart);
}

/**
  * @name   setStreamMode
  * @brief  A method to set the IQS7211E device into streaming mode.
  * @param  stopOrRestart -> Specifies whether the communications window must 
  *                          be kept open or must be closed after this action.
  *                          Use the STOP and RESTART definitions.
  * @retval None.
  * @note   All other bits at the IQS7211E_MM_SYS_CONTROL register address 
  *         are preserved.
  */
void iqs7211e_setStreamMode(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.

  /* First read the bytes at the memory address so that they can be preserved. */
  iqs7211e_readRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 2, transferBytes, RESTART);
  /* Set/Clear the IQS7211E_EVENT_MODE_BIT in SYS_CONTROL register */
  transferBytes[1] = iqs7211e_clearBit(transferBytes[1], IQS7211E_EVENT_MODE_BIT);
  /* Write the bytes back to the device */
  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 2, transferBytes, stopOrRestart);
}

/**
  * @name   setEventMode
  * @brief  A method to set the IQS7211E device into event mode.
  * @param  stopOrRestart ->  Specifies whether the communications window must 
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   All other bits at the IQS7211E_MM_CONFIG_SETTINGS register address 
  *         are preserved.
  */
void iqs7211e_setEventMode(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  uint8_t transferByte[2]; // The array which will hold the bytes which are transferred.

  /* First read the bytes at the memory address so that they can be preserved */
  iqs7211e_readRandomBytes(instance, IQS7211E_MM_CONFIG_SETTINGS, 2, transferByte, RESTART);
  /* Set/Clear the IQS7211E_EVENT_MODE_BIT in SYS_CONTROL register  */
  transferByte[1] = iqs7211e_setBit(transferByte[1], IQS7211E_EVENT_MODE_BIT);
  /* Write the bytes back to the device */
  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_CONFIG_SETTINGS, 2, transferByte, stopOrRestart);
}

/**
 * @name   updateInfoFlags
 * @brief  A method which reads the IQS7211E info flags and assigns them to the
 *         INFO_FLAGS memory map local variable
 * @param  stopOrRestart -> Specifies whether the communications window must be
 *                          kept open or must be closed after this action.
 *              			      Use the STOP and RESTART definitions.
 * @retval None.
 * @note   The INFO_FLAGS memory map local variable is altered with the new 
 *         value of the info flags register retrieved from the IQS7211E.
 */
void iqs7211e_updateInfoFlags(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  /* The array which will hold the bytes to be transferred. */
  uint8_t transferBytes[2]; 

  /* Read the info flags. */
  iqs7211e_readRandomBytes(instance, IQS7211E_MM_INFO_FLAGS, 2, transferBytes, stopOrRestart);
  /* Assign the info flags to the INFO_FLAGS memory map local variable */
  instance->IQSMemoryMap.INFO_FLAGS[0] = transferBytes[0];
  instance->IQSMemoryMap.INFO_FLAGS[1] = transferBytes[1];
}

/**
  * @name   iqs7211e_getPowerMode
  * @brief  A method which reads the INFO_FLAGS memory map local variable and 
  *         returns the current power mode.
  * @param  instance -> Pointer to the IQS7211E instance
  * @param  stopOrRestart -> Specifies whether the communications window must be 
  *                          kept open or must be closed after this action.
  * @retval Returns the current iqs7211e_power_modes state the device is in.
  * @note   See Datasheet on power mode options and timeouts. 
  *         Normal Power, Low Power and Ultra Low Power (ULP).
  */
iqs7211e_power_modes iqs7211e_getPowerMode(iqs7211e_instance_t *instance, bool stopOrRestart)
{
  uint8_t buffer = iqs7211e_getBit(instance->IQSMemoryMap.INFO_FLAGS[0], IQS7211E_CHARGING_MODE_BIT_0);
  buffer += iqs7211e_getBit(instance->IQSMemoryMap.INFO_FLAGS[0], IQS7211E_CHARGING_MODE_BIT_1) << 1;
  buffer += iqs7211e_getBit(instance->IQSMemoryMap.INFO_FLAGS[0], IQS7211E_CHARGING_MODE_BIT_2) << 2;

  if(buffer == IQS7211E_ACTIVE_BITS)
  {
    return IQS7211E_ACTIVE;
  }
  else if(buffer == IQS7211E_IDLE_TOUCH_BITS)
  {
    return IQS7211E_IDLE_TOUCH;
  }
  else if(buffer == IQS7211E_IDLE_BITS)
  {
    return IQS7211E_IDLE;
  }
    else if(buffer == IQS7211E_LP1_BITS)
  {
    return IQS7211E_LP1;
  }
    else if(buffer == IQS7211E_LP2_BITS)
  {
    return IQS7211E_LP2;
  }
  else
  {
    return IQS7211E_POWER_UNKNOWN;
  }
}

/**
 * @name   updateAbsCoordinates
 * @brief  A method which reads the IQS7211E x and y coordinates from the IQS7211E 
 *         and assigns them to the requested finger number's X and Y register 
 *         in the local memory map variables
 * @param  stopOrRestart -> Specifies whether the communications window must be 
 *                          kept open or closed after retrieving the information.
 *                          Use the STOP and RESTART definitions.
 * @param  fingerNum     -> Specifies the finger number. Finger 1 is the first 
 *                          finger to touch the trackpad, finger 2 is the second
 *                          to touch the trackpad.
 * @retval None.
 * @note   The FINGER_1_X and FINGER_1_Y local memory map variables are altered 
 *         with the new value of the coordinates registers from the IQS7211E. 
 *         The user can use the getAbsXCoordinate and getAbsYCoordinate methods 
 *         to return the value.
 */
void iqs7211e_updateAbsCoordinates(iqs7211e_instance_t *instance, bool stopOrRestart, uint8_t fingerNum)
{
  /* The temporary address which will hold the bytes from the 
  IQS7211E_MM_FINGER_1_X register address. */
  uint8_t transferBytes[4]; 
  if (fingerNum == FINGER_1)
  {
    /* Read the bytes using the readRandomBytes method to read bytes at the 
      Finger 1 address. */
    iqs7211e_readRandomBytes(instance, IQS7211E_MM_FINGER_1_X, 4, transferBytes, stopOrRestart);
    /*  Assign the bytes to the FINGER_1_X and FINGER_1_Y local memory map 
    variables. */
    instance->IQSMemoryMap.FINGER_1_X[0] = transferBytes[0];
    instance->IQSMemoryMap.FINGER_1_X[1] = transferBytes[1];
    instance->IQSMemoryMap.FINGER_1_Y[0] = transferBytes[2];
    instance->IQSMemoryMap.FINGER_1_Y[1] = transferBytes[3];
  }
  else if (fingerNum == FINGER_2)
  {
    /* Read the bytes using the readRandomBytes method to read bytes at the 
    Finger 2 address. */
    iqs7211e_readRandomBytes(instance, IQS7211E_MM_FINGER_2_X, 4, transferBytes, stopOrRestart);
    /*  Assign the bytes to the FINGER_2_X and FINGER_2_Y local memory map 
    variables. */
    instance->IQSMemoryMap.FINGER_2_X[0] = transferBytes[0];
    instance->IQSMemoryMap.FINGER_2_X[1] = transferBytes[1];
    instance->IQSMemoryMap.FINGER_2_Y[0] = transferBytes[2];
    instance->IQSMemoryMap.FINGER_2_Y[1] = transferBytes[3];
  }
}

/**
 * @name	 getAbsYCoordinate
 * @brief  A method that returns the constructed 16-bit coordinate value
 * @param  fingerNum     -> Specifies the finger number. Finger 1 is the first 
 *                          finger to touch the trackpad, finger 2 is the second
 *                          to touch the trackpad.
 * @retval Returns 16-bit Y coordinate value.
 */
uint16_t iqs7211e_getAbsYCoordinate(iqs7211e_instance_t *instance, uint8_t fingerNum)
{
  /* The 16-bit return value. */
  uint16_t absYCoordReturn = 0; 

  /* Construct the 16-bit return value. */
  if (fingerNum == FINGER_1)
  {
    absYCoordReturn = (uint16_t)(instance->IQSMemoryMap.FINGER_1_Y[0]);
    absYCoordReturn |= (uint16_t)(instance->IQSMemoryMap.FINGER_1_Y[1] << 8);
  }
  else if (fingerNum == FINGER_2)
  {
    absYCoordReturn = (uint16_t)(instance->IQSMemoryMap.FINGER_2_Y[0]);
    absYCoordReturn |= (uint16_t)(instance->IQSMemoryMap.FINGER_2_Y[1] << 8);
  }
  /* Invalid finger number requested */
  else
  {
    return 0xFFFF;
  }

  /*- Return the coordinate value. 
    - Note that a value of 65535 (0xFFFF) means there is no touch. */
  return absYCoordReturn;
}

/**
 * @name	getAbsXCoordinate
 * @brief  A method that returns the constructed 16-bit coordinate value
 * @param  fingerNum     -> Specifies the finger number. Finger 1 is the first 
 *                          finger to touch the trackpad, finger 2 is the second
 *                          to touch the trackpad.
 * @retval Returns 16-bit X coordinate value.
 */
uint16_t iqs7211e_getAbsXCoordinate(iqs7211e_instance_t *instance, uint8_t fingerNum)
{
  /* The 16-bit return value. */
  uint16_t absXCoordReturn = 0; 

  /* Construct the 16-bit return value. */
  if (fingerNum == FINGER_1)
  {
    absXCoordReturn = (uint16_t)(instance->IQSMemoryMap.FINGER_1_X[0]);
    absXCoordReturn |= (uint16_t)(instance->IQSMemoryMap.FINGER_1_X[1] << 8);
  }
  else if (fingerNum == FINGER_2)
  {
    absXCoordReturn = (uint16_t)(instance->IQSMemoryMap.FINGER_2_X[0]);
    absXCoordReturn |= (uint16_t)(instance->IQSMemoryMap.FINGER_2_X[1] << 8);
  }
  /* Invalid finger number requested */
  else
  {
    return 0xFFFF;
  }

  /*- Return the coordinate value. 
    - Note that a value of 65535 (0xFFFF) means there is no touch. */
  return absXCoordReturn;
}

/**
  * @name   touchpad_event_occurred
  * @brief  A method that tells the user if an event occurred on the touchpad
  * @retval Returns true if the slider event occurred bit has been set by the 
  *         IQS7211E, false if not.
  */
bool iqs7211e_touchpad_event_occurred(iqs7211e_instance_t *instance)
{
    return iqs7211e_getBit(instance->IQSMemoryMap.INFO_FLAGS[1], IQS7211E_TP_MOVEMENT_BIT);
}

/**
  * @name   get_touchpad_event
  * @brief  A method that gives the type of event that occurred on the touchpad.
  * @retval Returns the slider event that has occurred defined as iqs7211e_gestures_e.
  * @note   See the iqs7211e_gestures_e typedef for all possible slider events 
  *         that can occur.
  */
iqs7211e_gestures_e iqs7211e_get_touchpad_event(iqs7211e_instance_t *instance)
{
  /* Find the slider event that occurred */
  if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[0], IQS7211E_GESTURE_SINGLE_TAP_BIT))
  {
    return IQS7211E_GESTURE_SINGLE_TAP;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[0], IQS7211E_GESTURE_DOUBLE_TAP_BIT))
  {
    return IQS7211E_GESTURE_DOUBLE_TAP;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[0], IQS7211E_GESTURE_TRIPLE_TAP_BIT))
  {
    return IQS7211E_GESTURE_TRIPLE_TAP;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[0], IQS7211E_GESTURE_PRESS_HOLD_BIT))
  {
    return IQS7211E_GESTURE_PRESS_HOLD;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[0], IQS7211E_GESTURE_PALM_GESTURE_BIT))
  {
    return IQS7211E_GESTURE_PALM_GESTURE;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[1], IQS7211E_GESTURE_SWIPE_X_POSITIVE_BIT))
  {
    return IQS7211E_GESTURE_SWIPE_X_POSITIVE;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[1], IQS7211E_GESTURE_SWIPE_X_NEGATIVE_BIT))
  {
    return IQS7211E_GESTURE_SWIPE_X_NEGATIVE;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[1], IQS7211E_GESTURE_SWIPE_Y_POSITIVE_BIT))
  {
    return IQS7211E_GESTURE_SWIPE_Y_POSITIVE;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[1], IQS7211E_GESTURE_SWIPE_Y_NEGATIVE_BIT))
  {
    return IQS7211E_GESTURE_SWIPE_Y_NEGATIVE;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[1], IQS7211E_GESTURE_SWIPE_HOLD_X_POSITIVE_BIT))
  {
    return IQS7211E_GESTURE_SWIPE_HOLD_X_POSITIVE;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[1], IQS7211E_GESTURE_SWIPE_HOLD_X_NEGATIVE_BIT))
  {
    return IQS7211E_GESTURE_SWIPE_HOLD_X_NEGATIVE;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[1], IQS7211E_GESTURE_SWIPE_HOLD_Y_POSITIVE_BIT))
  {
    return IQS7211E_GESTURE_SWIPE_HOLD_Y_POSITIVE;
  }
  else if(iqs7211e_getBit(instance->IQSMemoryMap.GESTURES[1], IQS7211E_GESTURE_SWIPE_HOLD_Y_NEGATIVE_BIT))
  {
    return IQS7211E_GESTURE_SWIPE_HOLD_Y_NEGATIVE;
  }
  else
  {
    return IQS7211E_GESTURE_NONE;
  }
}

/**
  * @name   getNumFingers
  * @brief  A method that returns the number of fingers active on the trackpad. 
  * @param  None.
  * @retval Returns an 8-bit unsigned integer value of the number of fingers 
  *         on the trackpad. 
  */
uint8_t iqs7211e_getNumFingers(iqs7211e_instance_t *instance)
{
  uint8_t buffer = iqs7211e_getBit(instance->IQSMemoryMap.INFO_FLAGS[1], IQS7211E_NUM_FINGERS_BIT_0);
  buffer += iqs7211e_getBit(instance->IQSMemoryMap.INFO_FLAGS[1], IQS7211E_NUM_FINGERS_BIT_1);

  return buffer;
}

/*****************************************************************************/
/*								        	ADVANCED PUBLIC METHODS			    						     */
/*****************************************************************************/

/**
 * @name   writeMM
 * @brief  Function to write the whole memory map to the device (writable) 
 *         registers
 * @param  stopOrRestart -> Specifies whether the communications window must 
 *                          be kept open or must be closed after this action.
 *                          Use the STOP and RESTART definitions.
 * @retval None.
 * @note   IQS7211E_init.h -> exported GUI init.h file
 */
void iqs7211e_writeMM(iqs7211e_instance_t *instance, bool stopOrRestart)
{

  uint8_t transferBytes[30]; // Temporary array which holds the bytes to be transferred.

  /* Change the ALP ATI Compensation */
  /* Memory Map Position 0x1F - 0x20 */
  transferBytes[0] = ALP_COMPENSATION_A_0;
  transferBytes[1] = ALP_COMPENSATION_A_1;
  transferBytes[2] = ALP_COMPENSATION_B_0;
  transferBytes[3] = ALP_COMPENSATION_B_1;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_ALP_ATI_COMP_A, 4, transferBytes, RESTART);
  LOG_INF("\t\t1. Write ALP Compensation");

  /* Change the ATI Settings */
  /* Memory Map Position 0x21 - 0x27 */
  transferBytes[0] = TP_ATI_MULTIPLIERS_DIVIDERS_0;
  transferBytes[1] = TP_ATI_MULTIPLIERS_DIVIDERS_1;
  transferBytes[2] = TP_COMPENSATION_DIV;
  transferBytes[3] = TP_REF_DRIFT_LIMIT;
  transferBytes[4] = TP_ATI_TARGET_0;
  transferBytes[5] = TP_ATI_TARGET_1;
  transferBytes[6] = TP_MIN_COUNT_REATI_0;
  transferBytes[7] = TP_MIN_COUNT_REATI_1;
  transferBytes[8] = ALP_ATI_MULTIPLIERS_DIVIDERS_0;
  transferBytes[9] = ALP_ATI_MULTIPLIERS_DIVIDERS_1;
  transferBytes[10] = ALP_COMPENSATION_DIV;
  transferBytes[11] = ALP_LTA_DRIFT_LIMIT;
  transferBytes[12] = ALP_ATI_TARGET_0;
  transferBytes[13] = ALP_ATI_TARGET_1;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_TP_GLOBAL_MIRRORS, 14, transferBytes, RESTART);
  LOG_INF("\t\t2. Write ATI Settings");

  /* Change the RR and Timing Settings */
  /* Memory Map Position 0x28 - 0x32 */
  transferBytes[0] = ACTIVE_MODE_REPORT_RATE_0;
  transferBytes[1] = ACTIVE_MODE_REPORT_RATE_1;
  transferBytes[2] = IDLE_TOUCH_MODE_REPORT_RATE_0;
  transferBytes[3] = IDLE_TOUCH_MODE_REPORT_RATE_1;
  transferBytes[4] = IDLE_MODE_REPORT_RATE_0;
  transferBytes[5] = IDLE_MODE_REPORT_RATE_1;
  transferBytes[6] = LP1_MODE_REPORT_RATE_0;
  transferBytes[7] = LP1_MODE_REPORT_RATE_1;
  transferBytes[8] = LP2_MODE_REPORT_RATE_0;
  transferBytes[9] = LP2_MODE_REPORT_RATE_1;
  transferBytes[10] = ACTIVE_MODE_TIMEOUT_0;
  transferBytes[11] = ACTIVE_MODE_TIMEOUT_1;
  transferBytes[12] = IDLE_TOUCH_MODE_TIMEOUT_0;
  transferBytes[13] = IDLE_TOUCH_MODE_TIMEOUT_1;
  transferBytes[14] = IDLE_MODE_TIMEOUT_0;
  transferBytes[15] = IDLE_MODE_TIMEOUT_1;
  transferBytes[16] = LP1_MODE_TIMEOUT_0;
  transferBytes[17] = LP1_MODE_TIMEOUT_1;
  transferBytes[18] = REATI_RETRY_TIME;
  transferBytes[19] = REF_UPDATE_TIME;
  transferBytes[20] = I2C_TIMEOUT_0;
  transferBytes[21] = I2C_TIMEOUT_1;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_ACTIVE_MODE_RR, 22, transferBytes, RESTART);
  LOG_INF("\t\t3. Write Report rates and timings");

  /* Change the System Settings */
  /* Memory Map Position 0x33 - 0x35 */
  transferBytes[0] = SYSTEM_CONTROL_0;
  transferBytes[1] = SYSTEM_CONTROL_1;
  transferBytes[2] = CONFIG_SETTINGS0;
  transferBytes[3] = CONFIG_SETTINGS1;
  transferBytes[4] = OTHER_SETTINGS_0;
  transferBytes[5] = OTHER_SETTINGS_1;
  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_SYS_CONTROL, 6, transferBytes, RESTART);
  LOG_INF("\t\t4. Write System control settings");

  /* Change the ALP Settings */
  /* Memory Map Position 0x36 - 0x37 */
  transferBytes[0] = ALP_SETUP_0;
  transferBytes[1] = ALP_SETUP_1;
  transferBytes[2] = ALP_TX_ENABLE_0;
  transferBytes[3] = ALP_TX_ENABLE_1;
  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_ALP_SETUP, 4, transferBytes, RESTART);
  LOG_INF("\t\t5. Write ALP Settings");

  /* Change the Threshold Settings */
  /* Memory Map Position 0x38 - 0x3A */
  transferBytes[0] = TRACKPAD_TOUCH_SET_THRESHOLD;
  transferBytes[1] = TRACKPAD_TOUCH_CLEAR_THRESHOLD;
  transferBytes[2] = ALP_THRESHOLD_0;
  transferBytes[3] = ALP_THRESHOLD_1;
  transferBytes[4] = ALP_SET_DEBOUNCE;
  transferBytes[5] = ALP_CLEAR_DEBOUNCE;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_TP_TOUCH_SET_CLEAR_THR, 6, transferBytes, RESTART);
  LOG_INF("\t\t6. Write Threshold settings");

  /* Change the Button and ALP count and LTA betas */
  /* Memory Map Position 0x3B - 0x3C */
  transferBytes[0] = ALP_COUNT_BETA_LP1;
  transferBytes[1] = ALP_LTA_BETA_LP1;
  transferBytes[2] = ALP_COUNT_BETA_LP2;
  transferBytes[3] = ALP_LTA_BETA_LP2;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_LP1_FILTERS, 4, transferBytes, RESTART);
  LOG_INF("\t\t7. Write Filter Betas");

  /* Change the Hardware Settings */
  /* Memory Map Position 0x3D - 0x40 */
  transferBytes[0] = TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
  transferBytes[1] = TP_CONVERSION_FREQUENCY_FRACTION_VALUE;
  transferBytes[2] = ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
  transferBytes[3] = ALP_CONVERSION_FREQUENCY_FRACTION_VALUE;
  transferBytes[4] = TRACKPAD_HARDWARE_SETTINGS_0;
  transferBytes[5] = TRACKPAD_HARDWARE_SETTINGS_1;
  transferBytes[6] = ALP_HARDWARE_SETTINGS_0;
  transferBytes[7] = ALP_HARDWARE_SETTINGS_1;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_TP_CONV_FREQ, 8, transferBytes, RESTART);
  LOG_INF("\t\t8. Write Hardware settings");

  /* Change the TP Setup */
  /* Memory Map Position 0x41 - 0x49 */
  transferBytes[0] = TRACKPAD_SETTINGS_0_0;
  transferBytes[1] = TRACKPAD_SETTINGS_0_1;
  transferBytes[2] = TRACKPAD_SETTINGS_1_0;
  transferBytes[3] = TRACKPAD_SETTINGS_1_1;
  transferBytes[4] = X_RESOLUTION_0;
  transferBytes[5] = X_RESOLUTION_1;
  transferBytes[6] = Y_RESOLUTION_0;
  transferBytes[7] = Y_RESOLUTION_1;
  transferBytes[8] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_0;
  transferBytes[9] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_1;
  transferBytes[10] = XY_DYNAMIC_FILTER_TOP_SPEED_0;
  transferBytes[11] = XY_DYNAMIC_FILTER_TOP_SPEED_1;
  transferBytes[12] = XY_DYNAMIC_FILTER_BOTTOM_BETA;
  transferBytes[13] = XY_DYNAMIC_FILTER_STATIC_FILTER_BETA;
  transferBytes[14] = STATIONARY_TOUCH_MOV_THRESHOLD;
  transferBytes[15] = FINGER_SPLIT_FACTOR;
  transferBytes[16] = X_TRIM_VALUE;
  transferBytes[17] = Y_TRIM_VALUE;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_TP_RX_SETTINGS, 18, transferBytes, RESTART);
  LOG_INF("\t\t9. Write TP Settings");

  /* Change the Settings Version Numbers */
  /* Memory Map Position 0x4A - 0x4A */
  transferBytes[0] = MINOR_VERSION;
  transferBytes[1] = MAJOR_VERSION;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_SETTINGS_VERSION, 2, transferBytes, RESTART);
  LOG_INF("\t\t10. Write Version numbers");

  /* Change the Gesture Settings */
  /* Memory Map Position 0x4B - 0x55 */
  transferBytes[0] = GESTURE_ENABLE_0;
  transferBytes[1] = GESTURE_ENABLE_1;
  transferBytes[2] = TAP_TOUCH_TIME_0;
  transferBytes[3] = TAP_TOUCH_TIME_1;
  transferBytes[4] = TAP_WAIT_TIME_0;
  transferBytes[5] = TAP_WAIT_TIME_1;
  transferBytes[6] = TAP_DISTANCE_0;
  transferBytes[7] = TAP_DISTANCE_1;
  transferBytes[8] = HOLD_TIME_0;
  transferBytes[9] = HOLD_TIME_1;
  transferBytes[10] = SWIPE_TIME_0;
  transferBytes[11] = SWIPE_TIME_1;
  transferBytes[12] = SWIPE_X_DISTANCE_0;
  transferBytes[13] = SWIPE_X_DISTANCE_1;
  transferBytes[14] = SWIPE_Y_DISTANCE_0;
  transferBytes[15] = SWIPE_Y_DISTANCE_1;
  transferBytes[16] = SWIPE_X_CONS_DIST_0;
  transferBytes[17] = SWIPE_X_CONS_DIST_1;
  transferBytes[18] = SWIPE_Y_CONS_DIST_0;
  transferBytes[19] = SWIPE_Y_CONS_DIST_1;
  transferBytes[20] = SWIPE_ANGLE;
  transferBytes[21] = PALM_THRESHOLD;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_GESTURE_ENABLE, 22, transferBytes, RESTART);
  LOG_INF("\t\t11. Write Gesture Settings");

  /* Change the RxTx Mapping */
  /* Memory Map Position 0x56 - 0x5C */
  transferBytes[0] = RX_TX_MAP_0;
  transferBytes[1] = RX_TX_MAP_1;
  transferBytes[2] = RX_TX_MAP_2;
  transferBytes[3] = RX_TX_MAP_3;
  transferBytes[4] = RX_TX_MAP_4;
  transferBytes[5] = RX_TX_MAP_5;
  transferBytes[6] = RX_TX_MAP_6;
  transferBytes[7] = RX_TX_MAP_7;
  transferBytes[8] = RX_TX_MAP_8;
  transferBytes[9] = RX_TX_MAP_9;
  transferBytes[10] = RX_TX_MAP_10;
  transferBytes[11] = RX_TX_MAP_11;
  transferBytes[12] = RX_TX_MAP_12;
  transferBytes[13] = RX_TX_MAP_FILLER;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_RX_TX_MAPPING_0_1, 14, transferBytes, RESTART);
  LOG_INF("\t\t12. Write Rx Tx Map Settings");

  /* Change the Allocation of channels into cycles 0-9 */
  /* Memory Map Position 0x5D - 0x6B */
  transferBytes[0] = PLACEHOLDER_0;
  transferBytes[1] = CH_1_CYCLE_0;
  transferBytes[2] = CH_2_CYCLE_0;
  transferBytes[3] = PLACEHOLDER_1;
  transferBytes[4] = CH_1_CYCLE_1;
  transferBytes[5] = CH_2_CYCLE_1;
  transferBytes[6] = PLACEHOLDER_2;
  transferBytes[7] = CH_1_CYCLE_2;
  transferBytes[8] = CH_2_CYCLE_2;
  transferBytes[9] = PLACEHOLDER_3;
  transferBytes[10] = CH_1_CYCLE_3;
  transferBytes[11] = CH_2_CYCLE_3;
  transferBytes[12] = PLACEHOLDER_4;
  transferBytes[13] = CH_1_CYCLE_4;
  transferBytes[14] = CH_2_CYCLE_4;
  transferBytes[15] = PLACEHOLDER_5;
  transferBytes[16] = CH_1_CYCLE_5;
  transferBytes[17] = CH_2_CYCLE_5;
  transferBytes[18] = PLACEHOLDER_6;
  transferBytes[19] = CH_1_CYCLE_6;
  transferBytes[20] = CH_2_CYCLE_6;
  transferBytes[21] = PLACEHOLDER_7;
  transferBytes[22] = CH_1_CYCLE_7;
  transferBytes[23] = CH_2_CYCLE_7;
  transferBytes[24] = PLACEHOLDER_8;
  transferBytes[25] = CH_1_CYCLE_8;
  transferBytes[26] = CH_2_CYCLE_8;
  transferBytes[27] = PLACEHOLDER_9;
  transferBytes[28] = CH_1_CYCLE_9;
  transferBytes[29] = CH_2_CYCLE_9;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_PROXA_CYCLE0, 30, transferBytes, RESTART);
  LOG_INF("\t\t13. Write Cycle 0 - 9 Settings");

  /* Change the Allocation of channels into cycles 10-19 */
  /* Memory Map Position 0x6C - 0x7A */
  transferBytes[0] = PLACEHOLDER_10;
  transferBytes[1] = CH_1_CYCLE_10;
  transferBytes[2] = CH_2_CYCLE_10;
  transferBytes[3] = PLACEHOLDER_11;
  transferBytes[4] = CH_1_CYCLE_11;
  transferBytes[5] = CH_2_CYCLE_11;
  transferBytes[6] = PLACEHOLDER_12;
  transferBytes[7] = CH_1_CYCLE_12;
  transferBytes[8] = CH_2_CYCLE_12;
  transferBytes[9] = PLACEHOLDER_13;
  transferBytes[10] = CH_1_CYCLE_13;
  transferBytes[11] = CH_2_CYCLE_13;
  transferBytes[12] = PLACEHOLDER_14;
  transferBytes[13] = CH_1_CYCLE_14;
  transferBytes[14] = CH_2_CYCLE_14;
  transferBytes[15] = PLACEHOLDER_15;
  transferBytes[16] = CH_1_CYCLE_15;
  transferBytes[17] = CH_2_CYCLE_15;
  transferBytes[18] = PLACEHOLDER_16;
  transferBytes[19] = CH_1_CYCLE_16;
  transferBytes[20] = CH_2_CYCLE_16;
  transferBytes[21] = PLACEHOLDER_17;
  transferBytes[22] = CH_1_CYCLE_17;
  transferBytes[23] = CH_2_CYCLE_17;
  transferBytes[24] = PLACEHOLDER_18;
  transferBytes[25] = CH_1_CYCLE_18;
  transferBytes[26] = CH_2_CYCLE_18;
  transferBytes[27] = PLACEHOLDER_19;
  transferBytes[28] = CH_1_CYCLE_19;
  transferBytes[29] = CH_2_CYCLE_19;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_PROXA_CYCLE10, 30, transferBytes, RESTART);
  LOG_INF("\t\t14. Write Cycle 10 - 19 Settings");

  /* Change the Allocation of channels into cycles 20 */
  /* Memory Map Position 0x7B - 0x7C */
  transferBytes[0] = PLACEHOLDER_20;
  transferBytes[1] = CH_1_CYCLE_20;
  transferBytes[2] = CH_2_CYCLE_20;

  iqs7211e_writeRandomBytes(instance, IQS7211E_MM_PROXA_CYCLE20, 3, transferBytes, RESTART);
  LOG_INF("\t\t15. Write Cycle 20  Settings");
}

/*****************************************************************************/
/*                              PRIVATE METHODS                              */
/*****************************************************************************/

/**
 * @name    readRandomBytes
 * @brief   A method that reads a specified number of bytes from a specified 
 *          address and saves it into a user-supplied array. This method is used 
 *          by all other methods in this class which read data from the IQS7211E 
 *          device.
 * @param   memoryAddress -> The memory address from which to start reading bytes   
 *                           from. See the "iqs7211e_addresses.h" file.
 * @param   numBytes      -> The number of bytes that must be read.
 * @param   bytesArray    -> The array which will store the bytes to be read, 
 *                           his array will be overwritten.
 * @param   stopOrRestart -> A boolean that specifies whether the communication 
 *                           window should remain open or be closed after transfer.
 *                           False keeps it open, true closes it. Use the STOP 
 *                           and RESTART definitions.
 * @retval  No value is returned, however, the user-supplied array is overwritten.
 * @note    Uses standard Arduino "Wire" library which is for I2C communication.
 *          Take note that C++ cannot return an array, therefore, the array which 
 *          is passed as an argument is overwritten with the required values.
 *          Pass an array to the method by using only its name, e.g. "bytesArray", 
 *          without the brackets, this passes a pointer to the array.
 */
void iqs7211e_readRandomBytes(iqs7211e_instance_t *instance, uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart)
{
  uint8_t i = 0;  // A simple counter to assist with loading bytes into the user-supplied array.

  // Select the device with the address and start communication.
  wire_beginTransmission(&instance->wire, instance->_deviceAddress);
  // Send a bit asking for the "memoryAddress" register.
  wire_write(&instance->wire, memoryAddress);
  // Complete the selection and communication initialization.
  wire_endTransmission(&instance->wire);  // End transmission before reading

  // Request "numBytes" bytes from the device
  int result;
  do
  {
    result = wire_requestFrom(&instance->wire, instance->_deviceAddress, numBytes); 
  } while(result != 0);  // Wait for successful response

  // Load the received bytes into the array
  while(wire_available(&instance->wire) > 0 && i < numBytes)
  {
    // Load the received bytes into the user-supplied array
    bytesArray[i] = wire_read(&instance->wire);
    i++;
  }

  /* Always manually close the RDY window after a STOP is sent to prevent 
     writing while the RDY window closes */
  if(stopOrRestart == STOP)
  {
    iqs7211e_deviceRDY = false;
  }
}

/**
 * @name   writeRandomBytes
 * @brief  A method that writes a specified number of bytes to a specified 
 *         address, the bytes to write are supplied using an array pointer.
 *         This method is used by all other methods of this class which 
 *         write data to the IQS7211E device.
 * @param  memoryAddress -> The memory address at which to start writing the bytes 
 *                          to. See the "iqs7211e_addresses.h" file.
 * @param  numBytes      -> The number of bytes that must be written.
 * @param  bytesArray    -> The array which stores the bytes which will be 
 *                          written to the memory location.
 * @param  stopOrRestart -> A boolean that specifies whether the communication 
 *                          window should remain open or be closed of transfer.
 *                          False keeps it open, true closes it. Use the STOP 
 *                          and RESTART definitions.
 * @retval No value is returned, only the IQS device registers are altered.
 * @note   Uses standard Arduino "Wire" library which is for I2C communication.
 *         Take note that a full array cannot be passed to a function in C++.
 *         Pass an array to the function by using only its name, e.g. "bytesArray", 
 *         without the square brackets, this passes a pointer to the array. 
 *         The values to be written must be loaded into the array prior 
 *         to passing it to the function.
 */
void iqs7211e_writeRandomBytes(iqs7211e_instance_t *instance, uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart)
{
  /* Select the device with the address and start communication. */
  wire_beginTransmission(&instance->wire, instance->_deviceAddress);
  /* Specify the memory address where the IQS7211E must start saving the data, 
    as designated by the "memoryAddress" variable. */
  wire_write(&instance->wire, memoryAddress);
  /* Write the bytes as specified in the array. */
  for (int i = 0; i < numBytes; i++)
  {
    wire_write(&instance->wire, bytesArray[i]);
  }
  /* End the transmission, user decides to STOP or RESTART. */
  wire_endTransmission_stop(&instance->wire, stopOrRestart);

  /* Always manually close the RDY window after a STOP is sent to prevent 
    writing while the RDY window closes */
  if(stopOrRestart == STOP)
  {
    iqs7211e_deviceRDY = false;
  }
}

/**
 * @name   writeRandomBytes16
 * @brief  A method that writes a specified number of bytes to a specified 
 *         address, the bytes to write are supplied using an array pointer.
 *         This method is used by all other methods of this class which 
 *         write data to the IQS7211E device.
 * @param  memoryAddress -> The memory address at which to start writing the 
 *                          bytes to. See the "iqs7211e_addresses.h" file.
 * @param  numBytes      -> The number of bytes that must be written.
 * @param  bytesArray    -> The array which stores the bytes which will be 
 *                          written to the memory location.
 * @param  stopOrRestart -> A boolean that specifies whether the communication 
 *                          window should remain open or be closed of transfer.
 *                          False keeps it open, true closes it. Use the STOP 
 *                          and RESTART definitions.
 * @retval No value is returned, only the IQS device registers are altered.
 * @note   Uses standard Arduino "Wire" library which is for I2C communication.
 *         Take note that a full array cannot be passed to a function in C++.
 *         Pass an array to the function by using only its name, e.g. "bytesArray", 
 *         without the square brackets, this passes a pointer to the array. 
 *         The values to be written must be loaded into the array prior 
 *         to passing it to the function.
 */
static void iqs7211e_writeRandomBytes16(iqs7211e_instance_t *instance, uint16_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart)
{
  /* Select the device with the address and start communication. */
  wire_beginTransmission(&instance->wire, instance->_deviceAddress);
  /* Specify the memory address where the IQS7211E must start saving the data, 
    as designated by the "memoryAddress" variable. */
  uint8_t addr_h, addr_l;
  addr_h = memoryAddress >> 8;
  addr_l = memoryAddress;
  wire_write(&instance->wire, addr_h);
  wire_write(&instance->wire, addr_l);
  /* Write the bytes as specified in the array. */
  for (int i = 0; i < numBytes; i++)
  {
    wire_write(&instance->wire, bytesArray[i]);
  }
  /* End the transmission, and the user decides to STOP or RESTART. */
  wire_endTransmission_stop(&instance->wire, stopOrRestart);

  /* Always manually close the RDY window after a STOP is sent to prevent 
    writing while the RDY window closes */
  if(stopOrRestart == STOP)
  {
    iqs7211e_deviceRDY = false;
  }
}

/**
  * @name   iqs7211e_getBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval The boolean value of the specific bit requested. 
  */
bool iqs7211e_getBit(uint8_t data, uint8_t bit_number)
{
  return (data & ( 1 << bit_number )) >> bit_number;
}

/**
  * @name   iqs7211e_setBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval Returns an 8-bit unsigned integer value of the given data byte with 
  *         the requested bit set.
  */
uint8_t iqs7211e_setBit(uint8_t data, uint8_t bit_number)
{
	return (data |= 1UL << bit_number);
}

/**
  * @name   iqs7211e_clearBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval Returns an 8-bit unsigned integer value of the given data byte with 
  *         the requested bit cleared.
  */
uint8_t iqs7211e_clearBit(uint8_t data, uint8_t bit_number)
{
	return (data &= ~(1UL << bit_number));
}

/**
  * @name   iqs7211e_force_I2C_communication
  * @brief  A method which writes data 0x00 to memory address 0xFF to open a 
  *         communication window on the IQS7211E.
  * @param  instance -> Pointer to the IQS7211E instance
  * @retval None.
  * @note   Uses the wire wrapper for I2C communication.
  */
void iqs7211e_force_I2C_communication(iqs7211e_instance_t *instance)
{
  LOG_INF("Force I2C communication called - iqs7211e_deviceRDY=%s", iqs7211e_deviceRDY ? "TRUE" : "FALSE");
  
  /*Ensure RDY is HIGH at the moment*/
  if (!iqs7211e_deviceRDY)
  {
    LOG_INF("Executing force I2C communication (RDY is not ready)");
    
    /* Select the device with the address and start communication. */
    wire_beginTransmission(&instance->wire, instance->_deviceAddress);

    /* Write to memory address 0xFF that will prompt the IQS7211E to open a 
    communication window.*/
    wire_write(&instance->wire, 0xFF);
    wire_write(&instance->wire, 0x00);

    /* End the transmission, user decides to STOP or RESTART. */
    wire_endTransmission_stop(&instance->wire, true);  // Always STOP for force communication
    
    LOG_INF("Force I2C communication sent - 0xFF 0x00 to address 0x%02X", instance->_deviceAddress);
  } else {
    LOG_INF("Skipping force I2C communication (RDY is already ready)");
  }
}