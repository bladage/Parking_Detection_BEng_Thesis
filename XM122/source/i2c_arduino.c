/**
 * @brief Functions to communicate with an Arduino Uno R3 via I2C or TWI, respectively.  
 *
 * @author Bennet Ladage
 * @date 2022-09-11
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 */

#include "i2c_arduino.h"

// Using TWI_1 here, because TWI_0 uses the same base address as SPI_0 which is propably used by the A111 chip
#define TWI_INSTANCE_ID 1
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID); 

// A flag to indicate the transfer state
static volatile bool m_xfer_done = false;

/**************************************************************

                    Event / Interrupt Handler

**************************************************************/

void ard_twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    // if data transmission or receiving is finished
    if(p_event->type == NRF_DRV_TWI_EVT_DONE)
    {
      m_xfer_done = true; // reset the flag
    }
}


/**************************************************************

  Write Commands and Receive a Reply from the Arduino

**************************************************************/

bool ard_get_data(uint8_t * addr, uint16_t * dist, uint16_t * strength, int16_t * chip_temp)
{
    uint32_t cmnd = OBTAIN_DATA_FRAME;
    uint8_t reply[TFMP_REPLY_SIZE + 1] = {0};
    uint16_t chkSum = 0;       // calculate the check sum byte.
    uint8_t replyLen = 9;      // store reply data length
    //uint8_t cmndLen = 5;       // store command data length
    uint8_t cmndData[6] = {0x5A, 0x05, 0x00, 0x01, 0x60}; // store command data    
    
    int16_t dist_i = 0;
    int16_t strength_i = 0;
    int16_t chip_temp_i = 0;
    
    ret_code_t err_code;
    m_xfer_done = false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Send the command data array to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Transmit the bytes and a stop message to release the I2C bus
    err_code = nrf_drv_twi_tx(&m_twi, *addr, cmndData, cmndData[1], false); // stop condition is set
    while(m_xfer_done == false); //Wait for the transmission to get completed
    
    if(err_code != NRF_SUCCESS)
    {
        return TFMP_FAIL;
    }
    m_xfer_done = false; // reset the flag that new transmission is possible
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Get command reply data back from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    // Request reply data from the device and close the I2C interface.
    err_code = nrf_drv_twi_rx(&m_twi, *addr, reply, replyLen);  // Receive the data
    while(m_xfer_done == false);
	
    // if data was successfully read, return true else return false
    if(NRF_SUCCESS != err_code)
    {
        return TFMP_FAIL;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Perform a checksum test.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Add together all bytes but the last...
    chkSum = 0;
    for(uint8_t i = 0; i < (replyLen - 1); i++)
    {
      chkSum += reply[i];
    }
    // If the low order byte of the Sum does not equal the last byte...
    if(reply[replyLen - 1] != (uint8_t)chkSum)
    {
      return TFMP_CHECKSUM; // checksum error
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 4 - Reassamble the bytes
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    dist_i = reply[2] + (reply[ 3] << 8);
    strength_i = reply[4] + (reply[ 5] << 8);
    chip_temp_i = reply[6] + (reply[ 7] << 8);
    // Convert temp code to degrees Celsius.
    chip_temp_i = (chip_temp_i >> 3) - 256;
    
    // - - Evaluate Abnormal Data Values - -
    if(dist_i == -1) // Signal strength <= 100
    {
      dist_i = 0;
      strength_i = 0;
      return TFMP_WEAK;
    }
    else if(strength_i == -1) // Signal Strength saturation
    {
      dist_i = 0;
      strength_i = 0;
      return TFMP_STRONG;
    }
    else if(dist_i == -4) // Ambient Light saturation
    {
      dist_i = 0;
      strength_i = 0;
      return TFMP_FLOOD;
    }
    else // data ok
    {
      *dist = (uint16_t)dist_i;
      *strength = (uint16_t)strength_i;
      *chip_temp = (uint16_t)chip_temp_i;
    }   

    return TFMP_READY;  
}


/**************************************************************

  Functions to Handle the Sensor and its Communication via TWI

***************************************************************/

ret_code_t ard_twi_master_init(void)
{
  ret_code_t err_code;
  
  const nrf_drv_twi_config_t twi_config = {
    .scl                = ARD_SCL,               // SCL=23, XB122
    .sda                = ARD_SDA,               // SDA=21, XB122
    .frequency          = NRF_DRV_TWI_FREQ_100K,  // communication speed to 100k, (250k, 400k possible too)
    .interrupt_priority = APP_IRQ_PRIORITY_MID,   // interrupt priority, not high if soft-device is used
    .clear_bus_init     = false                   // automatic bus clearing 
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, ard_twi_handler, NULL); // initialize the twi
  APP_ERROR_CHECK(err_code);                                           // check if any error occured during initialization

  if(err_code == NRF_SUCCESS)
  {
    nrf_drv_twi_enable(&m_twi); // enable communication via TWI
    //while(nrf_drv_twi_is_busy(&m_twi)); // wait if TWI is busy
  }
  return err_code;
}

bool ard_init(uint8_t * addr, uint16_t * sampling_frequency)
{
  
  return true;
}