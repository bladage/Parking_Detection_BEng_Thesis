/**
 * @brief Functions to communicate with the LiDAR sensor TFmini Plus A05 via I2C or TWI, respectively.  
 *
 * @author Bennet Ladage
 * @date 2022-09-10
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 * The source code is strongly based on the code of Bud Ryerson: https://github.com/budryerson/TFMini-Plus-I2C
 * Many thanks for your work, Mr Ryerson!
 */

#include "tfmini_plus_uart.h"


/**************************************************************

                    Event / Interrupt Handler

**************************************************************/

void uart_error_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[TFMP_REPLY_SIZE + 1];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;
            NRF_LOG_DEBUG("[%d] = %x", index, data_array[index]);

            if (index == 9)
            {
              NRF_LOG_DEBUG("Ready to send data over BLE NUS");
              NRF_LOG_HEXDUMP_DEBUG(data_array, index);
              index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            NRF_LOG_ERROR("Communication error occurred while handling UART: %08X", p_event->data.error_communication );
            if( p_event->data.error_communication & UART_ERRORSRC_BREAK_Msk )
            {
                NRF_LOG_ERROR("   Break");
            }
            if( p_event->data.error_communication & UART_ERRORSRC_FRAMING_Msk )
            {
                NRF_LOG_ERROR("   Framing");
            }
            if( p_event->data.error_communication & UART_ERRORSRC_PARITY_Msk )
            {
                NRF_LOG_ERROR("   Parity");
            }
            if( p_event->data.error_communication & UART_ERRORSRC_OVERRUN_Msk )
            {
                NRF_LOG_ERROR("   Overrun");
            }
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }

    //if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) // not so fatal errors ... https://devzone.nordicsemi.com/f/nordic-q-a/40697/how-to-resolve-the-error-nrf_error_no_mem-at-the-case-app_uart_communication_error-in-uart_event_handle/160847#160847
    //{
    //    //APP_ERROR_HANDLER(p_event->data.error_communication);
    //    NRF_LOG_ERROR("Communication error occurred while handling UART: %08X", p_event->data.error_communication );
    //        if( p_event->data.error_communication & UART_ERRORSRC_BREAK_Msk )
    //        {
    //            NRF_LOG_ERROR("   Break");
    //        }
    //        if( p_event->data.error_communication & UART_ERRORSRC_FRAMING_Msk )
    //        {
    //            NRF_LOG_ERROR("   Framing");
    //        }
    //        if( p_event->data.error_communication & UART_ERRORSRC_PARITY_Msk )
    //        {
    //            NRF_LOG_ERROR("   Parity");
    //        }
    //        if( p_event->data.error_communication & UART_ERRORSRC_OVERRUN_Msk )
    //        {
    //            NRF_LOG_ERROR("   Overrun");
    //        }
    //}
    //else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    //{
    //    APP_ERROR_HANDLER(p_event->data.error_code);
    //}


}

/**************************************************************

  Write Commands and Receive a Reply from the Sensor

**************************************************************/

uint8_t tfmp_write_cmd(uint32_t cmnd, uint32_t param, uint8_t * version) // uint32_t cmnd, uint32_t param, uint8_t addr
{
    uint8_t reply[TFMP_REPLY_SIZE + 1] = {0};

    uint16_t chkSum = 0;       // calculate the check sum byte.
    uint8_t replyLen = 0;      // store reply data length
    uint8_t cmndLen = 0;       // store command data length
    uint8_t cmndData[TFMP_COMMAND_MAX] = {0}; // store command data    
    ret_code_t err_code;
    //m_xfer_done = false;

    memcpy(&cmndData[0], &cmnd, 4); // Copy 4 bytes of data: reply length, command length, command number and a one byte parameter, all encoded as a 32 bit unsigned integer.
    replyLen = cmndData[0];        // Save the first byte as reply length.
    cmndLen = cmndData[1];         // Save the second byte as command length.
    cmndData[0] = 0x5A;            // Set the first byte to the header character.
    
    if(cmnd == SET_FRAME_RATE)          // If the command is to Set Frame Rate...
    {
      memcpy(&cmndData[3], &param, 2);  // add the 2 byte Frame Rate parameter.
    }
    else if(cmnd == SET_BAUD_RATE)      // If the command is to Set Baud Rate...
    {
      memcpy(&cmndData[3], &param, 4);  // add the 3 byte Baud Rate parameter.
    }   

    // Create a checksum byte for the command data array.
    // Add together all bytes but the last...
    for(uint8_t i = 0; i < (cmndLen - 1); i++)
    {
      chkSum += cmndData[i];
    }
    // and save it as the last byte of command data.
    cmndData[cmndLen - 1] = (uint8_t)chkSum;
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Send the command data array to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Transmit the bytes and a stop message to release the I2C bus
    //err_code = nrf_drv_twi_tx(&m_twi, *addr, cmndData, cmndLen, false); // stop condition is set
    //while(m_xfer_done == false); //Wait for the transmission to get completed
    
    //if(err_code != NRF_SUCCESS)
    //{
    //    return TFMP_FAIL;
    //}
    //m_xfer_done = false; // reset the flag that new transmission is possible

    // If no reply data expected, then go home. Otherwise,
    // wait for device to process the command and continue.
    if(replyLen == 0)
    {
      return TFMP_READY;
    }
    else
    {
      nrf_delay_ms(500); // TODO change to 100ms as suggested in the data sheet
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Get command reply data back from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    // Request reply data from the device and close the I2C interface.
    //err_code = nrf_drv_twi_rx(&m_twi, *addr, reply, replyLen);  // Receive the data
    //while(m_xfer_done == false);
	
    // if data was successfully read, return true else return false
    if(NRF_SUCCESS != err_code)
    {
        return TFMP_FAIL;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 4 - Perform a checksum test.
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
    // Step 5 - Interpret different command responses.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if( cmnd == GET_FIRMWARE_VERSION && version != NULL)
    {
        version[0] = reply[5];  // set firmware version.
        version[1] = reply[4];
        version[2] = reply[3];
    }
    else
    {
        if( cmnd == SOFT_RESET ||
            cmnd == HARD_RESET ||
            cmnd == SAVE_SETTINGS )
        {
            if( reply[3] == 1)      // If PASS/FAIL byte not zero ...
            { 
                return TFMP_FAIL;   // set status `FAIL`...
            }
        }
    }
    return TFMP_READY;
}

uint8_t tfmp_cmd_reply(uint32_t cmnd, uint32_t param, uint8_t * reply) // uint32_t cmnd, uint32_t param, uint8_t addr
{
    //uint8_t reply[TFMP_REPLY_SIZE + 1] = {0};

    uint16_t chkSum = 0;       // calculate the check sum byte.
    uint8_t replyLen = 0;      // store reply data length
    uint8_t cmndLen = 0;       // store command data length
    uint8_t cmndData[TFMP_COMMAND_MAX] = {0}; // store command data    
    ret_code_t err_code;
    //m_xfer_done = false;

    memcpy(&cmndData[0], &cmnd, 4); // Copy 4 bytes of data: reply length, command length, command number and a one byte parameter, all encoded as a 32 bit unsigned integer.
    replyLen = cmndData[0];        // Save the first byte as reply length.
    cmndLen = cmndData[1];         // Save the second byte as command length.
    cmndData[0] = 0x5A;            // Set the first byte to the header character.
    
    if(cmnd == SET_FRAME_RATE)          // If the command is to Set Frame Rate...
    {
      memcpy(&cmndData[3], &param, 2);  // add the 2 byte Frame Rate parameter.
    }
    else if(cmnd == SET_BAUD_RATE)      // If the command is to Set Baud Rate...
    {
      memcpy(&cmndData[3], &param, 4);  // add the 3 byte Baud Rate parameter.
    }

    // Create a checksum byte for the command data array.
    // Add together all bytes but the last...
    for(uint8_t i = 0; i < (cmndLen - 1); i++)
    {
      chkSum += cmndData[i];
    }
    // and save it as the last byte of command data.
    cmndData[cmndLen - 1] = (uint8_t)chkSum;
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Send the command data array to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Transmit the bytes and a stop message to release the I2C bus
    //err_code = nrf_drv_twi_tx(&m_twi, *addr, cmndData, cmndLen, false); // stop condition is set
    //while(m_xfer_done == false); //Wait for the transmission to get completed
    
    //if(err_code != NRF_SUCCESS)
    //{
    //    return TFMP_FAIL;
    //}
    //m_xfer_done = false; // reset the flag that new transmission is possible

    // If no reply data expected, then go home. Otherwise,
    // wait for device to process the command and continue.
    if(replyLen == 0)
    {
      return TFMP_READY;
    }
    else if(cmnd != OBTAIN_DATA_FRAME)
    {
      nrf_delay_ms(500); // TODO change to 100ms as suggested in the data sheet
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Get command reply data back from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    // Request reply data from the device and close the I2C interface.
    //err_code = nrf_drv_twi_rx(&m_twi, *addr, reply, replyLen);  // Receive the data
    //while(m_xfer_done == false);
	
    //// if data was successfully read, return true else return false
    //if(NRF_SUCCESS != err_code)
    //{
    //    return TFMP_FAIL;
    //}

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 4 - Perform a checksum test.
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

    // see if something went wrong ...
    if( cmnd == SOFT_RESET ||
        cmnd == HARD_RESET ||
        cmnd == SAVE_SETTINGS )
    {
      if( reply[3] == 1)    // If PASS/FAIL byte not zero ...
      { 
        return TFMP_FAIL;   // set status `FAIL`...
      }
    }
    return TFMP_READY;
}

uint8_t tfmp_get_data(uint16_t * dist, uint16_t * strength, int16_t * chip_temp)
{
    uint32_t cmnd = OBTAIN_DATA_FRAME;
    uint8_t reply[TFMP_REPLY_SIZE + 1] = {0};
    uint16_t chkSum = 0;       // calculate the check sum byte.
    uint8_t replyLen = 9;      // store reply data length
    //uint8_t cmndLen = 5;       // store command data length
    uint8_t cmndData[6] = {0x5A, 0x05, 0x00, 0x01, 0x60}; // store command data    
    uint8_t temp_v = 0;

    int16_t dist_i = 0;
    int16_t strength_i = 0;
    int16_t chip_temp_i = 0;
    
    ret_code_t err_code;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Send the command data array to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Transmit the bytes and a stop message to release the I2C bus
    //err_code = nrf_drv_twi_tx(&m_twi, *addr, cmndData, cmndData[1], false); // stop condition is set
    //while(m_xfer_done == false); //Wait for the transmission to get completed
    
    //if(err_code != NRF_SUCCESS)
    //{
    //    return TFMP_FAIL;
    //}
    //m_xfer_done = false; // reset the flag that new transmission is possible
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Get command reply data back from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    // Request reply data from the device and close the I2C interface.
    //err_code = nrf_drv_twi_rx(&m_twi, *addr, reply, replyLen);  // Receive the data
    //while(m_xfer_done == false);
	
    // if data was successfully read, return true else return false
    //if(NRF_SUCCESS != err_code)
    //{
    //    return TFMP_FAIL;
    //}
    
    while(true) 
    {
      //app_uart_get(&temp_v); // wait until byte was read successfully; idx=0 for the first iteration
      while(app_uart_get(&temp_v) != NRF_SUCCESS);

      if(temp_v == 0x59) // check if frame is data frame
      {
        temp_v = 0;
        while(app_uart_get(&temp_v) != NRF_SUCCESS);
        //app_uart_get(&temp_v);
        //while(app_uart_get(&reply[1]) != NRF_SUCCESS); // check if second byte has stored the magical number 0x59 too
        if(temp_v == 0x59)
        {
          reply[0] = 0x59; // yes, it's a data frame, now save the rest
          reply[1] = 0x59;

          for(uint8_t k = 2; k < replyLen; k++)
          {
            app_uart_get(&temp_v);
            reply[k] = temp_v;
          }
          break;
        }
      }
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


uint8_t tfmp_init(uint16_t * sampling_frequency)
{
  uint8_t tfmp_err_code = TFMP_READY;
  ret_code_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          UART_RX,
          UART_TX,
          UART_RTS,
          UART_CTS,
          UART_HWFC,
          false,
          NRF_UART_BAUDRATE_115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_HIGH,
                         err_code);
    APP_ERROR_CHECK(err_code);

    //NRF_GPIO->PIN_CNF[UART_RX] |= 3<<2; 
    //nrf_gpio_cfg_input(UART_RX, NRF_GPIO_PIN_PULLUP); // enable pull up rx pin

  if(err_code == NRF_SUCCESS)
  {
  //  tfmp_err_code = tfmp_write_cmd(SOFT_RESET, 0, addr, NULL); // perform a soft reset
  //  if(tfmp_err_code == TFMP_READY)
  //  {
  //    // The condition sampling_freq = 1000 Hz / n should hold where n is a positive integer.
  //    tfmp_err_code = tfmp_write_cmd(SET_FRAME_RATE, *sampling_frequency, addr, NULL);
  //    if(tfmp_err_code == TFMP_READY)
  //    {        
  //      tfmp_err_code = tfmp_write_cmd(SAVE_SETTINGS, 0, addr, NULL); // Save settings
  //    }
  //  }
  }
  else
  {
    return TFMP_TIMEOUT;
  }
  return tfmp_err_code;
}