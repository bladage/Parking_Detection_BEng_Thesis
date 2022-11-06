/**
 * @brief Functions to communicate with the magnetic sensor via I2C or TWI, respectively.
 *        The magnetic sensor is the MMC5983MA which has got 3 axis.  
 *
 * @author Bennet Ladage
 * @date 2022-08-03
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 * The source code is partially based on this code: https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library
 */

#include "memsic_mmc5983ma.h"

/**************************************************************

  Help Functions, i.e., write/read registers, is a bit set? ...

**************************************************************/

bool mmc_read_register(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t register_address, uint8_t * rx_data, uint8_t number_of_bytes)
{
    ret_code_t err_code;
    *m_xfer_done = false;
    
    err_code = nrf_drv_twi_tx(m_twi, MMC_I2C_ADDRESS, &register_address, 1, true); // send reg address, no stop condition
    while(*m_xfer_done == false); //Wait for the transmission to get completed
    
    if(NRF_SUCCESS != err_code)
    {
        return false;
    }
    *m_xfer_done = false; // reset the flag that new transmission is possible
	  
    err_code = nrf_drv_twi_rx(m_twi, MMC_I2C_ADDRESS, rx_data, number_of_bytes);  // Receive the data
    while(*m_xfer_done == false);
	
    // if data was successfully read, return true else return false
    if(NRF_SUCCESS != err_code)
    {
        return false;
    }
    return true;  
}

bool mmc_write_register(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t register_address, uint8_t tx_data)
{
    ret_code_t err_code;
    *m_xfer_done = false;
    
    // I2C "protocol style": 1. register address 2. data
    uint8_t tx_buffer[2];
    tx_buffer[0] = register_address;
    tx_buffer[1] = tx_data;
    
    err_code = nrf_drv_twi_tx(m_twi, MMC_I2C_ADDRESS, tx_buffer, 2, false); // stop condition is set
    while(*m_xfer_done == false); //Wait for the transmission to get completed
    
    if(err_code != NRF_SUCCESS)
    {
        return false;
    }
    *m_xfer_done = false; // reset the flag that new transmission is possible
    return true;
}

void mmc_set_register_bit(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask)
{
  uint8_t reg = 0x00;
  if(mmc_read_register(m_twi, m_xfer_done, register_address, &reg, 1))
  { 
    reg |= bit_mask; // OR --> set bit
    mmc_write_register(m_twi, m_xfer_done, register_address, reg);
  }
}

void mmc_clear_register_bit(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask)
{
  uint8_t reg = 0x00;
  if(mmc_read_register(m_twi, m_xfer_done, register_address, &reg, 1))
  { 
    reg &= ~bit_mask; // AND, ~ is NOT --> clear bit
    mmc_write_register(m_twi, m_xfer_done, register_address, reg);
  }
}

bool mmc_is_bit_set(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask)
{
  uint8_t reg = 0x00;
  if(mmc_read_register(m_twi, m_xfer_done, register_address, &reg, 1))
  { 
    if((reg & bit_mask) == 0) // AND
    {
      return false;
    }
    return true;
  }
}

void mmc_set_register_bit_wro(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask, uint8_t * backup_reg)
{
  uint8_t reg = 0x00;
  if(backup_reg != NULL)
  {
    reg = *backup_reg; // take the backup and store that temporarily
    reg |= bit_mask;  // AND, ~ is NOT --> clear the bit
    *backup_reg = reg; // the register byte is saved to remember which bits were changed. Thus, backup_reg is a pseudo register  
  }
  else
  {
    reg |= bit_mask; // AND, ~ is NOT --> clear the bit
  }
  mmc_write_register(m_twi, m_xfer_done, register_address, reg);
}

void mmc_clear_register_bit_wro(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask, uint8_t * backup_reg)
{
  uint8_t reg = 0x00;
  if(backup_reg != NULL)
  {
    reg = *backup_reg; // take the backup and store that temporarily
    reg &= ~bit_mask;  // AND, ~ is NOT --> clear the bit
    *backup_reg = reg; // the register byte is saved to remember which bits were changed. Thus, backup_reg is a pseudo register  
  }
  else
  {
    reg &= ~bit_mask; // AND, ~ is NOT --> clear the bit
  }
  mmc_write_register(m_twi, m_xfer_done, register_address, reg);
}

bool mmc_is_bit_set_backup_reg(uint8_t * backup_reg, const uint8_t bit_mask)
{
  if((*backup_reg & bit_mask) == 0)
  {
    return false;
  }
  return true;
}


/**************************************************************

  Functions to Handle the Sensor and its Communication via TWI

***************************************************************/

bool mmc_is_connected(nrf_drv_twi_t const * m_twi, bool * m_xfer_done)
{
  uint8_t rx_byte;
  if(mmc_read_register(m_twi, m_xfer_done, MMC_PRODUCT_ID, &rx_byte, 1))
  {
    if(rx_byte == MMC_I2C_ADDRESS) // the product ID = I2C address
      return true; 
  }
  return false;
}

float mmc_get_die_temperature(nrf_drv_twi_t const * m_twi, bool * m_xfer_done)
{
  uint8_t temp_reg = 0;
  float temp = -1.0f;

  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, TM_T, NULL); // start measurement, TM_T clears itself
  do
  {
    //nrf_delay_ms(5); // prevent too much requests
  } while(mmc_is_bit_set(m_twi, m_xfer_done, MMC_STATUS, MEAS_T_DONE) != 1); // wait until measurement is done
  
  if(mmc_read_register(m_twi, m_xfer_done, MMC_TOUT, &temp_reg, 1))
  {
    temp = -75.0f + ((float)temp_reg * (200.0f / 255.0f));
  }
  return temp;
}

void mmc_soft_reset(nrf_drv_twi_t const * m_twi, bool * m_xfer_done)
{
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, SW_RST, NULL);
  nrf_delay_ms(15); // power on time = 10 ms, so 5 ms as backup
}

/*
  Interrupt on/off functions, nt
*/
void mmc_enable_interrupt_pin(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0)
{
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, INT_MEAS_DONE_EN, backup_internal_ctrl_0);
}

void mmc_disable_interrupt_pin(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0)
{
  mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, INT_MEAS_DONE_EN, backup_internal_ctrl_0);
}

/*
  SET/RESET operation functions
*/
void mmc_execute_set_operation(nrf_drv_twi_t const * m_twi, bool * m_xfer_done)
{
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, SET_OPERATION, NULL); // SET bit clears itself
  nrf_delay_ms(1); // time of the current flow = 0.5 ms, so +0.5 ms as backup
}

void mmc_execute_reset_operation(nrf_drv_twi_t const * m_twi, bool * m_xfer_done)
{
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, RESET_OPERATION, NULL); // RESET bit clears itself
  nrf_delay_ms(1); // time of the current flow = 0.5 ms, so +0.5 ms as backup  
}

void mmc_enable_automatic_set_reset_operation(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0)
{
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, AUTO_SR_EN, backup_internal_ctrl_0);
}

void mmc_disable_automatic_set_reset_operation(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0)
{
  mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, AUTO_SR_EN, backup_internal_ctrl_0);
}

bool mmc_is_automatic_set_reset_enabled(uint8_t * backup_internal_ctrl_0)
{
  return mmc_is_bit_set_backup_reg(backup_internal_ctrl_0, AUTO_SR_EN);
}

/*
  Channel X
*/
void mmc_enable_x_channel(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1)
{
  mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, X_INHIBIT, backup_internal_ctrl_1); // on if X_INHIBIT=0
}

void mmc_disable_x_channel(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1)
{
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, X_INHIBIT, backup_internal_ctrl_1); // off if X_INHIBIT=1
}

bool mmc_is_x_channel_enabled(uint8_t * backup_internal_ctrl_1)
{
  return !mmc_is_bit_set_backup_reg(backup_internal_ctrl_1, X_INHIBIT); // on=0, off=1
}

/*
  Channel YZ
*/
void mmc_enable_yz_channels(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1)
{
  mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, YZ_INHIBIT, backup_internal_ctrl_1); // on if YZ_INHIBIT=0
}

void mmc_disable_yz_channels(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1)
{
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, YZ_INHIBIT, backup_internal_ctrl_1); // off if YZ_INHIBIT=1
}

bool mmc_are_yz_channels_enabled(uint8_t * backup_internal_ctrl_1)
{
  return !mmc_is_bit_set_backup_reg(backup_internal_ctrl_1, YZ_INHIBIT); // on=0, off=1
}

/*
  Decimation Filter
*/

void mmc_set_filter_bandwidth(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint16_t bandwidth, uint8_t * backup_internal_ctrl_1)
{
  switch(bandwidth)
  {
    case 200:
      // 200 Hz = 4 ms --> BW1=0, BW0=1
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, BW1, backup_internal_ctrl_1);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, BW0, backup_internal_ctrl_1);
    break;

    case 400:
      // 400 Hz = 2 ms --> BW1=1, BW0=0
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, BW1, backup_internal_ctrl_1);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, BW0, backup_internal_ctrl_1);
    break;

    case 800:
      // 800 Hz = 0.5 ms --> BW1=1, BW0=1
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, BW1, backup_internal_ctrl_1);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, BW0, backup_internal_ctrl_1);
    break;
    
    default:
      // 100 Hz = 8 ms --> BW1=0, BW0=0
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, BW1, backup_internal_ctrl_1);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_1, BW0, backup_internal_ctrl_1);
    break;
  }
}

uint16_t mmc_get_filter_bandwidth(uint8_t * backup_internal_ctrl_1)
{
  uint16_t bandwidth = 100;
  if(!mmc_is_bit_set_backup_reg(backup_internal_ctrl_1, BW1) 
     && mmc_is_bit_set_backup_reg(backup_internal_ctrl_1, BW0))
  {
    bandwidth = 200;
  }
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_1, BW1) 
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_1, BW0))
  {
    bandwidth = 400;
  }
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_1, BW1) 
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_1, BW0))
  {
    bandwidth = 800;
  }
  return bandwidth;
}


/*
  Continuous Measurement Mode
*/
bool mmc_enable_cont_mode(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1, uint8_t * backup_internal_ctrl_2)
{
  if(mmc_get_filter_bandwidth(backup_internal_ctrl_1) == 100) // the continuous mode need 100 Hz filter bandwidth
  {
     // CM_Frequ[2:0] cannot be 0b000! Thus, 1 Hz = 0b001 is default frequency
    mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_2, backup_internal_ctrl_2);
    mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_1, backup_internal_ctrl_2);
    mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_0, backup_internal_ctrl_2);
  
    // Enable the continuous measurement mode
    mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CMM_EN, backup_internal_ctrl_2);
    return true;
  }
  return false;
}

bool mmc_is_cont_mode_enabled(uint8_t * backup_internal_ctrl_2)
{
  return mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CMM_EN); // on=1, off=0
}

void mmc_set_frequency_cont_mode(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint16_t freq, uint8_t * backup_internal_ctrl_1, uint8_t * backup_internal_ctrl_2)
{
  mmc_set_filter_bandwidth(m_twi, m_xfer_done, 100, backup_internal_ctrl_1); // the most frequencies needs 100 Hz bandwidth of the decimation filter

  switch(freq)
  {
    case 10: // = 010 [CM_FREQ_2 CM_FREQ_1 CM_FREQ_0]
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_2, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_1, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_0, backup_internal_ctrl_2);
    break;

    case 20: // = 011
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_2, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_1, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_0, backup_internal_ctrl_2);
    break;

    case 50: // = 100
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_2, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_1, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_0, backup_internal_ctrl_2);
    break;

    case 100: // = 101
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_2, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_1, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_0, backup_internal_ctrl_2);
    break;

    case 200: // = 110
      mmc_set_filter_bandwidth(m_twi, m_xfer_done, 200, backup_internal_ctrl_1); // BW=01 is required for this, thus 200 Hz is set

      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_2, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_1, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_0, backup_internal_ctrl_2);
    break;

    case 1000: // = 111
      mmc_set_filter_bandwidth(m_twi, m_xfer_done, 800, backup_internal_ctrl_1); // BW=11 is required for this, thus 800 Hz is set

      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_2, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_1, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_0, backup_internal_ctrl_2);
    break;

    default: // 1 Hz = 001
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_2, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_1, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CM_FREQ_0, backup_internal_ctrl_2);
    break;
  }
}

uint16_t mmc_get_frequency_cont_mode(uint8_t * backup_internal_ctrl_2)
{
  uint16_t freq = 1; // = 001

  if(!mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_2) 
     && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_1)
     && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_0))
  {
    freq = 10; // = 010 [CM_FREQ_2 CM_FREQ_1 CM_FREQ_0]
  }
  else if(!mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_2) 
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_1)
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_0))
  {
    freq = 20; // = 011
  } 
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_2) 
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_1)
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_0))
  {
    freq = 50; // = 100
  } 
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_2) 
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_1)
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_0))
  {
    freq = 100; // = 101
  } 
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_2) 
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_1)
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_0))
  {
    freq = 200; // = 110
  } 
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_2) 
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_1)
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_0))
  {
    freq = 1000; // = 111
  }
  else if(!mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_2) 
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_1)
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CM_FREQ_0))
  {
    freq = 0; // = 000, this is not allowed for the continuous mode, i.e., the mode is off
  }    
  return freq;
}


/*
  Periodic SET operations to remove the effects of a strong external magnetic field, i.e., H_ext > 10 Gauss
*/

void mmc_enable_pso(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0, uint8_t * backup_internal_ctrl_2)
{
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, AUTO_SR_EN, backup_internal_ctrl_0); // 1=on
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CMM_EN, backup_internal_ctrl_2);     // 1=on
  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, EN_PRD_SET, backup_internal_ctrl_2); // 1=on
}

void mmc_disable_pso(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0, uint8_t * backup_internal_ctrl_2)
{
  mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, AUTO_SR_EN, backup_internal_ctrl_0); // 0=off
  mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, CMM_EN, backup_internal_ctrl_2);     // 0=off
  mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, EN_PRD_SET, backup_internal_ctrl_2); // 0=off
}

bool mmc_is_pso_enabled(uint8_t * backup_internal_ctrl_0, uint8_t * backup_internal_ctrl_2)
{
  if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_0, AUTO_SR_EN) 
     && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, CMM_EN)
     && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, EN_PRD_SET))
  {
    return true;
  }
  return false;
}

void mmc_set_repetition_number_pso(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint16_t rep_number, uint8_t * backup_internal_ctrl_2)
{
  switch(rep_number)
  {
    case 25: // = 001 [PRD_SET_2 PRD_SET_1 PRD_SET_0]
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_2, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_1, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_0, backup_internal_ctrl_2);
    break;

    case 75: // = 010
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_2, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_1, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_0, backup_internal_ctrl_2);
    break;

    case 100: // = 011
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_2, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_1, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_0, backup_internal_ctrl_2);
    break;
    
    case 250: // = 100
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_2, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_1, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_0, backup_internal_ctrl_2);
    break;

    case 500: // = 101
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_2, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_1, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_0, backup_internal_ctrl_2);
    break;

    case 1000: // = 110
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_2, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_1, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_0, backup_internal_ctrl_2);
    break;

    case 2000: // = 111
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_2, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_1, backup_internal_ctrl_2);
      mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_0, backup_internal_ctrl_2);
    break;

    default: // after 1 measurement = 000 [PRD_SET_2 PRD_SET_1 PRD_SET_0]
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_2, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_1, backup_internal_ctrl_2);
      mmc_clear_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_2, PRD_SET_0, backup_internal_ctrl_2);
    break;
  }
}

uint16_t mmc_get_repetition_number_pso(uint8_t * backup_internal_ctrl_2)
{
  uint16_t rep_number = 25; // = 001

  if(!mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_2) 
     && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_1)
     && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_0))
  {
    rep_number = 75; // = 010 [PRD_SET_2 PRD_SET_1 PRD_SET_0]
  }
  else if(!mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_2) 
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_1)
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_0))
  {
    rep_number = 100; // = 011
  } 
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_2) 
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_1)
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_0))
  {
    rep_number = 250; // = 100
  } 
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_2) 
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_1)
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_0))
  {
    rep_number = 500; // = 101
  } 
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_2) 
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_1)
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_0))
  {
    rep_number = 1000; // = 110
  } 
  else if(mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_2) 
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_1)
          && mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_0))
  {
    rep_number = 2000; // = 111
  }
  else if(!mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_2) 
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_1)
          && !mmc_is_bit_set_backup_reg(backup_internal_ctrl_2, PRD_SET_0))
  {
    rep_number = 1; // = 000
  }    
  return rep_number;
}

/*
  Get magnetic field measurements
*/

uint32_t mmc_get_magnetic_field_x(nrf_drv_twi_t const * m_twi, bool * m_xfer_done)
{
  uint32_t measurement = 0;
  uint32_t temp = 0;
  uint8_t rx_buffer[7] = {0};

  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, TM_M, NULL); // start measurement, TM_M clears itself
  do
  {
    nrf_delay_ms(5); // prevent too much requests
  } while(mmc_is_bit_set(m_twi, m_xfer_done, MMC_STATUS, MEAS_M_DONE) != 1); // wait until measurement is done
  
  if(mmc_read_register(m_twi, m_xfer_done, MMC_XOUT_0, rx_buffer, 7)) // cycle 12 to 18 (7 bytes) of the data sheet "example measurement"
  {
    temp = (uint32_t)(rx_buffer[MMC_XOUT_0]);
    temp = temp << XYZ_0_SHIFT;
    measurement |= temp;

    temp = (uint32_t)(rx_buffer[MMC_XOUT_1]);
    temp = temp << XYZ_1_SHIFT;
    measurement |= temp;

    temp = (uint32_t)(rx_buffer[MMC_XYZOUT_2]);
    temp &= X2_MASK;
    temp = temp >> 6;
    measurement |= temp;    
  }

  return measurement;  
}

uint32_t mmc_get_magnetic_field_y(nrf_drv_twi_t const * m_twi, bool * m_xfer_done)
{
  uint32_t measurement = 0;
  uint32_t temp = 0;
  uint8_t rx_buffer = 0;

  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, TM_M, NULL); // start measurement, TM_M clears itself
  do
  {
    nrf_delay_ms(5); // prevent too much requests
  } while(mmc_is_bit_set(m_twi, m_xfer_done, MMC_STATUS, MEAS_M_DONE) != 1); // wait until measurement is done
  
  if(mmc_read_register(m_twi, m_xfer_done, MMC_YOUT_0, &rx_buffer, 1)) // if this register is readable, the other should be readable too, i.e., the sensor is connected
  {
    temp = (uint32_t)rx_buffer;
    temp = temp << XYZ_0_SHIFT;
    measurement |= temp;
    
    mmc_read_register(m_twi, m_xfer_done, MMC_YOUT_1, &rx_buffer, 1); // read Yout1 register
    temp = (uint32_t)rx_buffer;
    temp = temp << XYZ_1_SHIFT;
    measurement |= temp;
    
    mmc_read_register(m_twi, m_xfer_done, MMC_XYZOUT_2, &rx_buffer, 1); // read XYZout2 register
    temp = (uint32_t)rx_buffer;   
    temp &= Y2_MASK;
    temp = temp >> 4;
    measurement |= temp;              
  }
  return measurement;   
}

uint32_t mmc_get_magnetic_field_z(nrf_drv_twi_t const * m_twi, bool * m_xfer_done)
{
  uint32_t measurement = 0;
  uint32_t temp = 0;
  uint8_t rx_buffer = 0;

  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, TM_M, NULL); // start measurement, TM_M clears itself
  do
  {
    nrf_delay_ms(5); // prevent too much requests
  } while(mmc_is_bit_set(m_twi, m_xfer_done, MMC_STATUS, MEAS_M_DONE) != 1); // wait until measurement is done
  
  if(mmc_read_register(m_twi, m_xfer_done, MMC_ZOUT_0, &rx_buffer, 1)) // if this register is readable, the other should be readable too, i.e., the sensor is connected
  {
    temp = (uint32_t)rx_buffer;
    temp = temp << XYZ_0_SHIFT;
    measurement |= temp;
    
    mmc_read_register(m_twi, m_xfer_done, MMC_ZOUT_1, &rx_buffer, 1); // read Zout1 register
    temp = (uint32_t)rx_buffer;
    temp = temp << XYZ_1_SHIFT;
    measurement |= temp;
    
    mmc_read_register(m_twi, m_xfer_done, MMC_XYZOUT_2, &rx_buffer, 1); // read XYZout2 register
    temp = (uint32_t)rx_buffer;   
    temp &= Z2_MASK;
    temp = temp >> 2;
    measurement |= temp;              
  }
  return measurement; 
}

void mmc_get_magnetic_field_xyz(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint32_t * x, uint32_t * y, uint32_t * z)
{
  uint32_t measurement = 0;
  uint32_t temp = 0;
  uint8_t rx_buffer[7] = {0};

  mmc_set_register_bit_wro(m_twi, m_xfer_done, MMC_INTERNAL_CONTROL_0, TM_M, NULL); // start measurement, TM_M clears itself
  do
  {
    //nrf_delay_ms(5); // prevent too much requests
  } while(mmc_is_bit_set(m_twi, m_xfer_done, MMC_STATUS, MEAS_M_DONE) != 1); // wait until measurement is done
  
  if(mmc_read_register(m_twi, m_xfer_done, MMC_XOUT_0, rx_buffer, 7)) // cycle 12 to 18 (7 bytes) of the data sheet "example measurement"
  {
    /*
      Extract the value of the x-axis
    */
    temp = (uint32_t)(rx_buffer[MMC_XOUT_0]);
    temp = temp << XYZ_0_SHIFT;
    measurement |= temp;

    temp = (uint32_t)(rx_buffer[MMC_XOUT_1]);
    temp = temp << XYZ_1_SHIFT;
    measurement |= temp;

    temp = (uint32_t)(rx_buffer[MMC_XYZOUT_2]);
    temp &= X2_MASK;
    temp = temp >> 6;
    measurement |= temp;
    
    *x = measurement; // save magnetic field of the x-axis
    measurement = 0;

    /*
      Extract the value of the y-axis
    */
    temp = (uint32_t)(rx_buffer[MMC_YOUT_0]);
    temp = temp << XYZ_0_SHIFT;
    measurement |= temp;
    
    temp = (uint32_t)(rx_buffer[MMC_YOUT_1]);
    temp = temp << XYZ_1_SHIFT;
    measurement |= temp;
    
    temp = (uint32_t)(rx_buffer[MMC_XYZOUT_2]);   
    temp &= Y2_MASK;
    temp = temp >> 4;
    measurement |= temp;  

    *y = measurement; // save magnetic field of the y-axis
    measurement = 0;
    
    /*
      Extract the value of the z-axis
    */
    temp = (uint32_t)(rx_buffer[MMC_ZOUT_0]);
    temp = temp << XYZ_0_SHIFT;
    measurement |= temp;
    
    temp = (uint32_t)(rx_buffer[MMC_ZOUT_1]);
    temp = temp << XYZ_1_SHIFT;
    measurement |= temp;
    
    temp = (uint32_t)(rx_buffer[MMC_XYZOUT_2]);    
    temp &= Z2_MASK;
    temp = temp >> 2;
    measurement |= temp; 

    *z = measurement; // save magnetic field of the z-axis
  }  
}

void mmc_get_magnetic_field_xyz_gauss(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * x, float * y, float * z)
{
  uint32_t x32 = 0;
  uint32_t y32 = 0;
  uint32_t z32 = 0;
  mmc_get_magnetic_field_xyz(m_twi, m_xfer_done, &x32, &y32, &z32);
  *x = (float)(x32) / MMC_COUNTS_PER_GAUSS_18_BIT_FL;
  *y = (float)(y32) / MMC_COUNTS_PER_GAUSS_18_BIT_FL;
  *z = (float)(y32) / MMC_COUNTS_PER_GAUSS_18_BIT_FL;
}

void mmc_get_magnetic_field_xyz_milli_tesla(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * x, float * y, float * z)
{
  uint32_t x32 = 0;
  uint32_t y32 = 0;
  uint32_t z32 = 0;
  mmc_get_magnetic_field_xyz(m_twi, m_xfer_done, &x32, &y32, &z32);
  *x = (float)(x32) / (MMC_COUNTS_PER_GAUSS_18_BIT_FL*10.0); // 1 Gauss = 0.1 mT
  *y = (float)(y32) / (MMC_COUNTS_PER_GAUSS_18_BIT_FL*10.0);
  *z = (float)(y32) / (MMC_COUNTS_PER_GAUSS_18_BIT_FL*10.0);
}

void mmc_convert_samples_to_gauss(uint32_t * samples, uint16_t length, float * gauss_signal)
{
  for(uint16_t k = 0; k < length; k++)
  {
    gauss_signal[k] = (float)(samples[k]) / MMC_COUNTS_PER_GAUSS_18_BIT_FL;
  }
}

void mmc_convert_samples_to_milli_tesla(uint32_t * samples, uint16_t length, float * mT_signal)
{
  for(uint16_t k = 0; k < length; k++)
  {
    mT_signal[k] = (float)(samples[k]) / (MMC_COUNTS_PER_GAUSS_18_BIT_FL*10.0); // 1 Gauss = 0.1 mT
  }
}

void mmc_get_bridge_offset(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint32_t * offset_xyz, uint32_t * magnetic_field_xyz)
{
  uint32_t output_1[3] = {0}; // [0]=x, [1]=y, [2]=z  
  uint32_t output_2[3] = {0};

  // 1) set internal magnetization in the direction of the SET field  
  mmc_execute_set_operation(m_twi, m_xfer_done); 
  
  // 2) perform a measurement
  mmc_get_magnetic_field_xyz(m_twi, m_xfer_done, &output_1[0], &output_1[1], &output_1[2]); 
  
  // 3) set internal magnetization in the direction of the RESET field which is the opposite to the SET field (180°)
  mmc_execute_reset_operation(m_twi, m_xfer_done); 

  // 4) perform a measurement
  mmc_get_magnetic_field_xyz(m_twi, m_xfer_done, &output_2[0], &output_2[1], &output_2[2]); 

  // 5) calculate the offset for all directions and all magnetic fields without the offset
  for(uint8_t k = 0; k < 3; k++)
  {
    /*
      Derivation of the equations:
      O:=offset,  out0:=output_0, out1:=output_1,  B:=magnetic field

      out0 = B + O    This is measured after the SET operation
      out1 = -B + O   This is measured after the RESET operation whose magnetic field is opposite to the SET field. 
                      Thus, -B holds if B is the magnetic field measured after the SET operation

      This leads to the offset and the magnetic field without the offset:
      
      [out0 + out1] / 2 = [(B+O) + (-B+O)] / 2 = [B+O-B+O] / 2 = 2O / 2 = O --> offset equation
      [out0 - out1] / 2 = [(B+O) - (-B+O)] / 2 = [B+O+B-O] / 2 = 2 B/ 2 = B --> magnetic field without offset equation
    */
    offset_xyz[k] = (output_1[k] + output_2[k]) / 2; // offset equation

    if(output_1[k] > output_2[k]) // unsigned hence positive difference only;  magnetic field without offset
    {
      magnetic_field_xyz[k] = (output_1[k] - output_2[k]) / 2;
    }
    else
    {
      magnetic_field_xyz[k] = (output_2[k] - output_1[k]) / 2;
    }
  }
  // 6) SET operation again that further measurements are based on the direction of the SET field
  mmc_execute_set_operation(m_twi, m_xfer_done);
}

void mmc_get_bridge_offset_gauss(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * offset_xyz_gauss, float * mg_field_xyz_gauss)
{
  uint32_t offset_xyz[3] = {0};   // raw samples
  uint32_t mg_field_xyz[3] = {0}; // raw samples
  mmc_get_bridge_offset(m_twi, m_xfer_done, offset_xyz, mg_field_xyz);
  mmc_convert_samples_to_gauss(offset_xyz, 3, offset_xyz_gauss);
  mmc_convert_samples_to_gauss(mg_field_xyz, 3, mg_field_xyz_gauss);
}

void mmc_get_bridge_offset_milli_tesla(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * offset_xyz_mT, float * mg_field_xyz_mT)
{
  uint32_t offset_xyz[3] = {0};   // raw samples
  uint32_t mg_field_xyz[3] = {0}; // raw samples
  mmc_get_bridge_offset(m_twi, m_xfer_done, offset_xyz, mg_field_xyz);
  mmc_convert_samples_to_milli_tesla(offset_xyz, 3, offset_xyz_mT);
  mmc_convert_samples_to_milli_tesla(mg_field_xyz, 3, mg_field_xyz_mT);
}

void mmc_get_bridge_offset_only(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint32_t * offset_xyz)
{
  uint32_t output_1[3] = {0}; // [0]=x, [1]=y, [2]=z  
  uint32_t output_2[3] = {0};

  // 1) set internal magnetization in the direction of the SET field  
  mmc_execute_set_operation(m_twi, m_xfer_done); 
  
  // 2) perform a measurement
  mmc_get_magnetic_field_xyz(m_twi, m_xfer_done, &output_1[0], &output_1[1], &output_1[2]); 
  
  // 3) set internal magnetization in the direction of the RESET field which is the opposite to the SET field (180°)
  mmc_execute_reset_operation(m_twi, m_xfer_done); 

  // 4) perform a measurement
  mmc_get_magnetic_field_xyz(m_twi, m_xfer_done, &output_2[0], &output_2[1], &output_2[2]); 

  // 5) calculate the offset for all directions and all magnetic fields without the offset
  for(uint8_t k = 0; k < 3; k++)
  {
    /*
      Derivation of the equations:
      O:=offset,  out0:=output_0, out1:=output_1,  B:=magnetic field

      out0 = B + O    This is measured after the SET operation
      out1 = -B + O   This is measured after the RESET operation which magnetic field is opposite to the SET field. 
                      Thus, -B holds if B is the magnetic field measured after the SET operation

      This leads to the offset:
      
      [out0 + out1] / 2 = [(B+O) + (-B+O)] / 2 = [B+O-B+O] / 2 = 2O / 2 = O --> offset equation
    */
    offset_xyz[k] = (output_1[k] + output_2[k]) / 2; // offset equation
  }
  // 6) SET operation again that further measurements are based on the direction of the SET field
  mmc_execute_set_operation(m_twi, m_xfer_done);
}         

void mmc_get_bridge_offset_only_gauss(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * offset_xyz_gauss)
{
  uint32_t offset_xyz[3] = {0};   // raw samples
  mmc_get_bridge_offset_only(m_twi, m_xfer_done, offset_xyz);
  mmc_convert_samples_to_gauss(offset_xyz, 3, offset_xyz_gauss);
}

void mmc_get_bridge_offset_only_milli_tesla(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * offset_xyz_mT)
{
  uint32_t offset_xyz[3] = {0};   // raw samples
  mmc_get_bridge_offset_only(m_twi, m_xfer_done, offset_xyz);
  mmc_convert_samples_to_milli_tesla(offset_xyz, 3, offset_xyz_mT);
}  

/*
  Initialization of the sensor
*/ 
void mmc_init(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, mmc_config * cnfg_mmc, mmc_backup_registers * shadows)
{
  //ret_code_t err_code = twi_master_init(); // Start TWI/SPI
  //APP_ERROR_CHECK(err_code);

  if(mmc_is_connected(m_twi, m_xfer_done))
  {
    mmc_soft_reset(m_twi, m_xfer_done); // soft reset is recommended after starting the sensor
    mmc_set_filter_bandwidth(m_twi, m_xfer_done, cnfg_mmc->filter_bandwidth, &(shadows->internal_ctrl_reg_1)); // the the measuring duration
    
    // periodic set reset operation due to strong magnetic fields (B > 10 Gauss)
    if(cnfg_mmc->periodic_set_operation) 
    {
      mmc_enable_pso(m_twi, m_xfer_done, &(shadows->internal_ctrl_reg_0), &(shadows->internal_ctrl_reg_2));
      mmc_set_repetition_number_pso(m_twi, m_xfer_done, cnfg_mmc->repetition_number_pso, &(shadows->internal_ctrl_reg_2));
    }
    //mmc_execute_set_operation(); // restore sensor characteristics if a magnetic field > 10 Gauss was applied    
  }  
}


/*
  Apply / remove extra current; TODO if necessary
*/


/*
  Change from I2C mode to SPI; TODO if necessary
*/