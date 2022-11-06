/**
 * @brief Functions to communicate with an Arduino Uno R3 via I2C or TWI, respectively.  
 *
 * @author Bennet Ladage
 * @date 2022-09-11
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 */

#ifndef I2C_ARDUINO_
#define I2C_ARDUINO_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "board_xm122.h"

// Logger:
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ARD_I2C_ADDR          0x08
#define ARD_SDA               TWI_DATA_Pin // this pin is defined in the board_xm122.h file (Acconeer XM122)
#define ARD_SCL               TWI_CLK_Pin  // this pin is defined in the board_xm122.h file (Acconeer XM122)

bool ard_get_data(uint8_t * addr, uint16_t * dist, uint16_t * strength, int16_t * chip_temp);

/**
 * @brief Initialize the I2C communication
 */
ret_code_t ard_twi_master_init(void);

bool ard_init(uint8_t * addr, uint16_t * sample_frequency);

#endif