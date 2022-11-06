/**
 * @brief Functions to communicate with the LiDAR sensor TFmini Plus A05 via I2C or TWI, respectively.  
 *
 * @author Bennet Ladage
 * @date 2022-10-08
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 * The source code is strongly based on the code of Bud Ryerson: https://github.com/budryerson/TFMini-Plus-I2C
 * Many thanks for your work, Mr Ryerson!
 */

#ifndef TFMINI_PLUS_H_
#define TFMINI_PLUS_H_

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

#define TFMP_DEFAULT_ADDRESS   0x10         // default I2C slave address

// Buffer size definitions
#define TFMP_FRAME_SIZE         9   // Size of data frame = 9 bytes
#define TFMP_REPLY_SIZE         8   // Longest command reply = 8 bytes
#define TFMP_COMMAND_MAX        8   // Longest command = 8 bytes

// Timeout Limits definitions for various functions
#define TFMP_MAX_READS           20   // readData() sets SERIAL error
#define MAX_BYTES_BEFORE_HEADER  20   // getData() sets HEADER error
#define MAX_ATTEMPTS_TO_MEASURE  20

// Error Status Condition definitions
#define TFMP_READY           0  // no error
#define TFMP_SERIAL          1  // serial timeout
#define TFMP_HEADER          2  // no header found
#define TFMP_CHECKSUM        3  // checksum doesn't match
#define TFMP_TIMEOUT         4  // I2C timeout
#define TFMP_PASS            5  // reply from some system commands
#define TFMP_FAIL            6  //           "
#define TFMP_I2CREAD         7
#define TFMP_I2CWRITE        8
#define TFMP_I2CLENGTH       9
#define TFMP_WEAK           10  // Signal Strength ≤ 100
#define TFMP_STRONG         11  // Signal Strength saturation
#define TFMP_FLOOD          12  // Ambient Light saturation
#define TFMP_MEASURE        13
#define TFMP_SAMPLING_FREQ  14  // wrong sampling frequency

// Command Definitions
/* - - - - -  TFMini Plus Data & Command Formats  - - - - -
  Data Frame format:
  Byte0  Byte1  Byte2   Byte3   Byte4   Byte5   Byte6   Byte7   Byte8
  0x59   0x59   Dist_L  Dist_H  Flux_L  Flux_H  Temp_L  Temp_H  CheckSum_
  Data Frame Header character: Hex 0x59, Decimal 89, or "Y"

  Command format:
  Byte0  Byte1   Byte2   Byte3 to Len-2  Byte Len-1
  0x5A   Length  Cmd ID  Payload if any   Checksum
 - - - - - - - - - - - - - - - - - - - - - - - - - */
 #define TFMP_BYTE01 0x5A

// The library 'sendCommand( cmnd, param)' function
// defines a command (cmnd) in the the following format:
// 0x     00       00       00       00
//     one byte  command  command   reply
//     payload   number   length    length
#define    SET_SERIAL_MODE            0x000A0500   // return no reply data
#define    SET_I2C_MODE               0x010A0500   //           "

#define    GET_FIRMWARE_VERSION       0x00010407   // return 3 byte firmware version

#define    SET_FRAME_RATE             0x00030606   // return an echo of the command
#define    STANDARD_FORMAT_CM         0x01050505   //           "
#define    STANDARD_FORMAT_MM         0x06050505   //           "
#define    SET_BAUD_RATE              0x00060808   //           "
#define    ENABLE_OUTPUT              0x01070505   //           "
#define    DISABLE_OUTPUT             0x00070505   //           "
#define    SET_I2C_ADDRESS            0x100B0505   //           "

#define    SOFT_RESET                 0x00020405   // echo and pass(0)/fail(1) byte
#define    HARD_RESET                 0x00100405   //           "
#define    SAVE_SETTINGS              0x00110405   //           "

#define    I2C_FORMAT_CM              0x01000500   // return 9 byte data frame
#define    I2C_FORMAT_MM              0x06000500   //           "
#define    TRIGGER_DETECTION          0x00040400   // return 9 byte serial data
                                                   // frame rate set to zero
#define    OBTAIN_DATA_FRAME          0x01000509


// Command Parameter Definitions
#define    FRAME_0            0x0000    // internal measurement rate
#define    FRAME_1            0x0001    // expressed in hexidecimal
#define    FRAME_2            0x0002
#define    FRAME_5            0x0005
#define    FRAME_10           0x000A
#define    FRAME_20           0x0014
#define    FRAME_25           0x0019
#define    FRAME_50           0x0032
#define    FRAME_100          0x0064
#define    FRAME_125          0x007D
#define    FRAME_200          0x00C8
#define    FRAME_250          0x00FA
#define    FRAME_500          0x01F4
#define    FRAME_1000         0x03E8

/**
 * @brief Send a command to the LiDAR sensor. The command may contain parameters. If there aren't any parameters, just pass 0.
 * If the used command is GET_FIRMWARE_VERSION, the array version is set which must hold 3 elements.
 *
 * @param[in] cmnd:   I2C command, use the defined ones above
 * @param[in] param:  parameter of the used command
 * @param[in] addr:   I2C address of the device
 *
 * @return TFMP_FAIL:     if I2C communication errors occurs
 * @return TFMP_CHECKSUM: if checksum errors occurs, i.e., transmission is temporary interfered 
 * @return TFMP_READY:    everything should be good
 */
uint8_t tfmp_write_cmd(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint32_t cmnd, uint32_t param, uint8_t * addr, uint8_t * version);

/**
 * @brief This function has the same function as the tfmp_write_cmd function. It only returns the reply message of the
 * LiDAR sensor. The reply array should be defined as 
 *  uint8_t reply[TFMP_REPLY_SIZE + 1] = {0};
 *
 * @return TFMP_FAIL:     if I2C communication errors occurs
 * @return TFMP_CHECKSUM: if checksum errors occurs, i.e., transmission is temporary interfered 
 * @return TFMP_READY:    everything should be good
 */
uint8_t tfmp_cmd_reply(nrf_drv_twi_t const * m_twi, bool *m_xfer_done, uint32_t cmnd, uint32_t param, uint8_t * addr, uint8_t * reply);

/**
 * @brief This function returns the distance in cm, the signal strength and the LiDAR chip temperature.
 *
 * @param[in] addr:       I2C address of the device
 * @param[in] dist:       distance in cm, value range [0; 1200]
 * @param[in] strength:   signal strength, value range [0; 65,535]
 * @param[in] chip_temp:  chip temperature in degree celsius
 *
 * @return TFMP_FAIL:     if I2C communication errors occurs
 * @return TFMP_CHECKSUM: if checksum errors occurs, i.e., transmission is temporary interfered 
 * @return TFMP_READY:    everything should be good
 *
 * @return TFMP_WEAK:     signal strength < 100, i.e., unreliable data. Thus, dist=strength=0
 * @return TFMP_STRONG:   signal strength saturation, i.e., unreliable data. Thus, dist=strength=0
 * @return TFMP_FLOOD:    ambient light saturation, i.e., unreliable data. Thus, dist=strength=0
 */
uint8_t tfmp_get_data(nrf_drv_twi_t const * m_twi, bool *m_xfer_done, uint8_t * addr, uint16_t * dist, uint16_t * strength, int16_t * chip_temp);

/**
 * @brief Initialize the I2C communication
 */
//ret_code_t tfmp_twi_master_init(void);

/**
 * @brief Perform a soft reset and set the internal sampling frequency of the LiDAR sensor
 *
 * @param[in] addr:             I2C address of the device
 * @param[in] sample_frequency: internal sampling frequency in Hz, this should satisfy the condition that the 
 *                              sample_frequency = 1000 Hz / n where n is a positive integer
 *                              maximum 1000 Hz; minimum 0 Hz, i.e., no sampling
 *
 * @return TFMP_READY:    everything should be good
 * @return TFMP_TIMEOUT:  the function tfmp_twi_master_init fails
 * @return TFMP_FAIL:     if I2C communication errors occurs
 * @return TFMP_CHECKSUM: if checksum errors occurs, i.e., transmission is temporary interfered 
 */
uint8_t tfmp_init(nrf_drv_twi_t const * m_twi, bool *m_xfer_done, uint8_t * addr, uint16_t * sample_frequency);

//void tfmp_arduino_bridge(void);

#endif