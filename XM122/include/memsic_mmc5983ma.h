/**
 * @brief Functions to communicate with the magnetic sensor via I2C or TWI, respectively.
 *        The magnetic sensor is the MMC5983MA which has got 3 axis.  
 *
 * @author Bennet Ladage
 * @date 2022-10-08
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 * The source code is partially based on this code: https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library
 */

#ifndef MEMSIC_MMC5983MA_H_
#define MEMSIC_MMC5983MA_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "board_xm122.h"

/*
  Define Registers' Addresses
*/
#define MMC_XOUT_0              0x00
#define MMC_XOUT_1              0x01
#define MMC_YOUT_0              0x02
#define MMC_YOUT_1              0x03
#define MMC_ZOUT_0              0x04
#define MMC_ZOUT_1              0x05
#define MMC_XYZOUT_2            0x06
#define MMC_TOUT                0x07
#define MMC_STATUS              0x08
#define MMC_INTERNAL_CONTROL_0  0x09
#define MMC_INTERNAL_CONTROL_1  0x0A
#define MMC_INTERNAL_CONTROL_2  0x0B
#define MMC_INTERNAL_CONTROL_3  0x0C
#define MMC_PRODUCT_ID          0x2F

/*
  Values to calculate the real magnetic field in Gauss (1 Gauss = 10^-4 Tesla)
*/
#define MMC_COUNTS_PER_GAUSS_18_BIT     16384
#define MMC_COUNTS_PER_GAUSS_16_BIT     4096
#define MMC_COUNTS_PER_GAUSS_18_BIT_FL  16384.0
#define MMC_COUNTS_PER_GAUSS_16_BIT_FL  4096.0

/*
  I2C Address & Pins
*/
#define MMC_I2C_ADDRESS         0x30         // 7 bit address=Product ID! 0x30 default

/*
  Define Bits for the Bit Masks
*/
#define MEAS_M_DONE                 (1 << 0)
#define MEAS_T_DONE                 (1 << 1)
#define OTP_READ_DONE               (1 << 4)
#define TM_M                        (1 << 0)
#define TM_T                        (1 << 1)
#define INT_MEAS_DONE_EN            (1 << 2)
#define SET_OPERATION               (1 << 3)
#define RESET_OPERATION             (1 << 4)
#define AUTO_SR_EN                  (1 << 5)
#define OTP_READ                    (1 << 6)
#define BW0                         (1 << 0)
#define BW1                         (1 << 1)
#define X_INHIBIT                   (1 << 2)
#define YZ_INHIBIT                  (3 << 3)
//#define Y_INHIBIT                   (1 << 3)
//#define Z_INHIBIT                   (1 << 4)
#define SW_RST                      (1 << 7)
#define CM_FREQ_0                   (1 << 0)
#define CM_FREQ_1                   (1 << 1)
#define CM_FREQ_2                   (1 << 2)
#define CMM_EN                      (1 << 3)
#define PRD_SET_0                   (1 << 4)
#define PRD_SET_1                   (1 << 5)
#define PRD_SET_2                   (1 << 6)
#define EN_PRD_SET                  (1 << 7)
#define ST_ENP                      (1 << 1)
#define ST_ENM                      (1 << 2)
#define SPI_3W                      (1 << 6)
#define X2_MASK                     (3 << 6)
#define Y2_MASK                     (3 << 4)
#define Z2_MASK                     (3 << 2)
#define XYZ_0_SHIFT                 10
#define XYZ_1_SHIFT                 2
#define MMC_CLEAR_REG               0b00000000 // MMC_CLEAR_REG & reg = 0x00, always

/**
 * @brief Struct for the backup register. Some registers are write only (wro). Therefore, their
 * status can't be check by reading the real registers of the sensor. Thus, we need a 
 * backup of the set bits. We may use this struct to archive this. The backup registers are occasionally
 * called shadow registers.
 */
struct mmc_bckp_regs
{
  uint8_t internal_ctrl_reg_0;
  uint8_t internal_ctrl_reg_1;
  uint8_t internal_ctrl_reg_2;
  uint8_t internal_ctrl_reg_3;
};
typedef struct mmc_bckp_regs mmc_backup_registers;  

/**
 * @brief Struct for the storage of the magnetic sensor configuration.
 */
struct m_cnfg
{
  uint16_t sample_period_ms;
  uint16_t filter_bandwidth;        
  bool periodic_set_operation;   
  uint16_t repetition_number_pso; 
};
typedef struct m_cnfg mmc_config;  


/**************************************************************

  Help Functions, i.e., write/read registers, is a bit set? ...

/**************************************************************/

/**
 * @brief Read byte(s) via TWI.
 */
bool mmc_read_register(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t register_address, uint8_t * rx_data, uint8_t number_of_bytes);

/**
 * @brief Write a byte via TWI
 */
bool mmc_write_register(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t register_address, uint8_t tx_data);

/**
 * @brief Sets a single bit in a specific 8 bit register. The register must be readable. 
 */
void mmc_set_register_bit(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask);

/**
 * @brief Clears a single bit in a specific register. The register must be readable.
 */
void mmc_clear_register_bit(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask);

/**
 * @brief Returns true if a specific bit is set in a register. The register must be readable.
 */
bool mmc_is_bit_set(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask);

/**
 * @brief Sets a single bit in a specific 8 bit register. This is for write only (wro) registers.
 * The register byte is saved to remember which bits were changed. This is the backup_reg variable, i.e., a pseudo register.
 * When no backup register is needed, pass the null pointer NULL as backup_reg.
 */
void mmc_set_register_bit_wro(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask, uint8_t * backup_reg);

/**
 * @brief Clears a single bit in a specific register. This is for write only (wro) registers. 
 * The register byte is saved to remember which bits were changed. This is the backup_reg variable, i.e., a pseudo register.
 * When no backup register is needed, pass the null pointer NULL as backup_reg.
 */
void mmc_clear_register_bit_wro(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, const uint8_t register_address, const uint8_t bit_mask, uint8_t * backup_reg);

/**
 * @brief Returns true if a specific bit is set in the backup register. This pseudo register is stored into the RAM!
 */ 
bool mmc_is_bit_set_backup_reg(uint8_t * backup_reg, const uint8_t bit_mask);


/**************************************************************

  Functions to Handle the Sensor and its Communication via TWI

***************************************************************/
 
/**
 * @brief initialize the twi communication
 */
//ret_code_t mmc_twi_master_init(void);

/**
 * @brief Read the sensor's product ID and compare if it is equal to the I2C address; return true if theses values match
 */
bool mmc_is_connected(nrf_drv_twi_t const * m_twi, bool * m_xfer_done);

/**
 * @brief Soft reset device
 */ 
void mmc_soft_reset(nrf_drv_twi_t const * m_twi, bool * m_xfer_done);

/**
 * @brief Returns die temperature. Range is -75C to 125C
 */
float mmc_get_die_temperature(nrf_drv_twi_t const * m_twi, bool * m_xfer_done);


/*
  Interrupt on/off functions
*/

/**
 * @brief  An interrupt will be send to the master via the INT pin if a measurement (magnetic or temperature) is finished
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 */
void mmc_enable_interrupt_pin(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0);

/**
 * @brief Disable the interrput.
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 */ 
void mmc_disable_interrupt_pin(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0);


/*
  SET/RESET operation functions
*/

/**
 * @brief Restore the sensor characteristics if a strong magnetic field occured (i.e., more than 10 Gauss)
 * A high and pulsed current flows through the sensor if this function is called.
 */
void mmc_execute_set_operation(nrf_drv_twi_t const * m_twi, bool * m_xfer_done);

/**
 * @brief Same as the set operation, but the current flows in the other direction. Thus, the internal magnetization direction
 * is opposite to the set field (180 degree opposed).
 */
void mmc_execute_reset_operation(nrf_drv_twi_t const * m_twi, bool * m_xfer_done);

/**
 * @brief Enable the restoring of the sensor characteristics if a strong magnetic field occured, but this is done automatically!
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 */
void mmc_enable_automatic_set_reset_operation(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0);

/**
 * @brief Disable the restoring of the sensor characteristics if a strong magnetic field occured, but this is done automatically!
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 */
void mmc_disable_automatic_set_reset_operation(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0);

/**
 * @brief Check the backup register if automatic set/reset operation is enabled.
 */
bool mmc_is_automatic_set_reset_enabled(uint8_t * backup_internal_ctrl_0);


/*
  Channel X functions
*/

/**
 * @brief Enable the x channel of the magnetic field measurement.
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 */
void mmc_enable_x_channel(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1);

/**
 * @brief Disable the x channel of the magnetic field measurement.
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 */
void mmc_disable_x_channel(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1);

/**
 * @brief Check the backup register if the x channel is enabled.
 */
bool mmc_is_x_channel_enabled(uint8_t * backup_internal_ctrl_1);


/*
  Channel YZ functions
*/

/**
 * @brief Enable the y&z channel of the magnetic field measurement.
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 */
void mmc_enable_yz_channels(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1);

/**
 * @brief Disable the y&z channel of the magnetic field measurement.
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 *
 * @warning Bug? y-channel can't be switched off??
 */
void mmc_disable_yz_channels(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1);

/**
 * @brief Check the backup register if the y&z channel are enabled.
 */
bool mmc_are_yz_channels_enabled(uint8_t * backup_internal_ctrl_1);


/*
  Decimation Filter
*/

/**
 * @brief Set the bandwidth of the decimation filter. This controls the measurement time of each measurement.
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 * @param[in] bandwidth 100 Hz = 8.0 ms
 *                      200 Hz = 4.0 ms
 *                      400 Hz = 2.0 ms
 *                      800 Hz = 0.5 ms
 *                      default case is 100 Hz
 * @param[in] backup_internal_ctrl_1 Backup register (or shadow register) of MMC_INTERNAL_CONTROL_1
 */
void mmc_set_filter_bandwidth(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint16_t bandwidth, uint8_t * backup_internal_ctrl_1);

/**
 * @brief Returns the bandwidth (in Hz) of the decimation filter. The data is extracted from the backup register.
 */
uint16_t mmc_get_filter_bandwidth(uint8_t * backup_internal_ctrl_1);


/*
  Continuous Measurement Mode
*/

/**
 * @brief Enables the continuous measurement mode.
 * The bandwidth has to be 100 Hz otherwise the measurement frequencies doesn't match. TBD: 100 Hz automatically?!?
 *
 * @param[in] backup_internal_ctrl_1: backup register which has the information of the filter bandwidth
 * @param[in] backup_internal_ctrl_2: backup register which has the information of the continuous measurement mode
 *
 * @return true:  Bandwidth is 100 Hz of the backup register backup_internal_ctrl_1! (wro register, so we know just his shadow...)
 * @return false: Bandwidth isn't 100 Hz of the backup register. Thus, the mode isn't enabled.
 */
bool mmc_enable_cont_mode(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_1, uint8_t * backup_internal_ctrl_2);

/**
 * @brief Check the backup register if the continuous mode is enabled.
 */
bool mmc_is_cont_mode_enabled(uint8_t * backup_internal_ctrl_2);

/**
 * @brief Set the measurement frequency of the continuous mode. This frequency depends on the bandwidth of the
 * decimation filter. This could be changed by this function. Thus, the backup register backup_internal_ctrl_1
 * is passed to save changes. The backup register backup_internal_ctrl_2 contains bits to control the continuous mode.
 * A backup is recommendable, because these bit values won't clear itself and the used register is write only.
 *
 * @param[in] freq:                   measurement frequency, freq={1, 20, 50, 100, 200, 1000} Hz
 * @param[in] backup_internal_ctrl_1: backup register for MMC_INTERNAL_CONTROL_1 (filter bandwidth etc.)
 * @param[in] backup_internal_ctrl_2: backup register for MMC_INTERNAL_CONTROL_2 (continuous mode etc.)
 */
void mmc_set_frequency_cont_mode(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint16_t freq, uint8_t * backup_internal_ctrl_1, uint8_t * backup_internal_ctrl_2);

/**
 * @brief Return which frequency is used in the continuous mode. The values comes from the backup register MMC_INTERNAL_CONTROL_2
 */
uint16_t mmc_get_frequency_cont_mode(uint8_t * backup_internal_ctrl_2);


/*
  Periodic SET operations to remove the effects of a strong external magnetic field, i.e., H_ext > 10 Gauss = 10*
*/

/**
 * @brief Enables the Periodic SET Operation (pso), i.e., the chip performs a SET operation after every measurement (default).
 * The repetiton number can be changed by the mmc_set_repetition_number_pso function. It must be changed bits of the two registers
 * to start this periodic function. The needed registers are MMC_INTERNAL_CONTROL_0 and MMC_INTERNAL_CONTROL_2.
 * A backup is recommendable, because these bit values won't clear itself and the used register is write only.
 */
void mmc_enable_pso(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0, uint8_t * backup_internal_ctrl_2);

/**
 * @brief Disable the periodic SET operations (pso).
 */
void mmc_disable_pso(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint8_t * backup_internal_ctrl_0, uint8_t * backup_internal_ctrl_2);

/**
 * @brief Check the backup register if the periodic SET operation (pso) is enabled.
 */
bool mmc_is_pso_enabled(uint8_t * backup_internal_ctrl_0, uint8_t * backup_internal_ctrl_2);

/**
 * @brief How much measurements has to be passed that a SET operation is started automatically? The answer to this question
 * is the repetition number, i.e., the variable rep_number. Possible numbers = {1, 25, 75, 100, 250, 500, 1000, 2000}.
 * A backup is recommendable, because this bit value won't clear itself and the used register is write only.
 */
void mmc_set_repetition_number_pso(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint16_t rep_number, uint8_t * backup_internal_ctrl_2);

/**
 * @brief Return which repetition number is used in the pso mode. The values comes from the backup register MMC_INTERNAL_CONTROL_2.
 */
uint16_t mmc_get_repetition_number_pso(uint8_t * backup_internal_ctrl_2);


/*
  Get magnetic field measurements
*/

/**
 * @brief Return the magnetic field value of the x-axis in raw samples. One measurement will be performed.
 */
uint32_t mmc_get_magnetic_field_x(nrf_drv_twi_t const * m_twi, bool * m_xfer_done);

/**
 * @brief Return the magnetic field value of the y-axis in raw samples. One measurement will be performed.
 */
uint32_t mmc_get_magnetic_field_y(nrf_drv_twi_t const * m_twi, bool * m_xfer_done);

/**
 * @brief Return the magnetic field value of the z-axis in raw samples. One measurement will be performed.
 */
uint32_t mmc_get_magnetic_field_z(nrf_drv_twi_t const * m_twi, bool * m_xfer_done);

/**
 * @brief Get the raw samples of the the magnetic field values for all axes. One measurement will be performed.
 * The usage of this function is recommendable instead of using mmc_get_magnetic_field_x(), mmc_get_magnetic_field_y()
 * and mmc_get_magnetic_field_z() in a row.
 */
void mmc_get_magnetic_field_xyz(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint32_t * x, uint32_t * y, uint32_t * z);

/**
 * @brief Get the magnetic field values in Gauss for all axes. One measurement will be performed.
 */
void mmc_get_magnetic_field_xyz_gauss(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * x, float * y, float * z);

/**
 * @brief Get the magnetic field values in milli Tesla for all axes. One measurement will be performed.
 */
void mmc_get_magnetic_field_xyz_milli_tesla(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * x, float * y, float * z);

/**
 * @brief Converts an array of raw samples (pointer samples) to an array with gauss units.
 * The length of both arrays is the parameter length. 
 */
void mmc_convert_samples_to_gauss(uint32_t * samples, uint16_t length, float * gauss_signal);

/**
 * @brief Converts an array of raw samples (pointer samples) to an array with milli Tesla (mT) units.
 * The length of both arrays is the parameter length. 
 */
void mmc_convert_samples_to_milli_tesla(uint32_t * samples, uint16_t length, float * mT_signal);

/**
 * @brief Get the bridge offset for every axis and the magnetic field without the offset.
 * This routine is based on the data sheet example "Using SET and RESET to remove bridge offse".
 * The offset and magnetic field are raw samples here.
 */
void mmc_get_bridge_offset(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint32_t * offset_xyz, uint32_t * magnetic_field_xyz);

/**
 * @brief Get the bridge offset for every axis and the magnetic field without the offset.
 * The offset and magnetic field have got the unit Gauss here.
 */
void mmc_get_bridge_offset_gauss(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * offset_xyz_gauss, float * mg_field_xyz_gauss);

/**
 * @brief Get the bridge offset for every axis and the magnetic field without the offset.
 * The offset and magnetic field have got the unit milli Tesla (mT) here.
 */
void mmc_get_bridge_offset_milli_tesla(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * offset_xyz_mT, float * mg_field_xyz_mT);

/**
 * @brief Get the bridge offset for every axis.
 * This routine is based on the data sheet example "Using SET and RESET to remove bridge offse".
 * The offset and magnetic field are raw samples here.
 */
void mmc_get_bridge_offset_only(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, uint32_t * offset_xyz);

/**
 * @brief Get the bridge offset for every axis.
 * The offset and magnetic field have got the unit Gauss here.
 */
void mmc_get_bridge_offset_only_gauss(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * offset_xyz_gauss);

/**
 * @brief Get the bridge offset for every axis.
 * The offset and magnetic field have got the unit milli Tesla (mT) here.
 */
void mmc_get_bridge_offset_only_milli_tesla(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, float * offset_xyz_mT);

/*
  Initialization of the sensor
*/ 

//@brief Start the TWI1 instance to communicate with the Memsic sensor and initialize the sensor
void mmc_init(nrf_drv_twi_t const * m_twi, bool * m_xfer_done, mmc_config * cnfg_mmc, mmc_backup_registers * shadows);

/*
  Apply / remove extra current; TODO if necessary
*/


/*
  Change from I2C mode to SPI; TODO if necessary
*/


// maybe: add parameters like filter bandwidth pso repetition number...
//void mmc_init_sensor(void);


#endif
