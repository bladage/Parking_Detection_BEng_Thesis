/**
 * @brief This file contains functions which can be used to create messages for the Nordic UART Service (NUS) 
 * for Bluetooth Low Energy (BLE). These functions usually map 16 bit oder 32 bit arrays to 8 bit array. These
 * byte arrays are required due to the NUS. A frame of this service has got 8 data bits as the UART connection
 * via cable. Therefore, a byte array is needed for a transmission. Unsigned bytes are transmittable only.
 * The data of the Acconneer's RADAR sensor A111 and the signals of the Memsic's magnetic field sensor MMC5983MA 
 * are mapped to byte arrays. Metadata frames for the two sensors are created additionally.
 *
 * @author Bennet Ladage
 * @date 2022-09-18
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 */

#ifndef PD_BLE_MESSAGES_H_
#define PD_BLE_MESSAGES_H_

#include "pd_project_defines.h"
#include "acc_service_envelope.h" //TODO remove later
#include "acc_service_iq.h"
#include "memsic_mmc5983ma.h"
#include "nrf_log.h"
#include <math.h>
#include <complex.h>

// Message types for the master device (BLE's central role, raspi) OLD, nec?
//TODO remove later, if old functions aren't nec anymore
//#define MSG_SEND_SIGNALS      'a'
#define MSG_SEND_RADAR_META   'd'
#define MSG_SEND_MMC_SIG      'e'
#define MSG_SEND_MMC_META     'f'
#define MSG_PARKING_SPOT_OCCU 'g'   // parking space is occupied
#define MSG_PARKING_SPOT_VACA 'h'   // parking space is vacant

// Messages from the master device (BLE's central role, raspi)
#define MSG_SETUP_CONFIG        0xC0
#define MSG_UPDATE_RADAR_CONFIG 0xC1
#define MSG_UPDATE_MGF_CONFIG   0xC2
#define MSG_UPDATE_LIDAR_CONFIG 0xC3
#define MSG_UPDATE_TIMESTAMP    0xC4
#define MSG_UPDATE_SLEEP_PERIOD 0xC5

// Messages from the slave device (BLE's peripheral role, xm122)
#define MSG_SEND_RADAR          0xA0
#define MSG_SEND_MGF            0xA1
#define MSG_SEND_LIDAR          0xA2
#define MSG_SEND_SIGNALS        0xA3 //nec?

// Flags which are the same at the master & slave device
#define MSG_SEND_META_DATA      0xB0

// How much bytes contain a message from the master device? See pd_sensor_cnfg.py (raspberry)
#define BYTES_MSG_SETUP_CONFIG          25
#define BYTES_MSG_UPDATE_RADAR_CONFIG   11
#define BYTES_MSG_UPDATE_MGF_CONFIG     7
#define BYTES_MSG_UPDATE_LIDAR_CONFIG   3
#define BYTES_MSG_UPDATE_TIMESTAMP      5
#define BYTES_MSG_UPDATE_SLEEP_PERIOD   3

#define BYTES_32BIT_DATA_GENERAL_SETTINGS   4
#define BYTES_16BIT_DATA_GENERAL_SETTINGS   2
#define BYTES_16BIT_DATA_RADAR_SETTINGS     4
#define BYTES_16BIT_DATA_MGF_SETTINGS       4
#define BYTES_16BIT_DATA_LIDAR_SETTINGS     2
#define BYTES_8BIT_DATA_RADAR_SETTINGS      6
#define BYTES_8BIT_DATA_MGF_SETTINGS        2

#define BYTES_32BIT_DATA_SETUP_CONFIG       BYTES_32BIT_DATA_GENERAL_SETTINGS  
#define BYTES_16BIT_DATA_SETUP_CONFIG       BYTES_16BIT_DATA_GENERAL_SETTINGS + BYTES_16BIT_DATA_RADAR_SETTINGS + BYTES_16BIT_DATA_MGF_SETTINGS + BYTES_16BIT_DATA_LIDAR_SETTINGS
#define BYTES_8BIT_DATA_SETUP_CONFIG        BYTES_8BIT_DATA_RADAR_SETTINGS + BYTES_8BIT_DATA_MGF_SETTINGS
#define BYTES_8BIT_START_SETUP_CONFIG       BYTES_32BIT_DATA_SETUP_CONFIG + BYTES_16BIT_DATA_SETUP_CONFIG + 1 // +1 message flag 0xC0

// How much bytes contain a message from the slave device? (xm122)
#define BYTES_OFFSET_MSG_SEND_META_DATA     9 * NUMBER_MGF_SENSORS                                              // 3axes x 3bytes x K sensors, old: 3axes x 6sensors x 4bytes=72
#define BYTES_RADAR_MSG_SEND_META_DATA      9                                                                    // see create_msg_send_metadata()
#define BYTES_MSG_SEND_META_DATA            BYTES_OFFSET_MSG_SEND_META_DATA + BYTES_RADAR_MSG_SEND_META_DATA + 1 // +1 message flag 0xB0

// Messages to transfer magnetic field sensors' data
#define BYTES_GENERAL_INFORMATION    8                        // 8bytes = 1byte flag + 1byte occupied status + 4byte timestamp + 2byte time offset in ms
#define NUMBER_MGF_SENSORS           1                        // Number of connected magnetic field sensors; max6?
#define BYTES_MGF_XYZ_DATA           NUMBER_MGF_SENSORS * 12  // M sensors x 3axes x 3bytes; 6x12=72bytes; TODO should be 3x3
#define BYTES_MGF_TEMP_DATA          NUMBER_MGF_SENSORS * 2   // M sensors x 1dietemp x 2bytes; 6x2=12bytes 
#define BYTES_MGF_TOTAL              BYTES_GENERAL_INFORMATION + BYTES_MGF_XYZ_DATA + BYTES_MGF_TEMP_DATA  // 92bytes

// Messages to transfer LiDAR sensors' data
#define NUMBER_LIDAR_SENSORS        2      // Number of connected LIDAR sensors;9 max??
#define BYTES_LIDAR_TOTAL           BYTES_GENERAL_INFORMATION + NUMBER_LIDAR_SENSORS * 6 // 26bytes; 18bytes = M sensors x (2bytes x [1xdistance + 1xstrength + 1xtemperature])

/**
 * @brief Struct for the storage of the RADAR sensor configuration.
 */
struct r_cnfg
{
  uint16_t sample_period_ms;
  float start_m;
  float length_m;
  uint8_t profile;
  uint8_t hwaas;
  uint16_t downsampling_factor;
  uint32_t power_save_mode;
  float running_avg;
  float cutoff_lowpass;
};
typedef struct r_cnfg radar_config; 

/**
 * @brief Struct for the storage of the LiDAR sensor configuration.
 */
struct l_cnfg
{
  uint16_t sample_period_ms;
  uint16_t internal_sampling_freq_Hz;
};
typedef struct l_cnfg lidar_config; 

uint16_t maximum(uint16_t a, uint16_t b);

void process_msg_setup_config(uint8_t const * msg, radar_config * cnfg_radar, mmc_config * cnfg_mmc, lidar_config * cnfg_lidar, uint16_t * sleep_period, uint32_t * timestamp);

void create_msg_send_metadata(uint8_t * byte_msg, acc_service_iq_metadata_t * metadata_radar, uint32_t * offset_xyz);

void create_msg_send_radar(uint8_t * byte_msg, bool * occupied, uint32_t * timestamp, uint16_t * time_offset_ms, float complex * cmpl_signal, const uint16_t signal_length);

void create_msg_send_mmc(uint8_t * byte_msg, bool * occupied, uint32_t * timestamp, uint16_t * time_offset_ms, uint32_t * mgf_x, uint32_t * mgf_y, uint32_t * mgf_z, float * die_temp);

void create_msg_send_lidar(uint8_t * byte_msg, bool * occupied, uint32_t * timestamp, uint16_t * time_offset_ms, uint16_t * dist, uint16_t * strength, uint16_t * temperature);



void map_8bit_to_32bit(uint8_t const * bytes, const uint16_t number_of_bytes, uint32_t * signal_32);

void map_8bit_to_24bit(uint8_t const * bytes, const uint16_t number_of_bytes, uint32_t * signal_24);

void map_8bit_to_16bit(uint8_t const * bytes, const uint16_t number_of_bytes, uint16_t * signal_16);


/** 
  @brief  This function maps the metadata to uint8_t and it builts 
          the message which can be send via UART/NUS. This is only the metadata of the RADAR sensor.
          
          The message has got the following frame:
            byte_msg[] = {8, 2, start_m, length_m, data_length_MSB, data_length_LSB, 
                                                   stitch_count_MSB, stitch_count LSB,
                                                   step_length_m_MSB, step_length_m_LSB}

          The real values of the metadata are mapped to unsigned numbers due to the NUS (UART) style
          of messages. Float values are multiplied by a constant that the decimal digit "disappears". Therefore,
          some conversion are necessary to restore the real values. This has to be done by the receiver of the
          messages. These equations are needed then:

          start_m_real = (start_m/10.0)-0.7
          length_m_real = length_m/10.0

          data_length_real = uint16(data_length_MSB, data_length_LSB)
          stitch_count_real = uint16(stitch_count_MSB, stitch_count_LSB)

          step_length_m_real = uint16(step_length_m_MSB, step_length_m_LSB)/1000000.0

          
          The word is 131 (16bit) for the detection that a signal message is received. 
          This word is split into 2x8bit, i.e., 8 (higher byte) and 2 (lower byte).

  @pre    byte_msg must hold 10 elements!
**/ 
void create_metadata_msg(uint8_t *byte_msg, acc_service_envelope_metadata_t *metadata);

/** 
  @brief  The function maps the uint16_t signal to uint8_t and it builts 
          the message, which contains the RADAR signal.

          The message has got the following frame:
            byte_msg[] = {8, 1, MSB0, LSB0, MSB1, LSB1, MSB2, LSB1, ...}
          
          The word is 65 (16bit) for the detection that a signal message is received. 
          This word is split into 2x8bit, i.e., 8 (higher byte) and 1 (lower byte).
**/ 
void create_signal_msg(uint8_t *byte_msg, const uint16_t msg_size, uint16_t *signal);

/** 
  @brief  The function maps a uint32_t vector to uint8_t
**/ 
void map_32bit_to_8bit(uint8_t * byte_vector, const uint16_t number_of_bytes, uint32_t * signal);

/** 
  @brief  The function maps a uint32_t vector (just 3 bytes) to uint8_t
**/ 
void map_24bit_to_8bit(uint8_t * byte_vector, const uint16_t number_of_bytes, uint32_t * signal);

/** 
  @brief  The function maps a uint16_t vector to uint8_t
**/ 
void map_16bit_to_8bit(uint8_t * byte_vector, const uint16_t number_of_bytes, uint16_t * signal);

/** 
  @brief  The function maps the metadata of the RADAR and MMC sensor to uint8_t and it builds 
          the message which can be send via NUS/UART. The meta data is
          the radar's meta data and the magnetic field sensor's bridge offset.
          
          The message has got the following frame:
            byte_vector[] = {8, 2, start_m, length_m, data_length_MSB, data_length_LSB, 
                                                      stitch_count_MSB, stitch_count LSB,
                                                      step_length_m_MSB, step_length_m_LSB
                                                      offset_x_byte3_MSB, offset_x_byte2, offset_x_byte1, offset_x_byte0_LSB,
                                                      offset_y_byte3_MSB, offset_y_byte2, offset_y_byte1, offset_y_byte0_LSB,
                                                      offset_z_byte3_MSB, offset_z_byte2, offset_z_byte1, offset_z_byte0_LSB,
                                                      bytes_mgf_vector_MSB, bytes_mgf_vector_LSB,
                                                      die_temperature_MSB, die_temperature_LSB,
                                                      filter_bandwidth_MSB, filter_bandwidth_LSB}

          The real values of the metadata are mapped to unsigned numbers due to the NUS (UART) style
          of messages. Float values are multiplied by a constant that the decimal digit "disappears". Therefore,
          some conversion are necessary to restore the real values. This has to be done by the receiver of the
          messages. These equations are needed then:

          start_m_real = (start_m/100.0)-0.7
          length_m_real = length_m/10.0

          data_length_real = uint16(data_length_MSB, data_length_LSB)
          stitch_count_real = uint16(stitch_count_MSB, stitch_count_LSB)

          step_length_m_real = uint16(step_length_m_MSB, step_length_m_LSB)/1000000.0

          offset_x_real = uint32(offset_x_byte3_MSB, offset_x_byte2, offset_x_byte1, offset_x_byte0_LSB)
          offset_y_real = uint32(offset_y_byte3_MSB, offset_y_byte2, offset_y_byte1, offset_y_byte0_LSB)
          offset_z_real = uint32(offset_z_byte3_MSB, offset_z_byte2, offset_z_byte1, offset_z_byte0_LSB)

          bytes_mgf_vector_real = uint16(bytes_mgf_vector_MSB, bytes_mgf_vector_LSB)
          die_temperature_real = (uint16(die_temperature_MSB, die_temperature_LSB)-75.0)/10.0
          filter_bandwidth_real = uint16(filter_bandwidth_MSB, filter_bandwidth_LSB)

          The word is 131 (16bit) for the detection that a signal message is received. 
          This word is split into 2x8bit, i.e., 8 (higher byte) and 2 (lower byte).          

  @pre    byte_vector must hold 10+12+2+2+2=28 elements (BYTES_META_DATA_ALL) and offset_xyz 3
**/ 
void create_metadata_msg_radar_mmc(uint8_t * byte_msg, acc_service_envelope_metadata_t *metadata, uint32_t * offset_xyz, uint16_t length_mgf, float die_temp, uint16_t filter_bandwidth);

/** 
  @brief  The function maps the uint16_t signal to uint8_t or uint32_t to uint8_t due to UART/NUS.
          Also, it builds the message, which contains the RADAR signal and the magnetic field (mgf) signal.

          The message has got the following frame:
            byte_msg[] = {8, 1, radar_sample0_MSB, radar_sample0_LSB,
                                radar_sample1_MSB, radar_sample1_LSB,
                                radar_sample2_MSB, radar_sample2_LSB,
                                ...
                                mgf_x_sample0_byte3_MSB, mgf_x_sample0_byte2, mgf_x_sample0_byte1, mgf_x_sample0_byte0_LSB,
                                mgf_x_sample1_byte3_MSB, mgf_x_sample1_byte2, mgf_x_sample1_byte1, mgf_x_sample1_byte0_LSB,
                                ...
                                mgf_y_sample0_byte3_MSB, mgf_y_sample0_byte2, mgf_y_sample0_byte1, mgf_y_sample0_byte0_LSB,
                                mgf_y_sample1_byte3_MSB, mgf_y_sample1_byte2, mgf_y_sample1_byte1, mgf_y_sample1_byte0_LSB,
                                ...
                                mgf_z_sample0_byte3_MSB, mgf_z_sample0_byte2, mgf_z_sample0_byte1, mgf_z_sample0_byte0_LSB,
                                mgf_z_sample1_byte3_MSB, mgf_z_sample1_byte2, mgf_z_sample1_byte1, mgf_z_sample1_byte0_LSB}
          
          That is, the reconstruction of a measured sample is...
          
          ... for the radar sensor: radar_sample0 = uint16(radar_sample0_MSB, radar_sample0_LSB)
          ... for the mgf sensor:   mgf_?_sample0 = uint32(mgf_?_sample0_byte3_MSB, mgf_?_sample0_byte2, mgf_?_sample0_byte1, mgf_?_sample0_byte0_LSB) where ? = {x,y,z}
**/ 
void create_radar_mmc_signal_msg(uint8_t * byte_msg, uint16_t * radar_signal, 
                                        uint32_t * mgf_x, uint32_t * mgf_y, uint32_t * mgf_z,
                                        const uint16_t length_radar_signal, const uint16_t length_mgf);

#endif