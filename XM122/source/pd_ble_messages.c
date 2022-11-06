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

#include "pd_ble_messages.h"


uint16_t maximum(uint16_t a, uint16_t b)
{
  uint16_t max = ((a+b)/2.0) + fabs((a-b)/2.0);
  return max;
}

void process_msg_setup_config(uint8_t const * msg, radar_config * cnfg_radar, mmc_config * cnfg_mmc, lidar_config * cnfg_lidar, uint16_t * sleep_period, uint32_t * timestamp)
{
  uint16_t signal_16[BYTES_16BIT_DATA_SETUP_CONFIG] = {0};
  uint32_t signal_32[BYTES_32BIT_DATA_SETUP_CONFIG] = {0};
  
  map_8bit_to_32bit(msg+1, BYTES_32BIT_DATA_SETUP_CONFIG, signal_32); // +1 to skip the flag 0xC0
  map_8bit_to_16bit(msg+1+BYTES_32BIT_DATA_SETUP_CONFIG, BYTES_16BIT_DATA_SETUP_CONFIG, signal_16);

  // 32 bit data
  *timestamp = signal_32[0];
  *sleep_period = signal_16[0];
  
  // 16 bit data
  cnfg_radar->sample_period_ms = signal_16[1];
  cnfg_lidar->internal_sampling_freq_Hz = signal_16[2]; // old: /100.0
  cnfg_mmc->sample_period_ms = signal_16[3];
  cnfg_mmc->repetition_number_pso = signal_16[4];
  cnfg_lidar->sample_period_ms = signal_16[5];

  // 8 bit data
  cnfg_radar->start_m = (msg[BYTES_8BIT_START_SETUP_CONFIG]-7)/100.0; // convert cm in m; TODO -70 cm??
  cnfg_radar->length_m = msg[BYTES_8BIT_START_SETUP_CONFIG+1]/100.0;
  cnfg_radar->downsampling_factor = msg[BYTES_8BIT_START_SETUP_CONFIG+2];  
  cnfg_radar->profile = msg[BYTES_8BIT_START_SETUP_CONFIG+3];
  cnfg_radar->hwaas = msg[BYTES_8BIT_START_SETUP_CONFIG+4];
  cnfg_radar->cutoff_lowpass = (float)msg[BYTES_8BIT_START_SETUP_CONFIG+5]/100.0;

  cnfg_mmc->filter_bandwidth = 100*msg[BYTES_8BIT_START_SETUP_CONFIG+6];
  cnfg_mmc->periodic_set_operation = msg[BYTES_8BIT_START_SETUP_CONFIG+7];
}

void create_msg_send_metadata(uint8_t * byte_msg, acc_service_iq_metadata_t * metadata_radar, uint32_t * offset_xyz)
{
  uint8_t offset_xyz_8bit[BYTES_OFFSET_MSG_SEND_META_DATA] = {0};// 3axes x 6sensors x 4bytes=72; offset_xyz[0]=offset x0, offset_xyz[1]=offset y0, offset_xyz[2]=offset z0,
                                                                 //                               offset_xyz[0]=offset x1, offset_xyz[1]=offset y1, offset_xyz[2]=offset z1,   
                                                                 //                               ...
                                                                 //                               offset_xyz[0]=offset x5, offset_xyz[1]=offset y5, offset_xyz[2]=offset z5,    
  uint8_t k; // index loop
  float temp_v = ceilf((metadata_radar->start_m+0.7) * 100) / 100; // set precision of the floating point number; +0.7 because min. is -0.7;

  // Set flag that the message is MSG_SEND_METADATA
  byte_msg[0] = MSG_SEND_META_DATA;

  // map the floats to uint16
  byte_msg[1] = (uint8_t)(100*metadata_radar->depth_lowpass_cutoff_ratio);
  byte_msg[2] = (100*temp_v);          
  byte_msg[3] = (uint8_t)((10.0)*(metadata_radar->length_m)); // length_tx
  
  // map 1x16bit |--> 2x8bit
  byte_msg[4] = (uint8_t)(metadata_radar->data_length >> 8);
  byte_msg[5] = (uint8_t)metadata_radar->data_length;
  byte_msg[6] = (uint8_t)(metadata_radar->stitch_count >> 8);
  byte_msg[7] = (uint8_t)metadata_radar->stitch_count;

  // map the step_length (float) also to uint16 and then to 2x8bit
  uint16_t step_length_tx = (uint16_t)((1000000.0)*(metadata_radar->step_length_m));
  byte_msg[8] = (uint8_t)(step_length_tx >> 8);
  byte_msg[9] = (uint8_t)step_length_tx;

  // Add the mmcs' offsets
  map_24bit_to_8bit(offset_xyz_8bit, BYTES_OFFSET_MSG_SEND_META_DATA, offset_xyz); // map the 32 bit signal to 8 bit due to the 8 bit UART frames
  for(k = 0; k < BYTES_OFFSET_MSG_SEND_META_DATA; k++)
  { 
    byte_msg[k+10] = offset_xyz_8bit[k]; // append the offset which is split into 8 bit
  }  
}

void create_msg_send_radar(uint8_t * byte_msg, bool * occupied, uint32_t * timestamp, uint16_t * time_offset_ms, float complex * cmpl_signal, const uint16_t signal_length)
{
  int16_t phi;
  float magf, phif;
  uint16_t mag16, phi16;
  uint16_t phase[signal_length];
  uint16_t mag[signal_length];

  for(uint16_t k = 0; k < signal_length; k++)
  {
    magf = cabsf(cmpl_signal[k])*1000.0f; // magnitude, 3 decimal digits
    mag16 = (uint16_t)magf;
    phif = cargf(cmpl_signal[k])*10000.0f;  // phase, 4 decimal digits
    phi = (int16_t)phif;
    phi16 = (uint16_t)(phi); // bits are the same either uint16 or int16, interpretation is important hence use signed in python
    
    mag[k] = mag16;
    phase[k] = phi16;

    //if(phi >= 0)
    //{
    //  byte_msg[8+(4*signal_length)+k] = 0; // save the sign "+"; like BPSK modulation: (-1)^0=1, (-1)^1=-1; saves signal_length bytes per transmission!
    //}
    //else
    //{
    //  byte_msg[8+(4*signal_length)+k] = 1; // sign "-"
    //}
  }

  byte_msg[0] = MSG_SEND_RADAR;                     // Set flag that the message is MSG_SEND_RADAR
  byte_msg[1] = (uint8_t)(*occupied);               // is parking spot occupied?; Y of the regression model
  map_32bit_to_8bit(byte_msg+2, 4, timestamp);      // timestamp = byte_msg[2,3,4,5]
  map_16bit_to_8bit(byte_msg+6, 2, time_offset_ms); // time_offset_ms = byte_msg[6,7]  

  map_16bit_to_8bit(byte_msg+8, 2*signal_length, mag); // send whole magnitudes first, then phases, after this the phases' signs (saved above in loop)
  map_16bit_to_8bit(byte_msg+8+(2*signal_length), 2*signal_length, phase);
}

void create_msg_send_mmc(uint8_t * byte_msg, bool * occupied, uint32_t * timestamp, uint16_t * time_offset_ms, uint32_t * mgf_x, uint32_t * mgf_y, uint32_t * mgf_z, float * die_temp)
{
  //uint8_t byte_msg[BYTES_MGF_TOTAL] = {0};
  uint16_t die_temp_16[NUMBER_MGF_SENSORS] = {0};

  // Convert float to 16 bit
  for(uint8_t k=0; k < NUMBER_MGF_SENSORS; k++)
  {
    die_temp[k] = (die_temp[k] * 10.0)+75.0; // -75°C is minimum, 1 decimal digit thus *10.0
    die_temp_16[k] = (uint16_t)(die_temp[k]);
  }

  byte_msg[0] = MSG_SEND_MGF;                       // Set flag that the message is MSG_SEND_MGF
  byte_msg[1] = (uint8_t)(*occupied);               // is parking spot occupied?; Y of the regression model
  map_32bit_to_8bit(byte_msg+2, 4, timestamp);      // timestamp = byte_msg[2,3,4,5]
  map_16bit_to_8bit(byte_msg+6, 2, time_offset_ms); // time_offset_ms = byte_msg[6,7]

  map_24bit_to_8bit(byte_msg+8, 3*NUMBER_MGF_SENSORS, mgf_x);  // mgf_x[0] = byte_msg[8,9,10,11], mgf_x[1] = byte_msg[12,13,14,15]
  map_24bit_to_8bit(byte_msg+8+(3*NUMBER_MGF_SENSORS), 3*NUMBER_MGF_SENSORS, mgf_y); 
  map_24bit_to_8bit(byte_msg+8+(6*NUMBER_MGF_SENSORS), 3*NUMBER_MGF_SENSORS, mgf_z); 
  map_16bit_to_8bit(byte_msg+8+(9*NUMBER_MGF_SENSORS), 2*NUMBER_MGF_SENSORS, die_temp_16);
}

void create_msg_send_lidar(uint8_t * byte_msg, bool * occupied, uint32_t * timestamp, uint16_t * time_offset_ms, uint16_t * dist, uint16_t * strength, uint16_t * temperature)
{
  byte_msg[0] = MSG_SEND_LIDAR;                     // Set flag that the message is MSG_SEND_LIDAR
  byte_msg[1] = (uint8_t)(*occupied);               // is parking spot occupied?; Y of the regression model 
  map_32bit_to_8bit(byte_msg+2, 4, timestamp);      // timestamp = byte_msg[2,3,4,5]
  map_16bit_to_8bit(byte_msg+6, 2, time_offset_ms); // time_offset_ms = byte_msg[6,7]
  
  map_16bit_to_8bit(byte_msg+8, 2*NUMBER_LIDAR_SENSORS, dist);      // dist[sensor 0]=byte_msg[8,9], dist[sensor 1]=byte_msg[10,11], dist[sensor 2]=byte_msg[12,13]   
  map_16bit_to_8bit(byte_msg+8+(2*NUMBER_LIDAR_SENSORS), 2*NUMBER_LIDAR_SENSORS, strength); // strength[sensor 0]=byte_msg[14,15], strength[sensor 1]=byte_msg[16,17], strength[sensor 2]=byte_msg[18,19]   
  map_16bit_to_8bit(byte_msg+8+(4*NUMBER_LIDAR_SENSORS), 2*NUMBER_LIDAR_SENSORS, temperature); // temperature[sensor 0]=byte_msg[20,21], temperature[sensor 1]=byte_msg[22,23], temperature[sensor 2]=byte_msg[24,25]   
}

void map_8bit_to_32bit(uint8_t const * bytes, const uint16_t number_of_bytes, uint32_t * signal_32)
{
  for(uint16_t k=0; k < number_of_bytes/4; k++)
  {
    signal_32[k] |= (bytes[2*k] << 24);
    signal_32[k] |= (bytes[2*k+1] << 16);
    signal_32[k] |= (bytes[2*k+2] << 8);
    signal_32[k] |= bytes[2*k+3];
    //NRF_LOG_INFO("32bit[%d]: %x", k, signal_32[k]);
  } 
}

void map_8bit_to_24bit(uint8_t const * bytes, const uint16_t number_of_bytes, uint32_t * signal_24)
{
  for(uint16_t k=0; k < number_of_bytes/3; k++)
  {
    signal_24[k] |= (bytes[2*k+1] << 16);
    signal_24[k] |= (bytes[2*k+2] << 8);
    signal_24[k] |= bytes[2*k+3];
    NRF_LOG_INFO("24bit[%d]: %x", k, signal_24[k]);
  } 
}

void map_8bit_to_16bit(uint8_t const * bytes, const uint16_t number_of_bytes, uint16_t * signal_16)
{
  for(uint16_t k=0; k < number_of_bytes/2; k++)
  {
    signal_16[k] |= (bytes[2*k] << 8);
    signal_16[k] |= bytes[2*k+1];
  }  
}

// obsolete, except for the mapping functions
void create_metadata_msg(uint8_t *byte_msg, acc_service_envelope_metadata_t *metadata)
{
  // The sync-word is 131 (16bit) which is split into 2x8bit, i.e., 8 (higher byte) and 2 (lower byte)
  byte_msg[0] = 8;
  byte_msg[1] = 2;
  
  // map the floats to uint16
  byte_msg[2] = (uint8_t)((10.0)*(metadata->start_m+0.7));   // start_tx
  byte_msg[3] = (uint8_t)((10.0)*(metadata->length_m));      // length_tx
  
  // map 1x16bit |--> 2x8bit
  byte_msg[4] = (uint8_t)(metadata->data_length >> 8);
  byte_msg[5] = (uint8_t)metadata->data_length;
  byte_msg[6] = (uint8_t)(metadata->stitch_count >> 8);
  byte_msg[7] = (uint8_t)metadata->stitch_count;

  // map the step_length (float) also to uint16 and then to 2x8bit
  uint16_t step_length_tx = (uint16_t)((1000000.0)*(metadata->step_length_m));
  byte_msg[8] = (uint8_t)(step_length_tx >> 8);
  byte_msg[9] = (uint8_t)step_length_tx;
}

void create_signal_msg(uint8_t *byte_msg, const uint16_t msg_size, uint16_t *signal)
{
  // The sync-word is 65 (16bit) which is split into 2x8bit, i.e., 8 (higher byte) and 1 (lower byte)
  byte_msg[0] = 8;
  byte_msg[1] = 1;

  for(uint16_t idx=2; idx < (msg_size+2)/2; idx++)
  {
    byte_msg[2*idx-2] = (uint8_t)(signal[idx-2] >> 8);   // MSB first, i.e. "higher bits", buffer's index is even: 2,4,6,...
    byte_msg[2*idx-1] = (uint8_t)signal[idx-2];          // LSB last,  i.e. "lower bits", buffer's index is odd: 3,5,7,...
  }
}

void map_32bit_to_8bit(uint8_t * byte_vector, const uint16_t number_of_bytes, uint32_t * signal)
{
  for(uint16_t idx=1; idx <= number_of_bytes/4; idx++)
  {
    byte_vector[4*idx-4] = (uint8_t)(signal[idx-1] >> 24);   // MSB first, i.e. "higher bits", byte_vector's index is 0, 4, 8,12,16,...
    byte_vector[4*idx-3] = (uint8_t)(signal[idx-1] >> 16);   //                                byte_vector's index is 1, 5, 9,13,17,...
    byte_vector[4*idx-2] = (uint8_t)(signal[idx-1] >> 8);    //                                byte_vector's index is 2, 6,10,14,18,...
    byte_vector[4*idx-1] = (uint8_t)signal[idx-1];           // LSB last,  i.e. "lower bits",  byte_vector's index is 3, 7,11,15,19,...
  }
}

void map_24bit_to_8bit(uint8_t * byte_vector, const uint16_t number_of_bytes, uint32_t * signal)
{
  for(uint16_t idx=1; idx <= number_of_bytes/3; idx++)
  { 
    byte_vector[3*idx-3] = (uint8_t)(signal[idx-1] >> 16);   // MSB first, i.e. "higher bits", byte_vector's index is 0, 3, 6, 9,...
    byte_vector[3*idx-2] = (uint8_t)(signal[idx-1] >> 8);    //                                byte_vector's index is 1, 4, 7, 10,...
    byte_vector[3*idx-1] = (uint8_t)signal[idx-1];           // LSB last,  i.e. "lower bits",  byte_vector's index is 2, 5, 8, 11,...
  }
}

void map_16bit_to_8bit(uint8_t * byte_vector, const uint16_t number_of_bytes, uint16_t * signal)
{
  for(uint16_t idx=1; idx <= number_of_bytes/2; idx++)
  {
    byte_vector[2*idx-2] = (uint8_t)(signal[idx-1] >> 8);   // MSB first, i.e. "higher bits"
    byte_vector[2*idx-1] = (uint8_t)signal[idx-1];          // LSB last,  i.e. "lower bits"
  }
}

void create_metadata_msg_radar_mmc(uint8_t * byte_msg, acc_service_envelope_metadata_t *metadata, uint32_t * offset_xyz, uint16_t length_mgf, float die_temp, uint16_t filter_bandwidth)
{
  uint8_t offset_xyz_8bit[12] = {0};        // 12=4 bytes x 3; offset_xyz[0]=offset x, offset_xyz[1]=offset y, offset_xyz[2]=offset z
  uint16_t bytes_mgf = 4*length_mgf; // 4 bytes required to store a 32 bit number
  die_temp = (die_temp * 10.0)+75.0;        // -75°C is minimum, 1 decimal digit thus *10.0
  uint16_t die_temp_16bit = (uint16_t)die_temp;
  uint8_t k = 0; // index loop

  NRF_LOG_DEBUG("die temp tx = %d|%x", die_temp_16bit, die_temp_16bit);

  // The sync-word is 131 (16bit) which is split into 2x8bit, i.e., 8 (higher byte) and 2 (lower byte)
  byte_msg[0] = 8;
  byte_msg[1] = 2;
  
  // map the floats to uint16
  byte_msg[2] = (uint8_t)(100*(metadata->start_m+0.7));   // start_tx; +0.7 because min. is -0.7;
  byte_msg[3] = (uint8_t)((10.0)*(metadata->length_m));      // length_tx
  
  // map 1x16bit |--> 2x8bit
  byte_msg[4] = (uint8_t)(metadata->data_length >> 8);
  byte_msg[5] = (uint8_t)metadata->data_length;
  byte_msg[6] = (uint8_t)(metadata->stitch_count >> 8);
  byte_msg[7] = (uint8_t)metadata->stitch_count;

  // map the step_length (float) also to uint16 and then to 2x8bit
  uint16_t step_length_tx = (uint16_t)((1000000.0)*(metadata->step_length_m));
  byte_msg[8] = (uint8_t)(step_length_tx >> 8);
  byte_msg[9] = (uint8_t)step_length_tx;

  // Add the mmc's offset
  map_32bit_to_8bit(offset_xyz_8bit, 12, offset_xyz); // map the 32 bit signal to 8 bit due to the 8 bit UART frames
  for(k = 0; k < 12; k++)
  { 
    byte_msg[k+10] = offset_xyz_8bit[k]; // append the offset which is split into 8 bit
  }

  // Bytes of the magnetic field vector
  byte_msg[k+10] = (uint8_t)(bytes_mgf >> 8);
  NRF_LOG_DEBUG("MSB bytes_mgf %d", byte_msg[k+10]);
  byte_msg[k+11] = (uint8_t)bytes_mgf;
  NRF_LOG_DEBUG("LSB bytes_mgf %d", byte_msg[k+11]);

  // Bytes of the die temperature
  byte_msg[k+12] = (uint8_t)(die_temp_16bit >> 8);
  byte_msg[k+13] = (uint8_t)die_temp_16bit;

  // Bytes of the filter bandwidth
  byte_msg[k+14] = (uint8_t)(filter_bandwidth >> 8);
  byte_msg[k+15] = (uint8_t)filter_bandwidth;

  // Test data
  //uint32_t test_offset[3] = {0x12345678, 0x12345678, 0x12345678};
  //map_32bit_to_8bit(offset_xyz_8bit, 12, test_offset); // map the 32 bit signal to 8 bit due to the 8 bit UART frames
  //for(uint8_t k = 0; k < 12; k++)
  //{ 
  //  byte_msg[k+10] = offset_xyz_8bit[k]; // append the offset which is split into 8 bit
  //}
}

void create_radar_mmc_signal_msg(uint8_t * byte_msg, uint16_t * radar_signal, 
                                        uint32_t * mgf_x, uint32_t * mgf_y, uint32_t * mgf_z,
                                        const uint16_t length_radar_signal, const uint16_t length_mgf)
{
  uint16_t idx = 0;
  uint16_t bytes_mmc = 4*length_mgf;//LENGTH_MGF_VECTOR;
  uint16_t bytes_radar = 2*length_radar_signal;

  // The sync-word is 65 (16bit) which is split into 2x8bit, i.e., 8 (higher byte) and 1 (lower byte)
  byte_msg[0] = 8;
  byte_msg[1] = 1;
  
  //uint16_t length_radar_signal_ = 2;
  //radar_signal[0] = 0x0012;
  //radar_signal[1] = 0x0034;

  //for(idx=2; idx < (2*length_radar_signal+2)/2; idx++) // add the radar signal
  for(idx=2; idx < (length_radar_signal+2); idx++) // add the radar signal
  {
    byte_msg[2*idx-2] = (uint8_t)(radar_signal[idx-2] >> 8);   // MSB first, i.e. "higher bits", buffer's index is even: 2,4,6,...
    byte_msg[2*idx-1] = (uint8_t)radar_signal[idx-2];          // LSB last,  i.e. "lower bits", buffer's index is odd: 3,5,7,...
  }
  
  // Convert the 32 bit data to 8 bit and add the data to the message
  map_32bit_to_8bit(byte_msg+bytes_radar+2, bytes_mmc, mgf_x);                // send x-axis first: start address is after 2 sync bytes and after the radar bytes
  map_32bit_to_8bit(byte_msg+bytes_radar+bytes_mmc+2, bytes_mmc, mgf_y);      // send y-axis
  map_32bit_to_8bit(byte_msg+bytes_radar+(2*bytes_mmc)+2, bytes_mmc, mgf_z);  // send z-axis last
}