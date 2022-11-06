/**
 * @brief Sending the A111's data via serial (UART/USB) to the PC.
 *
 * @author Bennet Ladage
 * @date 2022-06-26
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 */

#include <stdbool.h>
#include <stdlib.h>

#include "acc_hal_integration_xm122.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"

// Bennet's new includes for UART
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "board_xm122.h"
#include "app_error.h"
#include "nrf.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

// Acconeer's includes low power envelope
#include <stddef.h>
//#include <stdint.h>
//#include <stdio.h>
//#include <stdlib.h>

#ifdef BROADCAST_BATTERY_INFO
#include "acc_battery_info.h"
#endif

#include "acc_bluetooth_beacon_xm122.h"
#include "acc_definitions_common.h"
#include "acc_hal_definitions.h"
#include "acc_hal_integration.h"
#include "acc_integration.h"
#include "acc_rss.h"
#include "acc_service.h"
#include "acc_service_envelope.h"
#include "acc_version.h"

// Bennet's new includes for I2C/TWI
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_twis.h"
#include "memsic_mmc5983ma.h"


// Defines for UART
#define UART_TX_BUFF_SIZE 4096
#define UART_RX_BUFF_SIZE 4096

// Defines for low power envelope service
#ifndef SUSPEND_TIME_BETWEEN_UPDATES_MS
#define SUSPEND_TIME_BETWEEN_UPDATES_MS (10000) // 0.1Hz
#endif

#ifndef USE_BLE_ADVERTISING
#define USE_BLE_ADVERTISING (0) //1=BT LE on
#endif

#ifndef POWER_SAVE_MODE
#define POWER_SAVE_MODE OFF
#endif

#ifndef RANGE_LENGTH
#define RANGE_LENGTH (0.8f)
#endif

#ifndef SERVICE_PROFILE
#define SERVICE_PROFILE 1
#endif

#define PASTER(x, y)    x ## y
#define EVALUATOR(x, y) PASTER(x, y)

#define SELECTED_POWER_SAVE_MODE (EVALUATOR(ACC_POWER_SAVE_MODE_, POWER_SAVE_MODE))
#define SELECTED_SERVICE_PROILE  (EVALUATOR(ACC_SERVICE_PROFILE_, SERVICE_PROFILE))

#ifndef HWAAS
#define HWAAS (10)
#endif

#ifndef DOWNSAMPLING_FACTOR
#define DOWNSAMPLING_FACTOR (1)
#endif

// Defines for the 8 bit frame
#define START_BYTES_8BFR 2
#define METADATA_WORDS_8BFR 4

// Defines for the MEMSIC sensor
#define LENGTH_MGF_VECTOR    100  // length of the MaGnetic Field vector
#define BYTES_META_DATA_ALL  24    // how much bytes contain the meta data of the radar & magnetic field sensor

/*
  Acconeer's functions of the main-template
*/
int _write(int file, char *ptr, int len)
{
  static char log_buffer[256];
  static int  log_index;

  for (int i = 0; i < len; i++)
  {
    log_buffer[log_index] = *ptr++;
    if (log_buffer[log_index] == '\n' || (log_index == (sizeof(log_buffer) - 2)))
    {
      if (log_buffer[log_index] != '\n')
      {
        log_index++;
      }

      log_buffer[log_index] = 0;
      // The additional "\n" is needed when running from SEGGER Embedded Studio
      NRF_LOG_INFO("%s\n", NRF_LOG_PUSH(log_buffer));
      log_index = 0;
    }
    else
    {
      log_index++;
    }
  }

  return len;
}


extern void *_sbrk(int incr);


extern void *__HeapBase;
extern void *__HeapLimit;


// The built in version for newlib of _sbrk() does not check
// the heap for overflow so we need our own version of it
void *_sbrk(int incr)
{
  static void *heap = NULL;
  void        *prev_heap;
  void        *next_heap;

  if (heap == NULL)
  {
    heap = (void * )&__HeapBase;
  }

  prev_heap = heap;
  next_heap = (void *)((unsigned int)heap + incr);

  if (next_heap >= (void *)&__HeapLimit)
  {
    return (void *)-1;
  }
  else
  {
    heap = next_heap;
    return prev_heap;
  }
}

/*
  New functions by Bennet
*/

//! @brief A simple error handler for uart if something goes wrong...
void uart_err_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/** 
  @brief  The function maps the metadata to uint8_t and it builts 
          the message which can be send via uart
  @pre    byte_msg must hold 10 elements!
**/ 
static void create_metadata_msg(uint8_t *byte_msg, acc_service_envelope_metadata_t *metadata)
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

/** 
  @brief  The function maps the uint16_t signal to uint8_t and it builts 
          the message, which contains the RADAR signal
**/ 
static void create_signal_msg(uint8_t *byte_msg, const uint16_t msg_size, uint16_t *signal)
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

/** 
  @brief  The function maps a uint32_t vector to uint8_t
**/ 
static void map_32bit_to_8bit(uint8_t * byte_vector, const uint16_t number_of_bytes, uint32_t * signal)
{
  for(uint16_t idx=1; idx <= number_of_bytes/4; idx++)
  {
    byte_vector[4*idx-4] = (uint8_t)(signal[idx-1] >> 24);   // MSB first, i.e. "higher bits", byte_vector's index is 0, 4, 8,12,16,...
    byte_vector[4*idx-3] = (uint8_t)(signal[idx-1] >> 16);   //                                byte_vector's index is 1, 5, 9,13,17,...
    byte_vector[4*idx-2] = (uint8_t)(signal[idx-1] >> 8);    //                                byte_vector's index is 2, 6,10,14,18,...
    byte_vector[4*idx-1] = (uint8_t)signal[idx-1];           // LSB last,  i.e. "lower bits",  byte_vector's index is 3, 7,11,15,19,...
  }
}

/** 
  @brief  The function maps the metadata to uint8_t and it builts 
          the message which can be send via uart. The meta data is
          the radar's meta data and the magnetic field sensor's bridge offset.
  @pre    byte_msg must hold 10+12+2=24 elements (BYTES_META_DATA_ALL) and offset_xyz 3
**/ 
static void create_metadata_msg_radar_mmc(uint8_t * byte_msg, acc_service_envelope_metadata_t *metadata, uint32_t * offset_xyz)
{
  uint8_t offset_xyz_8bit[12] = {0}; // 12=4x3; offset_xyz[0]=offset x, offset_xyz[1]=offset y, offset_xyz[2]=offset z
  uint16_t bytes_mgf = 4*LENGTH_MGF_VECTOR; // 4 bytes required to store a 32 bit number
  uint8_t k = 0; // index loop

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

  // Add the mmc's offset
  map_32bit_to_8bit(offset_xyz_8bit, 12, offset_xyz); // map the 32 bit signal to 8 bit due to the 8 bit UART frames
  for(k = 0; k < 12; k++)
  { 
    byte_msg[k+10] = offset_xyz_8bit[k]; // append the offset which is split into 8 bit
  }

  // Bytes of the magnetic field vector
  byte_msg[k+10] = (uint8_t)(bytes_mgf >> 8);
  byte_msg[k+11] = (uint8_t)bytes_mgf;

  // Test data
  //uint32_t test_offset[3] = {0x12345678, 0x12345678, 0x12345678};
  //map_32bit_to_8bit(offset_xyz_8bit, 12, test_offset); // map the 32 bit signal to 8 bit due to the 8 bit UART frames
  //for(uint8_t k = 0; k < 12; k++)
  //{ 
  //  byte_msg[k+10] = offset_xyz_8bit[k]; // append the offset which is split into 8 bit
  //}
}

/** 
  @brief  The function maps the uint16_t signal to uint8_t and it builds 
          the message, which contains the RADAR signal
**/ 
static void create_radar_mmc_signal_msg(uint8_t * byte_msg, uint16_t * radar_signal, 
                                        uint32_t * mgf_x, uint32_t * mgf_y, uint32_t * mgf_z,
                                        const uint16_t length_radar_signal) // , const uint16_t length_mgf
{
  uint16_t idx = 0;
  uint16_t bytes_mmc = 4*LENGTH_MGF_VECTOR;
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
  map_32bit_to_8bit(byte_msg+bytes_radar+2, bytes_mmc, mgf_x);                        // send x-axis first: start address is after 2 sync bytes and after the radar bytes
  map_32bit_to_8bit(byte_msg+bytes_radar+bytes_mmc+2, bytes_mmc, mgf_y);      // send y-axis
  map_32bit_to_8bit(byte_msg+bytes_radar+(2*bytes_mmc)+2, bytes_mmc, mgf_z);  // send z-axis last
}

/*
  Acconeer's functions: low power envelope, modified by Bennet
*/

//! @brief setup envelope service
static acc_service_handle_t create_envelope_service(void)
{
  bool                 success        = true;
  acc_service_handle_t service_handle = NULL;

  float range_start_m  = 0.2f;
  float range_length_m = RANGE_LENGTH;

  acc_service_configuration_t envelope_configuration = acc_service_envelope_configuration_create();

  if (envelope_configuration == NULL)
  {
    printf("Could not create envelope configuration\n");
    success = false;
  }

  if (success)
  {
    acc_service_requested_start_set(envelope_configuration, range_start_m);
    acc_service_requested_length_set(envelope_configuration, range_length_m);
    acc_service_power_save_mode_set(envelope_configuration, SELECTED_POWER_SAVE_MODE);
    acc_service_profile_set(envelope_configuration, SELECTED_SERVICE_PROILE);
    acc_service_hw_accelerated_average_samples_set(envelope_configuration, HWAAS);
    acc_service_envelope_downsampling_factor_set(envelope_configuration, DOWNSAMPLING_FACTOR);

    //acc_service_mur_set(envelope_configuration, ACC_SERVICE_MUR_9); // change MUR=17.3 m instead of 11.5 m
                

    service_handle = acc_service_create(envelope_configuration);

    if (service_handle == NULL)
    {
      printf("Could not create envelope service\n");
      success = false;
    }
  }

  acc_service_envelope_configuration_destroy(&envelope_configuration);

  return success ? service_handle : NULL;
}



int main(void)
{
  /*
    General Setup by Acconeer
  */ 
  nrf_drv_clock_lfclk_request(NULL);          // lfclk: low-frequency clock
  APP_ERROR_CHECK(nrf_drv_clock_init());      // initializing the nrf_drv_clock module 
  APP_ERROR_CHECK(nrf_drv_power_init(NULL));  // power module driver processes all the interrupts from power system
  
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));        // initializing the logs
  NRF_LOG_DEFAULT_BACKENDS_INIT();            // initializing default backends, for logging

  APP_ERROR_CHECK(app_timer_init());          // initializing the timer module
  APP_ERROR_CHECK(nrf_pwr_mgmt_init());       // initializing power management
      
  nrf_delay_ms(10);                           // Delay in order to handle rampup of voltage from buck converter
  acc_hal_integration_xm122_init();

  /*
    UART setup, modified by Bennet
  */
  uint32_t err_code_UART;                        // error value for UART communication
  const app_uart_comm_params_t config_UART =     // struct to hold the uart configurations
  {
    UART_RX, 
    UART_TX,
    UART_RTS,
    UART_CTS,
    APP_UART_FLOW_CONTROL_DISABLED, // hardware flow control disabled
    false, // parity = none
    #if defined (UART_PRESENT)
    NRF_UART_BAUDRATE_115200
    #else
    NRF_UARTE_BAUDRATE_115200
    #endif
  };

  // Initialize the UART module:
  APP_UART_FIFO_INIT(&config_UART, 
  UART_RX_BUFF_SIZE, 
  UART_TX_BUFF_SIZE, 
  uart_err_handle, 
  APP_IRQ_PRIORITY_LOWEST, 
  err_code_UART);
  NRF_LOG_INFO("Error Code UART: %d", err_code_UART);
  APP_ERROR_CHECK(err_code_UART); // check if everything initialized correctly

  /*
    Setup Envelope Service
  */
  printf("Acconeer software version %s\n", acc_version_get());
  acc_hal_t hal = *acc_hal_integration_get_implementation();
  acc_integration_set_periodic_wakeup(SUSPEND_TIME_BETWEEN_UPDATES_MS);
  hal.log.log_level = ACC_LOG_LEVEL_ERROR;

  if(!acc_rss_activate(&hal))
  {
    return EXIT_FAILURE;
  }

  acc_service_handle_t handle = create_envelope_service();
  if(handle == NULL)
  {
    printf("acc_service_create() failed\n");
    return EXIT_FAILURE;
  }

  if(!acc_service_activate(handle))
  {
    printf("acc_service_activate() failed\n");
    acc_service_destroy(&handle);
    acc_rss_deactivate();
    return EXIT_FAILURE;
  }

  /*
    Data structures of the Envelope Service; --> in INTERRUPT!       
  */
  uint8_t byte_msg_metadata[BYTES_META_DATA_ALL];             // the metada as a message in bytes for uart
  acc_service_envelope_metadata_t metadata = { 0 };           // Start/length of sweep, signal length, stitch_count, Delta data point in m
  acc_service_envelope_get_metadata(handle, &metadata);
  //create_metadata_msg(byte_msg_metadata, &metadata);// TBD: replace with meta data radar + offset xyz

  uint16_t envelope_data[metadata.data_length];               // raw data of the RX-signal
  //uint16_t *envelope_data;                                  // raw data of the RX-signal, point to the memory which is allocated by the RSS
  acc_service_envelope_result_info_t result_info;             // missed_data, sensor_communication_error, data_saturated, data_quality_warning
  
  // 2* due to 1x16 bit |--> 2x8 bit; +2 due to two sync bytes; 
  // 3*4=12 due to 3 axis of the magnetic field sensor (x,y,z) and the sensor's data is 32 bit which needs 4 transmission bytes:
  const uint16_t msg_size = 2*metadata.data_length + 2 + 12*LENGTH_MGF_VECTOR;
  // bytes of the uart message which contain the radar data & the magnetic field data:         
  uint8_t signal_msg[msg_size];                               

  /*  
    Setup Memsic MMC5983MA Magnetic Field Sensor (connected via TWI/I2C), see memsic_mmc5983ma.h
  */
  ret_code_t err_twi;                       // error code twi
  mmc_backup_registers shadows = {0,0,0,0}; // shadow registers, i.e., backup of internal control registers 0 to 3
        
  //float mmc_die_temperature = 0.0;
  uint32_t offset_xyz[3] = {0};
  uint32_t mgf_x[LENGTH_MGF_VECTOR] = {0}; // MaGnetic Field x-axis
  uint32_t mgf_y[LENGTH_MGF_VECTOR] = {0}; // MaGnetic Field y-axis
  uint32_t mgf_z[LENGTH_MGF_VECTOR] = {0}; // MaGnetic Field z-axis
  bool mmc_ready = false;
        
  err_twi = twi_master_init(); // start TWI_1; the TWI_DATA_Pin (21)=SDA and TWI_CLK_Pin (23)=SCL are used of the XB122
  if(err_twi == NRF_SUCCESS)
  {
    if(mmc_is_connected())
    {
      mmc_soft_reset(); // adjust parameters like the filter bandwidth after this line
      //mmc_execute_set_operation(); // restore sensor characteristics if a magnetic field > 10 Gauss was applied
      mmc_ready = true; // tbd: check if sensor still there
      
      mmc_get_bridge_offset_only(offset_xyz); // get the offset of the Wheatstone Bridge in raw samples; additional meta data to the radar's meta data
      create_metadata_msg_radar_mmc(byte_msg_metadata, &metadata, offset_xyz);
    }
  }

  /*
    Setup misc
  */
  uint16_t idx;                 // index for loops
  uint8_t ack = 'z';            // acknowledge which received from the pc
  nrf_gpio_cfg_output(LED_2);   // XB122's LED
        
  /*
    Toggle the LED as a restart indication
  */
  for(idx=0; idx < 10; idx++)
  {
    nrf_gpio_pin_toggle(LED_2);
    nrf_delay_ms(50);
  }
        
  /*
    Wait until the acknowledge arrives from the pc and send the metadata periodically
  */
  while(true) 
  {
    for(idx=0; idx < BYTES_META_DATA_ALL; idx++)
    {
      while(app_uart_put(byte_msg_metadata[idx]) != NRF_SUCCESS); // send metadata, 24 bytes (BYTES_META_DATA_ALL)
    }

    app_uart_get(&ack); // get the PC's ACK='o'
    if(ack == 'o')
    {
      while(app_uart_put('k') != NRF_SUCCESS); // send the uC's ACK="k"
      break;
    }
  } 
        
  /*
    Send the radar signal & the mmc signal periodically
  */  
  while(true)
  {
    // get the next RX-signal; by_reference "optimization"    
    if(acc_service_envelope_get_next(handle, envelope_data, metadata.data_length, &result_info))
    {
      // get the magnetic field of the Memsic sensor
      if(mmc_ready)
      {
        for(idx=0; idx < LENGTH_MGF_VECTOR; idx++)
        {
          mmc_get_magnetic_field_xyz(&mgf_x[idx], &mgf_y[idx], &mgf_z[idx]);
        }
      }

      // built the uart data frame (signal_msg) which is a 8bit array       
      //mgf_x[0] = 0x12345678;
      //mgf_x[1] = 0x9ABCDEF0;
      //mgf_y[0] = 0x87654321;
      //mgf_y[1] = 0x0FEDCBA9;
      //mgf_z[0] = 0xA1B2C3D4;
      //mgf_z[1] = 0xE5F6A7B8;
      
      //uint16_t radar_signal[2] = {0xABCD, 0xEFEF};
      //uint16_t len_radar_ = 2;
      //const uint16_t msg_size_ = 2*len_radar_ + 2 + 12*LENGTH_MGF_VECTOR; //12=3 dimension x 4 bytes
      //uint8_t signal_msg_[msg_size_];

      create_radar_mmc_signal_msg(signal_msg, envelope_data, mgf_x, mgf_y, mgf_z, metadata.data_length);
      //create_radar_mmc_signal_msg(signal_msg_, radar_signal, mgf_x, mgf_y, mgf_z, len_radar_);
      
      // send the radar signal via UART
      //for(idx=0; idx < msg_size; idx++)
      for(idx=0; idx < msg_size; idx++)
      {
        while(app_uart_put(signal_msg[idx]) != NRF_SUCCESS);
      }
    }
    //acc_integration_sleep_until_periodic_wakeup(); // save energy and sleep a bit
  }      
        
  // shut down the envelope service, by Acconeer
  acc_service_deactivate(handle);
  acc_service_destroy(&handle);
  acc_rss_deactivate();

  return EXIT_SUCCESS;
}
