/**
 * @brief Sending the A111's data via serial (UART/USB) to the PC.
 *
 * @author Bennet Ladage
 * @date 2022-05-14
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


// Defines for UART
#define UART_TX_BUFF_SIZE 4096
#define UART_RX_BUFF_SIZE 4096

#define SIGNAL_SIZE 256                    // uint16 vector
#define BYTE_BUFFER_SIZE 514 //2*SIGNAL_SIZE+2  // uint8 vector, hence 2*uint16; +2 because two sync bytes

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
#define RANGE_LENGTH (0.6f)
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
  @brief  The function maps every data type to uint8_t and it builts 
          the frame, which contains the RADAR signal and the RADAR's 
          metadata
**/ 
//static void create_8bit_frame(uint8_t *byte_buffer, uint16_t *signal, acc_service_envelope_metadata_t *metadata)
static void create_8bit_frame(uint8_t *byte_buffer, const uint16_t frame_size, uint16_t *signal, acc_service_envelope_metadata_t *metadata)
{
  // The sync-word is 65 (16bit) which is split into 2x8bit, i.e. 8 (higher byte) and 1 (lower byte)
  byte_buffer[0] = 8;
  byte_buffer[1] = 1;
  
  // map the floats to uint16
  byte_buffer[2] = (uint8_t)((10.0)*(metadata->start_m+0.7));   // start_tx
  byte_buffer[3] = (uint8_t)((10.0)*(metadata->length_m));      // length_tx
  //uint8_t start_tx = (uint8_t)((10.0)*(metadata->start_m+0.7));
  //uint8_t length_tx = (uint8_t)((10.0)*(metadata->length_m));
  
  // map 1x16bit |--> 2x8bit
  byte_buffer[4] = (uint8_t)(metadata->data_length >> 8);
  byte_buffer[5] = (uint8_t)metadata->data_length;
  byte_buffer[6] = (uint8_t)(metadata->stitch_count >> 8);
  byte_buffer[7] = (uint8_t)metadata->stitch_count;

  // map the step_length (float) also to uint16 and then to 2x8bit
  uint16_t step_length_tx = (uint16_t)((1000000.0)*(metadata->step_length_m));
  byte_buffer[8] = (uint8_t)(step_length_tx >> 8);
  byte_buffer[9] = (uint8_t)step_length_tx;

  
  //data_length = (((*frame_size-START_BYTES_8BFR)/2)-METADATA_WORDS_8BFR)/2
  uint16_t tempH, tempL, M, idx;
  M = metadata->data_length/2;
  printf("M=%d\n", M);

  for(idx=10; idx < 1240; idx++)
  //for(uint16_t idx=2; idx <= *frame_size/2; idx++)
  //for(uint16_t idx=10; idx <= M; idx++)
  {
    byte_buffer[2*idx-10] = (uint8_t)(signal[idx-10] >> 8);   // MSB first, i.e. "higher bits", buffer's index even: 10,12,14,...
    byte_buffer[2*idx-9] = (uint8_t)signal[idx-10];          // LSB last,  i.e. "lower bits", buffer's index odd: 11,13,15,...
    
    tempH = 2*idx-10;
    tempL = 2*idx-9;
    //byte_buffer[2*idx-2] = (uint8_t)(signal[idx-2] >> 8);   // MSB first, i.e. "higher bits", buffer's index even: 2,4,6,...
    //byte_buffer[2*idx-1] = (uint8_t)signal[idx-2];          // LSB last,  i.e. "lower bits", buffer's index odd: 3,5,7,...
    //NRF_LOG_INFO("%d", signal[idx-10]);
    //NRF_LOG_INFO("idx=%d, H%d: %d", idx, tempH, byte_buffer[2*idx-10]);
    //NRF_LOG_INFO("idx=%d, L%d: %d", idx, tempL, byte_buffer[2*idx-9]);
  }  
    NRF_LOG_INFO("%d", signal[idx-1-10]);
    NRF_LOG_INFO("idx=%d, H%d: %d", idx, tempH, byte_buffer[2*(idx-1)-10]);
    NRF_LOG_INFO("idx=%d, L%d: %d", idx, tempL, byte_buffer[2*(idx-1)-9]);
}

/*
  Acconeer's functions low power envelope, modified by Bennet
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

//! @brief find the peak and its index by using the envelope service; nec.?
static void peakfinder_envelope_service(acc_service_handle_t handle, uint16_t *result1, uint16_t *result2)
{
	acc_service_envelope_metadata_t metadata = { 0 };

	acc_service_envelope_get_metadata(handle, &metadata);

	uint16_t                           envelope_data[metadata.data_length];
	acc_service_envelope_result_info_t result_info;

	uint16_t max_amplitude_index = 0;
	uint16_t max_amplitude       = 0;

	if (acc_service_envelope_get_next(handle, envelope_data, metadata.data_length, &result_info))
	{
		max_amplitude_index = 0;
		max_amplitude       = 0;

		for (uint16_t index = 0; index < metadata.data_length; index++)
		{
			if (envelope_data[index] > max_amplitude)
			{
				max_amplitude       = envelope_data[index];
				max_amplitude_index = index;
			}
		}
	}

	*result1 = max_amplitude;
	*result2 = max_amplitude_index;
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
          Data structures of the Envelope Service       
        */
	acc_service_envelope_metadata_t metadata = { 0 };           // Start/length of sweep, signal length, stitch_count, Delta data point in m
	acc_service_envelope_get_metadata(handle, &metadata);

	uint16_t envelope_data[metadata.data_length];               // raw data of the RX-signal
        //uint16_t *envelope_data;                                      // raw data of the RX-signal, point to the memory which is allocated by the RSS
	acc_service_envelope_result_info_t result_info;             // missed_data, sensor_communication_error, data_saturated, data_quality_warning

        /*
          Setup misc
        */
        uint16_t idx; //, tempH, tempL;

        
        /*
          The action begins here!
        */
	while(true)
	{   
          // get the next RX-signal    
          if(acc_service_envelope_get_next(handle, envelope_data, metadata.data_length, &result_info))
          //if(acc_service_envelope_get_next_by_reference(handle, &envelope_data, &result_info))
          {
            const uint16_t frame_size = START_BYTES_8BFR+2*(metadata.data_length+METADATA_WORDS_8BFR); // 2 start bytes, 4 words metadata; 2*() because 16bit |--> 2x8bit
            uint8_t byte_buffer[frame_size];

            // build the uart buffer (byte_buffer) which is a 8bit array       
            create_8bit_frame(byte_buffer, frame_size, envelope_data, &metadata);


            // send the data via UART
            //for(idx=0; idx < BYTE_BUFFER_SIZE; idx++)
            for(idx=0; idx < frame_size; idx++)
            {
              //app_uart_put(byte_buffer[idx]);
              //while(app_uart_put(byte_buffer[idx]) != NRF_SUCCESS);
              while(app_uart_put(byte_buffer[idx]) != NRF_SUCCESS);
              //nrf_delay_ms(50); 
            }
          }

          //acc_integration_sleep_until_periodic_wakeup(); // save energy and sleep a bit
	}
        
        // shut down the envelope service
	acc_service_deactivate(handle);
	acc_service_destroy(&handle);
	acc_rss_deactivate();

	return EXIT_SUCCESS;
}
