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

// Bennet's new includes for TWI/I2C
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


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

// Defines for the Memsic sensor
//#define POWERPIN      NRF_GPIO_PIN_MAP(0, 22)
#define TWI_SDA       NRF_GPIO_PIN_MAP(0, 24)
#define TWI_SCL       NRF_GPIO_PIN_MAP(0, 22)
#define TWI_INSTANCE_ID 0 // TWI instance ID
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID); // create a handle which will point to TWI instance, in this case its TWI_1
//static const nrf_drv_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID); // create a handle which will point to TWI instance, in this case its TWI_1

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
 * Function for configuring UICR_REGOUT0 register
 * to set GPIO output voltage to 3.0V.
 */
//static void gpio_output_voltage_setup(void)
//{
//    // Configure UICR_REGOUT0 register only if it is set to default value.
//    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
//        (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))
//    {
//        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
//        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

//        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
//                            (UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos);

//        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
//        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

//        // System reset is needed to update UICR registers.
//        NVIC_SystemReset();
//    }
//}

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

// Init TWI/I2C
void twi_init(void)
{
  ret_code_t err_code; // a variable to hold error code

// Create a struct with configurations and pass the values to these configurations.
  const nrf_drv_twi_config_t twi_config = {
    .scl                = TWI_SCL, 
    .sda                = TWI_SDA,
    .frequency          = NRF_DRV_TWI_FREQ_100K, // set the communication speed to 100K, we can select 250k or 400k as well
    .interrupt_priority = APP_IRQ_PRIORITY_MID, // Interrupt priority is set to high, keep in mind to change it if you are using a soft-device
    .clear_bus_init     = false // automatic bus clearing 

  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL); // initialize the twi
  APP_ERROR_CHECK(err_code); // check if any error occured during initialization

  nrf_drv_twi_enable(&m_twi); // enable the twi comm so that its ready to communicate with the sensor

}

int main(void)
{       
        //gpio_output_voltage_setup();
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
        uint8_t byte_msg_metadata[10];                              // the metada as a message in bytes for uart
	acc_service_envelope_metadata_t metadata = { 0 };           // Start/length of sweep, signal length, stitch_count, Delta data point in m
	acc_service_envelope_get_metadata(handle, &metadata);
        create_metadata_msg(byte_msg_metadata, &metadata);

	uint16_t envelope_data[metadata.data_length];               // raw data of the RX-signal
        //uint16_t *envelope_data;                                  // raw data of the RX-signal, point to the memory which is allocated by the RSS
	acc_service_envelope_result_info_t result_info;             // missed_data, sensor_communication_error, data_saturated, data_quality_warning
        
        /*
          Setup Memsic
        */
        //nrf_gpio_cfg_output(POWERPIN); // power supply for the sensor
        //nrf_gpio_pin_set(POWERPIN);    // should always be on. maybe later not
        ret_code_t err_code; // a variable to hold error code value
        uint8_t address = 0x61; // address of the sensor
        uint8_t sample_data = 0x00; // sample data initialized with 0 value.

        // initialize the Logger so that we can print msgs on the logger
        APP_ERROR_CHECK(NRF_LOG_INIT(NULL)); 
        NRF_LOG_DEFAULT_BACKENDS_INIT();

        NRF_LOG_INFO("Application Started");

        NRF_LOG_FLUSH(); // flushing is necessary if deferred is set to 1(check this video tutorial to know it better)

        twi_init(); // call the twi initialization function

        //// read some data from the sensor
        //err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        //if(err_code == NRF_SUCCESS) // if reading data was successful
        //{

        //  NRF_LOG_INFO("Successfully detected a device at address: 0x%x", address); // let the users know its working
  
        //}

        //NRF_LOG_FLUSH();        

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
          for(idx=0; idx < 10; idx++)
          {
            while(app_uart_put(byte_msg_metadata[idx]) != NRF_SUCCESS); // send metadata, 10 bytes
          }

          app_uart_get(&ack); // get the PC's ACK='o'
          if(ack == 'o')
          {
            while(app_uart_put('k') != NRF_SUCCESS); // send the uC's ACK="k"
            break;
          }
        } 
        
        /*
          Send the radar signal periodically
        */  
	while(true)
	{
          // get the next RX-signal; by_reference "optimization"    
          if(acc_service_envelope_get_next(handle, envelope_data, metadata.data_length, &result_info))
          {
            const uint16_t msg_size = 2*metadata.data_length+2; // 2* due to 1x16 bit |--> 2x8 bit and +2 due to two sync bytes
            uint8_t signal_msg[msg_size];

            // built the uart data frame (signal_msg) which is a 8bit array       
            create_signal_msg(signal_msg, msg_size, envelope_data);

            // send the radar signal via UART
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