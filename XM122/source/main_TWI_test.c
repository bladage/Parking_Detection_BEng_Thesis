/**
 * @brief Test the TWI-Connection
 *
 * @author Bennet Ladage
 * @date 2022-06-15
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

// Bennet's new includes
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

// Bennet's new includes for I2C/TWI
#include "boards.h"
#include "app_util_platform.h"
//#include "app_error.h"
//#include "nrfx.h"
//#include "nrfx_twi.h"
//#include "nrfx_twim.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_twis.h"
#include "memsic_mmc5983ma.h"

//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"

// Using TWI_1 here, because TWI_0 uses the same base address as SPI_0 which is propably used by the A111 chip
#define TWI_INSTANCE_ID 1
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID); 


static ret_code_t twi_init(void)
{
  ret_code_t err_code;
  
  const nrf_drv_twi_config_t twi_config = {
  .scl                = TWI_CLK_Pin,            // SCL=23, XB122
  .sda                = TWI_DATA_Pin,           // SDA=21, XB122
  .frequency          = NRF_DRV_TWI_FREQ_100K,  // communication speed to 100k, (250k, 400k possible too)
  .interrupt_priority = APP_IRQ_PRIORITY_MID,   // Interrupt priority, not high if soft-device is used
  .clear_bus_init     = false                   // automatic bus clearing 
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL); // initialize the twi
  APP_ERROR_CHECK(err_code);                                    // check if any error occured during initialization

  if(err_code == NRF_SUCCESS)
  {
    nrf_drv_twi_enable(&m_twi); // enable communication via TWI
  }
  return err_code;
}

extern int acconeer_main(int argc, char *argv[]);


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

        nrf_gpio_cfg_output(LED_2);

        /*
          TWI/I2C
        */
        ret_code_t err_twi;                 // error code twi
        uint8_t address = MMC_I2C_ADDRESS;  // 7 bit address of the sensor
        uint8_t rx_byte = 0x00;             // receive byte for the twi transmission

        
        err_twi = twi_init(); // start TWI_1
        if(err_twi == NRF_SUCCESS)
        {
          while(nrf_drv_twi_is_busy(&m_twi)); // wait if TWI is busy

          err_twi = nrf_drv_twi_rx(&m_twi, address, &rx_byte, sizeof(rx_byte)); // read some data from the sensor
          if(err_twi == NRF_SUCCESS) // test if ACK is received
          {
            NRF_LOG_INFO("Successfully detected a device at address: 0x%x", address); // let the users know its working
            NRF_LOG_INFO("Sample Data: %d", rx_byte);
          }          
        }
        while(true);

        return 0;
}