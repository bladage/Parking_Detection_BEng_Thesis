/**
 * @brief An example source code that shows the handling of the MEMSIC MMC5983MA sensor via TWI (I2C).
 * The sensor is connected to the Acconeer XB122 or XM122, respectively. 
 *
 * @author Bennet Ladage
 * @date 2022-06-20
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
#include "nrf_drv_twi.h"
#include "nrf_drv_twis.h"
#include "memsic_mmc5983ma.h"
#include <inttypes.h> // printf %"PRIu32" for uint32_t

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
        ret_code_t err_twi;                       // error code twi
        mmc_backup_registers shadows = {0,0,0,0}; // shadow registers, i.e., backup of internal control registers 0 to 3
        
        float temperature = 0.0;
        float offset_xyz_mT[3] = {0};
        float mg_field_xyz_mT[3] = {0};
        char xyz [3] = {'x', 'y', 'z'};
        
        err_twi = twi_master_init(); // start TWI_1
        if(err_twi == NRF_SUCCESS)
        {
          if(mmc_is_connected())
          {
            NRF_LOG_INFO("Sensor is connected! I soft reset the device first.");
            mmc_soft_reset(); // adjust parameters like the filter bandwidth after this line

            //mmc_execute_set_operation(); // restore sensor characteristics if a magnetic field > 10 Gauss was applied
            
            /*
              Start the measurements
            */  
            temperature = mmc_get_die_temperature();
            printf("MMC5983MA's die temperature: %f C\n", temperature);   
            
            for(int16_t k = 0; k < 100; k++) 
            {
              // Measure the magnetic field and its offset due to the Wheatstone bridge circuit:
              mmc_get_bridge_offset_milli_tesla(offset_xyz_mT, mg_field_xyz_mT);
              printf("B field \t%d: \t(x|y|z)=(%f|%f|%f) mT\n", k, mg_field_xyz_mT[0], mg_field_xyz_mT[1], mg_field_xyz_mT[2]);
              printf("Offset \t%d: \t(x|y|z)=(%f|%f|%f) mT\n", k, offset_xyz_mT[0], offset_xyz_mT[1], offset_xyz_mT[2]);
            }
          }         
        }
        while(true);

        return 0;
}