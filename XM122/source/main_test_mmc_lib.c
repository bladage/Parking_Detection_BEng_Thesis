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
        mmc_backup_registers shadows = {0,0,0,0};

        
        err_twi = twi_master_init(); // start TWI_1
        if(err_twi == NRF_SUCCESS)
        {
          if(mmc_is_connected())
          {
            NRF_LOG_INFO("Sensor is connected! I soft reset the device first.");
            mmc_soft_reset();

            if(!mmc_is_bit_set(MMC_STATUS, MEAS_T_DONE))
            {
              NRF_LOG_INFO("BIT is 0!");  
            }
              
            if(mmc_is_bit_set(MMC_PRODUCT_ID, (1 << 4)))
            {
              NRF_LOG_INFO("BIT is 1!");  
            }
            
            mmc_backup_registers backup_reg = {0,0,0,0};
            backup_reg.internal_ctrl_reg_0 = 0b00001000;
            if(mmc_is_bit_set_backup_reg(&backup_reg.internal_ctrl_reg_0, (1 << 3)))
            {
              NRF_LOG_INFO("BIT is 1!"); 
            }
            
            float die_temp = mmc_get_die_temperature();
            //printf("Die temp: %.6f", die_temp); 

            mmc_enable_interrupt_pin(&shadows.internal_ctrl_reg_0);
            mmc_disable_interrupt_pin(&shadows.internal_ctrl_reg_0);
            
            backup_reg.internal_ctrl_reg_1 = 0b00011000;
            if(mmc_are_yz_channels_enabled(&backup_reg.internal_ctrl_reg_1))
            {
              NRF_LOG_INFO("Channel is on!"); 
            }
            
            uint16_t bw = mmc_get_filter_bandwidth(&backup_reg.internal_ctrl_reg_1);
            NRF_LOG_INFO("B=%d\n", bw);
            
            backup_reg.internal_ctrl_reg_2 = 0b00000000;
            if(mmc_is_cont_mode_enabled(&backup_reg.internal_ctrl_reg_2))
            {
              NRF_LOG_INFO("Cont. mode is on!"); 
            }
            
            //uint16_t freq = 13;
            //backup_reg.internal_ctrl_reg_2 = 0b00000000;
            //for(uint8_t k=0; k < 8; k++)
            //{
            //  freq = mmc_get_frequency_cont_mode(&backup_reg.internal_ctrl_reg_2);
            //  NRF_LOG_INFO("k=%d: f=%d\n", k, freq);
            //  backup_reg.internal_ctrl_reg_2++;
            //}
            
            backup_reg.internal_ctrl_reg_0 = 0b00100000;
            backup_reg.internal_ctrl_reg_2 = 0b10001000;
            if(mmc_is_pso_enabled(&backup_reg.internal_ctrl_reg_0, &backup_reg.internal_ctrl_reg_2))
            {
              NRF_LOG_INFO("PSO is enabled!");
            }

            uint16_t rep_number = 13;
            backup_reg.internal_ctrl_reg_2 = 0b00000000;
            for(uint8_t k=1; k <= 8; k++)
            {
              rep_number = mmc_get_repetition_number_pso(&backup_reg.internal_ctrl_reg_2);
              NRF_LOG_INFO("k=%d: rep_num=%d, reg=0x%x\n", k, rep_number, backup_reg.internal_ctrl_reg_2);
              backup_reg.internal_ctrl_reg_2 = (k << 4);
            }
          }         
        }
        while(true);

        return 0;
}