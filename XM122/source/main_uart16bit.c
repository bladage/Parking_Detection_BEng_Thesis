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


#define UART_TX_BUFF_SIZE 128
#define UART_RX_BUFF_SIZE 128

#define SIGNAL_SIZE 32                   // uint16 vector
#define BYTE_BUFFER_SIZE 2*SIGNAL_SIZE+2  // uint8 vector, hence 2*uint16; +2 because two sync bytes


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

// A simple error handler for uart if something goes wrong...
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
        const app_uart_comm_params_t config_UART = // struct to hold the uart configurations
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

        /*
          Signal setup & map 1x16 bit vector to a 2x8 bit vector (uint16 |--> 2x uint8)
        */
        uint16_t signal[SIGNAL_SIZE];
        uint8_t byte_buffer[BYTE_BUFFER_SIZE];
        uint16_t idx; //, tempH, tempL;
        
        NRF_LOG_INFO("Discrete Sinus:%f", M_PI);
        for(idx=0; idx < SIGNAL_SIZE; idx++)
        {
          signal[idx] = abs(1000*sin(2.0*M_PI*idx*2.0/((double)SIGNAL_SIZE))); 
          NRF_LOG_INFO("%d", signal[idx]);
        }
        
        byte_buffer[0] = 8;
        byte_buffer[1] = 1;
        for(idx=2; idx <= BYTE_BUFFER_SIZE/2; idx++)
        {
          //tempH = 2*idx-2;
          //tempL = 2*idx-1;
          byte_buffer[2*idx-2] = (uint8_t)(signal[idx-2] >> 8);   // MSB first, i.e. "higher bits", buffer's index even: 2,4,6,...
          byte_buffer[2*idx-1] = (uint8_t)signal[idx-2];          // LSB last,  i.e. "lower bits", buffer's index odd: 3,5,7,...
          //NRF_LOG_INFO("idx=%d, H%d: %d", idx, tempH, byte_buffer[2*idx-2]);
          //NRF_LOG_INFO("idx=%d, L%d: %d", idx, tempL, byte_buffer[2*idx-1]);
        }
        // Initialize the UART module:
        APP_UART_FIFO_INIT(&config_UART, 
                            UART_RX_BUFF_SIZE, 
                            UART_TX_BUFF_SIZE, 
                            uart_err_handle, 
                            APP_IRQ_PRIORITY_LOWEST, 
                            err_code_UART);
        NRF_LOG_INFO("Error Code UART: %d", err_code_UART);
        APP_ERROR_CHECK(err_code_UART); // check if everything initialized correctly
        
        
        nrf_gpio_cfg_output(LED_2);               // LED_PIN is an output now

        /*
          The action begins here
        */
	NRF_LOG_INFO("The action begins here!");
        printf("Hello PC from nordic Device!!\r\n");

       
        while(true)
        {
          for(idx=0; idx < BYTE_BUFFER_SIZE; idx++)
          {
            //app_uart_put(byte_buffer[idx]);
            while(app_uart_put(byte_buffer[idx]) != NRF_SUCCESS);
            //nrf_delay_ms(50); 
          }
          //nrf_gpio_pin_toggle(LED_2);
        }

        return 0;//acconeer_main(0, NULL);  // find maximum amplitude and its index, but 0 means do nothing??
}