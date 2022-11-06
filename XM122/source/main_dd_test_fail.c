/**
 * @brief Sending the A111's data via serial (UART/USB) to the PC.
 *
 * @author Bennet Ladage
 * @date 2022-05-14
 *
 * @details Part of the author's Bachelor Thesis, South Westphalia University of Applied Sciences Meschede
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_detector_distance.h"
#include "acc_hal_definitions.h"
#include "acc_hal_integration.h"
#include "acc_rss.h"
#include "acc_version.h"

// "old includes"
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
#include "board_xm122.h"
#include "app_error.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/** \example example_detector_distance.c
 * @brief This is an example on how the distance detector can be used
 * @n
 * The example executes as follows:
 *   - Activate Radar System Software (RSS)
 *   - Create a distance detector configuration
 *   - Create a distance detector using the previously created configuration
 *   - Destroy the distance detector configuration
 *   - Activate the distance detector
 *   - Get the result and print it 5 times
 *   - Deactivate and destroy the distance detector
 *   - Deactivate Radar System Software (RSS)
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


static void print_distances(acc_detector_distance_result_t *result, uint16_t reflection_count);
static void print_distances(acc_detector_distance_result_t *result, uint16_t reflection_count)
{
	printf("Found %u peaks:\n", (unsigned int)reflection_count);

	for (uint16_t i = 0; i < reflection_count; i++)
	{
		printf("Amplitude %u at %u mm\n", (unsigned int)result[i].amplitude,
		       (unsigned int)(result[i].distance_m * 1000));
	}
}


int main(void)
{
	NRF_LOG_INFO("Acconeer software version %s", acc_version_get());

	const acc_hal_t *hal = acc_hal_integration_get_implementation();

	if (!acc_rss_activate(hal))
	{
		printf("acc_rss_activate() failed\n");
		return EXIT_FAILURE;
	}

	acc_detector_distance_configuration_t distance_configuration = acc_detector_distance_configuration_create();

	if (distance_configuration == NULL)
	{
		printf("acc_detector_distance_configuration_create() failed\n");
		acc_rss_deactivate();
		return EXIT_FAILURE;
	}

	acc_detector_distance_handle_t distance_handle = acc_detector_distance_create(distance_configuration);

	if (distance_handle == NULL)
	{
		printf("acc_detector_distance_create() failed\n");
		acc_detector_distance_configuration_destroy(&distance_configuration);
		acc_rss_deactivate();
		return EXIT_FAILURE;
	}

	acc_detector_distance_configuration_destroy(&distance_configuration);

	if (!acc_detector_distance_activate(distance_handle))
	{
		printf("acc_detector_distance_activate() failed\n");
		acc_detector_distance_destroy(&distance_handle);
		acc_rss_deactivate();
		return EXIT_FAILURE;
	}

	bool                                success         = true;
	const int                           iterations      = 5;
	uint16_t                            number_of_peaks = 5;
	acc_detector_distance_result_t      result[number_of_peaks];
	acc_detector_distance_result_info_t result_info;

	for (int i = 0; i < iterations; i++)
	{
		success = acc_detector_distance_get_next(distance_handle, result, number_of_peaks, &result_info);

		if (!success)
		{
			printf("acc_detector_distance_get_next() failed\n");
			break;
		}

		print_distances(result, result_info.number_of_peaks);
	}

	bool deactivated = acc_detector_distance_deactivate(distance_handle);

	acc_detector_distance_destroy(&distance_handle);

	acc_rss_deactivate();

	if (deactivated && success)
	{
		printf("Application finished OK\n");
		return EXIT_SUCCESS;
	}

	return EXIT_FAILURE;
        //while(true)
        //{

        //}

        //return 0;
}