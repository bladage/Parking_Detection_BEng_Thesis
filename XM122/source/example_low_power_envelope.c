// Copyright (c) Acconeer AB, 2019-2022
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

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

/**
 * @brief Example that shows a single update and then enters low power mode
 *
 */

#ifndef SUSPEND_TIME_BETWEEN_UPDATES_MS
#define SUSPEND_TIME_BETWEEN_UPDATES_MS (10000) // 0.1Hz
#endif

#ifndef USE_BLE_ADVERTISING
#define USE_BLE_ADVERTISING (1) //1=BT LE on
#endif

#ifndef POWER_SAVE_MODE
#define POWER_SAVE_MODE OFF
#endif

#ifndef RANGE_LENGTH
#define RANGE_LENGTH (0.6f)
#endif

#ifndef SERVICE_PROFILE
#define SERVICE_PROFILE 2
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


/**
 * @brief Function for creating envelope service
 */
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


/**
 * @brief Function for running the envelope service
 */
static void execute_envelope_service(acc_service_handle_t handle, uint16_t *result1, uint16_t *result2)
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


int acconeer_main(int argc, char *argv[]);


int acconeer_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	printf("Acconeer software version %s\n", acc_version_get());

	acc_hal_t hal = *acc_hal_integration_get_implementation();

	acc_integration_set_periodic_wakeup(SUSPEND_TIME_BETWEEN_UPDATES_MS);

	hal.log.log_level = ACC_LOG_LEVEL_ERROR;

	if (!acc_rss_activate(&hal))
	{
		return EXIT_FAILURE;
	}

	acc_service_handle_t handle = create_envelope_service();
	if (handle == NULL)
	{
		printf("acc_service_create() failed\n");
		return EXIT_FAILURE;
	}

	if (!acc_service_activate(handle))
	{
		printf("acc_service_activate() failed\n");
		acc_service_destroy(&handle);
		acc_rss_deactivate();
		return EXIT_FAILURE;
	}

#ifdef BROADCAST_BATTERY_INFO
	acc_battery_info_init();
#endif

#if (USE_BLE_ADVERTISING == 1)
	// The advertising interval is not used in this case, the RTC interrupt will wake us up in time
	acc_bluetooth_beacon_init(0);
#endif

	while (true)
	{
		uint16_t result[2] = {0};

#ifdef BROADCAST_BATTERY_INFO
		// Sample the supply voltage before taxing the battery with radar operations
		float voltage = 0.0f;
		acc_battery_info_sample_voltage(&voltage, false);
#endif

		execute_envelope_service(handle, &result[0], &result[1]);

#ifdef BROADCAST_BATTERY_INFO
		// Replace result[0] with the sampled supply voltage in millivolts.
		// Casting a negative float to unsigned integer invokes undefined
		// behavior, so cast to a signed integer type first.
		result[0] = (uint16_t)(int16_t)((voltage * 1000.0f) + 0.5f);
#endif

#if (USE_BLE_ADVERTISING == 1)
		acc_bluetooth_beacon_update(result, sizeof(result) / sizeof(result[0]));
#else
		printf("Service data %" PRIu16 " , %" PRIu16 "\n", result[0], result[1]);
#endif

		acc_integration_sleep_until_periodic_wakeup();
	}

	acc_service_deactivate(handle);
	acc_service_destroy(&handle);
	acc_rss_deactivate();

	return EXIT_SUCCESS;
}
