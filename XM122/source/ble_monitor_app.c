// Copyright (c) Acconeer AB, 2022
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_battery_info.h"
#include "acc_bluetooth_beacon_xm122.h"
#include "acc_definitions_common.h"
#include "acc_detector_presence.h"
#include "acc_hal_definitions.h"
#include "acc_hal_integration.h"
#include "acc_integration.h"
#include "acc_rss.h"
#include "acc_service.h"
#include "acc_version.h"

#include "nrf_soc.h"


#define SUSPEND_TIME_BETWEEN_UPDATES_MS (500) // 2Hz


/**
 * @brief Function for creating presence detector
 */
static acc_detector_presence_handle_t create_presence_detector(void)
{
	bool                           success = true;
	acc_detector_presence_handle_t handle  = NULL;

	acc_detector_presence_configuration_t presence_configuration = acc_detector_presence_configuration_create();

	if (presence_configuration == NULL)
	{
		printf("Failed to create configuration\n");
		success = false;
	}

	if (success)
	{
		acc_detector_presence_configuration_start_set(presence_configuration, 0.2f);
		acc_detector_presence_configuration_length_set(presence_configuration, 3.0f);
		acc_detector_presence_configuration_power_save_mode_set(presence_configuration, ACC_POWER_SAVE_MODE_OFF);
		acc_detector_presence_configuration_update_rate_set(presence_configuration, 1000.0f / (float)SUSPEND_TIME_BETWEEN_UPDATES_MS);
		acc_detector_presence_configuration_service_profile_set(presence_configuration, ACC_SERVICE_PROFILE_3);
		acc_detector_presence_configuration_detection_threshold_set(presence_configuration, 1.5f);

		handle = acc_detector_presence_create(presence_configuration);
		if (handle == NULL)
		{
			printf("Could not create presence detector\n");
			success = false;
		}
	}

	acc_detector_presence_configuration_destroy(&presence_configuration);

	return success ? handle : NULL;
}


/**
 * @brief Function for running the sparse service
 */
static void execute_presence_detector(acc_detector_presence_handle_t handle, bool *presence_detected)
{
	acc_detector_presence_result_t result;

	*presence_detected = false;

	if (acc_detector_presence_get_next(handle, &result))
	{
		*presence_detected = (uint16_t)result.presence_detected;
	}
}


/**
 * @brief Function for getting remaining battery capacity in percentage from voltage for CR2744 coin cell batteries
 *
 * This function is a simplified version that just translates nominal battery voltage to remaning capacity linearly.
 * The actual discharge curve is not linear and temperature and discharge current also affects remaining capacity.
 */
static uint16_t get_remaining_battery_capacity(float voltage)
{
	const float max_voltage = 3.0f;
	const float min_voltage = 2.0f;

	uint16_t remaining;

	if (voltage >= max_voltage)
	{
		remaining = 100U;
	}
	else if (voltage <= min_voltage)
	{
		remaining = 0;
	}
	else
	{
		remaining = (uint16_t)(100 * ((voltage - min_voltage) / (max_voltage - min_voltage)));
	}

	return remaining;
}


static int16_t read_die_temperature(void)
{
	int32_t temperature;

	uint32_t err_code = sd_temp_get(&temperature);

	if (err_code == NRF_SUCCESS)
	{
		// Die temperature is in 0.25 degrees Celsius.
		return (int16_t)(temperature / 4);
	}
	else
	{
		return 0;
	}
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

	acc_detector_presence_handle_t detector_handle = NULL;
	detector_handle = create_presence_detector();
	if (detector_handle == NULL)
	{
		printf("create_presence_detector() failed\n");
		return EXIT_FAILURE;
	}

	if (!acc_detector_presence_activate(detector_handle))
	{
		printf("acc_detector_presence_activate() failed\n");
		acc_detector_presence_destroy(&detector_handle);
		acc_rss_deactivate();
		return EXIT_FAILURE;
	}

	acc_battery_info_init();
	acc_bluetooth_beacon_init(0);

	while (true)
	{
		uint16_t adv_data[3];
		bool     presence = false;
		uint16_t battery  = 0;

		// Sample the supply voltage before taxing the battery with radar operations
		float voltage = 0.0f;
		acc_battery_info_sample_voltage(&voltage, false);
		battery = get_remaining_battery_capacity(voltage);

		int16_t temperature = read_die_temperature();

		execute_presence_detector(detector_handle, &presence);

		adv_data[0] = battery;
		// This cast from int to uint will be handled on the other side of the BLE transmission
		// to support negative temperatures
		adv_data[1] = (uint16_t)temperature;
		adv_data[2] = (uint16_t)presence;

		acc_bluetooth_beacon_update(adv_data, sizeof(adv_data) / sizeof(adv_data[0]));

		acc_integration_sleep_until_periodic_wakeup();
	}

	acc_detector_presence_deactivate(detector_handle);
	acc_detector_presence_destroy(&detector_handle);
	acc_rss_deactivate();

	return EXIT_SUCCESS;
}
