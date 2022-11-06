// Copyright (c) Acconeer AB, 2020
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stddef.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_soc.h"
#include "nrfx_saadc.h"
#include "sdk_config.h"

#include "acc_battery_info.h"
#include "board_xm122.h"

#define ADC_CHANNEL 0

#define INV_VOLTAGE_RATIO    3.0f
#define INV_GAIN             3.0f
#define INTERNAL_REF_VOLTAGE 0.6f

#define MAX_INPUT_VOLTAGE (INV_VOLTAGE_RATIO * INV_GAIN * INTERNAL_REF_VOLTAGE)

#define NUM_QUANTIZATION_LEVELS (1 << 10)

#define ADC_CONVERSION_FACTOR (MAX_INPUT_VOLTAGE / NUM_QUANTIZATION_LEVELS)

#define VIN_ADC_EN_DELAY_US 300


static volatile bool adc_calibration_done = false;


static void adc_event_handler(nrfx_saadc_evt_t const *event)
{
	if (event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
	{
		adc_calibration_done = true;
		NRF_LOG_DEBUG("ADC calibration done");
	}
}


static bool calibrate_adc(void)
{
	adc_calibration_done = false;
	NRF_LOG_DEBUG("ADC calibration started");

	nrfx_err_t err_code = nrfx_saadc_calibrate_offset();

	if (err_code != NRFX_SUCCESS)
	{
		NRF_LOG_ERROR("ADC calibration error %d", (int)err_code);
		return false;
	}

	while (nrfx_saadc_is_busy())
	{
		sd_app_evt_wait();
	}

	return adc_calibration_done;
}


void acc_battery_info_init(void)
{
	// Configure ADC to use 10-bit resolution and no oversampling
	nrfx_saadc_config_t adc_config =
	{
		.resolution         = NRF_SAADC_RESOLUTION_10BIT,
		.oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,
		.interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY,
		.low_power_mode     = true
	};

	// Configure ADC channel to use internal reference (0.6 V) and 1/3 gain.
	// An acquisition time of 15 microseconds has been selected based on that
	// the source resistance is roughly 200 kiloohm.
	nrf_saadc_channel_config_t adc_channel_config =
	{
		.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
		.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
		.gain       = NRF_SAADC_GAIN1_3,
		.reference  = NRF_SAADC_REFERENCE_INTERNAL,
		.acq_time   = NRF_SAADC_ACQTIME_15US,
		.mode       = NRF_SAADC_MODE_SINGLE_ENDED,
		.burst      = NRF_SAADC_BURST_DISABLED,
		.pin_p      = NRF_SAADC_INPUT_AIN7,
		.pin_n      = NRF_SAADC_INPUT_DISABLED
	};

	nrfx_saadc_init(&adc_config, adc_event_handler);
	nrfx_saadc_channel_init(ADC_CHANNEL, &adc_channel_config);

	// Configure the VIN_ADC_EN pin
	nrf_gpio_cfg_output(VIN_ADC_EN_Pin);
}


bool acc_battery_info_sample_voltage(float *voltage, bool recalibrate)
{
	bool result = true;

	// Connect supply voltage to VIN_ADC by asserting VIN_ADC_EN
	nrf_gpio_pin_write(VIN_ADC_EN_Pin, true);

	// Wait 300 microseconds after asserting VIN_ADC_EN before trying to sample
	// VIN_ADC to accommodate for delay in the power switch
	nrf_delay_us(VIN_ADC_EN_DELAY_US);

	// Perform calibration if needed
	if (recalibrate || !adc_calibration_done)
	{
		result = calibrate_adc();
	}

	if (result && voltage != NULL)
	{
		// Sample VIN_ADC once
		nrf_saadc_value_t adc_sample = 0;
		nrfx_err_t        err_code   = nrfx_saadc_sample_convert(ADC_CHANNEL, &adc_sample);

		if (err_code != NRFX_SUCCESS)
		{
			NRF_LOG_ERROR("ADC conversion error %d", (int)err_code);
			result = false;
		}
		else
		{
			// Convert sample to voltage, compensating for quantization error
			*voltage = ADC_CONVERSION_FACTOR * ((float)adc_sample + 0.5f);
		}
	}

	// Disconnect supply voltage from VIN_ADC by deasserting VIN_ADC_EN
	nrf_gpio_pin_write(VIN_ADC_EN_Pin, false);

	return result;
}
