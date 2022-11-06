// Copyright (c) Acconeer AB, 2018-2022
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "acc_hal_integration_xm122.h"

#include "acc_definitions_common.h"
#include "acc_hal_definitions.h"
#include "acc_hal_integration.h"
#include "acc_integration.h"
#include "acc_integration_log.h"

#include "app_timer.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_soc.h"
#include "nrfx_gpiote.h"
#include "nrfx_spim.h"

#include "board_xm122.h"

#define MODULE "driver_hal"

/**
 * @brief The number of sensors available on the board
 */
#define SENSOR_COUNT 1

/**
 * @brief Size of SPI transfer buffer
 */
#define SPI_MAX_TRANSFER_SIZE 4096

/**
 * @brief The reference frequency used by this board
 *
 * This assumes 26 MHz on the Sparkfun A111 Board
 * if BOARD_PCA10056 is defined, otherwise XM122.
 */
#if defined(BOARD_PCA10056)
// Assuming sparkfun breakout card
#define ACC_BOARD_REF_FREQ           26000000
#define SENSOR_DEFAULT_SPI_FREQUENCY NRF_SPIM_FREQ_1M
#else
// XM122
#define ACC_BOARD_REF_FREQ           24000000
#define SENSOR_DEFAULT_SPI_FREQUENCY NRF_SPIM_FREQ_32M
#endif


#define SPI_INSTANCE 3                                            /**< SPI instance index. */
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool     spi_done;
static volatile bool     timed_out;

/*
 * Workaround for nRF52840 anomaly [198] SPIM: SPIM3 transmit data might be corrupted
 * Symptoms:
 * Data accessed by CPU location in the same RAM block as where the SPIM3 TXD.PTR is pointing,
 * and CPU does a read or write operation at the same clock cycle as the SPIM3 EasyDMA is fetching data.
 * Workaround:
 * Reserve dedicated RAM blocks for the SPIM3 transmit buffer, not overlapping with application data
 * used by the CPU. In addition, synchronize so that the CPU is not writing data to the transmit buffer
 * while SPIM is transmitting data.
 *
 * One RAM block in nRF52840 is 8k bytes.
 *
 * See more in the Errata document located at https://infocenter.nordicsemi.com/
 */
uint8_t tx_buffer[0x2000] __attribute__((section(".spim3_tx_buffer")));

APP_TIMER_DEF(int_timeout);


static void spim_event_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
	(void)p_event;
	(void)p_context;
	spi_done = true;
}


static void spi_init()
{
	nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;

	spi_config.frequency      = SENSOR_DEFAULT_SPI_FREQUENCY;
	spi_config.ss_pin         = A111_SPI_CS_N_Pin;
	spi_config.miso_pin       = A111_SPI_MISO_Pin;
	spi_config.mosi_pin       = A111_SPI_MOSI_Pin;
	spi_config.sck_pin        = A111_SPI_CLK_Pin;
	spi_config.dcx_pin        = NRFX_SPIM_PIN_NOT_USED;
	spi_config.use_hw_ss      = true;
	spi_config.ss_active_high = false;
	APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL));

	/* Override the pin configurations made by nrfx_spim_init()
	 * so that we use High drive '0' and high-drive '1' on the
	 * pins in order to handle 32 MHz SPI clock properly.
	 */
	nrf_gpio_cfg(spi_config.sck_pin,
	             NRF_GPIO_PIN_DIR_OUTPUT,
	             NRF_GPIO_PIN_INPUT_CONNECT,
	             NRF_GPIO_PIN_NOPULL,
	             NRF_GPIO_PIN_H0H1,
	             NRF_GPIO_PIN_NOSENSE);
}


static void int_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	// Do nothing, we read the GPIO pin anyway
}


void timeout_handler(void *p_context)
{
	timed_out = true;
}


static void gpio_init(void)
{
	if (!nrfx_gpiote_is_init())
	{
		APP_ERROR_CHECK(nrfx_gpiote_init());
	}

	nrfx_gpiote_in_config_t int_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	int_config.pull = NRF_GPIO_PIN_PULLDOWN;
	APP_ERROR_CHECK(nrfx_gpiote_in_init(A111_SENSOR_INTERRUPT_Pin, &int_config, int_pin_handler));
	nrfx_gpiote_in_event_enable(A111_SENSOR_INTERRUPT_Pin, true);

	nrfx_gpiote_out_config_t out_pin_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
	nrfx_gpiote_out_init(A111_ENABLE_Pin, &out_pin_config);
	nrfx_gpiote_out_init(A111_PS_ENABLE_Pin, &out_pin_config);
	nrfx_gpiote_out_init(A111_CTRL_Pin, &out_pin_config);
	nrfx_gpiote_out_clear(A111_CTRL_Pin);

	nrfx_gpiote_out_init(LED_1, &out_pin_config);
	nrfx_gpiote_out_set(LED_1);
	nrfx_gpiote_out_init(LED_2, &out_pin_config);
	nrfx_gpiote_out_set(LED_2);

	// Reset DFU button GPIO in case bootloader didn't
	nrf_gpio_cfg_default(BUTTON_1);
}


static void timer_init(void)
{
	APP_ERROR_CHECK(app_timer_create(&int_timeout, APP_TIMER_MODE_SINGLE_SHOT, timeout_handler));
}


//----------------------------------------
// Implementation of RSS HAL handlers
//----------------------------------------


static void acc_hal_integration_sensor_transfer(acc_sensor_id_t sensor_id, uint8_t *buffer, size_t buffer_size)
{
	APP_ERROR_CHECK_BOOL(buffer_size <= sizeof(tx_buffer));
	memcpy(tx_buffer, buffer, buffer_size);

	spi_done = false;

	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_buffer, buffer_size, buffer, buffer_size);

	nrfx_spim_xfer(&spi, &xfer_desc, 0);

	while (!spi_done)
	{
		nrf_pwr_mgmt_run();
	}
}


static void acc_hal_integration_sensor_power_on(acc_sensor_id_t sensor_id)
{
	(void)sensor_id; // Ignore parameter

	nrfx_gpiote_out_set(A111_PS_ENABLE_Pin);
	nrfx_gpiote_out_set(A111_ENABLE_Pin);

	// The U4 power switch (see the XM122 schematic) controlled by PS_ENABLE has a 0.3 ms rise time
	// on its output, which decreases the time between VIO_1_2 actually going high and us trying to
	// use the sensor by 0.3 ms.
	// Once VIO_1_2 is 1.8 v we wait another 2 ms for the crystal to stabilize.
	acc_integration_sleep_us(2300);

	spi_init();
}


static void acc_hal_integration_sensor_power_off(acc_sensor_id_t sensor_id)
{
	(void)sensor_id; // Ignore parameter

	nrfx_gpiote_out_clear(A111_ENABLE_Pin);
	nrfx_gpiote_out_clear(A111_PS_ENABLE_Pin);

	nrfx_spim_uninit(&spi);

	// Wait after power off to leave the sensor in a known state
	// in case the application intends to enable the sensor directly
	acc_integration_sleep_us(2000);
}


static void acc_hal_integration_sensor_hibernate_enter(acc_sensor_id_t sensor_id)
{
	for (uint32_t i = 0; i < ACC_NBR_CLOCK_CYCLES_REQUIRED_HIBERNATE_ENTER; i++)
	{
		nrfx_gpiote_out_set(A111_CTRL_Pin);
		nrfx_gpiote_out_clear(A111_CTRL_Pin);
	}

	// Turn off sensor supplies VIO_1 and VIO_2
	nrfx_gpiote_out_clear(A111_PS_ENABLE_Pin);

	nrfx_spim_uninit(&spi);
}


static void acc_hal_integration_sensor_hibernate_exit(acc_sensor_id_t sensor_id)
{
	// Turn on sensor supplies VIO_1 and VIO_2
	nrfx_gpiote_out_set(A111_PS_ENABLE_Pin);
	acc_integration_sleep_us(300);

	spi_init();

	// When using the SPIM3 block of NRF52840 we get strange behaviour that
	// there is no activity on the SPI bus if we don't restart the SPI block
	// in this way with a dummy read before waking the sensor up from hibernate.
	uint8_t buffer[] = {0x30, 0, 0, 0, 0, 0};
	acc_hal_integration_sensor_transfer(1, buffer, sizeof(buffer));

	for (uint32_t i = 0; i < ACC_NBR_CLOCK_CYCLES_REQUIRED_STEP_1_HIBERNATE_EXIT; i++)
	{
		nrfx_gpiote_out_set(A111_CTRL_Pin);
		nrfx_gpiote_out_clear(A111_CTRL_Pin);
	}

	acc_integration_sleep_us(ACC_WAIT_TIME_HIBERNATE_EXIT_MS * 1000);

	for (uint32_t i = 0; i < ACC_NBR_CLOCK_CYCLES_REQUIRED_STEP_2_HIBERNATE_EXIT; i++)
	{
		nrfx_gpiote_out_set(A111_CTRL_Pin);
		nrfx_gpiote_out_clear(A111_CTRL_Pin);
	}
}


static bool acc_hal_integration_wait_for_sensor_interrupt(acc_sensor_id_t sensor_id, uint32_t timeout_ms)


{
	if (timeout_ms > 0)
	{
		uint32_t ticks = APP_TIMER_TICKS(timeout_ms);
		timed_out = false;
		if (ticks < APP_TIMER_MIN_TIMEOUT_TICKS)
		{
			ticks = APP_TIMER_MIN_TIMEOUT_TICKS;
		}

		APP_ERROR_CHECK(app_timer_start(int_timeout, ticks, NULL));
	}
	else
	{
		timed_out = true;
	}

	while (!nrf_gpio_pin_read(A111_SENSOR_INTERRUPT_Pin) && !timed_out)
	{
		nrf_pwr_mgmt_run();
	}
	APP_ERROR_CHECK(app_timer_stop(int_timeout));

	return nrf_gpio_pin_read(A111_SENSOR_INTERRUPT_Pin);
}


bool acc_hal_integration_xm122_init(void)
{
	gpio_init();
	timer_init();

	return true;
}


static float acc_hal_integration_get_reference_frequency(void)
{
	return ACC_BOARD_REF_FREQ;
}


static const acc_hal_t hal =
{
	.properties.sensor_count          = SENSOR_COUNT,
	.properties.max_spi_transfer_size = SPI_MAX_TRANSFER_SIZE,

	.sensor_device.power_on                = acc_hal_integration_sensor_power_on,
	.sensor_device.power_off               = acc_hal_integration_sensor_power_off,
	.sensor_device.hibernate_enter         = acc_hal_integration_sensor_hibernate_enter,
	.sensor_device.hibernate_exit          = acc_hal_integration_sensor_hibernate_exit,
	.sensor_device.wait_for_interrupt      = acc_hal_integration_wait_for_sensor_interrupt,
	.sensor_device.transfer                = acc_hal_integration_sensor_transfer,
	.sensor_device.get_reference_frequency = acc_hal_integration_get_reference_frequency,

	.os.mem_alloc = malloc,
	.os.mem_free  = free,
	.os.gettime   = acc_integration_get_time,

	.log.log_level = ACC_LOG_LEVEL_INFO,
	.log.log       = acc_integration_log,

	.optimization.transfer16 = NULL,
};


const acc_hal_t *acc_hal_integration_get_implementation(void)
{
	return &hal;
}
