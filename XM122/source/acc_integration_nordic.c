// Copyright (c) Acconeer AB, 2019-2021
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include "nrf_delay.h"
#include "nrf_drv_rtc.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_uarte.h"

#include "acc_integration.h"


/**
 * @brief RTC compare channel used for periodic wakeup
 *
 */
#define WKUP_RTC_PERIODIC_SLEEP_CC 0


/**
 * @brief RTC compare channel used for wakeup in our delay function
 *
 */
#define WKUP_RTC_DELAY_CC 1


/**
 * @brief Get number of RTC ticks from time in us
 */
#define WKUP_RTC_TICKS(TIME_US) (RTC_US_TO_TICKS(((uint64_t)(TIME_US)), RTC_DEFAULT_CONFIG_FREQUENCY))


/**
 * @brief Get time in ms from number of RTC ticks
 */
#define WKUP_RTC_MS(TICKS) (uint32_t)(((uint64_t)(TICKS) * 1000U) / (RTC_DEFAULT_CONFIG_FREQUENCY))


/**
 * @brief RTC configuration
 */
static nrf_drv_rtc_config_t const rtc_cfg = NRF_DRV_RTC_DEFAULT_CONFIG;


/**
 * @brief RTC instance
 *
 * Instance of the RTC used for waking up the system.
 * We will use RTC2 since RTC0 is used by the softdevice
 * and RTC1 by app_timer.
 */
static nrf_drv_rtc_t const rtc = NRF_DRV_RTC_INSTANCE(2);


/**
 * @brief Set to true when RTC2 interrupt has triggered on CC0
 */
static volatile bool rtc_cc_periodic_sleep_irq_triggered = false;


/**
 * @brief Set to true when RTC2 interrupt has triggered on CC1
 */
static volatile bool rtc_cc_delay_irq_triggered = false;


/**
 * @brief The periodic wakeup time
 */
static uint32_t periodic_sleep_time_ms = 0;


/**
 * @brief Set to true when the rtc is enabled
 */
static bool rtc_enabled = false;

static volatile uint32_t overflow_counter;


/**
 * @brief Function for setting the next wakeup time from the RTC interrupt.
 */
static void rtc_set_next_wakeup_time(void)
{
	if (periodic_sleep_time_ms != 0)
	{
		uint32_t ctr_val = (nrf_drv_rtc_counter_get(&rtc) + WKUP_RTC_TICKS(periodic_sleep_time_ms * 1000)) & RTC_COUNTER_COUNTER_Msk;
		APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, WKUP_RTC_PERIODIC_SLEEP_CC, ctr_val, true));
	}
}


/**
 * @brief Function for handling the RTC interrupts.
 * Triggered on COMPARE0 and COMPARE1 match
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	if (int_type == WKUP_RTC_PERIODIC_SLEEP_CC)
	{
		rtc_cc_periodic_sleep_irq_triggered = true;
		rtc_set_next_wakeup_time();
	}
	else if (int_type == WKUP_RTC_DELAY_CC)
	{
		rtc_cc_delay_irq_triggered = true;
	}
	else if (int_type == NRFX_RTC_INT_OVERFLOW)
	{
		overflow_counter++;
	}
}


/**
 * @brief Function for initialization and configuration of RTC driver instance.
 */
static void rtc_enable(void)
{
	if (!rtc_enabled)
	{
		APP_ERROR_CHECK(nrf_drv_rtc_init(&rtc, &rtc_cfg, rtc_handler));

		rtc_cc_periodic_sleep_irq_triggered = false;
		rtc_cc_delay_irq_triggered          = false;

		nrf_drv_rtc_counter_clear(&rtc);

		nrfx_rtc_overflow_enable(&rtc, true);

		nrf_drv_rtc_enable(&rtc);
		rtc_enabled = true;
	}
}


// Public functions
void acc_integration_sleep_ms(uint32_t time_msec)
{
	acc_integration_sleep_us(time_msec * 1000);
}


/**
 * @brief Function for delaying execution for a specified amount of time
 *
 * It will use RTC compare function for sleeping the specified amount of time.
 * For shorter delays it will use busy-waiting.
 */
void acc_integration_sleep_us(uint32_t time_usec)
{
	if (time_usec != 0)
	{
		// For shorter sleep periods we will use a busy wait delay instead of using the RTC compare function
		if (time_usec < 200)
		{
			nrf_delay_us(time_usec);
		}
		else
		{
			rtc_enable();

			rtc_cc_delay_irq_triggered = false;
			uint32_t ctr_val = (nrf_drv_rtc_counter_get(&rtc) + WKUP_RTC_TICKS(time_usec)) & RTC_COUNTER_COUNTER_Msk;
			APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, WKUP_RTC_DELAY_CC, ctr_val, true));

			while (!rtc_cc_delay_irq_triggered)
			{
				nrf_pwr_mgmt_run();
			}

			rtc_cc_delay_irq_triggered = false;
			nrfx_rtc_cc_disable(&rtc, WKUP_RTC_DELAY_CC);
		}
	}
}


void acc_integration_set_periodic_wakeup(uint32_t time_msec)
{
	rtc_enable();
	periodic_sleep_time_ms = time_msec;
	rtc_set_next_wakeup_time();
}


void acc_integration_sleep_until_periodic_wakeup(void)
{
	// The periodic timer must be set prior to invoking this function
	if (periodic_sleep_time_ms != 0)
	{
		while (!rtc_cc_periodic_sleep_irq_triggered)
		{
			nrf_pwr_mgmt_run();
		}

		rtc_cc_periodic_sleep_irq_triggered = false;
	}
	else
	{
		NRF_LOG_WARNING("acc_integration_set_periodic_wakeup must be called prior to calling this function");
	}
}


uint32_t acc_integration_get_time(void)
{
	rtc_enable();

	uint32_t rtc_counter = nrf_drv_rtc_counter_get(&rtc);
	uint32_t result      = WKUP_RTC_MS(rtc_counter + overflow_counter *
	                                   (RTC_COUNTER_COUNTER_Msk >> RTC_COUNTER_COUNTER_Pos));
	return result;
}


void acc_integration_set_lowest_power_state(uint32_t req_power_state)
{
	(void)req_power_state;
}
