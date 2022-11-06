/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/**
 *
 * The file examples/ble_peripheral/ble_app_beacon/main.c from the Nordic SDK is used as a base
 * for this application.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_timer.h"
#include "ble_advdata.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_soc.h"

#define APP_BLE_CONN_CFG_TAG            1                         /**< A tag identifying the SoftDevice BLE configuration. */
#define NON_CONNECTABLE_ADV_INTERVAL(A) (MSEC_TO_UNITS((A), \
                                         UNIT_0_625_MS))          /**< The advertising interval for non-connectable advertisement (100 ms). */
#define APP_BLE_OBSERVER_PRIO           3                         /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define DEAD_BEEF                       0xACC0DEAD                /**< Value used as error code on stack dump. */

#ifdef USE_BLE_MONITOR_ADV_FORMAT
#define APP_CFG_ADV_DATA_LEN            22                             /**< Required length of the complete advertisement packet. This should be atleast 8 in order to accommodate flag field and other mandatory fields and one byte of manufacturer specific data. */

#define ADV_ENCODED_AD_TYPE_LEN     1                                  /**< Length of encoded ad type in advertisement data. */
#define ADV_ENCODED_AD_TYPE_LEN_LEN 1                                  /**< Length of the 'length field' of each ad type in advertisement data. */
#define ADV_FLAGS_LEN               1                                  /**< Length of flags field that will be placed in advertisement data. */
#define ADV_ENCODED_FLAGS_LEN       (ADV_ENCODED_AD_TYPE_LEN +       \
                                     ADV_ENCODED_AD_TYPE_LEN_LEN +   \
                                     ADV_FLAGS_LEN)                    /**< Length of flags field in advertisement packet. (1 byte for encoded ad type plus 1 byte for length of flags plus the length of the flags itself). */
#define ADV_ENCODED_COMPANY_ID_LEN  2                                  /**< Length of the encoded Company Identifier in the Manufacturer Specific Data part of the advertisement data. */
#define ADV_ADDL_MANUF_DATA_LEN     (APP_CFG_ADV_DATA_LEN -          \
                                     (ADV_ENCODED_FLAGS_LEN +        \
                                      (ADV_ENCODED_AD_TYPE_LEN +     \
                                       ADV_ENCODED_AD_TYPE_LEN_LEN + \
                                       ADV_ENCODED_COMPANY_ID_LEN)))   /**< Length of Manufacturer Specific Data field that will be placed on the air during advertisement. This is computed based on the value of APP_CFG_ADV_DATA_LEN (required advertisement data length). */


#if APP_CFG_ADV_DATA_LEN > BLE_GAP_ADV_SET_DATA_SIZE_MAX
    #error "The required advertisement data size is greater than the value allowed by stack (BLE_GAP_ADV_MAX_SIZE)."
#endif

#if ADV_ADDL_MANUF_DATA_LEN < 1
    #error "The required length of additional manufacturer specific data computed based on the user configured values is computed to be less than 1."
#endif

static uint8_t m_addl_adv_svc_data[ADV_ADDL_MANUF_DATA_LEN];           /**< Value of the additional manufacturer specific data that will be placed in air (initialized to all zeros). */

#else // USE_BLE_MONITOR_ADV_FORMAT

#define APP_BEACON_INFO_LENGTH          0x17                      /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                      /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                      /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                      /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                    /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0    /**< Proprietary UUID for Beacon. */
#define ACC_DATA_OFFSET_IN_BEACON_INFO  18                        /**< Position of the MSB of the Acconeer specific data in m_beacon_info array. */

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] = /**< Information advertised by the Beacon. */
{
	APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
	                     // implementation.
	APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
	                     // manufacturer specific data in this implementation.
	APP_BEACON_UUID,     // 128 bit UUID value.
	APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
	APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
	APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
	                     // this implementation.
};
#endif


static ble_gap_adv_params_t m_adv_params;                                    /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;   /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffers for storing encoded advertising sets. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
	.adv_data =
	{
		.p_data = m_enc_advdata[0],
		.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
	},
	.scan_rsp_data =
	{
		.p_data = NULL,
		.len    = 0
	}
};


static bool     ble_stack_initialized  = false;
static bool     advertisement_started  = false;
static uint32_t requested_adv_interval = 0;


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num    Line number of the failing ASSERT call.
 * @param[in]   p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(uint16_t *data, uint32_t data_length)
{
	uint32_t                 err_code;
	ble_advdata_t            advdata;
	ble_advdata_manuf_data_t manuf_specific_data;

#ifdef USE_BLE_MONITOR_ADV_FORMAT
	uint8_t flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	manuf_specific_data.company_identifier = 0xacc0;

	if (data != NULL && data_length == 3)
	{
		uint8_t acc_index = 0;
		// XM122 Presence ID
		m_addl_adv_svc_data[acc_index++] = 0x80;

		// battery_level, %
		m_addl_adv_svc_data[acc_index++] = data[0] & 0xFF;
		m_addl_adv_svc_data[acc_index++] = 0x00;

		// temperature, degrees Celcius
		m_addl_adv_svc_data[acc_index++] = data[1] & 0xFF;
		m_addl_adv_svc_data[acc_index++] = (data[1] >> 8) & 0xFF;

		// motion, bool
		m_addl_adv_svc_data[acc_index++] = data[2];
		m_addl_adv_svc_data[acc_index++] = 0x00;
	}

	manuf_specific_data.data.size   = ADV_ADDL_MANUF_DATA_LEN;
	manuf_specific_data.data.p_data = m_addl_adv_svc_data;

#else // USE_BLE_MONITOR_ADV_FORMAT

	uint8_t flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

	manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

	if (data != NULL && data_length == 2)
	{
		uint8_t acc_index = ACC_DATA_OFFSET_IN_BEACON_INFO;
		m_beacon_info[acc_index++] = MSB_16(data[0]);
		m_beacon_info[acc_index++] = LSB_16(data[0]);
		m_beacon_info[acc_index++] = MSB_16(data[1]);
		m_beacon_info[acc_index++] = LSB_16(data[1]);
	}

	manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;
	manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;
#endif // USE_BLE_MONITOR_ADV_FORMAT

	// Build and set advertising data.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type             = BLE_ADVDATA_NO_NAME;
	advdata.flags                 = flags;
	advdata.p_manuf_specific_data = &manuf_specific_data;

	// swap adv data buffer - from API doc of sd_ble_gap_adv_set_configure:
	// "In order to update advertising data while advertising, new advertising buffers must be provided"
	m_adv_data.adv_data.p_data = (m_adv_data.adv_data.p_data == m_enc_advdata[0])
	                             ? m_enc_advdata[1] : m_enc_advdata[0];

	err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
	APP_ERROR_CHECK(err_code);

	// To update advertising data while advertising, the advertising params paramater must be NULL
	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, advertisement_started ? NULL : &m_adv_params);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for start advertising.
 */
static void advertising_start(void)
{
	if (!advertisement_started)
	{
		ret_code_t err_code;

		err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
		APP_ERROR_CHECK(err_code);

		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
		APP_ERROR_CHECK(err_code);

		advertisement_started = true;
	}
}


/**@brief Function for stop advertising.
 */
static void advertising_stop(void)
{
	if (advertisement_started)
	{
		ret_code_t err_code;

		err_code = sd_ble_gap_adv_stop(m_adv_handle);
		APP_ERROR_CHECK(err_code);

		err_code = bsp_indication_set(BSP_INDICATE_IDLE);
		APP_ERROR_CHECK(err_code);

		advertisement_started = false;
	}
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
	(void)p_context;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_ADV_SET_TERMINATED:
			advertisement_started = false;
			break;
		default:
			// Intentionally do nothing here
			break;
	}
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
	if (!ble_stack_initialized)
	{
		ret_code_t err_code;

		err_code = nrf_sdh_enable_request();
		APP_ERROR_CHECK(err_code);

		// Configure the BLE stack using the default settings.
		// Fetch the start address of the application RAM.
		uint32_t ram_start = 0;
		err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
		APP_ERROR_CHECK(err_code);

		// Enable BLE stack.
		err_code = nrf_sdh_ble_enable(&ram_start);
		APP_ERROR_CHECK(err_code);

		// Register a handler for BLE events.
		NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

		ble_stack_initialized = true;
	}
}


/**@brief Function for initializing the advertisement params
 */
static void init_adv_params(uint32_t interval_ms)
{
	requested_adv_interval = interval_ms;

	// Maximum allowed advertising interval is 10.25s
	uint32_t limited_interval = NON_CONNECTABLE_ADV_INTERVAL(interval_ms);
	if (limited_interval > BLE_GAP_ADV_INTERVAL_MAX || interval_ms == 0)
	{
		limited_interval = BLE_GAP_ADV_INTERVAL_MAX;
	}

	// Initialize advertising parameters (used when starting advertising).
	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
	m_adv_params.p_peer_addr     = NULL; // Undirected advertisement.
	m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
	m_adv_params.interval        = limited_interval;
	m_adv_params.duration        = 0; // Never time out.
	m_adv_params.max_adv_evts    = interval_ms == 0 ? 1 : 0; // Advertise once, then terminate if interval is specified
}


void acc_bluetooth_beacon_init(uint32_t interval_ms)
{
	ble_stack_init();
	init_adv_params(interval_ms);
}


bool acc_bluetooth_beacon_stop(void)
{
	advertising_stop();

	return true;
}


bool acc_bluetooth_beacon_update(uint16_t *data, uint32_t data_length)
{
	if (!ble_stack_initialized)
	{
		return false;
	}

	// requested interval of 0 is used to signal that advertisement should be restarted
	// after each event
	if (requested_adv_interval == 0)
	{
		advertising_stop();
	}

	advertising_init(data, data_length);
	advertising_start();
	return true;
}
