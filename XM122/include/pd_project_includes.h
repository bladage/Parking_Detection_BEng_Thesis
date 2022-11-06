#include <stdbool.h>

#include "acc_hal_integration_xm122.h"
#include "app_uart.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nrf_gpio.h"

// Logger:
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"    // Application Timers
#include "nrf_pwr_mgmt.h" // power management

// Softdevice:
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

// BLE includes
#include "nrf_ble_qwr.h"
#include "nrf_ble_gatt.h"

// BLE advertisment
#include "ble_advdata.h"
#include "ble_advertising.h"

// BLE PPCP
#include "ble_conn_params.h"

// BLE NUS (Nordic UART Service)
#include "ble_nus.h"

#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "bsp_btn_ble.h"  // nec?

// Acconeer's includes low power envelope
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


#ifdef BROADCAST_BATTERY_INFO
#include "acc_battery_info.h"
#endif

//#include "acc_bluetooth_beacon_xm122.h"
#include "acc_definitions_common.h"
#include "acc_hal_definitions.h"
#include "acc_hal_integration.h"
#include "acc_integration.h"
#include "acc_rss.h"
#include "acc_service.h"
//#include "acc_service_envelope.h"
#include "acc_service_iq.h"
#include "acc_version.h"

// Bennet's new includes for I2C/TWI
#include "boards.h"
#include "board_xm122.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_twis.h"
#include "memsic_mmc5983ma.h"
#include <math.h>
#include <complex.h>
//#include "tfmini_plus.h"
//#include "tfmini_plus_uart.h"

//// UART
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

// Build frames/messages for the NUS BLE service
#include "pd_ble_messages.h"

// Application Timer (RTC1)
#include "app_timer.h"
#include "nrf_drv_clock.h"

