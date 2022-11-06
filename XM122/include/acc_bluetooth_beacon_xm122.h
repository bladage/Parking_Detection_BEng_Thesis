// Copyright (c) Acconeer AB, 2019-2022
// All rights reserved

#ifndef ACC_BLUETOOTH_BEACON_XM122_H_
#define ACC_BLUETOOTH_BEACON_XM122_H_

#include <stdbool.h>
#include <stdint.h>


/**@brief Function for initializing BLE stack and setting up advertisement interval
 *
 * @param[in] interval_ms  The advertising interval in ms. If set to 0 the event
 *                         will be advertised once and then disabled.
 */
void acc_bluetooth_beacon_init(uint32_t interval_ms);


/**@brief Function for starting and updating data in Bluetooth advertising.
 *
 * @param[in] data         Array of data to advertise
 * @param[in] data_length  Length of array of data to advertise
 *
 * @return True if successful.
 */
bool acc_bluetooth_beacon_update(uint16_t *data, uint32_t data_length);


/**@brief Function for stopping Bluetooth advertising.
 *
 * @return True if successful.
 */
bool acc_bluetooth_beacon_stop(void);


#endif
