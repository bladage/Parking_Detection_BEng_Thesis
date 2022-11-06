// Copyright (c) Acconeer AB, 2020-2021
// All rights reserved

#ifndef ACC_BATTERY_INFO_H_
#define ACC_BATTERY_INFO_H_

#include <stdbool.h>

/**
 * @defgroup battery_info Battery Information
 *
 * @brief API for sampling the supply voltage. Used for monitoring battery life.
 *
 * @{
 */


/**
 * Configure the ADC for sampling the supply voltage
 */
void acc_battery_info_init(void);


/**
 * Sample the supply voltage
 *
 * @param[out] voltage      A sample of the supply voltage in units of volts.
 *                          Resolution is approximately 5.3 mV on XM122.
 * @param[in]  recalibrate  A Boolean value indicating whether the ADC should be
 *                          (re)calibrated before sampling.
 *
 * @return True if sampling was successful
 */
bool acc_battery_info_sample_voltage(float *voltage, bool recalibrate);


/**
 * @}
 */

#endif
