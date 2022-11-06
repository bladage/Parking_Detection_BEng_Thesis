// Copyright (c) Acconeer AB, 2019-2021
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "acc_definitions_common.h"
#include "acc_integration_log.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define LOG_BUFFER_MAX_SIZE 150


void acc_integration_log(acc_log_level_t level, const char *module, const char *format, ...)
{
	char    log_buffer[LOG_BUFFER_MAX_SIZE];
	va_list ap;

	va_start(ap, format);

	int ret = vsnprintf(log_buffer, LOG_BUFFER_MAX_SIZE, format, ap);
	if (ret >= LOG_BUFFER_MAX_SIZE)
	{
		log_buffer[LOG_BUFFER_MAX_SIZE - 4] = '.';
		log_buffer[LOG_BUFFER_MAX_SIZE - 3] = '.';
		log_buffer[LOG_BUFFER_MAX_SIZE - 2] = '.';
		log_buffer[LOG_BUFFER_MAX_SIZE - 1] = 0;
	}

	switch (level)
	{
		case ACC_LOG_LEVEL_ERROR:
			NRF_LOG_ERROR("%s:%s", module, log_buffer);
			break;
		case ACC_LOG_LEVEL_WARNING:
			NRF_LOG_WARNING("%s:%s", module, log_buffer);
			break;
		case ACC_LOG_LEVEL_INFO:
			NRF_LOG_INFO("%s:%s", module, log_buffer);
			break;
		case ACC_LOG_LEVEL_VERBOSE:
			NRF_LOG_INFO("%s:%s", module, log_buffer);
			break;
		default:
		case ACC_LOG_LEVEL_DEBUG:
			NRF_LOG_DEBUG("%s:%s", module, log_buffer);
			break;
	}

	// Flush log immediately when user is using deferred logging.
	if (NRF_LOG_DEFERRED != 0)
	{
		NRF_LOG_FLUSH();
	}

	va_end(ap);
}
