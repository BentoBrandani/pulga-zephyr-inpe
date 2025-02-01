/*
 * Copyright (c) 2024 Kauê Rodrigues Barbosa
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SCD30_H_
#define _SCD30_H_

#include <zephyr/drivers/sensor.h>

/* Callback strutucture to be shared in application and driver */
typedef void (*scd30_callback_t)(const struct device *scd30_device,
                          		 const struct scd30_data *scd30_data);

void scd30_register_callback(scd30_callback_t cb);

/* Additional custom attributes */
enum scd30_attribute {
	/**
	 * Sensor sampling period, i.e. how many seconds there are between
	 * two subsequent measurements.
	 */
	SCD30_SENSOR_ATTR_SAMPLING_PERIOD = SENSOR_ATTR_PRIV_START,
};

#endif // SCD30_H
