/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-04     stackryan      first version
 */

#ifndef SENSOR_ST_LIS2DH12_H__
#define SENSOR_ST_LIS2DH12_H__

#include "sensor.h"
#include "lis2dh12.h"

int rt_hw_lis2dh12_init(const char *name, struct rt_sensor_config *cfg);

#endif
