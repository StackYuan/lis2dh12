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

#include "lis2dh12.h"

#include <rtthread.h>
#include <rtdevice.h>

#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif
#endif

int rt_hw_lis2dh12_init(const char *name, struct rt_sensor_config *cfg);

#endif
