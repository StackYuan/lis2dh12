#ifndef SENSOR_ST_LIS2DH12_H__
#define SENSOR_ST_LIS2DH12_H__

#include "sensor.h"
#include "lis2dh12.h"

// #define LSM6DSL_ADDR_DEFAULT (0xD6 >> 1)

int rt_hw_lis2dh12_init(const char *name, struct rt_sensor_config *cfg);

#endif
