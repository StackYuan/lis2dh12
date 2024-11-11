/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-04     stackryan      first version
 */

#include "st_lis2dh12_sensor_v1.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.st.lis2dh12"
#define DBG_COLOR
#include <rtdbg.h>

#define SENSOR_ACC_RANGE_2G   2000
#define SENSOR_ACC_RANGE_4G   4000
#define SENSOR_ACC_RANGE_8G   8000
#define SENSOR_ACC_RANGE_16G  16000

#include "lis2dh12_reg.h"
#include "board.h"

static uint8_t whoamI = 0x00;
static struct rt_spi_device *spi_dev_acce;
static LIS2DH12_Object_t lis2dh12_obj;

static int32_t rt_func_ok(void)
{
    return 0;
}

static int32_t get_tick(void)
{
    return rt_tick_get();
}

static int32_t rt_spi_write_reg(uint16_t address, uint16_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr = reg | 0x40;
    struct rt_spi_message msgs[2];

    msgs[0].send_buf  = &addr;
    msgs[0].recv_buf = RT_NULL;
    msgs[0].length   = 1;
    msgs[0].cs_take  = 1;
    msgs[0].cs_release = 0;
    msgs[0].next = &msgs[1];

    msgs[1].send_buf   = data;
    msgs[1].recv_buf   = RT_NULL;
    msgs[1].length     = len;
    msgs[1].cs_take    = 0;
    msgs[1].cs_release = 1;
    msgs[1].next       = RT_NULL;

    if (rt_spi_transfer_message(spi_dev_acce, msgs))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int32_t rt_spi_read_reg(uint16_t address, uint16_t reg, uint8_t *data, uint16_t len)
{
    // rt_uint8_t tmp = reg;
    uint8_t addr = reg | 0xC0;
    struct rt_spi_message msgs[2];

    msgs[0].send_buf   = &addr;
    msgs[0].recv_buf   = RT_NULL;
    msgs[0].length     = 1;
    msgs[0].cs_take    = 1;
    msgs[0].cs_release = 0;
    msgs[0].next       = &msgs[1];

    msgs[1].send_buf   = RT_NULL;
    msgs[1].recv_buf   = data;
    msgs[1].length     = len;
    msgs[1].cs_take    = 0;
    msgs[1].cs_release = 1;
    msgs[1].next       = RT_NULL;

    if (rt_spi_transfer_message(spi_dev_acce, msgs))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _lis2dh12_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t  id;
    LIS2DH12_IO_t io_ctx;

    spi_dev_acce = (struct rt_spi_device *)rt_device_find(intf->dev_name);
    if (spi_dev_acce == RT_NULL)
    {
        LOG_E("Can't find acce device");
        return -RT_ERROR;
    }

    /* Configure the accelero driver */
    io_ctx.BusType     = LIS2DH12_SPI_4WIRES_BUS; /* SPI */
    io_ctx.Address     = RT_NULL;
    io_ctx.Init        = rt_func_ok;
    io_ctx.DeInit      = rt_func_ok;
    io_ctx.ReadReg     = rt_spi_read_reg;
    io_ctx.WriteReg    = rt_spi_write_reg;
    io_ctx.GetTick     = get_tick;

    if (LIS2DH12_RegisterBusIO(&lis2dh12_obj, &io_ctx) != LIS2DH12_OK)
    {
        return -RT_ERROR;
    }
    else if (LIS2DH12_ReadID(&lis2dh12_obj, &id) != LIS2DH12_OK)
    {
        LOG_E("read id failed");
        return -RT_ERROR;
    }
    LOG_D("acce id:%d",id);
    if (LIS2DH12_Init(&lis2dh12_obj) != LIS2DH12_OK)
    {
        LOG_E("acce init failed");
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _lis2dh12_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        LIS2DH12_SetFullScale(&lis2dh12_obj, range / 1000);
        LOG_D("acce set range %d", range);
    }

    return RT_EOK;
}

static rt_err_t _lis2dh12_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        LIS2DH12_SetOutputDataRate(&lis2dh12_obj, odr);
        LOG_D("acce set odr %d", odr);
    }

    return RT_EOK;
}

static rt_err_t _lis2dh12_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        lis2dh12_fifo_mode_set(&lis2dh12_obj.Ctx, LIS2DH12_BYPASS_MODE);
        LOG_D("set mode to POLLING");
    }
    else if (mode == RT_SENSOR_MODE_INT)
    {
        LOG_D("set mode to RT_SENSOR_MODE_INT");
    }
    else if (mode == RT_SENSOR_MODE_FIFO)
    {
        lis2dh12_fifo_mode_set(&lis2dh12_obj.Ctx, LIS2DH12_FIFO_MODE);
        lis2dh12_fifo_trigger_event_set(&lis2dh12_obj.Ctx, LIS2DH12_INT1_GEN);
        LOG_D("set mode to RT_SENSOR_MODE_FIFO");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", mode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _lis2dh12_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
        {
            LIS2DH12_Disable(&lis2dh12_obj);
        }

        LOG_D("set power down");
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
        {
            LIS2DH12_Enable(&lis2dh12_obj);
        }
        LOG_D("set power normal");
    }
    else
    {
        LOG_W("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_size_t _lis2dh12_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        LIS2DH12_Axes_t acce;

        if(LIS2DH12_OK == LIS2DH12_GetAxes(&lis2dh12_obj, &acce))
        {
            data->type = RT_SENSOR_CLASS_ACCE;
            data->data.acce.x = acce.x;
            data->data.acce.y = acce.y;
            data->data.acce.z = acce.z;
            data->timestamp = rt_sensor_get_ts();
            return 1;
        }
    }
    return 0;
}

static rt_size_t _lis2dh12_fifo_get_data(rt_sensor_t sensor, struct rt_sensor_data *data, rt_size_t len)
{
    LIS2DH12_Axes_t acce;
    rt_uint8_t i;

    for (i = 0; i < len; i++)
    {
        if (LIS2DH12_GetAxes(&lis2dh12_obj, &acce) == 0)
        {
            data[i].type = RT_SENSOR_CLASS_ACCE;
            data[i].data.acce.x = acce.x;
            data[i].data.acce.y = acce.y;
            data[i].data.acce.z = acce.z;
            data[i].timestamp = rt_sensor_get_ts();
        }
        else
            break;
    }
    return i;
}

static rt_size_t _lis2dh12_interrupt_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
/**
 * @TODO add interrupt method
 * 
 */
    return RT_EOK;
}

static rt_size_t _lis2dh12_get_id(void *args)
{
    LIS2DH12_ReadID(&lis2dh12_obj, args);
    if ((*(uint8_t *)args) != LIS2DH12_ID)
    {
        LOG_D("Err: no sensor! whoamI = %d!\r\n", args);
        return -1;
    }
    return RT_EOK;
}

static RT_SIZE_TYPE lis2dh12_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _lis2dh12_polling_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_INT)
    {
        return _lis2dh12_interrupt_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_FIFO)
    {
        return _lis2dh12_fifo_get_data(sensor, buf, len);
    }
    else
        return 0;
}

static rt_err_t lis2dh12_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        result = _lis2dh12_get_id(args);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _lis2dh12_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = _lis2dh12_set_odr(sensor, (rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _lis2dh12_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _lis2dh12_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    lis2dh12_fetch_data,
    lis2dh12_control
};

int rt_hw_lis2dh12_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_acce = RT_NULL;

    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -1;

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_acce->info.model      = "lis2dh12_acc";
        sensor_acce->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_SPI;
        sensor_acce->info.range_max  = SENSOR_ACC_RANGE_16G;
        sensor_acce->info.range_min  = SENSOR_ACC_RANGE_2G;
        sensor_acce->info.period_min = 5;

        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        sensor_acce->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    result = _lis2dh12_init(&cfg->intf);
    if (result != RT_EOK)
    {
        LOG_E("_lis2dh12 init err code: %d", result);
        goto __exit;
    }

    LOG_I("lis2dh12 init success");
    return RT_EOK;

__exit:
    if (sensor_acce != RT_NULL)
        rt_free(sensor_acce);

    return -RT_ERROR;
}
