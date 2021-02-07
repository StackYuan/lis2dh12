# LIS2DH12 软件包

# 简介

本驱动包支持SPI方式访问 lis2dh12 此款3轴加速度计。

# 传感器介绍

**LIS2DH12 ** 是 ST（意法半导体）公司专为可穿戴设备和 IOT 市场开发的一款超低功耗传感器，尺寸小巧。该模块常用于穿戴/手机等消费类电子场合。

## 支持情况

| 包含设备         | 加速度计 |
| ---------------- | -------- |
| **通讯接口**     |          |
| IIC              |          |
| SPI              | √        |
| **工作模式**     |          |
| 轮询             | √        |
| 中断             |          |
| FIFO             |          |
| **电源模式**     |          |
| 掉电             | √        |
| 低功耗           |          |
| 普通             | √        |
| **数据输出速率** | √        |
| **测量范围**     | √        |
| **自检**         |          |
| **多实例**       |          |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- SPI 驱动：LIS2DH12设备使用 SPI 进行数据通讯，需要系统 SPI 驱动框架支持；
- PIN 驱动：用于处理设备中断引脚；

### 获取软件包

使用 LIS2DH12软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
--- LIS2DH12 sensor driver package, support: 3-axis accelerometer,tempature.
    [*]   Enable lis2dh12 accelerometer
          Version (latest)  --->
```

**Enable lis2dh12 accelerometer**： 配置开启加速度计功能

**Version**：软件包版本选择

### 使用软件包

LIS2DH12软件包初始化函数如下所示：

```
int rt_hw_lis2dh12_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备和中断引脚）；
- 注册相应的传感器设备，完成 LIS2DH12设备的注册；

#### 初始化示例

如下代码示例进行了spi的port和传感器初始化：

```c
//port example for lis2dh12
#include "drv_gpio.h"
#include "drv_spi.h"
#include "sensor.h"
#include "sensor_st_lis2dh12.h"
#define PIN_SPI2_CS             GET_PIN(B, 12)

rt_err_t lis2dh12_port()
{
    struct rt_sensor_config cfg;
    struct rt_spi_device *spi_dev_lis2d;

    cfg.intf.dev_name = "spi20";
    cfg.intf.user_data = (void *)RT_NULL;   //address
    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_pin_mode(PIN_SPI2_CS, PIN_MODE_OUTPUT);
    rt_pin_write(PIN_SPI2_CS, PIN_LOW);
    rt_hw_spi_device_attach("spi2", "spi20", GPIOB, GPIO_PIN_12);

    spi_dev_lis2d = (struct rt_spi_device *)rt_device_find("spi20");
    if (spi_dev_lis2d == RT_NULL)
    {
        rt_kprintf("Can't find acce device");
        return -RT_ERROR;
    }
    /* config spi */
    struct rt_spi_configuration spi_cfg;
    spi_cfg.data_width = 8;
    spi_cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    spi_cfg.max_hz = 5 * 1000 * 1000; /* 42M,SPI max 42MHz,lcd 4-wire spi */

    rt_spi_configure(spi_dev_lis2d, &spi_cfg);

    rt_hw_lis2dh12_init("lis2dh12",&cfg);
    return RT_EOK;
}
INIT_DEVICE_EXPORT(lis2dh12_port);
```

## 注意事项

中断方式实现数据读取暂需要自行实现。

## 联系人信息

维护人:

- [stackRyan](https://github.com/stackryan) 

