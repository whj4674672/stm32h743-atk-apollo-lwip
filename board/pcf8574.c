/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-21     SummerGift   first version
 * 2018-11-22     flybreak     Make the first version of pcf8574's package
 */

#include "pcf8574.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME     "pcf8574"
#define DBG_LEVEL            DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

uint8_t pcf8574_port_read(pcf8574_device_t dev)
{
    uint8_t value;

    rt_device_read(&dev->bus->parent, dev->i2c_addr, &value, 1);

    return value;
}

void pcf8574_port_write(pcf8574_device_t dev, uint8_t value)
{
    rt_device_write(&dev->bus->parent, dev->i2c_addr, &value, 1);
}

uint8_t pcf8574_pin_read(pcf8574_device_t dev, uint8_t bit)
{
    uint8_t data;
    data = pcf8574_port_read(dev);

    if (data & (1 << bit))
        return 1;
    else
        return 0;
}

void pcf8574_pin_write(pcf8574_device_t dev, uint8_t bit, uint8_t value)
{
    uint8_t data;
    data = pcf8574_port_read(dev);

    if (value == 0)
        data &= ~(1 << bit);
    else 
        data |= 1 << bit;

    pcf8574_port_write(dev, data);
}

pcf8574_device_t pcf8574_init(const char *dev_name, rt_uint8_t i2c_addr)
{
    uint8_t buffer[] = { 0xFF };
    pcf8574_device_t dev = RT_NULL;
    
    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct pcf8574_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for pcf8574 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = (struct rt_i2c_bus_device *)rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("i2c_bus %s for PCF8574 not found!", dev_name);
        goto __exit;
    }

    if (i2c_addr != RT_NULL)
        dev->i2c_addr = i2c_addr;
    else
        dev->i2c_addr = PCF8574_ADDR_DEFAULT;

    if (rt_device_open(&dev->bus->parent, RT_NULL) != RT_EOK)
    {
        LOG_D("i2c_bus %s for PCF8574 opened failed!", dev_name);
        goto __exit;
    }

    rt_device_write(&dev->bus->parent, dev->i2c_addr, &buffer, 1);

    LOG_D("pcf8574 init done", dev_name);
    return dev;

__exit:
    if (dev != RT_NULL)
        rt_free(dev);

    return RT_NULL;
}

void pcf8574_deinit(struct pcf8574_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}
