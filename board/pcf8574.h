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

#ifndef __PCF8574_H
#define __PCF8574_H

#include <rtthread.h>
#include <rtdevice.h>

#define PCF8574_ADDR_DEFAULT    0x20

/* pcf8574 device structure */
struct pcf8574_device
{
    struct rt_i2c_bus_device *bus;
    rt_uint8_t i2c_addr;
};
typedef struct pcf8574_device *pcf8574_device_t;

/**
 * This function initialize the pcf8574 device.
 *
 * @param dev_name the name of i2c bus device
 * @param i2c_addr the i2c device address for i2c communication,RT_NULL use default address
 *
 * @return the pointer of device structure, RT_NULL reprensents  initialization failed.
 */
pcf8574_device_t pcf8574_init(const char *dev_name, rt_uint8_t i2c_addr);

/**
 * This function releases memory
 *
 * @param dev the pointer of device structure
 */
void pcf8574_deinit(struct pcf8574_device *dev);

/**
 * This function read the data port of pcf8574.
 *
 * @param dev the pointer of device structure
 *
 * @return the state of data port. 0xFF meas all pin is high.
 */
uint8_t pcf8574_port_read(pcf8574_device_t dev);

/**
 * This function sets the status of the data port.
 *
 * @param dev the pointer of device structure
 * @param port_val the port value you want to set, 0xFF meas all pin output high.
 */
void pcf8574_port_write(pcf8574_device_t dev, uint8_t port_val);

/**
 * This function read the specified port pin of the pcf8574.
 *
 * @param dev the pointer of device structure
 * @param pin the specified pin of the data port
 *
 * @return the status of the specified data port pin, 0 is low, 1 is high.
 */
uint8_t pcf8574_pin_read(pcf8574_device_t dev, uint8_t pin);

/**
 * This function sets the status of the specified port pin.
 *
 * @param dev the pointer of device structure
 * @param pin_val the specified pin value you want to set, 0 is low, 1 is high.
 */
void pcf8574_pin_write(pcf8574_device_t dev, uint8_t pin, uint8_t pin_val);

#endif

