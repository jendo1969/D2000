/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <misc/printk.h>
#include <stdio.h>

#include <device.h>
#include <i2c.h>
#include <uart.h>

#define BMC150_I2C_ADDR 0x10
#define ACCD_TEMP_REG   0x08

float temp_val = 0.0;

static void read_temperature(struct device *dev)
{
	int8_t data;

	data = ACCD_TEMP_REG;
	if (i2c_write(dev, &data, sizeof(data), BMC150_I2C_ADDR) != 0) {
		printk("Error on i2c_write()\n");
		return;
	}

	data = 0;
	if (i2c_read(dev, &data, sizeof(data), BMC150_I2C_ADDR) != 0) {
		printk("Error on i2c_read()\n");
		return;
	}

	temp_val = ((float)data) / 2 + 23;
}

void main(void)
{
	uint8_t buf[10];
	union dev_config cfg;
	struct device *i2c_dev;
	struct device *uart_dev;

	printk("Start I2C BMC150 sample\n");

	cfg.raw = 0;
	cfg.bits.use_10_bit_addr = 0;
	cfg.bits.speed = I2C_SPEED_STANDARD;
	cfg.bits.is_master_device = 1;

	i2c_dev = device_get_binding("I2C_0");
	if (!i2c_dev) {
		printk("I2C0: Device not found.\n");
		return;
	}

	if (i2c_configure(i2c_dev, cfg.raw) != 0) {
		printk("Error on i2c_configure()\n");
		return;
	}

	uart_dev = device_get_binding("UART_0");

	while (1) {
		read_temperature(i2c_dev);

		sprintf(buf, "temp=%4.1f\n", temp_val);
		uart_fifo_fill(uart_dev, buf, 10);

		k_sleep(1000);
	}
}
