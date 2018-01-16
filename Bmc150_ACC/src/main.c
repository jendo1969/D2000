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

#define BMC150_I2C_ADDR 0x10
#define ACCD_TEMP_REG   0x08                	/* Temperature register */

#define ACCELEROMTER_RANGE 		0x0F            /* Accelerometer registers */
#define ACCELEROMETER_RANGE_2G  0b0011
#define ACCELEROMETER_RANGE_4G  0b0101
#define ACCELEROMETER_RANGE_8G  0b1000
#define ACCELEROMETER_RANGE_16G 0b1100
#define DATA_ACC_X_LSB 0x02
#define DATA_ACC_X_MSB 0x03
#define DATA_ACC_Y_LSB 0x04
#define DATA_ACC_Y_MSB 0x05
#define DATA_ACC_Z_LSB 0x06
#define DATA_ACC_Z_MSB 0x07

#define ACCELEROMTER_BANDWIDTH		 0x10       /* Accelerometer registers */
#define ACCELEROMTER_BANDWIDTH_64MS  0x8
#define ACCELEROMTER_BANDWIDTH_32MS  0x9
#define ACCELEROMTER_BANDWIDTH_16MS  0xA
#define ACCELEROMTER_BANDWIDTH_8MS   0xB
#define ACCELEROMTER_BANDWIDTH_4MS   0xC
#define ACCELEROMTER_BANDWIDTH_2MS   0xD
#define ACCELEROMTER_BANDWIDTH_1MS   0xE
#define ACCELEROMTER_BANDWIDTH_500uS 0xF

float temp_val = 0.0;
float acc_x_val = 0.0;
float acc_y_val = 0.0;
float acc_z_val = 0.0;
int16_t acc_x = 0;
int16_t acc_y = 0;
int16_t acc_z = 0;

struct device *i2c_dev;

static int i2c_init()
{
	union dev_config cfg;

	cfg.raw = 0;
	cfg.bits.use_10_bit_addr = 0;
	cfg.bits.speed = I2C_SPEED_STANDARD;
	cfg.bits.is_master_device = 1;

	i2c_dev = device_get_binding("I2C_0");
	if (!i2c_dev) {
		printk("I2C0: Device not found.\n");
		return -1;
	}

	if (i2c_configure(i2c_dev, cfg.raw) != 0) {
		printk("Error on i2c_configure()\n");
		return -1;
	}

	return 0;
}

static int read_register(uint16_t addr, uint8_t reg, uint8_t *const data, uint32_t len)
{
	if (i2c_write(i2c_dev, &reg, 1, addr) != 0) {
		printk("Error on i2c_write()\n");
		return 1;
	}

	if (i2c_read(i2c_dev, data, len, addr) != 0) {
		printk("Error on i2c_read()\n");
		return 1;
	}
	return 0;
}

static int write_register(uint16_t addr, uint8_t reg, uint8_t data)
{
	int8_t data1[2];

	data1[0] = reg;
	data1[1] = data;
	if (i2c_write(i2c_dev, &data1, sizeof(data1), addr) != 0)
	{
		printk("Error on i2c_write()\n");
		return 1;
	}
	return 0;
}


static void read_Temperature()
{
	int8_t data;

	if (read_register(BMC150_I2C_ADDR, ACCD_TEMP_REG, &data, 1)!= 0)
	{
		temp_val = 999;
		return;
	}
	temp_val = ((float)data) / 2 + 23;
}

static void write_Acc_range(uint8_t data)
{
	if (write_register(BMC150_I2C_ADDR, ACCELEROMTER_RANGE, data) != 0)
	{
		printk("Error on i2c_write()\n");
		return;
	}
}


static void write_Acc_bandwidth(uint8_t data)
{
	if (write_register(BMC150_I2C_ADDR, ACCELEROMTER_BANDWIDTH, data) != 0)
	{
		printk("Error on i2c_write()\n");
		return;
	}
}
static void read_Accelerometer()
{
	int8_t reg;
	int8_t data[6];
	int16_t lsb, msb;

	reg = DATA_ACC_X_LSB;
	if (i2c_write(i2c_dev, &reg, 1, BMC150_I2C_ADDR) != 0) {
		printk("Error on i2c_write()\n");
		return;
	}

	if (i2c_read(i2c_dev, data, 6, BMC150_I2C_ADDR) != 0) {
		printk("Error on i2c_read()\n");
		return;
	}

	lsb = data[0];
	msb = data[1];
	msb <<= 8;
	acc_x = (msb | lsb) >> 4;
	acc_x_val = ((float)acc_x) * 2 / 4096;

	lsb = data[2];
	msb = data[3];
	msb <<= 8;
	acc_y = (msb | lsb) >> 4;
	acc_y_val = ((float)acc_y) * 2 / 4096;

	lsb = data[4];
	msb = data[5];
	msb <<= 8;
	acc_z = (msb | lsb) >> 4;
	acc_z_val = ((float)acc_z) * 2 / 4096;
}

void main(void)
{
	uint8_t buf[64];

	printk("Start I2C BMC150 sample\n");

	i2c_init();


	write_Acc_range(ACCELEROMETER_RANGE_2G);
	write_Acc_bandwidth(ACCELEROMTER_BANDWIDTH_64MS);

	while (1) {
		read_Temperature();
		read_Accelerometer();
		sprintf(buf, "(t,x,y,z)=(%4.1f,%+04d,%+04d,%+04d)\n", temp_val, acc_x, acc_y, acc_z);
		printk(buf);

		k_sleep(500);
	}
}
