/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr.h>

#include <misc/printk.h>
#include <stdio.h>

#include <device.h>
#include <i2c.h>

typedef struct {
	int x, y, z;
} bmx1xx_mag_t;

typedef enum {
	BMX1XX_MAG_POWER_SUSPEND = 0x0,
	BMX1XX_MAG_POWER_ACTIVE = 0x1,
} bmx1xx_mag_power_t;

typedef enum {
	BMX1XX_MAG_PRESET_REGULAR,
	BMX1XX_MAG_PRESET_LOW_POWER,
	BMX1XX_MAG_PRESET_ENHANCED,
	BMX1XX_MAG_PRESET_HIGH_ACCURACY,
} bmx1xx_mag_preset_t;

#define BMC150_I2C_ADDR 	0x10
#define BMC150_I2C_MAG_ADDR 0x12

#define BMX1XX_REG_ACCEL_CHIPID (0x00)
#define ACCD_TEMP_REG   		0x08           	/* Temperature register */

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

#define BMX1XX_REG_MAG_X_LSB 		(0x42)
#define BMX1XX_REG_MAG_X_MSB		(0x43)
#define BMX1XX_REG_MAG_Y_LSB 		(0x44)
#define BMX1XX_REG_MAG_Y_MSB 		(0x45)
#define BMX1XX_REG_MAG_Z_LSB 		(0x46)
#define BMX1XX_REG_MAG_Z_MSB 		(0x47)
#define BMX1XX_REG_MAG_RHALL_LSB 	(0x48)
#define BMX1XX_REG_MAG_RHALL_MSB 	(0x49)
#define BMX1XX_REG_MAG_POWER_MODES 	(0x4B)
#define BMX1XX_REG_MAG_OPERATION_MODES (0x4C)
#define BMC150_MAGN_REG_INT          0x4D
#define BMC150_MAGN_REG_INT_DRDY     0x4E

#define BMX1XX_REG_MAG_REPETION_XY 	(0x51)
#define BMX1XX_REG_MAG_REPETION_Z 	(0x52)
#define BMX1XX_CHIP_ID_MAJOR 		(0xFA)

#define M_PI 3.14159265358979323846


#define EXTRACT_MAG_XY(msb, lsb) ((int16_t)((lsb) | (msb) << 8) >> 3)
#define EXTRACT_MAG_Z(msb, lsb) ((int16_t)((lsb) | (msb) << 8) >> 1)


/*
 * Values used for presets are from Table 6 (page 24) of the BMC150 datasheet.
 */
static const uint8_t operation_modes[] = {
    0,      /* 10 Hz */
    0,      /* 10 Hz */
    0,      /* 10 Hz */
    5 << 3, /* 20 Hz */
};



// ACCEROMETER
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

int bmx1xx_init()
{
	int rc;
	uint8_t data;

	rc = read_register(BMC150_I2C_ADDR, BMX1XX_REG_ACCEL_CHIPID, &data,
			   sizeof(data));
	if (rc != 0) {
		return rc;
	}

	if (data != BMX1XX_CHIP_ID_MAJOR) {
		return -ENODEV;
	}

	return 0;
}

int mag_init()
{
	/* Activate the magneto. */
	if (0 != write_register(BMC150_I2C_MAG_ADDR, BMX1XX_REG_MAG_POWER_MODES, BMX1XX_MAG_POWER_ACTIVE))
	{
		printk("Error: Unable to set BMC150 power state.");
		return 1;
	}

	k_sleep(10);

	// Change magnetometer mode in Active mode
	if (0 != write_register(BMC150_I2C_MAG_ADDR, BMX1XX_REG_MAG_OPERATION_MODES, 0x38))
	{
		printk("Error: Unable to set BMC150 Active mode.");
		return 1;
	}

	// Enable magnetometer axes
	if (0 != write_register(BMC150_I2C_MAG_ADDR, BMC150_MAGN_REG_INT_DRDY, 0x07))
	{
		printk("Error: Unable to set BMC150 magnetometer axes.");
		return 1;
	}

	//  Enable interrupts for every axis and override
	if (0 != write_register(BMC150_I2C_MAG_ADDR, BMC150_MAGN_REG_INT, 0xFF))
	{
		printk("Error: Unable to set BMC150 Enable interrupts.");
		return 1;
	}

	return 0;
}

/* Convert degrees into compass direction. */
static const char *degrees_to_direction(unsigned int deg)
{
	if (deg >= 360) {
		deg %= 360;
	}

	if (deg >= 338 || deg < 23) {
		return "N";
	} else if (deg < 68) {
		return "NE";
	} else if (deg < 113) {
		return "E";
	} else if (deg < 158) {
		return "SE";
	} else if (deg < 203) {
		return "S";
	} else if (deg < 248) {
		return "SW";
	} else if (deg < 293) {
		return "W";
	} else {
		return "NW";
	}
}

int bmx1xx_read_mag(bmx1xx_mag_t *const mag)
{
	int rc = 0;
	uint8_t raw_mag[6];
	int16_t x, y, z;

	rc = read_register(BMC150_I2C_MAG_ADDR, BMX1XX_REG_MAG_X_LSB, raw_mag,
			   sizeof(raw_mag));
	if (rc != 0) {
		return rc;
	}

	x = EXTRACT_MAG_XY(raw_mag[1], raw_mag[0]);
	y = EXTRACT_MAG_XY(raw_mag[3], raw_mag[2]);
	z = EXTRACT_MAG_Z(raw_mag[5], raw_mag[4]);

	mag->x = x;
	mag->y = y;
	mag->z = z;

	return 0;
}


void main(void)
{
	uint8_t buf[64];
	bmx1xx_mag_t mag = {0};
	double heading;
	int deg;
	/* Adjusted magnetometer readings */
	double mag_x, mag_y, mag_z;


	printk("Start I2C BMC150 sample\n");

	i2c_init();

	if(bmx1xx_init() != 0)
	{
		printk("Error I2C BMC150 sample\n");
		return;
	}

	mag_init();

	write_Acc_range(ACCELEROMETER_RANGE_2G);
	write_Acc_bandwidth(ACCELEROMTER_BANDWIDTH_64MS);

	while (1) {
		read_Temperature();
		read_Accelerometer();
		sprintf(buf, "ACC:x:%+04d, y:%+04d, z:%+04d, temp:%d\n", acc_x, acc_y, acc_z, (int)temp_val);
		printk(buf);

		/* Read the value from the magneto. */
		bmx1xx_read_mag(&mag);

		/* Calculate the heading. */
		heading = atan2((double)mag.y, (double)mag.x);
		if (heading < 0) {
			heading += (2 * M_PI);
		}

		/* Convert the heading into degrees. */
		deg = (int)((heading * 180) / M_PI);

		sprintf(buf, "MAG:x:%+04d, y:%+04d, z:%+04d, deg:%03d, dir:%s\n", mag.x, mag.y, mag.z, deg, degrees_to_direction(deg));
		printk(buf);

		k_sleep(500);
	}
}
