/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM.
 *
 * This app uses PWM[0].
 */

#include <zephyr.h>
#include <kernel.h>

#include <misc/printk.h>
#include <device.h>
#include <pwm.h>
#include <pinmux.h>

#include <board.h>
#include <device.h>
#include <gpio.h>
#include <misc/util.h>

#include <i2c.h>

#include "INA219.h"

//#include <INA219.h>


#define MSEC_100 100
#define MSEC_50 50
#define MSEC_10 10
#define MSEC_1 1

/* change this to use another GPIO port */
#define PORT	SW0_GPIO_NAME

/* change to use another GPIO pin interrupt config */
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_INT_ACTIVE_HIGH | GPIO_INT_DOUBLE_EDGE)

/* change this to enable pull-up/pull-down */
#define PULL_UP 0

#define PIN_A     10
#define PIN_B     5

#define PIN_SW    SW0_GPIO_PIN

#define PIN_DIR   9

/* change this to enable pull-up/pull-down */
#define PULL_UP 0

int count = 0;
int old_count = 0;
uint32_t val = 0;
uint32_t old_A = 0;
uint32_t old_B = 0;

uint32_t time_cnt = 0;

#define PWM_DRIVER CONFIG_PWM_QMSI_DEV_NAME
#define PWM_CHANNEL 1

#define PINMUX_NAME CONFIG_PINMUX_NAME
#define PWM1_PIN 24

/* in micro second */
//#define MIN_PERIOD	2000
//#define MIN_PERIOD	(USEC_PER_SEC / 5000) // 200us
//#define MIN_PERIOD	(USEC_PER_SEC / 800) // 1250us 60rpm
//#define MIN_PERIOD	(USEC_PER_SEC / 1600) // 625us 120rpm
//#define MIN_PERIOD	(USEC_PER_SEC / 3200) // 312.5us 240rpm
#define MIN_PERIOD	(USEC_PER_SEC / 6400) // 156.25us 480rpm

/* in micro second */
//#define MAX_PERIOD	2000000
//#define MAX_PERIOD	200000
#define MAX_PERIOD	1000000

//#define ACC_STEP	5000
#define ACC_STEP	10000




void rotA_Trig(struct device *gpiob, struct gpio_callback *cb,
		    uint32_t pins)
{
	gpio_pin_read(gpiob, PIN_A, &val);
	if(val == 1)
	{
		if(old_B == 1)
		{
			count++;
		}
		else
		{
			count--;
		}
	}
	else
	{
		if(old_B == 1)
		{
			count--;
		}
		else
		{
			count++;
		}
	}
	old_A = val;
}

void rotB_Trig(struct device *gpiob, struct gpio_callback *cb,
		    uint32_t pins)
{
	gpio_pin_read(gpiob, PIN_B, &val);
	if(val == 1)
	{
		if(old_A == 1)
		{
			count--;
		}
		else
		{
			count++;
		}
	}
	else
	{
		if(old_A == 1)
		{
			count++;
		}
		else
		{
			count--;
		}
	}
	old_B = val;
}




static struct gpio_callback gpio_cb_A;
static struct gpio_callback gpio_cb_B;


void main(void)
{
	uint32_t val = 0;
	uint32_t old_val = 0;
	uint8_t r_dir = 0;

	struct device *gpiob;
	union dev_config cfg;

	float shuntvoltage = 0;
	float busvoltage = 0;
	float current_mA = 0;
	float loadvoltage = 0;
	float power_mW = 0;

	printk("PWM demo STEP MOTER\n");

	gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	// RORTRY ENCODER
	gpio_pin_configure(gpiob, PIN_A, GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);
	gpio_init_callback(&gpio_cb_A, rotA_Trig, BIT(PIN_A));
	gpio_add_callback(gpiob, &gpio_cb_A);
	gpio_pin_enable_callback(gpiob, PIN_A);

	gpio_pin_configure(gpiob, PIN_B, GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);
	gpio_init_callback(&gpio_cb_B, rotB_Trig, BIT(PIN_B));
	gpio_add_callback(gpiob, &gpio_cb_B);
	gpio_pin_enable_callback(gpiob, PIN_B);

	// SW0
	gpio_pin_configure(gpiob, PIN_SW, GPIO_DIR_IN );

	// DIR
	gpio_pin_configure(gpiob, PIN_DIR, GPIO_DIR_OUT );
	gpio_pin_write(gpiob, PIN_DIR, r_dir);

	// PWM
	struct device *pwm_dev;
	int32_t period = MAX_PERIOD;
	uint8_t dir = 0;

	pwm_dev = device_get_binding(PWM_DRIVER);
	if (!pwm_dev) {
		printk("Cannot find %s!\n", PWM_DRIVER);
		return;
	}

	struct device *pinmux = device_get_binding(PINMUX_NAME);
	uint32_t function;

	if (!pinmux) {
		printk("Cannot get PINMUX\n");
		return;
	}

	if (pinmux_pin_set(pinmux, PWM1_PIN, PINMUX_FUNC_C)) {
		printk("Fail to set pin func, %u : %u\n",
			  PWM1_PIN, PINMUX_FUNC_C);
		return;
	}

	if (pinmux_pin_get(pinmux, PWM1_PIN, &function)) {
		printk("Fail to get pin func\n");
		return;
	}

	if (function != PINMUX_FUNC_C) {
		printk("Error. PINMUX get doesn't match PINMUX set\n");
		return;
	}

	// I2C
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

	// Set chip to large range config values to start
	setCalibration_32V_2A();
//	setCalibration_16V_400mA();

	while (1)
	{
		// SW2 READ
		gpio_pin_read(gpiob, PIN_SW, &val);

		// PWM SET
		if(period == MAX_PERIOD)
		{
			if (pwm_pin_set_usec(pwm_dev, PWM_CHANNEL, MAX_PERIOD, 0))
			{
				;
				//printk("pwm pin set fails\n");
				//return;
			}
		}
		else
		{
			if (pwm_pin_set_usec(pwm_dev, PWM_CHANNEL, period, period / 2))
			{
				printk("pwm pin set fails\n");
				return;
			}
		}

		if (dir)
		{
			period += ACC_STEP;

			if (period > MAX_PERIOD)
			{
				period = MAX_PERIOD;
			}
		}
		else
		{
			period -= ACC_STEP;

			if (period < MIN_PERIOD)
			{
				period = MIN_PERIOD;
			}
		}

		if((period == MAX_PERIOD) || (period == MIN_PERIOD))
		{
			if((val == 0) && (val == old_val))
			{
				if(period == MAX_PERIOD)
				{
					if(r_dir == 0)
					{
						r_dir = 1;
					}
					else
					{
						r_dir = 0;
					}
					gpio_pin_write(gpiob, PIN_DIR, r_dir);
				}

				if(dir == 0)
				{
					dir = 1;
				}
				else
				{
					dir = 0;
				}
			}
		}
		old_val = val;

		time_cnt++;
		k_sleep(MSEC_10);

		if(time_cnt%10 == 0)
		{
//			printk("Counter = %d, sp = %d\n", count, count - old_count);
			old_count = count;

			shuntvoltage = getShuntVoltage_mV();
			busvoltage = getBusVoltage_V();
			current_mA = getCurrent_mA();
			power_mW = getPower_mW();
			loadvoltage = busvoltage + (shuntvoltage / 1000);

			printk("BV=%dmV,SV=%duV,LV=%dmV,C=%dmA,P=%dmW\n",
					(int16_t)(busvoltage*1000),
					(int32_t)(shuntvoltage*1000),
					(int16_t)(loadvoltage*1000),
					(int16_t)(current_mA),
					(int16_t)(power_mW));
		}
	}
}
