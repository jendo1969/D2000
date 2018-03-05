/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <board.h>
#include <device.h>
#include <gpio.h>
#include <misc/util.h>
#include <misc/printk.h>

/* change this to use another GPIO port */
#ifdef SW0_GPIO_NAME
#define PORT	SW0_GPIO_NAME
#else
#error SW0_GPIO_NAME needs to be set in board.h
#endif

/* change this to use another GPIO pin */
#ifdef SW0_GPIO_PIN
#define PIN     SW0_GPIO_PIN
#else
#error SW0_GPIO_PIN needs to be set in board.h
#endif

/* change to use another GPIO pin interrupt config */
#ifdef SW0_GPIO_INT_CONF
#define EDGE    SW0_GPIO_INT_CONF
#else
/*
 * If SW0_GPIO_INT_CONF not defined used default EDGE value.
 * Change this to use a different interrupt trigger
 */
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_INT_ACTIVE_HIGH | GPIO_INT_DOUBLE_EDGE)
#endif

/* change this to enable pull-up/pull-down */
#define PULL_UP 0

/* Sleep time */
#define SLEEP_TIME	1000

/* Change this if you have an LED connected to a custom port */
#define PORT1	LED0_GPIO_PORT

/* Change this if you have an LED connected to a custom pin */
#define LED	LED0_GPIO_PIN

#define PIN_B     5


int count = 0;
uint32_t val = 0;
uint32_t old_A = 0;
uint32_t old_B = 0;

void rotA_Trig(struct device *gpiob, struct gpio_callback *cb,
		    uint32_t pins)
{
	gpio_pin_read(gpiob, PIN, &val);
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
		if(old_B == 1)
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

static struct gpio_callback gpio_cb;
static struct gpio_callback gpio_cb_B;

void main(void)
{
	struct device *gpiob;

	printk("Press the user defined button on the board\n");
	gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, PIN, GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);
	gpio_init_callback(&gpio_cb, rotA_Trig, BIT(PIN));
	gpio_add_callback(gpiob, &gpio_cb);
	gpio_pin_enable_callback(gpiob, PIN);

	gpio_pin_configure(gpiob, PIN_B, GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);
	gpio_init_callback(&gpio_cb_B, rotB_Trig, BIT(PIN_B));
	gpio_add_callback(gpiob, &gpio_cb_B);
	gpio_pin_enable_callback(gpiob, PIN_B);

	struct device *dev;
	dev = device_get_binding(PORT1);
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED, GPIO_DIR_OUT);


	while (1) {


		/* Set pin to HIGH/LOW */
		gpio_pin_write(dev, LED, !val);
		printk("Counter = %d\n", count);

		k_sleep(SLEEP_TIME);
	}
}
