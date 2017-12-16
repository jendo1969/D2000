/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <device.h>
#include <uart.h>
#include "board.h"
#include <misc/printk.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME 	1000

void main(void)
{
	struct device *uart_dev;

	uart_dev = device_get_binding("UART_0");

	while(1)
	{
		uart_poll_out(uart_dev, 'a');

		k_sleep(SLEEP_TIME);
	}
}
