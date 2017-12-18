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

#include <stdio.h>
#include <string.h>
#include <misc/byteorder.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME 	500
#define BUF_MAXSIZE	256

static struct device *uart0_dev;
static unsigned char rx_buf[BUF_MAXSIZE];
static unsigned char tx_buf[BUF_MAXSIZE];

static void msg_dump(const char *s, unsigned char *data, unsigned len)
{
	unsigned i;

	printf("%s: ", s);
	for (i = 0; i < len; i++) {
		printf("%02x ", data[i]);
	}
	printf("(%u bytes)\n", len);
	uart_fifo_fill(uart0_dev, data, len);
}

static void uart0_isr(struct device *x)
{
	int len = uart_fifo_read(uart0_dev, rx_buf, BUF_MAXSIZE);

	ARG_UNUSED(x);
	msg_dump(__func__, rx_buf, len);
}

static void uart0_init(void)
{
	uart0_dev = device_get_binding("UART_0");

	uart_irq_callback_set(uart0_dev, uart0_isr);
	uart_irq_rx_enable(uart0_dev);

	printf("%s() done\n", __func__);
}


void main(void)
{
	unsigned int *size = (unsigned int *)tx_buf;

	printf("Sample app running on: %s\n", CONFIG_ARCH);

	uart0_init();

	memcpy(tx_buf, "TEST\n", 5);

	uart_fifo_fill(uart0_dev, tx_buf, 5);

	while (1)
	{
		k_sleep(SLEEP_TIME);
	}
}
