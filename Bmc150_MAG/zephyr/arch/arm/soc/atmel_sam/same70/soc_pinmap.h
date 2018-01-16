/*
 * Copyright (c) 2016 Piotr Mienkowski
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Atmel SAM E70 MCU pin definitions.
 *
 * This file contains pin configuration data required by different MCU
 * modules to correctly configure GPIO controller.
 */

#ifndef _ATMEL_SAM_SOC_PINMAP_H_
#define _ATMEL_SAM_SOC_PINMAP_H_

#include <soc.h>

/* Ethernet MAC (GMAC) */

#define PINS_GMAC_MASK (PIO_PD0A_GMAC_GTXCK | PIO_PD1A_GMAC_GTXEN \
	  | PIO_PD2A_GMAC_GTX0 | PIO_PD3A_GMAC_GTX1 | PIO_PD4A_GMAC_GRXDV \
	  | PIO_PD5A_GMAC_GRX0 | PIO_PD6A_GMAC_GRX1 | PIO_PD7A_GMAC_GRXER \
	  | PIO_PD8A_GMAC_GMDC | PIO_PD9A_GMAC_GMDIO)
#define PIN_GMAC_SET1 {PINS_GMAC_MASK, PIOD, ID_PIOD, SOC_GPIO_FUNC_A}

#define PINS_GMAC0 {PIN_GMAC_SET1}

/* Universal Synchronous Asynchronous Receiver Transmitter (USART) */

#define PIN_USART0_RXD {PIO_PB0C_USART0_RXD0,  PIOB, ID_PIOB, SOC_GPIO_FUNC_C}
#define PIN_USART0_TXD {PIO_PB1C_USART0_TXD0,  PIOB, ID_PIOB, SOC_GPIO_FUNC_C}
#define PIN_USART0_CTS {PIO_PB2C_USART0_CTS0,  PIOB, ID_PIOB, SOC_GPIO_FUNC_C}
#define PIN_USART0_RTS {PIO_PB3C_USART0_RTS0,  PIOB, ID_PIOB, SOC_GPIO_FUNC_C}
#define PIN_USART0_SCK {PIO_PB13C_USART0_SCK0, PIOB, ID_PIOB, SOC_GPIO_FUNC_C}

#define PINS_USART0 {PIN_USART0_RXD, PIN_USART0_TXD, PIN_USART0_CTS, \
	  PIN_USART0_RTS, PIN_USART0_SCK}

#define PIN_USART1_RXD {PIO_PA21A_USART1_RXD1, PIOA, ID_PIOA, SOC_GPIO_FUNC_A}
#define PIN_USART1_TXD {PIO_PB4D_USART1_TXD1,  PIOB, ID_PIOB, SOC_GPIO_FUNC_D}
#define PIN_USART1_CTS {PIO_PA25A_USART1_CTS1, PIOA, ID_PIOA, SOC_GPIO_FUNC_A}
#define PIN_USART1_RTS {PIO_PA24A_USART1_RTS1, PIOA, ID_PIOA, SOC_GPIO_FUNC_A}
#define PIN_USART1_SCK {PIO_PA23A_USART1_SCK1, PIOA, ID_PIOA, SOC_GPIO_FUNC_A}

#define PINS_USART1 {PIN_USART1_RXD, PIN_USART1_TXD, PIN_USART1_CTS, \
	  PIN_USART1_RTS, PIN_USART1_SCK}

#define PIN_USART2_RXD {PIO_PD15B_USART2_RXD2, PIOD, ID_PIOD, SOC_GPIO_FUNC_B}
#define PIN_USART2_TXD {PIO_PD16B_USART2_TXD2, PIOD, ID_PIOD, SOC_GPIO_FUNC_B}
#define PIN_USART2_CTS {PIO_PD19B_USART2_CTS2, PIOD, ID_PIOD, SOC_GPIO_FUNC_B}
#define PIN_USART2_RTS {PIO_PD18B_USART2_RTS2, PIOD, ID_PIOD, SOC_GPIO_FUNC_B}
#define PIN_USART2_SCK {PIO_PD17B_USART2_SCK2, PIOD, ID_PIOD, SOC_GPIO_FUNC_B}

#define PINS_USART2 {PIN_USART2_RXD, PIN_USART2_TXD, PIN_USART2_CTS, \
	  PIN_USART2_RTS, PIN_USART2_SCK}

#endif /* _ATMEL_SAM_SOC_PINMAP_H_ */
