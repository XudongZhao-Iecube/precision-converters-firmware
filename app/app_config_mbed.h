/***************************************************************************//**
 *   @file    app_config_mbed.h
 *   @brief   Header file for Mbed platform configurations
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef APP_CONFIG_MBED_H_
#define APP_CONFIG_MBED_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include <PinNames.h>

#include "uart_extra.h"
#include "irq_extra.h"
#include "spi_extra.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Select b/w Arduino or SDP-120 pin header (default is Arduino) */
//#define  SDP_120

#ifdef SDP_120
/* Pin mapping of SDP-K1 w.r.t SDP-120 connector */
#define SPI_CSB			SDP_SPI_CS_A
#define SPI_HOST_SDO	SDP_SPI_MOSI
#define SPI_HOST_SDI	SDP_SPI_MISO
#define SPI_SCK			SDP_SPI_SCK

#define SYNC_INB		SDP_GPIO_1
#define DIG_AUX_1		SDP_GPIO_0
#define DIG_AUX_2		SDP_GPIO_2
#else
/* Pin mapping of SDP-K1 w.r.t Arduino connector */
#define SPI_CSB			D10
#define SPI_HOST_SDO	D11
#define SPI_HOST_SDI	D12
#define SPI_SCK			D13

#define SYNC_INB		D4
#define DIG_AUX_1		D2
#define DIG_AUX_2		D7
#endif

/* Common pin mapping on SDP-K1 */
#define UART_TX			USBTX
#define	UART_RX			USBRX
#define LED_GPO			LED3

/* Time period for periodic ticker interrupt (in usec) */
#define TICKER_INTERRUPT_PERIOD_uSEC	(50000)

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

extern mbed_irq_init_param mbed_ticker_int_extra_init_params;
extern mbed_irq_init_param mbed_ext_int_extra_init_params;
extern mbed_uart_init_param mbed_uart_extra_init_params;
extern mbed_spi_init_param mbed_spi_extra_init_params;

#endif /* APP_CONFIG_MBED_H_ */
