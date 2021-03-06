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
#include "spi_extra.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Pin mapping of SDP-K1 w.r.t Arduino connector */
#define SPI_CSB			D10
#define SPI_HOST_SDO	D11
#define SPI_HOST_SDI	D12
#define SPI_SCK			D13
#define LDAC_GPIO		D8

/* Common pin mapping on SDP-K1 */
#define UART_TX			USBTX
#define	UART_RX			USBRX

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

extern mbed_uart_init_param mbed_uart_extra_init_params;
extern mbed_spi_init_param mbed_spi_extra_init_params;

#endif /* APP_CONFIG_MBED_H_ */
