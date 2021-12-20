/***************************************************************************//**
 *   @file    app_config_stm32.h
 *   @brief   Header file for STM32 platform configurations
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef APP_CONFIG_STM32_H_
#define APP_CONFIG_STM32_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

#include "stm32_uart.h"
#include "stm32_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Pin mapping of SDP-K1 w.r.t Arduino connector */
#define SPI_CSB			11 //ARD_SPI2_CS_Pin
#define SPI_HOST_SDO	GPIO_PIN_15
#define SPI_HOST_SDI	GPIO_PIN_14
#define SPI_SCK			GPIO_PIN_12

/* Common pin mapping on SDP-K1 */
#define UART_TX			USB_UART_TX_Pin
#define	UART_RX			USB_UART_RX_Pin

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

extern struct stm32_uart_init_param stm32_uart_extra_init_params;
extern struct stm32_spi_init_param stm32_spi_extra_init_params;

#endif /* APP_CONFIG_STM32_H_ */
