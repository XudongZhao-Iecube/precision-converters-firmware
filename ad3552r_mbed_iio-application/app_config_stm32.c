/***************************************************************************//**
 *   @file    app_config_stm32.c
 *   @brief   Application configurations module for STM32 platform
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>
#include "app_config.h"
#include "app_config_stm32.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/******************************************************************************/
/******************** Variables and User Defined Data Types *******************/
/******************************************************************************/

/* UART STM32 platform specific init parameters */
struct stm32_uart_init_param stm32_uart_extra_init_params = {
#if defined(USE_VIRTUAL_COM_PORT)
	.virtual_com_enable = true,
	.vendor_id = VIRTUAL_COM_PORT_VID,
	.product_id = VIRTUAL_COM_PORT_PID,
	.serial_number = VIRTUAL_COM_SERIAL_NUM
#else
	.mode = UART_MODE_TX_RX,
	.hw_flow_ctl = UART_HWCONTROL_NONE,
	.over_sampling = 0
#endif
};

/* SPI STM32 platform specific parameters */
stm32_spi_init_param stm32_spi_extra_init_params = {
	.base = SPI2,
	.chip_select_port = GPIOA,
	.get_input_clock = HAL_RCC_GetSysClockFreq
};

/******************************************************************************/
/************************** Functions Declarations ****************************/
/******************************************************************************/

/******************************************************************************/
/************************** Functions Definitions *****************************/
/******************************************************************************/
