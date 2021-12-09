/***************************************************************************//**
 *   @file    app_config.c
 *   @brief   Application configurations module (platform-agnostic)
 *   @details This module performs the system configurations
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
#include "uart.h"
#include "error.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/******************************************************************************/
/******************** Variables and User Defined Data Types *******************/
/******************************************************************************/

/* UART init parameters */
struct uart_init_param uart_init_params = {
	.device_id = NULL,
	.baud_rate = IIO_UART_BAUD_RATE,
	.extra = &uart_extra_init_params
};

/* UART descriptor */
struct uart_desc *uart_desc;

/******************************************************************************/
/************************** Functions Declarations ****************************/
/******************************************************************************/

/******************************************************************************/
/************************** Functions Definitions *****************************/
/******************************************************************************/

/**
 * @brief 	Initialize the UART peripheral
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t init_uart(void)
{
	return uart_init(&uart_desc, &uart_init_params);
}


/**
 * @brief 	Initialize the system peripherals
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t init_system(void)
{
	if (init_uart() != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}
