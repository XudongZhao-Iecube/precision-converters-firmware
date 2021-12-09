/***************************************************************************//**
 * @file    ad3552r_user_config.c
 * @brief   User configurations for AD3552R No-OS driver
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

#include "ad3552r_user_config.h"
#include "app_config.h"
#include "gpio.h"

/******************************************************************************/
/********************* Macros and Constants Definition ************************/
/******************************************************************************/

/******************************************************************************/
/******************** Variables and User Defined Data Types *******************/
/******************************************************************************/

/* LDAC GPIO init parameters */
struct gpio_init_param ldac_gpio_init_param = {
	.number = LDAC_GPIO,
	.platform_ops = NULL,
	.extra = NULL
};

/* AD3552R No-OS driver init parameters */
struct ad3552r_init_param ad3552r_init_params = {
	.spi_param = {
		.max_speed_hz = 10000000,
		.mode = SPI_MODE_0,
		.chip_select = SPI_CSB,
		.extra = &spi_extra_init_params
	},
	.ldac_param = &ldac_gpio_init_param
};
