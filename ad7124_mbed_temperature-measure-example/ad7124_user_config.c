/*************************************************************************//**
 *   @file   ad7124_user_config.c
 *   @brief  User configuration file for AD7124 device
******************************************************************************
* Copyright (c) 2021 Analog Devices, Inc.
*
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*****************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

#include "app_config.h"
#include "ad7124_user_config.h"
#include "spi_extra.h"
#include "gpio_extra.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/* Define SPI extra init parameters structure */
mbed_spi_init_param spi_init_extra_params = {
	.spi_clk_pin = SPI_SCK,
	.spi_miso_pin = SPI_MISO,
	.spi_mosi_pin = SPI_MOSI
};

// Designated SPI Initialization Structure
spi_init_param	ad7124_spi_init = {
	.max_speed_hz = 10000000, // Max SPI Speed
	.chip_select = SPI_SS,    // Chip Select pin
	.mode = SPI_MODE_3,       // CPOL = 1, CPHA =1
	.extra = &spi_init_extra_params  // SPI extra configurations		
};

/* Define the AD7124 device init structure */
struct ad7124_init_param ad7124_init_params = {
	&ad7124_spi_init,  // spi_init_param type
	ad7124_regs,       // AD7124 Register Map
	10000              // SPI Ready Poll Count
};
