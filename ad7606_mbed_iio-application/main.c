/***************************************************************************//**
 *   @file    main.c
 *   @brief   Main module for AD7606 IIO application
 *   @details This module invokes the AD7606 IIO interfaces
 *            through forever loop.
********************************************************************************
 * Copyright (c) 2020 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include "platform_support.h"
#include "iio_ad7606.h"
#include "error.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief	Main entry point to application
 * @return	none
 */
int main(void)
{
	/* Initialize the AD7606 IIO interface */
	if (ad7606_iio_initialize() == FAILURE) {
		assert(false);
	}

	while (1) {
		/* Monitor the IIO client events */
		ad7606_iio_event_handler();
	}
}
