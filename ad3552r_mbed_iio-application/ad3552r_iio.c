/***************************************************************************//**
 *   @file    ad3552r_iio.c
 *   @brief   Implementation of AD3552R IIO application interfaces
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

#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include "ad3552r_iio.h"
#include "ad3552r_user_config.h"
#include "iio_ad3552r.h"
#include "app_config.h"
#include "error.h"

/******************************************************************************/
/********************* Macros and Constants Definition ************************/
/******************************************************************************/

/******************************************************************************/
/******************** Variables and User Defined Data Types *******************/
/******************************************************************************/

/* IIO interface descriptor */
static struct iio_desc *p_ad3552r_iio_desc;

/* IIO Device name */
static const char dev_name[] = IIO_DEVICE_NAME;

/* Pointer to the struct representing the IIO device */
struct ad3552r_desc *p_ad3552r_dev_inst = NULL;

/******************************************************************************/
/************************** Functions Declarations ****************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Release resources allocated for IIO device
 * @param desc[in] - IIO device descriptor
 * @return SUCCESS in case of success, FAILURE otherwise
 */
static int32_t iio_ad3552r_remove(struct iio_desc *desc)
{
	int32_t status;

	if (!desc) {
		return FAILURE;
	}

	status = iio_unregister(desc, (char *)dev_name);
	if (status != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}


/**
 * @brief	Initialize the IIO interface for AD3552R IIO device
 * @return	none
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad3552r_iio_initialize(void)
{
	int32_t init_status;

	/* IIO device descriptor */
	struct iio_device *p_iio_ad3552r_dev = &ad3552r_iio_descriptor;

	/* IIO interface init parameters */
	struct iio_init_param iio_init_params;

	/* Init the system peripherals */
	init_status = init_system();
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Initialize AD3552R device and peripheral interface */
	init_status = ad3552r_init(&p_ad3552r_dev_inst, &ad3552r_init_params);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Initialize the IIO interface */
	iio_init_params.phy_type = USE_UART;
	iio_init_params.uart_desc = uart_desc;
	init_status = iio_init(&p_ad3552r_iio_desc, &iio_init_params);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Register AD3552R IIO interface */
	init_status = iio_register(p_ad3552r_iio_desc, p_iio_ad3552r_dev,
				   (char *)dev_name, p_ad3552r_dev_inst, NULL, NULL);
	if (init_status != SUCCESS) {
		return iio_ad3552r_remove(p_ad3552r_iio_desc);
	}

	return init_status;
}


/**
 * @brief 	Run the AD3552R IIO event handler
 * @return	none
 * @details	This function monitors the new IIO client event
 */
void ad3552r_iio_event_handler(void)
{
	while(1) {
		(void)iio_step(p_ad3552r_iio_desc);
	}
}
