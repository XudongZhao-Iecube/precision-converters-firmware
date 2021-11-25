/***************************************************************************//**
 *   @file    app_config.c
 *   @brief   Application configurations module
 *   @details This module contains the configurations needed for IIO application
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
#include "adc_data_capture.h"
#include "ad4170_iio.h"
#include "error.h"
#include "uart.h"
#include "irq.h"
#include "gpio.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* UART init parameters */
struct uart_init_param uart_init_params = {
	.device_id = NULL,
	.baud_rate = IIO_UART_BAUD_RATE,
	.extra = &uart_extra_init_params
};

/* LED GPO init parameters */
static struct gpio_init_param led_gpio_init_params = {
	.number = LED_GPO,
	.extra = NULL
};

/* External interrupt init parameters */
static struct irq_init_param ext_int_init_params = {
	.irq_ctrl_id = EXTERNAL_INT_ID1,
	.extra = &ext_int_extra_init_params
};

/* Ticker interrupt init parameters */
static struct irq_init_param ticker_int_init_params = {
	.irq_ctrl_id = TICKER_INT_ID,
	.extra = &ticker_int_extra_init_params
};

/* External interrupt callback descriptor */
static struct callback_desc ext_int_callback_desc = {
	data_capture_callback,
	NULL,
	NULL
};

/* Ticker interrupt callback descriptor */
static struct callback_desc ticker_int_callback_desc = {
	ticker_callback,
	NULL,
	NULL
};

/* LED GPO descriptor */
gpio_desc *led_gpio_desc = NULL;

/* External interrupt descriptor */
struct irq_ctrl_desc *external_int_desc;

/* Ticker interrupt descriptor */
struct irq_ctrl_desc *ticker_int_desc;

/******************************************************************************/
/************************ Functions Prototypes ********************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief 	Initialize the GPIOs
 * @return	SUCCESS in case of success, FAILURE otherwise
 * @details	This function initialize the GPIOs used by application
 */
static int32_t init_gpio(void)
{
	do {
		/* Initialize the LED GPO */
		if (gpio_get_optional(&led_gpio_desc, &led_gpio_init_params) != SUCCESS) {
			break;
		}

		if (gpio_direction_output(led_gpio_desc, GPIO_HIGH) != SUCCESS) {
			break;
		}

		return SUCCESS;
	} while (0);

	return FAILURE;
}


/**
 * @brief 	Initialize the IRQ contoller
 * @return	SUCCESS in case of success, FAILURE otherwise
 * @details	This function initialize the interrupts for system peripherals
 */
static int32_t init_interrupt(void)
{
	do {
		/* Init interrupt controller for external interrupt (for monitoring
		 * conversion event on BUSY pin) */
		if (irq_ctrl_init(&external_int_desc, &ext_int_init_params) != SUCCESS) {
			break;
		}

		/* Register a callback function for external interrupt */
		if (irq_register_callback(external_int_desc,
					  EXTERNAL_INT_ID1,
					  &ext_int_callback_desc) != SUCCESS) {
			break;
		}

		/* Init interrupt controller for Ticker interrupt */
		if (irq_ctrl_init(&ticker_int_desc, &ticker_int_init_params) != SUCCESS) {
			break;
		}

		/* Register a callback function for Ticker interrupt */
		if (irq_register_callback(ticker_int_desc,
					  TICKER_INT_ID,
					  &ticker_int_callback_desc) != SUCCESS) {
			break;
		}

		return SUCCESS;
	} while (0);

	return FAILURE;
}


/**
 * @brief 	Initialize the system peripherals
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t init_system(void)
{
	if (init_gpio() != SUCCESS) {
		return FAILURE;
	}

	if (init_interrupt() != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}
