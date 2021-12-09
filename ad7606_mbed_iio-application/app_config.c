/***************************************************************************//**
 *   @file    app_config.c
 *   @brief   Application configurations module (platform-agnostic)
 *   @details This module performs the system configurations
********************************************************************************
 * Copyright (c) 2020-2021 Analog Devices, Inc.
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
#include "error.h"
#include "uart.h"
#include "gpio.h"
#include "irq.h"
#include "pwm.h"

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

/* External interrupt callback descriptor */
static struct callback_desc ext_int_callback_desc = {
	data_capture_callback,
	NULL,
	NULL
};

/* PWM init parameters */
static struct pwm_init_param pwm_init_params = {
	.id = PWM_TRIGGER,								// GPIO used for PWM
	.period_ns = CONV_TRIGGER_PERIOD_NSEC,			// PWM period in nsec
	.duty_cycle_ns = CONV_TRIGGER_DUTY_CYCLE_NSEC	// PWM duty cycle in nsec
};

/* LED GPO descriptor */
gpio_desc *led_gpio_desc;

/* External interrupt descriptor */
struct irq_ctrl_desc *ext_int_desc;

/* PWM descriptor */
struct pwm_desc *pwm_desc;

/******************************************************************************/
/************************** Functions Declarations ****************************/
/******************************************************************************/

/******************************************************************************/
/************************** Functions Definitions *****************************/
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
		/* Init interrupt controller for external interrupt */
		if (irq_ctrl_init(&ext_int_desc, &ext_int_init_params) != SUCCESS) {
			break;
		}

		/* Register a callback function for external interrupt */
		if (irq_register_callback(ext_int_desc,
					  EXTERNAL_INT_ID1,
					  &ext_int_callback_desc) != SUCCESS) {
			break;
		}

		return SUCCESS;
	} while (0);

	return FAILURE;
}


/**
 * @brief 	Initialize the PWM contoller
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t init_pwm(void)
{
	do {
		/* Initialize the PWM interface to generate PWM signal
		 * on conversion trigger event pin */
		if (pwm_init(&pwm_desc, &pwm_init_params) != SUCCESS) {
			break;
		}

		if (pwm_enable(pwm_desc) != SUCCESS) {
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

	if (init_pwm() != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}
