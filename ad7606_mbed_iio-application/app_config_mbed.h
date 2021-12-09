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
#include "irq_extra.h"
#include "spi_extra.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Pin mapping of SDP-K1 w.r.t Arduino connector */
#define SPI_CSB			D10
#define SPI_HOST_SDO	D11
#define SPI_HOST_SDI	D12
#define SPI_SCK			D13

/* Common pin mapping on SDP-K1 */
#define UART_TX			USBTX
#define	UART_RX			USBRX
#define LED_GPO			LED3

/* Pin mapping w.r.t. target */
#define	OSR0_PIN		D1
#define	OSR1_PIN		D2
#define	OSR2_PIN		D4
#define RESET_PIN		D5
#define CONVST_PIN		D6
#define BUSY_PIN		D7
#define RANGE_PIN		D8
#define STDBY_PIN		D9

/* Pins used to trigger and/or read a new (periodic) conversion event */
#define PWM_TRIGGER		D3
#define INT_EVENT		D3

/* Define the max possible sampling (or output data) rate for a given platform.
 * This is also used to find the time period to trigger a periodic conversion event.
 * Note: Max possible ODR is 32KSPS per channel for continuous data capture on
 * IIO client. This is derived by testing the firmware on SDP-K1 controller board
 * @22Mhz SPI clock. The max possible ODR can vary from board to board and
 * data continuity is not guaranteed above this ODR on IIO oscilloscope */
#define SAMPLING_RATE					(32000)
#define CONV_TRIGGER_PERIOD_NSEC		(((float)(1.0 / SAMPLING_RATE) * 1000000) * 1000)
#define CONV_TRIGGER_DUTY_CYCLE_NSEC	(CONV_TRIGGER_PERIOD_NSEC / 2)

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

extern mbed_irq_init_param mbed_ext_int_extra_init_params;
extern mbed_uart_init_param mbed_uart_extra_init_params;
extern mbed_spi_init_param mbed_spi_extra_init_params;

#endif /* APP_CONFIG_MBED_H_ */
