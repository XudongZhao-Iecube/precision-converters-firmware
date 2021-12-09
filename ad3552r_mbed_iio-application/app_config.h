/***************************************************************************//*
 * @file    app_config.h
 * @brief   Header file for application configurations (platform-agnostic)
******************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
******************************************************************************/

#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

/* List of supported platforms*/
#define	MBED_PLATFORM		1

/* Select the active platform */
#define ACTIVE_PLATFORM		MBED_PLATFORM

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

#define IIO_DEVICE_NAME	"ad3552r"

#if (ACTIVE_PLATFORM == MBED_PLATFORM)
#include "app_config_mbed.h"

/* Used to form a VCOM serial number */
#define	FIRMWARE_NAME	"ad3552r_mbed_iio_application"

/* Redefine the init params structure mapping w.r.t. platform */
#define uart_extra_init_params mbed_uart_extra_init_params
#define spi_extra_init_params mbed_spi_extra_init_params
#else
#error "No/Invalid active platform selected"
#endif

/****** Macros used to form a VCOM serial number ******/
#if !defined(DEVICE_NAME)
#define DEVICE_NAME		"DEV_AD3552R"
#endif

#if !defined(PLATFORM_NAME)
#define PLATFORM_NAME	"SDP_K1"
#endif
/******/

/* Enable the VirtualCOM port connection/interface. By default serial comminunication
 * is physical UART */
//#define USE_VIRTUAL_COM_PORT

#if defined(USE_VIRTUAL_COM_PORT)
/* Below USB configurations (VID and PID) are owned and assigned by ADI.
 * If intended to distribute software further, use the VID and PID owned by your
 * organization */
#define VIRTUAL_COM_PORT_VID	0x0456
#define VIRTUAL_COM_PORT_PID	0xb66c
/* Serial number string is formed as: application name + device (target) name + platform (host) name */
#define VIRTUAL_COM_SERIAL_NUM	(FIRMWARE_NAME "_" DEVICE_NAME "_" PLATFORM_NAME)
#endif

/* Baud rate for IIO application UART interface */
#define IIO_UART_BAUD_RATE	(230400)

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

extern struct uart_init_param uart_init_params;
extern struct uart_desc *uart_desc;

int32_t init_system(void);

#endif /* _APP_CONFIG_H_ */
