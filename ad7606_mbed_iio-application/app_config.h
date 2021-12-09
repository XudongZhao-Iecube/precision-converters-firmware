/*************************************************************************//**
 *   @file   app_config.h
 *   @brief  Header file for application configurations (platform-agnostic)
******************************************************************************
* Copyright (c) 2020-2021 Analog Devices, Inc.
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*****************************************************************************/

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

// **** Note for User: ACTIVE_DEVICE selection ****//
/* Define the device type here from the list of below device type defines
 * (one at a time. Defining more than one device can result into compile error).
 * e.g. #define DEV_AD7606B -> This will make AD7606B as an ACTIVE_DEVICE.
 * The ACTIVE_DEVICE is default set to AD7606B if device type is not defined.
 * */

//#define DEV_AD7606B

#if defined(DEV_AD7605_4)
#define ACTIVE_DEVICE		ID_AD7605_4
#define ACTIVE_DEVICE_NAME	"ad7605-4"
#elif defined(DEV_AD7606_4)
#define ACTIVE_DEVICE		ID_AD7606_4
#define ACTIVE_DEVICE_NAME	"ad7606-4"
#elif defined(DEV_AD7606_6)
#define ACTIVE_DEVICE		ID_AD7606_6
#define ACTIVE_DEVICE_NAME	"ad7606-6"
#elif defined(DEV_AD7606_8)
#define ACTIVE_DEVICE		ID_AD7606_8
#define ACTIVE_DEVICE_NAME	"ad7606-8"
#elif defined(DEV_AD7606B)
#define ACTIVE_DEVICE		ID_AD7606B
#define ACTIVE_DEVICE_NAME	"ad7606b"
#elif defined(DEV_AD7606C_16)
#define ACTIVE_DEVICE		ID_AD7606C_16
#define ACTIVE_DEVICE_NAME	"ad7606c-16"
#elif defined(DEV_AD7606C_18)
#define ACTIVE_DEVICE		ID_AD7606C_18
#define ACTIVE_DEVICE_NAME	"ad7606c-18"
#elif defined(DEV_AD7608)
#define ACTIVE_DEVICE		ID_AD7608
#define ACTIVE_DEVICE_NAME	"ad7608"
#elif defined(DEV_AD7609)
#define ACTIVE_DEVICE		ID_AD7609
#define ACTIVE_DEVICE_NAME	"ad7609"
#else
#warning No/Unsupported ADxxxxy symbol defined. AD7606B defined
#define DEV_AD7606B
#define ACTIVE_DEVICE		ID_AD7606B
#define ACTIVE_DEVICE_NAME	"ad7606b"
#endif

#if defined(DEV_AD7605_4)
#define	AD7606X_ADC_CHANNELS	4
#define AD7606X_ADC_RESOLUTION	16
#elif defined(DEV_AD7606_4)
#define	AD7606X_ADC_CHANNELS	4
#define AD7606X_ADC_RESOLUTION	16
#elif defined(DEV_AD7606_6)
#define	AD7606X_ADC_CHANNELS	6
#define AD7606X_ADC_RESOLUTION	16
#elif defined(DEV_AD7606_8)
#define	AD7606X_ADC_CHANNELS	8
#define AD7606X_ADC_RESOLUTION	16
#elif defined(DEV_AD7606B)
#define	AD7606X_ADC_CHANNELS	8
#define AD7606X_ADC_RESOLUTION	16
#elif defined(DEV_AD7606C_16)
#define	AD7606X_ADC_CHANNELS	8
#define AD7606X_ADC_RESOLUTION	16
#elif defined(DEV_AD7606C_18)
#define	AD7606X_ADC_CHANNELS	8
#define AD7606X_ADC_RESOLUTION	18
#elif defined(DEV_AD7608)
#define	AD7606X_ADC_CHANNELS	8
#define AD7606X_ADC_RESOLUTION	18
#elif defined(DEV_AD7609)
#define	AD7606X_ADC_CHANNELS	8
#define AD7606X_ADC_RESOLUTION	18
#else
/* Default config for AD7606B */
#define	AD7606X_ADC_CHANNELS	8
#define AD7606X_ADC_RESOLUTION	16
#endif

/* Macros for stringification */
#define XSTR(s)		STR(s)
#define STR(s)		#s

/****** Macros used to form a VCOM serial number ******/
#if !defined(DEVICE_NAME)
#define DEVICE_NAME		"DEV_AD760B"
#endif

#if !defined(PLATFORM_NAME)
#define PLATFORM_NAME	"SDP_K1"
#endif
/******/

#if (ACTIVE_PLATFORM == MBED_PLATFORM)
#include "app_config_mbed.h"

/* Used to form a VCOM serial number */
#define	FIRMWARE_NAME	"ad7606_mbed_iio_application"

/* Redefine the init params structure mapping w.r.t. platform */
#define ext_int_extra_init_params mbed_ext_int_extra_init_params
#define uart_extra_init_params mbed_uart_extra_init_params
#define spi_extra_init_params mbed_spi_extra_init_params
#else
#error "No/Invalid active platform selected"
#endif

/* ADC max count (full scale value) for unipolar inputs */
#define ADC_MAX_COUNT_UNIPOLAR	(uint32_t)((1 << AD7606X_ADC_RESOLUTION) - 1)

/* ADC max count (full scale value) for bipolar inputs */
#define ADC_MAX_COUNT_BIPOLAR	(uint32_t)(1 << (AD7606X_ADC_RESOLUTION-1))

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
extern struct gpio_desc *led_gpio_desc;

int32_t init_system(void);

#endif /* _APP_CONFIG_H_ */
