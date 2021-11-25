/*************************************************************************//**
 *   @file   app_config.h
 *   @brief  Configuration file for AD4170 device application
******************************************************************************
* Copyright (c) 2021 Analog Devices, Inc.
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

/* Demo mode configuration options */
#define		USER_DEFAULT_CONFIG		0
#define		RTD_2WIRE_CONFIG		1
#define		RTD_3WIRE_CONFIG		2
#define		RTD_4WIRE_CONFIG		3
#define		THERMISTOR_CONFIG		4
#define		THERMOCOUPLE_CONFIG		5
#define		ACCELEROMETER_CONFIG	6
#define		LOADCELL_CONFIG			7

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Name of active device */
#define ACTIVE_DEVICE_NAME	"ad4170"

/* Select the demo mode configuration (one at a time) */
#if !defined(ACTIVE_DEMO_MODE_CONFIG)
#define ACTIVE_DEMO_MODE_CONFIG		USER_DEFAULT_CONFIG
#endif

#if (ACTIVE_PLATFORM == MBED_PLATFORM)
#include "app_config_mbed.h"

/* Used to form a VCOM serial number */
#define	FIRMWARE_NAME	"ad4170_mbed_iio_application"

/* Redefine the init params structure mapping w.r.t. platform */
#define ticker_int_extra_init_params mbed_ticker_int_extra_init_params
#define ext_int_extra_init_params mbed_ext_int_extra_init_params
#define uart_extra_init_params mbed_uart_extra_init_params
#define spi_extra_init_params mbed_spi_extra_init_params
#else
#error "No/Invalid active platform selected"
#endif

/* Include user config files and params according to active/selected demo mode config */
#if (ACTIVE_DEMO_MODE_CONFIG == USER_DEFAULT_CONFIG)
#include "ad4170_user_config.h"
#define ad4170_init_params	ad4170_user_config_params
#elif ((ACTIVE_DEMO_MODE_CONFIG == RTD_2WIRE_CONFIG) || \
(ACTIVE_DEMO_MODE_CONFIG == RTD_3WIRE_CONFIG) || (ACTIVE_DEMO_MODE_CONFIG == RTD_4WIRE_CONFIG))
#include "ad4170_rtd_config.h"
#define ad4170_init_params	ad4170_rtd_config_params
#elif (ACTIVE_DEMO_MODE_CONFIG == THERMISTOR_CONFIG)
#include "ad4170_thermistor_config.h"
#define ad4170_init_params	ad4170_thermistor_config_params
#elif (ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG)
#include "ad4170_thermocouple_config.h"
#define ad4170_init_params	ad4170_thermocouple_config_params
#elif (ACTIVE_DEMO_MODE_CONFIG == ACCELEROMETER_CONFIG)
#include "ad4170_accelerometer_config.h"
#define ad4170_init_params	ad4170_accelerometer_config_params
#elif (ACTIVE_DEMO_MODE_CONFIG == LOADCELL_CONFIG)
#include "ad4170_loadcell_config.h"
#define ad4170_init_params	ad4170_loadcell_config_params
#else
#include "ad4170_user_config.h"
#define ad4170_init_params	ad4170_user_config_params
#warning "No/Invalid active demo config selected, user config used as default"
#endif

/* ADC resolution for active device */
#define ADC_RESOLUTION		24

/* ADC max count (full scale value) for unipolar inputs */
#define ADC_MAX_COUNT_UNIPOLAR	(uint32_t)((1 << ADC_RESOLUTION) - 1)

/* ADC max count (full scale value) for bipolar inputs */
#define ADC_MAX_COUNT_BIPOLAR	(uint32_t)(1 << (ADC_RESOLUTION-1))

/* Default ADC Vref voltage */
#define ADC_REF_VOLTAGE		2.5

/* Macros for stringification */
#define XSTR(s)		STR(s)
#define STR(s)		#s

/****** Macros used to form a VCOM serial number ******/
#if !defined(DEVICE_NAME)
#define DEVICE_NAME		"DEV_AD4170"
#endif

#if !defined(PLATFORM_NAME)
#define PLATFORM_NAME	"SDP_K1"
#endif

#if !defined(EVB_INTERFACE)
#define EVB_INTERFACE	"ARDUINO"
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
/* Serial number string is formed as: application name + device (target) name + platform (host) name + evb interface name */
#define VIRTUAL_COM_SERIAL_NUM	(FIRMWARE_NAME "_" DEVICE_NAME "_" PLATFORM_NAME "_" EVB_INTERFACE)
#endif

/* Baud rate for IIO application UART interface */
#define IIO_UART_BAUD_RATE	(230400)

/* Calculations for sampling frequency (used to define timeout in IIO client):
 * Note: Below calculations are based on default user configurations set in the
 *		  ad4170_xyz_config.h files. These configurations are used for data capturing.
 * Clock: Internal 16Mhz oscillotor
 * Filter Type: Selected in user config files
 * Filter FS: Selected in user config files
 * Filter ODR Average (as defined in datasheet): Selected in user config files
 **/
/* AD4170 default internal clock frequency (Fclock = 16Mhz)*/
#define AD4170_INTERNAL_CLOCK			(16000000U)

/* Default sampling frequency for AD4170 (in SPS) */
#define AD4170_DEFLT_SAMPLING_FREQEUNCY	(AD4170_INTERNAL_CLOCK / FS_TO_ODR_CONV_SCALER)

/******************************************************************************/
/************************ Public Declarations *********************************/
/******************************************************************************/

extern struct uart_init_param uart_init_params;
extern struct gpio_desc *led_gpio_desc;

int32_t init_system(void);

#endif //_APP_CONFIG_H_
