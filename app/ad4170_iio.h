/***************************************************************************//**
*   @file   ad4170_iio.h
*   @brief  Header file of ad4170_iio
********************************************************************************
* Copyright (c) 2021 Analog Devices, Inc.
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*******************************************************************************/
#ifndef _AD4170_IIO_H_
#define _AD4170_IIO_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdio.h>
#include <stdbool.h>

#include "iio.h"
#include "iio_types.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/* ADC calibration configs */
typedef struct {
	uint32_t gain_before_calib;
	uint32_t gain_after_calib;
	uint32_t offset_after_calib;
	uint32_t offset_before_calib;
} adc_calibration_configs;

/* Calibration status */
enum calib_status {
	CALIB_NOT_DONE,
	CALIB_DONE,
	CALIB_ERROR
};

/******************************************************************************/
/************************ Public Declarations *********************************/
/******************************************************************************/

/* AD4170 global device instance for accessing device specific APIs */
extern struct ad4170_dev *p_ad4170_dev_inst;

/* Calibration configs */
extern enum calib_status adc_calibration_status;
extern adc_calibration_configs adc_calibration_config[];

/* Init the IIO interface */
int32_t ad4170_iio_initialize(void);

/* Run the IIO event handler */
void ad4170_iio_event_handler(void);

void ticker_callback(void *ctx, uint32_t event, void *extra);

#endif /* _AD4170_IIO_H_ */
