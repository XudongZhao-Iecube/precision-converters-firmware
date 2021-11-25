/*************************************************************************//**
 *   @file   ad4170_user_config.h
 *   @brief  Header for AD4170 default user configurations file
******************************************************************************
* Copyright (c) 2021 Analog Devices, Inc.
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*****************************************************************************/

#ifndef _AD4170_USER_CONFIG_H_
#define _AD4170_USER_CONFIG_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include "ad4170.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Select the polarity for default config */
#define BIPOLAR		// comment to select unipolar mode

/* Select FS (or ODR) for default config */
#define AD4170_FS_CONFIG				16	// ODR = ~32KSPS

/* Scaler factor used in FS value to ODR conversion (for SINC5+Avg filter) */
#define FS_TO_ODR_CONV_SCALER			(32U * AD4170_FS_CONFIG)

/* Select the ADC continuous conversion mode for default config
 * Note: When supplying FIR coefficients, set this macro to 'AD4170_MODE_CONT_FIR' */
#define AD4170_CONT_CONV_MODE_CONFIG	AD4170_MODE_CONT

/* Select the PGA for default config */
#define AD4170_PGA_CONFIG				AD4170_PGA_GAIN_1

#if defined (BIPOLAR)
#define AD4170_BIPOLAR_MODE				(bool)(true)
#else
#define AD4170_BIPOLAR_MODE				(bool)(false)
#endif

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

extern struct ad4170_init_param ad4170_user_config_params;

#endif /* end of _AD4170_USER_CONFIG_H_ */
