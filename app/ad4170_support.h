/***************************************************************************//*
 * @file    ad4170_support.h
 * @brief   AD4170 No-OS driver support header file
******************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc. All Rights Reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
******************************************************************************/
#ifndef AD4170_SUPPORT_H_
#define AD4170_SUPPORT_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include "ad4170.h"

/******************************************************************************/
/********************* Macros and Constants Definitions ***********************/
/******************************************************************************/

#define AD4170_PGA_GAIN(x)	(1 << (x))

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

int32_t perform_sign_conversion(uint32_t adc_raw_data, bool bipolar);
float convert_adc_sample_into_voltage(uint32_t adc_raw, uint8_t chn);
float convert_adc_raw_into_rtd_resistance(uint32_t adc_raw, float rtd_res,
		uint8_t chn);
int32_t ad4170_disable_conversion(void);
int32_t ad4170_enable_input_chn(uint32_t input_chn);
int32_t ad4170_disable_input_chn(uint32_t input_chn);
int32_t ad4170_apply_excitation(void);
int32_t ad4170_remove_excitation(void);

#endif	/* end of AD4170_SUPPORT_H_ */
