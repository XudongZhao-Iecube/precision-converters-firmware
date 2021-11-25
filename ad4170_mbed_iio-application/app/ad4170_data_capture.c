/***************************************************************************//**
 *   @file    ad4170_data_capture.c
 *   @brief   Data capture interface for AD4170 IIO application
 *   @details This module handles the AD4170 data capturing for IIO client
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
#include <stdlib.h>

#include "app_config.h"
#include "ad4170_iio.h"
#include "ad4170_support.h"
#include "ad4170_data_capture.h"
#include "adc_data_capture.h"
#include "error.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/* Previously active channels */
static uint32_t ad4170_prev_active_chns = 0;

/* Polarity of channels */
static bool bipolar[AD4170_NUM_CHANNELS];

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Perform single sample read pre-operations */
static int32_t ad4170_single_sample_read_start_ops(uint8_t input_chn);

/* Perform conversion on input channel and read single conversion sample */
static int32_t ad4170_perform_conv_and_read_sample(uint32_t *adc_data,
		uint8_t chn);

/* Perform single sample read post-operations */
static int32_t ad4170_single_sample_read_stop_ops(uint8_t input_chn);

/* Perform continuous sample read pre-operations */
static int32_t ad4170_continuous_sample_read_start_ops(uint32_t chn_mask);

/* Read ADC sampled/conversion data */
static int32_t ad4170_read_converted_sample(uint32_t *adc_data,
		uint8_t input_chn);

/* Perform continuous sample read post-operations */
static int32_t ad4170_continuous_sample_read_stop_ops(void);

/* Define the variable for data_capture_ops structure */
struct data_capture_ops data_capture_ops = {
	/* Point AD4170 data capture functions to generic ADC data capture functions */
	.single_sample_read_start_ops = ad4170_single_sample_read_start_ops,
	.perform_conv_and_read_sample = ad4170_perform_conv_and_read_sample,
	.single_sample_read_stop_ops = ad4170_single_sample_read_stop_ops,
	.continuous_sample_read_start_ops = ad4170_continuous_sample_read_start_ops,
	.read_converted_sample = ad4170_read_converted_sample,
	.continuous_sample_read_stop_ops = ad4170_continuous_sample_read_stop_ops,
	.trigger_next_conversion = NULL
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/*!
 * @brief	Perform the operations required before single sample ADC read
 * @param	input_chn[out] - ADC input channel
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t ad4170_single_sample_read_start_ops(uint8_t input_chn)
{
	/* Save the previous active channels */
	ad4170_prev_active_chns = p_ad4170_dev_inst->config.channel_en;

	/* Disable all active channels */
	if (ad4170_set_channel_en(p_ad4170_dev_inst, 0) != SUCCESS) {
		return FAILURE;
	}

	/* Enable input channel */
	if (ad4170_enable_input_chn(input_chn) != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}


/*!
 * @brief	Perform conversion and read conversion sample
 * @param	adc_data[out] - Pointer to adc data read variable
 * @param	chn[in] - Channel for which data is to read
 * @return	SUCCESS in case of success, FAILURE otherwise
 * @details	This function performs the sampling on previously enabled channel
 *			and then read conversion result
 */
static int32_t ad4170_perform_conv_and_read_sample(uint32_t *adc_data,
		uint8_t chn)
{
	struct ad4170_adc_ctrl adc_ctrl;

	/* Apply excitation on the input (demo config specific) */
	if (ad4170_apply_excitation() != SUCCESS) {
		return FAILURE;
	}

	/* Enable single conversion mode */
	adc_ctrl = p_ad4170_dev_inst->config.adc_ctrl;
	adc_ctrl.mode = AD4170_MODE_SINGLE;
	if (ad4170_set_adc_ctrl(p_ad4170_dev_inst, adc_ctrl) != SUCCESS) {
		return FAILURE;
	}

	/* This function monitors RDY line to read ADC result */
	if (ad4170_read24(p_ad4170_dev_inst, adc_data, 1) != SUCCESS) {
		return FAILURE;
	}

	/* Remove excitation on the input (demo config specific) */
	if (ad4170_remove_excitation() != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}


/*!
 * @brief	Perform the operations required after single sample ADC read
 * @param	input_chn[out] - ADC input channel
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t ad4170_single_sample_read_stop_ops(uint8_t input_chn)
{
	/* Restore (re-enable) the previous active channels */
	if (ad4170_set_channel_en(p_ad4170_dev_inst,
				  ad4170_prev_active_chns) != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}


/*!
 * @brief	Perform the operations required before starting continuous sample read
 * @param	chn_mask[in] - active channels list received from IIO client
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t ad4170_continuous_sample_read_start_ops(uint32_t chn_mask)
{
	struct ad4170_adc_ctrl adc_ctrl = p_ad4170_dev_inst->config.adc_ctrl;
	uint8_t setup;
	uint32_t mask = 0x1;

	/* Disable ADC conversion */
	if (ad4170_disable_conversion() != SUCCESS) {
		return FAILURE;
	}

	/* Store the previous active channels */
	ad4170_prev_active_chns = p_ad4170_dev_inst->config.channel_en;

	/* Enable/Disable channels based on channel mask set in the IIO client and
	 * also save polarity of each channel setup */
	for (uint8_t chn = 0; chn < AD4170_NUM_CHANNELS; chn++) {
		if (chn_mask & mask) {
			setup = p_ad4170_dev_inst->config.setup[chn].setup_n;
			bipolar[chn] = p_ad4170_dev_inst->config.setups[setup].afe.bipolar;

			/* Enable the selected channel */
			if (ad4170_enable_input_chn(chn) != SUCCESS) {
				return FAILURE;
			}

			/* Apply calibrated coefficients before new sampling */
			if (adc_calibration_status == CALIB_DONE) {
				if (ad4170_spi_reg_write(p_ad4170_dev_inst, AD4170_REG_ADC_SETUPS_OFFSET(setup),
							 adc_calibration_config[chn].offset_after_calib) != SUCCESS) {
					return FAILURE;
				}

				if (ad4170_spi_reg_write(p_ad4170_dev_inst, AD4170_REG_ADC_SETUPS_GAIN(setup),
							 adc_calibration_config[chn].gain_after_calib) != SUCCESS) {
					return FAILURE;
				}
			}
		} else {
			/* Disable the selected channel */
			if (ad4170_disable_input_chn(chn) != SUCCESS) {
				return FAILURE;
			}
		}

		mask <<= 1;
	}

	/* Apply excitation sources (demo config specific) */
	if (ad4170_apply_excitation() != SUCCESS) {
		return FAILURE;
	}

	/* Select continuous conversion mode */
	adc_ctrl.mode = AD4170_CONT_CONV_MODE_CONFIG;

	/* Enable continuous read mode for faster data read (this mode is
	 * only allowed in continuous conversion) */
	adc_ctrl.cont_read = AD4170_CONT_READ_ON;
	adc_ctrl.cont_read_status_en = false;

	return ad4170_set_adc_ctrl(p_ad4170_dev_inst, adc_ctrl);
}


/*!
 * @brief	Read ADC raw data for recently sampled channel
 * @param	adc_data[out] - Pointer to adc data read variable
 * @param	input_chn[in] - Input channel
 * @return	SUCCESS in case of success, FAILURE otherwise
 * @note	This function is intended to call from the conversion end trigger
 *			event. Therefore, this function should just read raw ADC data
 *			without further monitoring conversion end event
 */
static int32_t ad4170_read_converted_sample(uint32_t *adc_data,
		uint8_t input_chn)
{
	uint8_t buf[AD4170_TRANSF_LEN(AD4170_REG_DATA_24b)];

	if (!adc_data) {
		return FAILURE;
	}

	/* Read data over spi interface (in continuous read mode) */
	memset(buf, 0, sizeof(buf));
	if (spi_write_and_read(p_ad4170_dev_inst->spi_desc,
			       buf,
			       sizeof(buf)) != SUCCESS) {
		return FAILURE;
	}

	/* Extract data:       MSB                       LSB */
	*adc_data = (buf[0] << 16) | (buf[1] << 8) | buf[2];

	return SUCCESS;
}


/*!
 * @brief	Perform the operations required after stopping continuous sample read
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t ad4170_continuous_sample_read_stop_ops(void)
{
	/* Disable ADC conversion */
	if (ad4170_disable_conversion() != SUCCESS) {
		return FAILURE;
	}

	/* Remove excitation sources (demo config specific) */
	if (ad4170_remove_excitation() != SUCCESS) {
		return FAILURE;
	}

	/* Restore (re-enable) the previous active channels */
	if (ad4170_set_channel_en(p_ad4170_dev_inst,
				  ad4170_prev_active_chns) != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}
