/***************************************************************************//**
 *   @file    ad7606_data_capture.c
 *   @brief   Data capture interface for AD7606 IIO application
 *   @details This module handles the AD7606 data capturing
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

#include <string.h>
#include <stdlib.h>

#include "app_config.h"
#include "ad7606_data_capture.h"
#include "iio_ad7606.h"
#include "ad7606_support.h"
#include "error.h"
#include "adc_data_capture.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

#if (AD7606X_ADC_RESOLUTION == 18)
#define	SAMPLE_SIZE_IN_BYTE		3
#else
#define	SAMPLE_SIZE_IN_BYTE		2
#endif

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/* Polarity of channels */
static polarity_e polarity[AD7606X_ADC_CHANNELS];

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Perform conversion on input channel and read single conversion sample */
static int32_t ad7606_perform_conv_and_read_sample(uint32_t *adc_data,
		uint8_t chn);

/* Perform continuous sample read pre-operations */
static int32_t ad7606_continuous_sample_read_start_ops(uint32_t ch_mask);

/* Read ADC sampled/conversion data */
static int32_t ad7606_read_converted_sample(uint32_t *adc_data,
		uint8_t input_chn);

/* Trigger next ADC conversion */
static int32_t ad7606_trigger_next_conversion(void);

/* Define the variable for data_capture_ops structure */
struct data_capture_ops data_capture_ops = {
	/* Point AD7606 data capture functions to generic ADC data capture functions */
	.single_sample_read_start_ops = NULL,
	.perform_conv_and_read_sample = ad7606_perform_conv_and_read_sample,
	.single_sample_read_stop_ops = NULL,
	.continuous_sample_read_start_ops = ad7606_continuous_sample_read_start_ops,
	.read_converted_sample = ad7606_read_converted_sample,
	.continuous_sample_read_stop_ops = NULL,
	.trigger_next_conversion = ad7606_trigger_next_conversion
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/*!
 * @brief	Remove the offset from data to change output data format to
 *			normal or stright  binary representation (needed for IIO client)
 * @param	adc_raw[out] - Pointer to adc data read variable
 * @param	bipolar[in] - Channel polarity
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t reformat_adc_raw_data(uint32_t adc_raw_data, polarity_e polarity)
{
	int32_t adc_data;

	/* Bipolar ADC Range:  (-FS) <-> 0 <-> (+FS) : 2^(ADC_RES-1) <-> 0 <-> 2^(ADC_RES-1)-1
	   Unipolar ADC Range: 0 <-> (+FS) : 0 <-> 2^ADC_RES
	 **/
	if (polarity == BIPOLAR) {
		/* Data output format is 2's complement for bipolar mode */
		if (adc_raw_data > ADC_MAX_COUNT_BIPOLAR) {
			/* Remove the offset from result to convert into negative reading */
			adc_data = ADC_MAX_COUNT_UNIPOLAR - adc_raw_data;
			adc_data = -adc_data;
		} else {
			adc_data = adc_raw_data;
		}
	} else {
		/* Data output format is straight binary for unipolar mode */
		adc_data = adc_raw_data;
	}

	return adc_data;
}


/*!
 * @brief	Perform conversion and read conversion sample
 * @param	adc_data[out] - Pointer to adc data read variable
 * @param	chn[in] - Channel for which data is to read
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad7606_perform_conv_and_read_sample(uint32_t *adc_data, uint8_t chn)
{
	uint32_t adc_raw[AD7606X_ADC_CHANNELS] = { 0 };
	uint8_t read_val;
	uint8_t chn_range;
	polarity_e polarity;

	/* Get input channel range */
	if (ad7606_spi_reg_read(p_ad7606_dev_inst,
				AD7606_REG_RANGE_CH_ADDR(chn),
				&read_val) != SUCCESS) {
		return FAILURE;
	}

	if (((chn) % 2) != 0) {
		read_val >>= CHANNEL_RANGE_MSK_OFFSET;
		chn_range = read_val;
	} else {
		chn_range = (read_val & AD7606_RANGE_CH_MSK(chn));
	}

	/* Get polarity based on input channel range */
	polarity = ad7606_get_input_polarity(chn_range);

	/* This function monitors BUSY line for EOC and read ADC result post that */
	if (ad7606_read(p_ad7606_dev_inst, adc_raw) != SUCCESS) {
		adc_raw[chn] = 0;
	}

	*adc_data = reformat_adc_raw_data(adc_raw[chn], polarity);

	return SUCCESS;
}


/*!
 * @brief	Perform the operations required before starting continuous sample read
 * @param	chn_mask[in] - active channels list received from IIO client
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad7606_continuous_sample_read_start_ops(uint32_t chn_mask)
{
	uint8_t read_val;
	uint8_t chn_range;

	for (uint8_t chn = 0; chn < AD7606X_ADC_CHANNELS; chn++) {
		/* Store the channels polarity */
		if (ad7606_spi_reg_read(p_ad7606_dev_inst,
					AD7606_REG_RANGE_CH_ADDR(chn),
					&read_val) == SUCCESS) {
			if (((chn) % 2) != 0) {
				read_val >>= CHANNEL_RANGE_MSK_OFFSET;
				chn_range = read_val;
			} else {
				chn_range = (read_val & AD7606_RANGE_CH_MSK(chn));
			}

			polarity[chn] = ad7606_get_input_polarity(chn_range);
		}
	}

	/* Trigger first conversion */
	return ad7606_trigger_next_conversion();
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
int32_t ad7606_read_converted_sample(uint32_t *adc_data,
				     uint8_t input_chn)
{
	uint32_t adc_raw;
	uint8_t bytes_to_read;
	uint8_t buffer_offset;

	if (!adc_data) {
		return FAILURE;
	}

	/* Get number of bytes to read count = chn_cnt * bytes per sample */
	bytes_to_read = AD7606X_ADC_CHANNELS * SAMPLE_SIZE_IN_BYTE;
	buffer_offset = input_chn * SAMPLE_SIZE_IN_BYTE;

	/* Read data over spi interface for all ADC channels */
	memset(p_ad7606_dev_inst->data, 0, sizeof(p_ad7606_dev_inst->data));
	spi_write_and_read(p_ad7606_dev_inst->spi_desc,
			   p_ad7606_dev_inst->data, bytes_to_read);

#if (AD7606X_ADC_RESOLUTION == 18)
	adc_raw =
		(((uint32_t)p_ad7606_dev_inst->data[buffer_offset] << 16) |		// MSB
		 ((uint32_t)p_ad7606_dev_inst->data[buffer_offset + 1] << 8) |
		 ((uint32_t)p_ad7606_dev_inst->data[buffer_offset + 2]));		// LSB
#else
	adc_raw =
		(uint16_t)(((uint16_t)p_ad7606_dev_inst->data[buffer_offset] << 8) | 	// MSB
			   p_ad7606_dev_inst->data[buffer_offset + 1]); 		// LSB
#endif

	*adc_data = reformat_adc_raw_data(adc_raw, polarity[input_chn]);

	return SUCCESS;
}


/*!
 * @brief	Trigger next ADC conversion
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad7606_trigger_next_conversion(void)
{
	if (ad7606_convst(p_ad7606_dev_inst) != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}
