/*************************************************************************//**
 *   @file   ad7606_support.c
 *   @brief  AD7606 device No-OS driver supports
******************************************************************************
* Copyright (c) 2020 Analog Devices, Inc.
*
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*****************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

#include "app_config.h"
#include "ad7606_support.h"
#include "ad7606_data_capture.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/*!
 * @brief	Function to convert adc raw data into equivalent voltage
 * @param	adc_raw[in] - ADC raw data
 * @param	scale[in] - ADC raw to voltage conversion scale
 * @return	equivalent voltage
 */
float convert_adc_raw_to_voltage(int32_t adc_raw, float scale)
{
	float voltage;

	/* Convert adc data into equivalent voltage.
	 * scale = (chn_range / MAX_ADC_CNT * 1000) as defined in iio_ad7606.c
	 * */
	voltage = (adc_raw  * (scale / 1000));

	return voltage;
}


/*!
 * @brief	Function to get the polarity of analog input
 * @param	chn_range_bits[in] - Bits from the channel range register
 * @return	UNIPOLAR or BIPOLAR
 */
polarity_e ad7606_get_input_polarity(uint8_t chn_range_bits)
{
	polarity_e polarity;

	if (chn_range_bits >= AD7606C_UNIPOLAR_RANGE_MIN
	    && chn_range_bits <= AD7606C_UNIPOLAR_RANGE_MAX) {
		polarity = UNIPOLAR;
	} else {
		polarity = BIPOLAR;
	}

	return polarity;
}


/*!
 * @brief	Read the num_of_bytes from previous conversion
 * @param	dev[in]- Device instance
 * @param	num_of_bytes[in] - Number of bytes to read from previous conversion
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad7606_read_conversion_data(struct ad7606_dev *dev,
				    uint8_t num_of_bytes)
{
	memset(dev->data, 0, sizeof(dev->data));
	return spi_write_and_read(dev->spi_desc, dev->data, num_of_bytes);
}
