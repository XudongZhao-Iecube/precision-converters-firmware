/***************************************************************************//*
 * @file    ad4170_support.c
 * @brief   AD4170 No-OS driver support file
******************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc. All Rights Reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "ad4170_support.h"
#include "app_config.h"
#include "ad4170_iio.h"
#include "error.h"

/******************************************************************************/
/********************* Macros and Constants Definitions ***********************/
/******************************************************************************/

/******************************************************************************/
/******************** Variables and User Defined Data Types *******************/
/******************************************************************************/

/******************************************************************************/
/************************** Functions Definitions *****************************/
/******************************************************************************/

/*!
 * @brief	Perform the sign conversion for handling negative voltages in
 *			bipolar mode
 * @param	adc_raw[out] - Pointer to adc data read variable
 * @param	bipolar[in] - Channel polarity
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t perform_sign_conversion(uint32_t adc_raw_data, bool bipolar)
{
	int32_t adc_data;

	/* Bipolar ADC Range:  (-FS) <-> 0 <-> (+FS) : 2^(ADC_RES-1) <-> 0 <-> 2^(ADC_RES-1)-1
	   Unipolar ADC Range: 0 <-> (+FS) : 0 <-> 2^ADC_RES
	 **/
	if (bipolar) {
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
 * @brief	Get the actual ADC gain decimal value
 * @param	chn[in] - ADC channel
 * @return	ADC programmable gain value
 */
float get_gain_value(uint8_t chn)
{
	float gain;
	enum ad4170_pga_gain pga = p_ad4170_dev_inst->config.setups[chn].afe.pga_gain;

	if (pga == AD4170_PGA_GAIN_1_PRECHARGE) {
		gain = 1.0;
	} else if (pga == AD4170_PGA_GAIN_0P5) {
		gain = 0.5;
	} else {
		gain = AD4170_PGA_GAIN(pga);
	}

	return gain;
}

/*!
 * @brief	Convert the ADC raw value into equivalent voltage
 * @param	adc_raw[in]- ADC raw data
 * @param	chn[in] - ADC channel
 * @return	ADC voltage value
 */
float convert_adc_sample_into_voltage(uint32_t adc_raw, uint8_t chn)
{
	float gain;
	int32_t adc_data;
	bool bipolar = p_ad4170_dev_inst->config.setups[chn].afe.bipolar;

	gain = get_gain_value(chn);
	adc_data = perform_sign_conversion(adc_raw, bipolar);

	if (bipolar) {
		return (adc_data * (ADC_REF_VOLTAGE / (ADC_MAX_COUNT_BIPOLAR * gain)));
	} else {
		return (adc_data * (ADC_REF_VOLTAGE / (ADC_MAX_COUNT_UNIPOLAR * gain)));
	}
}

/*!
 * @brief	Convert the ADC raw value into equivalent RTD resistance
 * @param	adc_raw[in] - ADC raw sample
 * @param	rtd_ref[in] - RTD reference resistance in ohms
 * @param	chn[in] - ADC channel
 * @return	RTD resistance value
 * @note	RTD is biased with constant excitation current. Below formula
 *			is based on ratiometric measurement, where fixed value of RTD RREF
 *			(reference resistor) and gain is taken into account
 */
float convert_adc_raw_into_rtd_resistance(uint32_t adc_raw, float rtd_ref,
		uint8_t chn)
{
	float gain;
	int32_t adc_data;
	bool bipolar = p_ad4170_dev_inst->config.setups[chn].afe.bipolar;

	gain = get_gain_value(chn);
	adc_data = perform_sign_conversion(adc_raw, bipolar);

	if (bipolar) {
		return (((float)adc_data * rtd_ref) / (ADC_MAX_COUNT_BIPOLAR * gain));
	} else {
		return (((float)adc_data * rtd_ref) / (ADC_MAX_COUNT_UNIPOLAR * gain));
	}
}

/*!
 * @brief	Disable ADC conversion
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad4170_disable_conversion(void)
{
	struct ad4170_adc_ctrl adc_ctrl = p_ad4170_dev_inst->config.adc_ctrl;

	/* Exit from continuous read mode */
	if (adc_ctrl.cont_read == AD4170_CONT_READ_ON) {
		if (ad4170_continuous_read_exit(p_ad4170_dev_inst) != SUCCESS) {
			return FAILURE;
		}

		adc_ctrl.cont_read = AD4170_CONT_READ_OFF;
	}

	adc_ctrl.mode = AD4170_MODE_STANDBY;
	return ad4170_set_adc_ctrl(p_ad4170_dev_inst, adc_ctrl);
}

/*!
 * @brief	Enable input channel
 * @param	input_chn[in] - Channel to be enabled
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad4170_enable_input_chn(uint32_t input_chn)
{
	uint16_t chn_enable_status;

	chn_enable_status = (p_ad4170_dev_inst->config.channel_en | AD4170_CHANNEL(
				     input_chn));
	return ad4170_set_channel_en(p_ad4170_dev_inst, chn_enable_status);
}

/*!
 * @brief	Disable input channel
 * @param	input_chn[in] - Channel to be disabled
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad4170_disable_input_chn(uint32_t input_chn)
{
	uint16_t chn_enable_status = 0;

	chn_enable_status = (p_ad4170_dev_inst->config.channel_en & ~AD4170_CHANNEL(
				     input_chn));
	return ad4170_set_channel_en(p_ad4170_dev_inst, chn_enable_status);
}

/*!
 * @brief	Apply the excitation sources
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad4170_apply_excitation(void)
{
	struct ad4170_current_source current_source;

#if (ACTIVE_DEMO_MODE_CONFIG == RTD_3WIRE_CONFIG)
	/* Apply excitation on AIN+ using current source 0 */
	current_source = p_ad4170_dev_inst->config.current_source[0];
	current_source.i_out_val = AD4170_I_OUT_500UA;
	if (ad4170_set_current_source(p_ad4170_dev_inst, 0,
				      current_source) != SUCCESS) {
		return FAILURE;
	}

	/* Apply excitation on AIN- using current source 1 */
	current_source = p_ad4170_dev_inst->config.current_source[1];
	current_source.i_out_val = AD4170_I_OUT_500UA;
	return ad4170_set_current_source(p_ad4170_dev_inst, 1, current_source);
#elif ((ACTIVE_DEMO_MODE_CONFIG == RTD_2WIRE_CONFIG) || (ACTIVE_DEMO_MODE_CONFIG == RTD_4WIRE_CONFIG)) || \
	((ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG) && defined(USE_CJC_AS_RTD))
	/* Apply excitation on AIN+ using current source 0 */
	current_source = p_ad4170_dev_inst->config.current_source[0];
	current_source.i_out_val = AD4170_I_OUT_500UA;
	return ad4170_set_current_source(p_ad4170_dev_inst, 0, current_source);
#endif

	return SUCCESS;
}


/*!
 * @brief	Remove the excitation sources
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad4170_remove_excitation(void)
{
	struct ad4170_current_source current_source;

#if (ACTIVE_DEMO_MODE_CONFIG == RTD_3WIRE_CONFIG)
	/* Remove excitation on AIN+ */
	current_source = p_ad4170_dev_inst->config.current_source[0];
	current_source.i_out_val = AD4170_I_OUT_0UA;
	if (ad4170_set_current_source(p_ad4170_dev_inst, 0,
				      current_source) != SUCCESS) {
		return FAILURE;
	}

	/* Remove excitation on AIN- */
	current_source = p_ad4170_dev_inst->config.current_source[1];
	current_source.i_out_val = AD4170_I_OUT_0UA;
	return ad4170_set_current_source(p_ad4170_dev_inst, 1, current_source);
#elif ((ACTIVE_DEMO_MODE_CONFIG == RTD_2WIRE_CONFIG) || (ACTIVE_DEMO_MODE_CONFIG == RTD_4WIRE_CONFIG)) || \
	((ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG) && defined(USE_CJC_AS_RTD))
	current_source = p_ad4170_dev_inst->config.current_source[0];
	current_source.i_out_val = AD4170_I_OUT_0UA;
	return ad4170_set_current_source(p_ad4170_dev_inst, 0, current_source);
#endif

	return SUCCESS;
}
