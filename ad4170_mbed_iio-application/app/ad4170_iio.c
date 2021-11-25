/***************************************************************************//**
 *   @file    ad4170_iio.c
 *   @brief   Implementation of AD4170 IIO application interfaces
 *   @details This module acts as an interface for AD4170 IIO application
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
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "ad4170_iio.h"
#include "app_config.h"
#include "tinyiiod.h"
#include "ad4170_regs.h"
#include "ad4170_data_capture.h"
#include "ad4170_support.h"
#include "ad4170_temperature_sensor.h"
#include "adc_data_capture.h"
#include "error.h"
#include "irq.h"
#include "irq_extra.h"
#include "delay.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/* ADC Raw to Voltage conversion default scale factor for IIO client */
#if defined(BIPOLAR)
#define AD4170_DEFAULT_SCALE	((ADC_REF_VOLTAGE / (ADC_MAX_COUNT_BIPOLAR * (1 << AD4170_PGA_CONFIG))) * 1000)
#else
#define AD4170_DEFAULT_SCALE	((ADC_REF_VOLTAGE / (ADC_MAX_COUNT_UNIPOLAR * (1 << AD4170_PGA_CONFIG))) * 1000)
#endif

/* Number of adc samples for loadcell calibration */
#define LOADCELL_SAMPLES_COUNT	10

/* Bytes per sample. This count should divide the total 256 bytes into 'n' equivalent
 * ADC samples as IIO library requests only 256bytes of data at a time in a given
 * data read query.
 * For 1 to 8-bit ADC, bytes per sample = 1 (2^0)
 * For 9 to 16-bit ADC, bytes per sample = 2 (2^1)
 * For 17 to 32-bit ADC, bytes per sample = 4 (2^2)
 **/
#define	BYTES_PER_SAMPLE	sizeof(uint32_t)	// For ADC resolution of 24-bits

/* Number of data storage bits (needed for IIO client to plot ADC data) */
#define CHN_REAL_BITS		(ADC_RESOLUTION)
#define CHN_STORAGE_BITS	(BYTES_PER_SAMPLE * 8)

#define	LED_TOGGLE_TIME			(500)		// 500msec
#define	LED_TOGGLE_TICK_CNTR	(LED_TOGGLE_TIME / (TICKER_INTERRUPT_PERIOD_uSEC / 1000))

#define		BYTE_SIZE		(uint32_t)8
#define		BYTE_MASK		(uint32_t)0xff

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* IIO interface descriptor */
static struct iio_desc *p_ad4170_iio_desc;

/**
 * Device name.
 */
static const char dev_name[] = ACTIVE_DEVICE_NAME;

/**
 * Pointer to the struct representing the AD4170 IIO device
 */
struct ad4170_dev *p_ad4170_dev_inst = NULL;

/* Device attributes with default values */

/* Scale attribute value per channel */
static float attr_scale_val[AD4170_NUM_CHANNELS] = {
#if (ACTIVE_DEMO_MODE_CONFIG == RTD_2WIRE_CONFIG) || (ACTIVE_DEMO_MODE_CONFIG == RTD_3WIRE_CONFIG) || \
	(ACTIVE_DEMO_MODE_CONFIG == RTD_4WIRE_CONFIG) || (ACTIVE_DEMO_MODE_CONFIG == THERMISTOR_CONFIG) || \
	(ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG)
	1.0,	// scale value is updated at run-time during temperature measurement
#if (ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG)
	1.0,	// scale value is updated at run-time during temperature measurement
#endif
#else
	AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE,
	AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE,
	AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE,
	AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE, AD4170_DEFAULT_SCALE
#endif
};

/* Diagnostic errors */
static const char *diagnostic_errors[] = {
	"ROM CRC Err", "Memory Map CRC Err", "SPI Err", "ADC Conv Err",
	"AINP OV/UV Err", "AINM OV/UV Err", "Ref OV/UV Err", "Ref Diff Min Err",
	"IOUT0 Compl Err", "IOUT1 Compl Err", "IOUT2 Compl Err", "IOUT3 Compl Err",
	"ALDO PSM Err", "DLDO PSM Err", "RES", "Device Init Err"
};

static const char *adc_modes[] = {
	"Continuous_Conversion",
	"Continuous_Conversion_FIR",
	"Continuous_Conversion_IIR",
	"RESERVED",
	"Single_Conversion",
	"Standby",
	"Power_Down",
	"Idle",
	"System_Offset_Calibration",
	"System_Gain_Calibration",
	"Self_Offset_Calibration",
};

/* Flag to trigger new data capture */
static bool adc_data_capture_started = false;

/* Diagnostic error status */
static uint16_t diag_err_status = 0;

/* To store the ADC register values during power-down */
static uint32_t adc_reg_data[ADC_REGISTER_COUNT];

/* ADC channels assigned to sensors for the measurement (each channel per sensor) */
enum sensor_channels {
	SENSOR_CHANNEL0,
#if (ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG)
	SENSOR_CHANNEL1,
#endif
	NUM_OF_SENSOR_CHANNELS
};

/* Attribute IDs */
enum ad4170_attr_id {
	IIO_RAW_ATTR_ID,
	IIO_SCALE_ATTR_ID,
	IIO_OFFSET_ATTR_ID,
	INTERNAL_CALIB_ID,
	SYSTEM_CALIB_ID,
	LOADCELL_OFFSET_CALIB_ID,
	LOADCELL_GAIN_CALIB_ID
};

/* Calibration state */
enum calibration_state {
	FULL_SCALE_CALIB_STATE,
	ZERO_SCALE_CALIB_STATE,
	CALIB_COMPLETE_STATE
};

static enum calibration_state system_calibration_state = ZERO_SCALE_CALIB_STATE;
static enum calibration_state internal_calibration_state =
	FULL_SCALE_CALIB_STATE;
enum calib_status adc_calibration_status;
adc_calibration_configs adc_calibration_config[NUM_OF_SENSOR_CHANNELS];

/* ADC raw averaged values from loadcell calibration */
static uint32_t adc_raw_offset;
static uint32_t adc_raw_gain;

/******************************************************************************/
/************************ Functions Prototypes ********************************/
/******************************************************************************/

static void perform_sensor_measurement_and_update_scale(uint32_t adc_raw,
		uint16_t chn);

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/*!
 * @brief	Getter/Setter for the demo config attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @param	id- Attribute ID (optional)
 * @return	Number of characters read/written
 */
static ssize_t get_demo_config(void *device,
			       char *buf,
			       size_t len,
			       const struct iio_ch_info *channel,
			       intptr_t id)
{
#if (ACTIVE_DEMO_MODE_CONFIG == RTD_2WIRE_CONFIG)
	return (ssize_t)sprintf(buf, "%s", "2-Wire RTD");
#elif (ACTIVE_DEMO_MODE_CONFIG == RTD_3WIRE_CONFIG)
	return (ssize_t)sprintf(buf, "%s", "3-Wire RTD");
#elif (ACTIVE_DEMO_MODE_CONFIG == RTD_4WIRE_CONFIG)
	return (ssize_t)sprintf(buf, "%s", "4-Wire RTD");
#elif (ACTIVE_DEMO_MODE_CONFIG == THERMISTOR_CONFIG)
	return (ssize_t)sprintf(buf, "%s", "Thermistor");
#elif (ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG)
	return (ssize_t)sprintf(buf, "%s", "Thermocouple");
#elif (ACTIVE_DEMO_MODE_CONFIG == ACCELEROMETER_CONFIG)
	return (ssize_t)sprintf(buf, "%s", "Accelerometer");
#elif (ACTIVE_DEMO_MODE_CONFIG == LOADCELL_CONFIG)
	return (ssize_t)sprintf(buf, "%s", "Loadcell");
#else
	return (ssize_t)sprintf(buf, "%s", "User Default");
#endif
}

static ssize_t set_demo_config(void *device,
			       char *buf,
			       size_t len,
			       const struct iio_ch_info *channel,
			       intptr_t id)
{
	/* Demo mode config is selected only at compiler time */
	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the sampling frequency attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @param	id- Attribute ID (optional)
 * @return	Number of characters read/written
 * @Note	This attribute is used to define the timeout period in IIO
 *			client during data capture.
 *			Timeout = (number of requested samples * (1/sampling frequency)) + 1sec
 *			e.g. if sampling frequency = 125KSPS and requested samples = 400
 *			Timeout = (400 * (1/125000)) + 1 = 1.00032sec = ~1sec
 */
static ssize_t get_sampling_frequency(void *device,
				      char *buf,
				      size_t len,
				      const struct iio_ch_info *channel,
				      intptr_t id)
{
	return (ssize_t) sprintf(buf, "%d", AD4170_DEFLT_SAMPLING_FREQEUNCY);
}

static ssize_t set_sampling_frequency(void *device,
				      char *buf,
				      size_t len,
				      const struct iio_ch_info *channel,
				      intptr_t id)
{
	/* NA - Sampling frequency is fixed in the firmware */
	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the raw, offset and scale attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @param	id- Attribute ID
 * @return	Number of characters read/written
 */
static ssize_t get_adc_raw(void *device,
			   char *buf,
			   size_t len,
			   const struct iio_ch_info *channel,
			   intptr_t id)
{
	static uint32_t adc_data_raw = 0;
	int32_t offset = 0;
	uint8_t setup = p_ad4170_dev_inst->config.setup[channel->ch_num].setup_n;

	switch (id) {
	case IIO_RAW_ATTR_ID:
		/* Apply calibrated coefficients before new sampling */
		if (adc_calibration_status == CALIB_DONE) {
			if (ad4170_spi_reg_write(p_ad4170_dev_inst, AD4170_REG_ADC_SETUPS_OFFSET(setup),
						 adc_calibration_config[channel->ch_num].offset_after_calib) != SUCCESS) {
				break;
			}

			if (ad4170_spi_reg_write(p_ad4170_dev_inst, AD4170_REG_ADC_SETUPS_GAIN(setup),
						 adc_calibration_config[channel->ch_num].gain_after_calib) != SUCCESS) {
				break;
			}
		}

		/* Capture the raw adc data */
		if (read_single_sample((uint32_t)channel->ch_num, &adc_data_raw) != FAILURE) {
			perform_sensor_measurement_and_update_scale(adc_data_raw, channel->ch_num);
			return (ssize_t) sprintf(buf, "%d", adc_data_raw);
		}
		break;

	case IIO_SCALE_ATTR_ID:
		return (ssize_t) snprintf(buf, len, "%.10f", attr_scale_val[channel->ch_num]);

	case IIO_OFFSET_ATTR_ID:
#if defined(BIPOLAR) && (ACTIVE_DEMO_MODE_CONFIG == USER_DEFAULT_CONFIG || \
		ACTIVE_DEMO_MODE_CONFIG == ACCELEROMETER_CONFIG || ACTIVE_DEMO_MODE_CONFIG == LOADCELL_CONFIG)
		if (adc_data_raw >= ADC_MAX_COUNT_BIPOLAR) {
			offset = -ADC_MAX_COUNT_UNIPOLAR;
		} else {
			offset = 0;
		}
#endif
		return (ssize_t) sprintf(buf, "%d", offset);

	default:
		break;
	}

	return -EINVAL;
}

static ssize_t set_adc_raw(void *device,
			   char *buf,
			   size_t len,
			   const struct iio_ch_info *channel,
			   intptr_t id)
{
	/* NA- Can't set raw value */
	return len;
}


/*!
 * @brief	Getter/Setter for the diagnostic error attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @param	id- Attribute ID (optional)
 * @return	Number of characters read/written
 */
static ssize_t get_diag_error(void *device,
			      char *buf,
			      size_t len,
			      const struct iio_ch_info *channel,
			      intptr_t id)
{
	uint16_t mask = 0x1;
	uint8_t err_cnt;

	/* Get the ADC error status bit (whichever found first starting from LSB) */
	if (ad4170_get_error(device, &diag_err_status) != FAILURE) {
		for (err_cnt = 0; err_cnt < ARRAY_SIZE(diagnostic_errors); err_cnt++) {
			if (diag_err_status & mask)
				break;
			mask <<= 1;
		}

		if (diag_err_status) {
			return sprintf(buf, "%s", diagnostic_errors[err_cnt]);
		} else {
			strcpy(buf, "No Error");
			return len;
		}
	}

	return -EINVAL;
}

static ssize_t set_diag_error(void *device,
			      char *buf,
			      size_t len,
			      const struct iio_ch_info *channel,
			      intptr_t id)
{
	/* NA- Can't set error value */
	return len;
}


/*!
 * @brief	Getter/Setter for the ADC mode available values
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @param	id- Attribute ID (optional)
 * @return	Number of characters read/written
 */
static ssize_t get_adc_mode_available(void *device,
				      char *buf,
				      size_t len,
				      const struct iio_ch_info *channel,
				      intptr_t id)
{
	return sprintf(buf,
		       "%s",
		       "Continuous_Conversion Continuous_Conversion_FIR Continuous_Conversion_IIR Standby Power_Down Idle");
}

static ssize_t set_adc_mode_available(void *device,
				      char *buf,
				      size_t len,
				      const struct iio_ch_info *channel,
				      intptr_t id)
{
	/* NA- Can't set available mode value */
	return len;
}


/*!
 * @brief	Getter/Setter for the ADC mode attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @param	id- Attribute ID (optional)
 * @return	Number of characters read/written
 */
static ssize_t get_adc_mode(void *device,
			    char *buf,
			    size_t len,
			    const struct iio_ch_info *channel,
			    intptr_t id)
{
	uint32_t reg_val;
	uint8_t adc_mode;
	struct ad4170_adc_ctrl adc_ctrl = p_ad4170_dev_inst->config.adc_ctrl;

	/* Do not read ADC mode when in power-down mode */
	if (adc_ctrl.mode != AD4170_MODE_POWER_DOWN) {
		if (ad4170_spi_reg_read(device, AD4170_REG_ADC_CTRL, &reg_val) != SUCCESS)
			return -EINVAL;

		adc_mode = (reg_val & AD4170_REG_CTRL_MODE_MSK);
	} else {
		adc_mode = AD4170_MODE_POWER_DOWN;
	}

	return sprintf(buf, "%s", adc_modes[adc_mode]);
}

static ssize_t set_adc_mode(void *device,
			    char *buf,
			    size_t len,
			    const struct iio_ch_info *channel,
			    intptr_t id)
{
	bool found = false;
	uint32_t reg;
	uint8_t new_adc_mode_indx;
	uint8_t current_adc_mode;
	struct ad4170_adc_ctrl adc_ctrl = p_ad4170_dev_inst->config.adc_ctrl;

	/* Search for valid adc mode by comparing mode string passed through input
	 * buffer with the mode string stored into RAM */
	for (new_adc_mode_indx = 0; new_adc_mode_indx < ARRAY_SIZE(adc_modes);
	     new_adc_mode_indx++) {
		if (!strncmp(buf,
			     adc_modes[new_adc_mode_indx],
			     strlen(buf))) {
			found = true;
			break;
		}
	}

	/* Get the current ADC mode */
	current_adc_mode = adc_ctrl.mode;

	if (found) {
		/* Check for old and new control mode */
		if ((new_adc_mode_indx != AD4170_MODE_POWER_DOWN)
		    && (adc_ctrl.mode == AD4170_MODE_POWER_DOWN)) {
			/* Reset SPI interface to exit out from power-down mode */
			if (ad4170_reset_spi_interface(device) != SUCCESS) {
				return -EINVAL;
			}

			/* Allow LDO to wake-up */
			mdelay(1000);

			/* Restore all the registers upon exit from power-down mode */
			for (reg = 0; reg < ADC_REGISTER_COUNT; reg++) {
				if (ad4170_spi_reg_write(device, ad4170_regs[reg],
							 adc_reg_data[reg]) != SUCCESS) {
					return -EINVAL;
				}
			}
		} else if ((new_adc_mode_indx == AD4170_MODE_POWER_DOWN)
			   && (current_adc_mode != AD4170_MODE_POWER_DOWN)) {
			/* Store all ADC registers before entering into power down mode */
			for (reg = 0; reg < ADC_REGISTER_COUNT; reg++) {
				if (ad4170_spi_reg_read(device, ad4170_regs[reg],
							&adc_reg_data[reg]) != SUCCESS) {
					return -EINVAL;
				}
			}

			/* Place ADC into standby mode first before entering into power-down mode */
			adc_ctrl.mode = AD4170_MODE_STANDBY;
			ad4170_set_adc_ctrl(device, adc_ctrl);
		} else {
			/* do nothing */
		}

		if ((new_adc_mode_indx != AD4170_MODE_CONT) && adc_data_capture_started) {
			adc_data_capture_started = false;
			stop_data_capture();
		}

		/* Write new ADC mode */
		adc_ctrl.mode = new_adc_mode_indx;
		ad4170_set_adc_ctrl(device, adc_ctrl);
	}

	return len;
}

/*!
 * @brief	Perform the ADC internal/system calibration
 * @param	chn[in] - ADC channel
 * @param	calib_mode[in] - Calibration mode
 * @return	SUCESS in case of SUCCESS, negative error code otherwise
 */
int32_t perform_adc_calibration(enum sensor_channels chn,
				enum ad4170_mode calib_mode)
{
	int32_t status;
	uint32_t data;
	struct ad4170_adc_ctrl adc_ctrl;
	uint8_t setup = p_ad4170_dev_inst->config.setup[chn].setup_n;
	uint8_t pga = p_ad4170_dev_inst->config.setups[setup].afe.pga_gain;

	/* Put ADC into standby mode */
	adc_ctrl = p_ad4170_dev_inst->config.adc_ctrl;
	adc_ctrl.mode = AD4170_MODE_STANDBY;
	status = ad4170_set_adc_ctrl(p_ad4170_dev_inst, adc_ctrl);
	if (status != SUCCESS) {
		return status;
	}

	/* Read the gain/offset coefficient value (pre calibrated) */
	if ((calib_mode == AD4170_MODE_SELF_GAIN_CAL)
	    || (calib_mode == AD4170_MODE_SYS_GAIN_CAL)) {
		status = ad4170_spi_reg_read(p_ad4170_dev_inst,
					     AD4170_REG_ADC_SETUPS_GAIN(setup),
					     &data);
		if (status != SUCCESS) {
			return status;
		}
		adc_calibration_config[chn].gain_before_calib = data;
	} else {
		status = ad4170_spi_reg_read(p_ad4170_dev_inst,
					     AD4170_REG_ADC_SETUPS_OFFSET(setup),
					     &data);
		if (status != SUCCESS) {
			return status;
		}
		adc_calibration_config[chn].offset_before_calib = data;
	}

	/* Enable channel for calibration */
	status = ad4170_enable_input_chn(chn);
	if (status != SUCCESS) {
		return status;
	}

	/* Apply excitation (for RTD sensor config) */
	status = ad4170_apply_excitation();
	if (status != SUCCESS) {
		return status;
	}

	if ((calib_mode == AD4170_MODE_SELF_GAIN_CAL)
	    || (calib_mode == AD4170_MODE_SYS_GAIN_CAL)) {
		if ((calib_mode == AD4170_MODE_SELF_GAIN_CAL)
		    && (pga != AD4170_PGA_GAIN_1 && pga != AD4170_PGA_GAIN_1_PRECHARGE)) {
			/* Internal gain calibration is NA at gain != 1 */
			adc_calibration_config[chn].gain_after_calib =
				adc_calibration_config[chn].gain_before_calib;
			return SUCCESS;
		}

		/* Perform internal/system gain (full-scale) calibration */
		adc_ctrl = p_ad4170_dev_inst->config.adc_ctrl;
		adc_ctrl.mode = calib_mode;
		status = ad4170_set_adc_ctrl(p_ad4170_dev_inst, adc_ctrl);
		if (status != SUCCESS) {
			return status;
		}

		/* Wait for conversion to finish */
		mdelay(100);

		/* Read the gain coefficient value (post calibrated) */
		status = ad4170_spi_reg_read(p_ad4170_dev_inst,
					     AD4170_REG_ADC_SETUPS_GAIN(setup), &data);
		if (status != SUCCESS) {
			return status;
		}
		adc_calibration_config[chn].gain_after_calib = data;

		/* Compare the pre and post adc calibration gain coefficients to check calibration status */
		if (adc_calibration_config[chn].gain_after_calib ==
		    adc_calibration_config[chn].gain_before_calib) {
			/* Error in gain calibration */
			return FAILURE;
		}
	} else {
		/* Perform internal/system offset (zero-scale) calibration */
		adc_ctrl = p_ad4170_dev_inst->config.adc_ctrl;
		adc_ctrl.mode = calib_mode;
		status = ad4170_set_adc_ctrl(p_ad4170_dev_inst, adc_ctrl);
		if (status != SUCCESS) {
			return status;
		}

		/* Wait for conversion to finish */
		mdelay(100);

		/* Read the coefficient value (post calibrated) */
		status = ad4170_spi_reg_read(p_ad4170_dev_inst,
					     AD4170_REG_ADC_SETUPS_OFFSET(setup), &data);
		if (status != SUCCESS) {
			return status;
		}
		adc_calibration_config[chn].offset_after_calib = data;

		/* Compare the pre and post adc calibration offset coefficients to check calibration status */
		if (adc_calibration_config[chn].offset_after_calib ==
		    adc_calibration_config[chn].offset_before_calib) {
			/* Error in offset calibration */
			return FAILURE;
		}
	}

	/* Remove excitation (for RTD sensor config) */
	status = ad4170_remove_excitation();
	if (status != SUCCESS) {
		return status;
	}

	/* Disable previously enabled channel */
	status = ad4170_disable_input_chn(chn);
	if (status != SUCCESS) {
		return status;
	}

	return SUCCESS;
}

/*!
 * @brief	Getter/Setter for the ADC internal/system calibration
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @param	id- Attribute ID
 * @return	Number of characters read/written
 */
static ssize_t get_calibration_status(void *device,
				      char *buf,
				      size_t len,
				      const struct iio_ch_info *channel,
				      intptr_t id)
{
	uint8_t buf_offset = 0;

	switch (id) {
	case SYSTEM_CALIB_ID:
	case INTERNAL_CALIB_ID:
		if (id == SYSTEM_CALIB_ID && system_calibration_state == CALIB_COMPLETE_STATE) {
			system_calibration_state = ZERO_SCALE_CALIB_STATE;
		} else if (id == INTERNAL_CALIB_ID
			   && internal_calibration_state == CALIB_COMPLETE_STATE) {
			internal_calibration_state = FULL_SCALE_CALIB_STATE;
		} else {
			/* Return NA to indicate that system calibration is not supported
			 * using IIO oscilloscope. Pyadi-iio script needs to be executed
			 * to perform a system calibration due to manual intervention */
			return snprintf(buf, len, "%s", "NA");
		}

		sprintf(buf + buf_offset, "%08x",
			adc_calibration_config[channel->ch_num].gain_before_calib);
		buf_offset += 8;
		sprintf(buf + buf_offset, "%08x",
			adc_calibration_config[channel->ch_num].gain_after_calib);
		buf_offset += 8;
		sprintf(buf + buf_offset, "%08x",
			adc_calibration_config[channel->ch_num].offset_before_calib);
		buf_offset += 8;
		sprintf(buf + buf_offset, "%08x",
			adc_calibration_config[channel->ch_num].offset_after_calib);
		buf_offset += 8;
		sprintf(buf + buf_offset, "%s", "calibration_done");
		buf_offset += (strlen("calibration_done") + 1);
		return buf_offset;

	default:
		return -EINVAL;
	}

	return len;
}

static ssize_t set_calibration_routine(void *device,
				       char *buf,
				       size_t len,
				       const struct iio_ch_info *channel,
				       intptr_t id)
{
	switch (id) {
	case INTERNAL_CALIB_ID:
		if (!strncmp(buf, "start_calibration", strlen(buf))) {
			switch (internal_calibration_state) {
			case FULL_SCALE_CALIB_STATE:
				if (perform_adc_calibration(channel->ch_num,
							    AD4170_MODE_SELF_GAIN_CAL) != SUCCESS) {
					adc_calibration_status = CALIB_ERROR;
					break;
				}
				internal_calibration_state = ZERO_SCALE_CALIB_STATE;
				break;

			case ZERO_SCALE_CALIB_STATE:
				if (perform_adc_calibration(channel->ch_num,
							    AD4170_MODE_SELF_OFFSET_CAL) != SUCCESS) {
					adc_calibration_status = CALIB_ERROR;
					internal_calibration_state = FULL_SCALE_CALIB_STATE;
					break;
				}
				adc_calibration_status = CALIB_DONE;
				internal_calibration_state = CALIB_COMPLETE_STATE;
				break;

			case CALIB_COMPLETE_STATE:
			default:
				internal_calibration_state = FULL_SCALE_CALIB_STATE;
				return -EINVAL;
			}
		}
		break;

	case SYSTEM_CALIB_ID:
		if (!strncmp(buf, "start_calibration", strlen(buf))) {
			switch (system_calibration_state) {
			case ZERO_SCALE_CALIB_STATE:
				if (perform_adc_calibration(channel->ch_num,
							    AD4170_MODE_SYS_OFFSET_CAL) != SUCCESS) {
					adc_calibration_status = CALIB_ERROR;
					break;
				}
				system_calibration_state = FULL_SCALE_CALIB_STATE;
				break;

			case FULL_SCALE_CALIB_STATE:
				if (perform_adc_calibration(channel->ch_num,
							    AD4170_MODE_SYS_GAIN_CAL) != SUCCESS) {
					adc_calibration_status = CALIB_ERROR;
					system_calibration_state = ZERO_SCALE_CALIB_STATE;
					break;
				}
				adc_calibration_status = CALIB_DONE;
				system_calibration_state = CALIB_COMPLETE_STATE;
				break;

			case CALIB_COMPLETE_STATE:
			default:
				system_calibration_state = ZERO_SCALE_CALIB_STATE;
				return -EINVAL;
			}
		}
		break;

	default:
		return -EINVAL;
	}

	return len;
}


/*!
 * @brief	Getter/Setter for the Loadcell offset/gain calibration
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @param	id- Attribute ID
 * @return	Number of characters read/written
 */
static ssize_t get_loadcell_calibration_status(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	switch (id) {
	case LOADCELL_OFFSET_CALIB_ID:
		return (ssize_t)sprintf(buf, "%d", adc_raw_offset);

	case LOADCELL_GAIN_CALIB_ID:
		return (ssize_t)sprintf(buf, "%d", adc_raw_gain);

	default:
		return -EINVAL;
	}

	return len;
}

static ssize_t set_loadcell_calibration_status(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	uint32_t adc_raw;
	uint8_t sample_cnt;
	uint64_t adc_raw_avg = 0;

	if (!strncmp(buf, "start_calibration", strlen(buf))) {
		switch (id) {
		case LOADCELL_OFFSET_CALIB_ID:
			for (sample_cnt = 0; sample_cnt < LOADCELL_SAMPLES_COUNT; sample_cnt++) {
				read_single_sample(SENSOR_CHANNEL0, &adc_raw);
				adc_raw_avg += adc_raw;
			}

			adc_raw_avg /= LOADCELL_SAMPLES_COUNT;
			adc_raw_offset = (uint32_t)adc_raw_avg;
			break;

		case LOADCELL_GAIN_CALIB_ID:
			for (sample_cnt = 0; sample_cnt < LOADCELL_SAMPLES_COUNT; sample_cnt++) {
				read_single_sample(SENSOR_CHANNEL0, &adc_raw);
				adc_raw_avg += adc_raw;
			}

			adc_raw_avg /= LOADCELL_SAMPLES_COUNT;
			adc_raw_gain = (uint32_t)adc_raw_avg;
			break;

		default:
			return -EINVAL;
		}
	}

	return len;
}


/*!
 * @brief	Search the debug register address in look-up table Or registers array
 * @param	addr- Register address to search for
 * @param	reg_addr_offset - Offset of register address from its base address for
 *			multi-byte register entity
 * @return	Index to register address from look-up detect
 */
static uint32_t debug_reg_search(uint32_t addr, uint32_t *reg_addr_offset)
{
	uint32_t curr_indx; 	// Indexing to registers array (look-up table)
	uint32_t reg_base_add; 	// Base register address
	bool found = false;		// Address found status flag

	/* Search for valid input register address in registers array */
	for (curr_indx = 0; curr_indx < ADC_REGISTER_COUNT; curr_indx++) {
		if (addr == AD4170_ADDR(ad4170_regs[curr_indx])) {
			*reg_addr_offset = 0;
			found = true;
			break;
		} else if (addr < AD4170_ADDR(ad4170_regs[curr_indx])) {
			/* Get the input address offset from its base address for
			 * multi-byte register entity and break the loop indicating input
			 * address is located somewhere in the previous indexed register */
			if (AD4170_TRANSF_LEN(ad4170_regs[curr_indx - 1]) > 1) {
				*reg_addr_offset = addr - AD4170_ADDR(ad4170_regs[curr_indx - 1]);
				found = true;
			}
			break;
		}
	}

	/* Get the base address of register entity (single or multi byte) */
	if (found) {
		if (*reg_addr_offset > 0) {
			reg_base_add = ad4170_regs[curr_indx - 1];
		} else {
			reg_base_add = ad4170_regs[curr_indx];
		}
	} else {
		reg_base_add = addr | AD4170_R1B;
	}

	return reg_base_add;
}


/*!
 * @brief	Read the debug register value
 * @param	dev- Pointer to IIO device instance
 * @param	reg- Register address to read from
 * @param	readval- Pointer to variable to read data into
 * @return	SUCCESS in case of success, negative value otherwise
 */
int32_t debug_reg_read(void *dev, uint32_t reg, uint32_t *readval)
{
	uint32_t reg_base_add; 		// Base register address
	uint32_t reg_addr_offset;	// Offset of input register address from its base

	if (reg <= MAX_REGISTER_ADDRESS) {
		reg_base_add = debug_reg_search(reg, &reg_addr_offset);

		/* Read data from device register */
		if ((ad4170_spi_reg_read(dev, reg_base_add, readval) != SUCCESS)) {
			return FAILURE;
		}

		/* Extract the specific byte location for register entity */
		*readval = (*readval >> (reg_addr_offset * BYTE_SIZE)) & BYTE_MASK;

		return SUCCESS;
	}

	return FAILURE;
}


/*!
 * @brief	Write into the debug register
 * @param	dev- Pointer to IIO device instance
 * @param	reg- Register address to write into
 * @param	writeval- Register value to write
 * @return	SUCCESS in case of success, negative value otherwise
 */
int32_t debug_reg_write(void *dev, uint32_t reg, uint32_t writeval)
{
	uint32_t reg_base_add; 		// Base register address
	uint32_t reg_addr_offset; 	// Offset of input register address from its base
	uint32_t data;				// Register data

	if (reg <= MAX_REGISTER_ADDRESS) {
		reg_base_add = debug_reg_search(reg, &reg_addr_offset);

		/* Read the register contents */
		if ((ad4170_spi_reg_read(dev, reg_base_add, &data) != SUCCESS)) {
			return FAILURE;
		}

		/* Modify the register contents to write user data at specific
		 * reister entity location */
		data &= ~(BYTE_MASK << (reg_addr_offset * BYTE_SIZE));
		data |= (uint32_t)((writeval & BYTE_MASK) << (reg_addr_offset * BYTE_SIZE));

		/* Write data into device register */
		if (ad4170_spi_reg_write(dev, reg_base_add, data) != SUCCESS) {
			return FAILURE;
		}

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief	Perform the sensor measurement as per current demo config and update
 *			the adc_raw value to sensor conversion scale factor for IIO client
 * @param	adc_raw[in] - ADC raw value
 * @param	chn[in] -  ADC channel
 * @return	none
 */
static void perform_sensor_measurement_and_update_scale(uint32_t adc_raw,
		uint16_t chn)
{
	float temperature = 0;
	int32_t cjc_raw_data;
	float cjc_temp;

#if (ACTIVE_DEMO_MODE_CONFIG == THERMISTOR_CONFIG)
	temperature = get_ntc_thermistor_temperature(adc_raw, SENSOR_CHANNEL0);
	attr_scale_val[SENSOR_CHANNEL0] = (temperature / perform_sign_conversion(
			adc_raw, AD4170_BIPOLAR_MODE)) * 1000.0;
#elif ((ACTIVE_DEMO_MODE_CONFIG == RTD_2WIRE_CONFIG) || \
	(ACTIVE_DEMO_MODE_CONFIG == RTD_3WIRE_CONFIG) || (ACTIVE_DEMO_MODE_CONFIG == RTD_4WIRE_CONFIG))
	temperature = get_rtd_temperature(adc_raw, SENSOR_CHANNEL0);
	attr_scale_val[SENSOR_CHANNEL0] = (temperature / perform_sign_conversion(
			adc_raw, AD4170_BIPOLAR_MODE)) * 1000.0;
#elif (ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG)
	if (chn == SENSOR_CHANNEL1) {
		/* Sample the TC channel (CJC channel is already sampled through get_raw() function) */
		cjc_raw_data = adc_raw;
		if (read_single_sample(SENSOR_CHANNEL0, (uint32_t *)&adc_raw) != SUCCESS) {
			return;
		}
	} else {
		/* Sample the CJC channel (TC channel is already sampled through get_raw() function) */
		if (read_single_sample(SENSOR_CHANNEL1, (uint32_t *)&cjc_raw_data) != SUCCESS) {
			return;
		}
	}

	/* Calculate the TC and CJC temperature and update scale factor */
	temperature = get_tc_temperature(adc_raw, cjc_raw_data,
					 SENSOR_CHANNEL0, SENSOR_CHANNEL1, &cjc_temp);
	attr_scale_val[SENSOR_CHANNEL0] = (temperature / perform_sign_conversion(
			adc_raw, AD4170_BIPOLAR_MODE)) * 1000.0;
	attr_scale_val[SENSOR_CHANNEL1] = (cjc_temp / perform_sign_conversion(
			cjc_raw_data, AD4170_BIPOLAR_MODE)) * 1000.0;
#endif
}


/**
 * @brief	Read buffer data corresponding to AD4170 IIO device
 * @param	dev_instance[in] - IIO device instance
 * @param	pbuf[out] - Pointer to output data buffer
 * @param	offset[in] - Data buffer offset
 * @param	bytes_count[in] - Number of bytes to read
 * @param	ch_mask[in] - Channels select mask
 * @return	SUCCESS in case of success or negative value otherwise
 */
static ssize_t iio_ad4170_read_data(void *dev_instance,
				    char *pbuf, size_t offset, size_t bytes_count, uint32_t ch_mask)
{
	if (adc_data_capture_started == false) {
		start_data_capture(ch_mask, AD4170_NUM_CHANNELS);
		adc_data_capture_started = true;
	}

	/* Read the data stored into acquisition buffers */
	return (ssize_t)read_buffered_data(pbuf, bytes_count, offset, ch_mask,
					   BYTES_PER_SAMPLE);
}


/**
 * @brief	Transfer the device data into memory (optional)
 * @param	dev_instance[in] - IIO device instance
 * @param	bytes_count[in] - Number of bytes to read
 * @param	ch_mask[in] - Channels select mask
 * @return	SUCCESS in case of success or negative value otherwise
 */
static ssize_t iio_ad4170_transfer_dev_data(void *dev_instance,
		size_t bytes_count, uint32_t ch_mask)
{
	/* The function insures that data is first read into memory from the device.
	 * This function doesn't do any sort of data transfer but it make sure data
	 * read and it's transfer to memory from device is happening in application through
	 * iio_ad4170_read_data() function */

	/* Store the requested samples count value for data capture */
	store_requested_samples_count(bytes_count, BYTES_PER_SAMPLE);

	return SUCCESS;
}


/**
 * @brief	Perform tasks before new data transfer
 * @param	dev_instance[in] - IIO device instance
 * @param	ch_mask[in] - Channels select mask
 * @return	SUCCESS in case of success or negative value otherwise
 */
static int32_t iio_ad4170_start_transfer(void *dev_instance, uint32_t ch_mask)
{
	return SUCCESS;
}


/**
 * @brief	Perform tasks before end of current data transfer
 * @param	dev_instance[in] - IIO device instance
 * @return	SUCCESS in case of success or negative value otherwise
 */
static int32_t iio_ad4170_stop_transfer(void *dev)
{
	adc_data_capture_started = false;
	stop_data_capture();

	return SUCCESS;
}

/*********************************************************
 *               IIO Attributes and Structures
 ********************************************************/

/* IIOD channels attributes list */
struct iio_attribute channel_input_attributes[] = {
	{
		.name = "raw",
		.show = get_adc_raw,
		.store = set_adc_raw,
		.priv = IIO_RAW_ATTR_ID
	},
	{
		.name = "scale",
		.show = get_adc_raw,
		.store = set_adc_raw,
		.priv = IIO_SCALE_ATTR_ID
	},
	{
		.name = "offset",
		.show = get_adc_raw,
		.store = set_adc_raw,
		.priv = IIO_OFFSET_ATTR_ID
	},
	{
		.name = "internal_calibration",
		.show = get_calibration_status,
		.store = set_calibration_routine,
		.priv = INTERNAL_CALIB_ID
	},
	{
		.name = "system_calibration",
		.show = get_calibration_status,
		.store = set_calibration_routine,
		.priv = SYSTEM_CALIB_ID
	},
#if (ACTIVE_DEMO_MODE_CONFIG == LOADCELL_CONFIG)
	{
		.name = "loadcell_offset_calibration",
		.show = get_loadcell_calibration_status,
		.store = set_loadcell_calibration_status,
		.priv = LOADCELL_OFFSET_CALIB_ID
	},
	{
		.name = "loadcell_gain_calibration",
		.show = get_loadcell_calibration_status,
		.store = set_loadcell_calibration_status,
		.priv = LOADCELL_GAIN_CALIB_ID
	},
#endif

	END_ATTRIBUTES_ARRAY
};

/* IIOD device (global) attributes list */
static struct iio_attribute global_attributes[] = {
	{
		.name = "demo_config",
		.show = get_demo_config,
		.store = set_demo_config
	},
	{
		.name = "sampling_frequency",
		.show = get_sampling_frequency,
		.store = set_sampling_frequency,
	},
	{
		.name = "diagnostic_error_status",
		.show = get_diag_error,
		.store = set_diag_error
	},
	{
		.name = "adc_mode_available",
		.show = get_adc_mode_available,
		.store = set_adc_mode_available
	},
	{
		.name = "adc_mode",
		.show = get_adc_mode,
		.store = set_adc_mode
	},

	END_ATTRIBUTES_ARRAY
};

/* IIOD debug attributes list */
static struct iio_attribute debug_attributes[] = {
	{
		.name = "direct_reg_access",
		.show = NULL,
		.store = NULL,
	},

	END_ATTRIBUTES_ARRAY
};

/* IIOD channels configurations */
struct scan_type chn_scan = {
#if defined(BIPOLAR)
	.sign = 's',
#else
	.sign = 'u',
#endif
	.realbits = CHN_REAL_BITS,
	.storagebits = CHN_STORAGE_BITS,
	.shift = 0,
	.is_big_endian = false
};

static struct iio_channel iio_ad4170_channels[] = {
#if (ACTIVE_DEMO_MODE_CONFIG == USER_DEFAULT_CONFIG)
	{
		.name = "voltage0",
		.ch_type = IIO_VOLTAGE,
		.channel = 0,
		.scan_index = 0,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true,
	},
	{
		.name = "voltage1",
		.ch_type = IIO_VOLTAGE,
		.channel = 1,
		.scan_index = 1,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage2",
		.ch_type = IIO_VOLTAGE,
		.channel = 2,
		.scan_index = 2,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage3",
		.ch_type = IIO_VOLTAGE,
		.channel = 3,
		.scan_index = 3,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage4",
		.ch_type = IIO_VOLTAGE,
		.channel = 4,
		.scan_index = 4,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage5",
		.ch_type = IIO_VOLTAGE,
		.channel = 5,
		.scan_index = 5,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage6",
		.ch_type = IIO_VOLTAGE,
		.channel = 6,
		.scan_index = 6,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage7",
		.ch_type = IIO_VOLTAGE,
		.channel = 7,
		.scan_index = 7,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage8",
		.ch_type = IIO_VOLTAGE,
		.channel = 8,
		.scan_index = 8,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true,
	},
	{
		.name = "voltage9",
		.ch_type = IIO_VOLTAGE,
		.channel = 9,
		.scan_index = 9,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage10",
		.ch_type = IIO_VOLTAGE,
		.channel = 10,
		.scan_index = 10,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage11",
		.ch_type = IIO_VOLTAGE,
		.channel = 11,
		.scan_index = 11,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage12",
		.ch_type = IIO_VOLTAGE,
		.channel = 12,
		.scan_index = 12,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage13",
		.ch_type = IIO_VOLTAGE,
		.channel = 13,
		.scan_index = 13,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage14",
		.ch_type = IIO_VOLTAGE,
		.channel = 14,
		.scan_index = 14,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
	{
		.name = "voltage15",
		.ch_type = IIO_VOLTAGE,
		.channel = 15,
		.scan_index = 15,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true
	},
#elif (ACTIVE_DEMO_MODE_CONFIG == ACCELEROMETER_CONFIG || ACTIVE_DEMO_MODE_CONFIG == LOADCELL_CONFIG)
	/* Note" Channel type is considered as voltage as IIO oscilloscope doesn't
	 * support accelerometer unit format of G and loadcell unit fomat of gram */
	{
		.name = "Sensor1",
		.ch_type = IIO_VOLTAGE,
		.channel = SENSOR_CHANNEL0,
		.scan_index = SENSOR_CHANNEL0,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true,
	},
#else
	{
		.name = "Sensor1",
		.ch_type = IIO_TEMP,
		.channel = SENSOR_CHANNEL0,
		.scan_index = SENSOR_CHANNEL0,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true,
	},
#if (ACTIVE_DEMO_MODE_CONFIG == THERMOCOUPLE_CONFIG)
	{
		.name = "CJC1",
		.ch_type = IIO_TEMP,
		.channel = SENSOR_CHANNEL1,
		.scan_index = SENSOR_CHANNEL1,
		.scan_type = &chn_scan,
		.attributes = channel_input_attributes,
		.ch_out = false,
		.indexed = true,
	},
#endif
#endif
};


/**
 * @brief	Init for reading/writing and parameterization of a
 * 			ad4170 IIO device
 * @param 	desc[in,out] - IIO device descriptor
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t iio_ad4170_init(struct iio_device **desc)
{
	struct iio_device *iio_ad4170_inst;

	iio_ad4170_inst = calloc(1, sizeof(struct iio_device));
	if (!iio_ad4170_inst) {
		return FAILURE;
	}

	iio_ad4170_inst->num_ch = sizeof(iio_ad4170_channels) / sizeof(
					  iio_ad4170_channels[0]);
	iio_ad4170_inst->channels = iio_ad4170_channels;
	iio_ad4170_inst->attributes = global_attributes;
	iio_ad4170_inst->debug_attributes = debug_attributes;

	iio_ad4170_inst->transfer_dev_to_mem = iio_ad4170_transfer_dev_data;
	iio_ad4170_inst->transfer_mem_to_dev = NULL;
	iio_ad4170_inst->read_data = iio_ad4170_read_data;
	iio_ad4170_inst->write_data = NULL;
	iio_ad4170_inst->prepare_transfer = iio_ad4170_start_transfer;
	iio_ad4170_inst->end_transfer = iio_ad4170_stop_transfer;
	iio_ad4170_inst->debug_reg_read = debug_reg_read;
	iio_ad4170_inst->debug_reg_write = debug_reg_write;

	*desc = iio_ad4170_inst;

	return SUCCESS;
}


/**
 * @brief Release resources allocated for IIO device
 * @param desc[in] - IIO device descriptor
 * @return SUCCESS in case of success, FAILURE otherwise
 */
static int32_t iio_ad4170_remove(struct iio_desc *desc)
{
	int32_t status;

	if (!desc) {
		return FAILURE;
	}

	status = iio_unregister(desc, (char *)dev_name);
	if (status != SUCCESS) {
		return FAILURE;
	}

	return SUCCESS;
}


/*!
 * @brief	This is an ISR (Interrupt Service Routine) for Ticker object
 * @param	*ctx[in] - Callback context (unused)
 * @param	event[in] - Callback event (unused)
 * @param	extra[in] - Callback extra (unused)
 * @return	none
 * @details	This function is periodically called based on the time period
 *			configured during Ticker instance creation/initialization.
 */
void ticker_callback(void *ctx, uint32_t event, void *extra)
{
	static uint32_t tick_cntr;
	static bool led_on = false;

	tick_cntr++;
	if (tick_cntr >= LED_TOGGLE_TICK_CNTR) {
		tick_cntr = 0;

		if (diag_err_status) {
			if (led_on) {
				/* Turn off LED */
				gpio_set_value(led_gpio_desc, GPIO_HIGH);
				led_on = false;
			} else {
				/* Turn on LED */
				gpio_set_value(led_gpio_desc, GPIO_LOW);
				led_on = true;
			}
		} else {
			/* Turn off LED */
			gpio_set_value(led_gpio_desc, GPIO_HIGH);
		}
	}
}


/**
 * @brief	Initialize the IIO interface for AD4170 IIO device
 * @return	none
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad4170_iio_initialize(void)
{
	int32_t init_status;

	/* IIO device descriptor */
	struct iio_device *p_iio_ad4170_dev;

	/**
	* IIO interface init parameters
	*/
	struct iio_init_param iio_init_params = {
		.phy_type = USE_UART,
		{
			&uart_init_params
		}
	};

	/* Initialize AD4170 device and peripheral interface */
	init_status = ad4170_init(&p_ad4170_dev_inst, &ad4170_init_params);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Initialize the IIO interface */
	init_status = iio_init(&p_ad4170_iio_desc, &iio_init_params);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Initialize the AD4170 IIO application interface */
	init_status = iio_ad4170_init(&p_iio_ad4170_dev);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Register AD4170 IIO interface */
	init_status = iio_register(p_ad4170_iio_desc,
				   p_iio_ad4170_dev,
				   (char *)dev_name,
				   p_ad4170_dev_inst,
				   NULL,
				   NULL);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Init the system peripherals */
	init_status = init_system();
	if (init_status != SUCCESS) {
		return init_status;
	}

	return init_status;
}


/**
 * @brief 	Run the AD4170 IIO event handler
 * @return	none
 * @details	This function monitors the new IIO client event
 */
void ad4170_iio_event_handler(void)
{
	while (1) {
		(void)iio_step(p_ad4170_iio_desc);
	}
}
