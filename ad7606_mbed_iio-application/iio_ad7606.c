/***************************************************************************//**
 *   @file    iio_ad7606.c
 *   @brief   Implementation of AD7606 IIO application interfaces
 *   @details This module acts as an interface for AD7606 IIO application
********************************************************************************
 * Copyright (c) 2020-2021 Analog Devices, Inc.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "app_config.h"
#include "iio_ad7606.h"
#include "platform_support.h"
#include "spi_extra.h"
#include "gpio_extra.h"
#include "uart_extra.h"
#include "irq_extra.h"
#include "error.h"

#include "ad7606.h"
#include "adc_data_capture.h"
#include "ad7606_support.h"
#include "ad7606_user_config.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/* ADC data to Voltage conversion scale factor for IIO client */
#define DEFAULT_SCALE		((DEFAULT_CHN_RANGE / ADC_MAX_COUNT_BIPOLAR) * 1000)

/* LSB Threshold to entry into open circuit detection as per datasheet */
#define MANUAL_OPEN_DETECT_ENTRY_TRHLD			350

/* Manual open circuit detect LSB threshold @50K Rpd as per datasheet */
#define MANUAL_OPEN_DETECT_THRESHOLD_RPD50K		20

/* Number of consecutive conversions (N) in manual open circuit detection */
#define MANUAL_OPEN_DETECT_CONV_CNTS			10

/* LSB Threshold b/w consecutive N conversions */
#define MANUAL_OPEN_DETECT_CONV_TRSHLD			10

/* Number of common mode conversions in manual open circuit detect */
#define MANUAL_OPEN_DETECT_CM_CNV_CNT			3

/* Max number of queue counts for auto mode open circuit detection */
#define AUTO_OPEN_DETECT_QUEUE_MAX_CNT			128
#define AUTO_OPEN_DETECT_QUEUE_EXTRA_CONV_CNT	15

/* Maximum ADC calibration gain value */
#define ADC_CALIBRATION_GAIN_MAX		64.0

#if defined(DEV_AD7606C_18)
#define	OFFSET_REG_RESOLUTION		4
#else
#define	OFFSET_REG_RESOLUTION		1
#endif

/* Bytes per sample. This count should divide the total 256 bytes into 'n' equivalent
 * ADC samples as IIO library requests only 256bytes of data at a time in a given
 * data read query.
 * For 1 to 8-bit ADC, bytes per sample = 1 (2^0)
 * For 9 to 16-bit ADC, bytes per sample = 2 (2^1)
 * For 17 to 32-bit ADC, bytes per sample = 4 (2^2)
 **/
#if (AD7606X_ADC_RESOLUTION == 18)
#define	BYTES_PER_SAMPLE	sizeof(uint32_t)	// For ADC resolution of 18-bits
#else
#define	BYTES_PER_SAMPLE	sizeof(uint16_t)	// For ADC resolution of 16-bits
#endif

/* Number of data storage bits (needed for IIO client to plot ADC data) */
#define CHN_STORAGE_BITS	(BYTES_PER_SAMPLE * 8)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* IIO interface descriptor */
static struct iio_desc *p_ad7606_iio_desc;

/**
 * Device name.
 */
static const char dev_name[] = ACTIVE_DEVICE_NAME;

/**
 * Pointer to the struct representing the AD7606 IIO device
 */
struct ad7606_dev *p_ad7606_dev_inst = NULL;


/* Device attributes with default values */

/* Power down mode values string representation (possible values specified in datasheet) */
static char *operating_mode_str[] = {
	"0  (Normal Mode)",
	"1  (Standby Mode)",
	"2  (Auto Standby Mode)",
	"3  (Shutdown Mode)"
};

/* Bandwidth values string */
static char *bandwidth_str[] = {
	"0  (Low)",
	"1  (High)"
};

/* Channel range values string representation (possible values specified in datasheet) */
static char *chn_range_str[] = {
#if defined(DEV_AD7606B)
	"0  (+/-2.5V SE)", "1  (+/-5.0V SE)", "2  (+/-10.0V SE)", "3  (+/-10.0V SE)",
	"4  (+/-10.0V SE)", "5  (+/-10.0V SE)", "6  (+/-10.0V SE)", "7  (+/-10.0V SE)",
	"8  (+/-10.0V SE)", "9  (+/-10.0V SE)", "10  (+/-10.0V SE)", "11  (+/-10.0V SE)",
#elif defined(DEV_AD7606C_18) || defined(DEV_AD7606C_16)
	"0  (+/-2.5V SE)", "1  (+/-5.0V SE)", "2  (+/-6.25V SE)", "3  (+/-10.0V SE)",
	"4  (+/-12.5V SE)", "5  (0 to 5V SE)", "6  (0 to 10V SE)", "7  (0 to 12.5V SE)",
	"8  (+/-5.0V Diff)", "9  (+/-10.0V Diff)", "10  (+/-12.5V Diff)", "11  (+/-20.0V Diff)"
#elif defined(DEV_AD7609)
	"0  (+/-10.0V SE)", "1  (+/-20.0V SE)"
#else
	"0  (+/-5.0V SE)", "1  (+/-10.0V SE)"
#endif
};

/* Oversampling values string representation (possible values specified in datasheet) */
static char *oversampling_val_str[] = {
	"0 (no oversampling)", "1 (oversampling by 2)", "2 (oversampling by 4)",
	"3 (oversampling by 8)", "4 (oversampling by 16)", "5 (oversampling by 32)",
	"6 (oversampling by 64)", "7 (oversampling by 128)", "8 (oversampling by 256)"
};


/* Channel range values string representation (possible values specified in datasheet) */
static float chn_range_val[] = {
#if defined(DEV_AD7606B)
	2.5, 5.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0
#elif defined(DEV_AD7606C_18) || defined(DEV_AD7606C_16)
	2.5, 5.0, 6.25, 10.0, 12.5, 5.0, 10.0, 12.5, 5.0, 10.0, 12.5, 20.0
#elif defined(DEV_AD7609)
	10.0, 20.0
#else
	5.0, 10.0
#endif
};

/* Range value per channel */
static float attr_chn_range[AD7606X_ADC_CHANNELS] = {
	DEFAULT_CHN_RANGE, DEFAULT_CHN_RANGE, DEFAULT_CHN_RANGE, DEFAULT_CHN_RANGE,
	DEFAULT_CHN_RANGE, DEFAULT_CHN_RANGE, DEFAULT_CHN_RANGE, DEFAULT_CHN_RANGE
};

/* Scale value per channel */
static float attr_scale_val[AD7606X_ADC_CHANNELS] = {
	DEFAULT_SCALE, DEFAULT_SCALE, DEFAULT_SCALE, DEFAULT_SCALE,
	DEFAULT_SCALE, DEFAULT_SCALE, DEFAULT_SCALE, DEFAULT_SCALE
};

/* Scale value per channel */
static polarity_e attr_polarity_val[AD7606X_ADC_CHANNELS] = {
	BIPOLAR, BIPOLAR, BIPOLAR, BIPOLAR,
	BIPOLAR, BIPOLAR, BIPOLAR, BIPOLAR
};

/* Channel range */
typedef enum {
	LOW,
	HIGH
} range_e;

/* Open detect auto mode QUEUE register count */
static uint8_t open_detect_queue_cnts[AD7606X_ADC_CHANNELS] = {
	0
};

/* ADC gain calibration Rfilter value (in Kohms) */
static uint8_t gain_calibration_reg_val[AD7606X_ADC_CHANNELS] = {
	0
};

/* Flag to trigger new background conversion and capture when READBUFF command is issued */
static bool adc_data_capture_started = false;

/* Gain calibration status */
static bool gain_calibration_done = false;

/* Open circuit mode detection flags */
static bool open_circuit_detection_done = false;
static bool open_circuit_detection_error = false;
static bool open_circuit_detect_read_done = false;

/******************************************************************************/
/************************ Functions Prototypes ********************************/
/******************************************************************************/

static float get_vltg_conv_scale_factor(float chn_range, polarity_e polarity);
static void save_local_attributes(void);

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/*!
 * @brief	Getter/Setter for the scale attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 */
static ssize_t get_chn_scale(void *device,
			     char *buf,
			     size_t len,
			     const struct iio_ch_info *channel,
			     intptr_t id)
{
	return (ssize_t) sprintf(buf, "%f", attr_scale_val[channel->ch_num]);
}

static ssize_t set_chn_scale(void *device,
			     char *buf,
			     size_t len,
			     const struct iio_ch_info *channel,
			     intptr_t id)
{
	float scale;

	(void)sscanf(buf, "%f", &scale);

	if (scale > 0.0) {
		attr_scale_val[channel->ch_num] = scale;
		return len;
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the sampling frequency attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @Note	This attribute is used to define the timeout period in IIO
 *			client during data capture.
 *			Timeout = (number of requested samples * (1/sampling frequency)) + 1sec
 *			e.g. if sampling frequency = 1KSPS and requested samples = 400
 *			Timeout = (400 * 0.001) + 1 = 1.4sec
 */
static ssize_t get_sampling_frequency(void *device,
				      char *buf,
				      size_t len,
				      const struct iio_ch_info *channel,
				      intptr_t id)
{
	return (ssize_t) sprintf(buf, "%d", SAMPLING_RATE);
}

static ssize_t set_sampling_frequency(void *device,
				      char *buf,
				      size_t len,
				      const struct iio_ch_info *channel,
				      intptr_t id)
{
	/* NA- Can't set sampling frequency value */
	return len;
}


/*!
 * @brief	Getter/Setter for the raw attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 */
static ssize_t get_chn_raw(void *device,
			   char *buf,
			   size_t len,
			   const struct iio_ch_info *channel,
			   intptr_t id)
{
	int32_t adc_data_raw;

	/* Capture the raw adc data */
	if (read_single_sample(channel->ch_num, (uint32_t *)&adc_data_raw) != SUCCESS) {
		return -EINVAL;
	}

	return (ssize_t) sprintf(buf, "%d", adc_data_raw);
}

static ssize_t set_chn_raw(void *device,
			   char *buf,
			   size_t len,
			   const struct iio_ch_info *channel,
			   intptr_t id)
{
	/* NA- Can't set raw value */
	return len;
}


/*!
 * @brief	Getter/Setter for the operating mode attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_operating_mode(void *device,
				  char *buf,
				  size_t len,
				  const struct iio_ch_info *channel,
				  intptr_t id)
{
	uint8_t read_val;
	uint8_t operating_mode_value;

	if (ad7606_spi_reg_read(device, AD7606_REG_CONFIG, &read_val) == SUCCESS) {
		operating_mode_value = (read_val & AD7606_CONFIG_OPERATION_MODE_MSK);

		if (operating_mode_value < sizeof(operating_mode_str) / sizeof(
			    operating_mode_str[0])) {
			return (ssize_t)sprintf(buf, "%s", operating_mode_str[operating_mode_value]);
		}
	}

	return -EINVAL;
}

static ssize_t set_operating_mode(void *device,
				  char *buf,
				  size_t len,
				  const struct iio_ch_info *channel,
				  intptr_t id)
{
	uint8_t operating_mode_value;

	(void)sscanf(buf, "%d", &operating_mode_value);

	if (operating_mode_value < sizeof(operating_mode_str) / sizeof(
		    operating_mode_str[0])) {
		if (ad7606_spi_write_mask(device,
					  AD7606_REG_CONFIG,
					  AD7606_CONFIG_OPERATION_MODE_MSK,
					  operating_mode_value) == SUCCESS) {
			return len;
		}
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the power down mode attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available for all devices except AD7606B and AD7606C
 */
static ssize_t get_power_down_mode(void *device,
				   char *buf,
				   size_t len,
				   const struct iio_ch_info *channel,
				   intptr_t id)
{
	uint8_t gpio_stby_val;
	uint8_t gpio_range_val;

	if (gpio_get_value(((struct ad7606_dev *)device)->gpio_stby_n,
			   &gpio_stby_val) == SUCCESS) {
		if (gpio_get_value(((struct ad7606_dev *)device)->gpio_range,
				   &gpio_range_val) == SUCCESS) {

			if (gpio_stby_val) {
				return sprintf(buf, "%s", operating_mode_str[AD7606_NORMAL]);
			} else {
				if (gpio_range_val) {
					return sprintf(buf, "%s", operating_mode_str[AD7606_STANDBY]);
				} else {
					return sprintf(buf, "%s", operating_mode_str[AD7606_SHUTDOWN]);
				}
			}
		}
	}

	return -EINVAL;
}

static ssize_t set_power_down_mode(void *device,
				   char *buf,
				   size_t len,
				   const struct iio_ch_info *channel,
				   intptr_t id)
{
	uint8_t power_down_mode_value;
	static enum ad7606_op_mode prev_power_down_mode = AD7606_NORMAL;
	struct ad7606_config dev_config;

	sscanf(buf, "%d", &power_down_mode_value);

	if (power_down_mode_value < (sizeof(operating_mode_str) / sizeof(
					     operating_mode_str[0]))) {

		dev_config.op_mode = power_down_mode_value;

		switch (power_down_mode_value) {
		case AD7606_NORMAL:
			if (ad7606_set_config(device, dev_config) == SUCCESS) {
				/* Reset the device if previous power down mode was either standby
				 * or shutdown */
				if (prev_power_down_mode != AD7606_NORMAL) {

					/* Power-up wait time */
					mdelay(1);

					/* Toggle reset pin */
					if (gpio_set_value(((struct ad7606_dev *)device)->gpio_reset,
							   GPIO_HIGH) == SUCCESS) {
						mdelay(1);

						if (gpio_set_value(((struct ad7606_dev *)device)->gpio_reset,
								   GPIO_LOW) == SUCCESS) {
							prev_power_down_mode = AD7606_NORMAL;
							return len;
						}
					}
				}
			}
			break;

		case AD7606_STANDBY:
			if (ad7606_set_config(device, dev_config) == SUCCESS) {
				prev_power_down_mode = AD7606_STANDBY;
				return len;
			}
			break;

		case AD7606_SHUTDOWN:
			if (ad7606_set_config(device, dev_config) == SUCCESS) {
				prev_power_down_mode = AD7606_SHUTDOWN;
				return len;
			}
			break;

		default:
			break;
		}
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the range attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available for all devices except AD7606B and AD7606C
 */
static ssize_t get_range(void *device,
			 char *buf,
			 size_t len,
			 const struct iio_ch_info *channel,
			 intptr_t id)
{
	uint8_t gpio_range_val;
	struct ad7606_dev *dev = device;

	if (gpio_get_value(dev->gpio_range, &gpio_range_val) == SUCCESS) {
		if (gpio_range_val) {
			return sprintf(buf, "%s", chn_range_str[HIGH]);
		} else {
			return sprintf(buf, "%s", chn_range_str[LOW]);
		}
	}

	return -EINVAL;
}

static ssize_t set_range(void *device,
			 char *buf,
			 size_t len,
			 const struct iio_ch_info *channel,
			 intptr_t id)
{
	uint8_t range_value;
	struct ad7606_dev *dev = device;

	(void)sscanf(buf, "%d", &range_value);

	if (range_value < (sizeof(chn_range_str) / sizeof(chn_range_str[0]))) {
		if (range_value == LOW) {
			if (gpio_set_value(dev->gpio_range, GPIO_LOW) == SUCCESS) {
				return len;
			}
		} else {
			if (gpio_set_value(dev->gpio_range, GPIO_HIGH) == SUCCESS) {
				return len;
			}
		}
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the oversampling attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available for all devices except AD7606B and AD7606C
 */
static ssize_t get_oversampling(void *device,
				char *buf,
				size_t len,
				const struct iio_ch_info *channel,
				intptr_t id)
{
	uint8_t oversampling_value;
	uint8_t read_val;
	uint8_t gpio_osr0_val;
	uint8_t gpio_osr1_val;
	uint8_t gpio_osr2_val;
	struct ad7606_dev *dev = device;

#if defined(DEV_AD7606B) || defined(DEV_AD7606C_18) || defined(DEV_AD7606C_16)
	if (ad7606_spi_reg_read(device,
				AD7606_REG_OVERSAMPLING,
				&read_val) == SUCCESS) {
		oversampling_value = (read_val & AD7606_OVERSAMPLING_MSK);

		if (oversampling_value < sizeof(oversampling_val_str) / sizeof(
			    oversampling_val_str[0])) {
			return (ssize_t)sprintf(buf, "%s", oversampling_val_str[oversampling_value]);
		}
	}
#else
	if (gpio_get_value(dev->gpio_os0, &gpio_osr0_val) == SUCCESS) {
		if (gpio_get_value(dev->gpio_os1, &gpio_osr1_val) == SUCCESS) {
			if (gpio_get_value(dev->gpio_os2, &gpio_osr2_val) == SUCCESS) {
				oversampling_value = (gpio_osr2_val << 2) | (gpio_osr1_val << 1) |
						     gpio_osr0_val;

				if (oversampling_value < (sizeof(oversampling_val_str) / sizeof(
								  oversampling_val_str[0]))) {
					return sprintf(buf, "%s", oversampling_val_str[oversampling_value]);
				}
			}
		}
	}
#endif

	return -EINVAL;
}

static ssize_t set_oversampling(void *device,
				char *buf,
				size_t len,
				const struct iio_ch_info *channel,
				intptr_t id)
{
	uint8_t oversampling_value;
	struct ad7606_oversampling oversampling_cfg;

	(void)sscanf(buf, "%d", &oversampling_value);

	if (oversampling_value < (sizeof(oversampling_val_str) / sizeof(
					  oversampling_val_str[0]))) {

		oversampling_cfg.os_pad = 0;
		oversampling_cfg.os_ratio = oversampling_value;

		ad7606_set_oversampling(device, oversampling_cfg);

		return len;
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the bandwidth attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available for only AD7606C
 */
static ssize_t get_bandwidth(void *device,
			     char *buf,
			     size_t len,
			     const struct iio_ch_info *channel,
			     intptr_t id)
{
	uint8_t bw_value;
	uint8_t read_val;

	if (ad7606_spi_reg_read(device,
				AD7606_REG_BANDWIDTH,
				&read_val) == SUCCESS) {
		bw_value = (read_val >> (channel->ch_num)) & 0x1;

		if (bw_value < sizeof(bandwidth_str) / sizeof(
			    bandwidth_str[0])) {
			return (ssize_t)sprintf(buf, "%s", bandwidth_str[bw_value]);
		}
	}

	return -EINVAL;
}

static ssize_t set_bandwidth(void *device,
			     char *buf,
			     size_t len,
			     const struct iio_ch_info *channel,
			     intptr_t id)
{
	uint8_t bw_value;
	uint8_t read_val;

	(void)sscanf(buf, "%d", &bw_value);

	if (bw_value < sizeof(bandwidth_str) / sizeof(
		    bandwidth_str[0])) {
		if (ad7606_spi_reg_read(device,
					AD7606_REG_BANDWIDTH,
					&read_val) == SUCCESS) {
			if (bw_value) {
				read_val |= (1 << (channel->ch_num));
			} else {
				read_val &= (~(1 << (channel->ch_num)));
			}

			if (ad7606_spi_reg_write(device,
						 AD7606_REG_BANDWIDTH,
						 read_val) == SUCCESS) {
				return len;
			}
		}
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel range attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_chn_range(void *device,
			     char *buf,
			     size_t len,
			     const struct iio_ch_info *channel,
			     intptr_t id)
{
	uint8_t read_val;
	uint8_t chn_range;

	if (ad7606_spi_reg_read(device, AD7606_REG_RANGE_CH_ADDR(channel->ch_num),
				&read_val) == SUCCESS) {
		if (((channel->ch_num) % 2) != 0) {
			read_val >>= CHANNEL_RANGE_MSK_OFFSET;
			chn_range = read_val;
		} else {
			chn_range = (read_val & AD7606_RANGE_CH_MSK(channel->ch_num));
		}

		if (chn_range < sizeof(chn_range_str) / sizeof(chn_range_str[0])) {
			attr_chn_range[channel->ch_num] = chn_range_val[chn_range];
			attr_polarity_val[channel->ch_num] = ad7606_get_input_polarity(chn_range);

			return (ssize_t)sprintf(buf, "%s", chn_range_str[chn_range]);
		}
	}

	return -EINVAL;
}

static ssize_t set_chn_range(void *device,
			     char *buf,
			     size_t len,
			     const struct iio_ch_info *channel,
			     intptr_t id)
{
	uint8_t chn_range;

	(void)sscanf(buf, "%d", &chn_range);

	if (chn_range < sizeof(chn_range_val) / sizeof(chn_range_val[0])) {

		/* Get the polarity of channel */
		attr_polarity_val[channel->ch_num] = ad7606_get_input_polarity(chn_range);

		attr_chn_range[channel->ch_num] = chn_range_val[chn_range];
		attr_scale_val[channel->ch_num] = get_vltg_conv_scale_factor(
				chn_range_val[chn_range],
				attr_polarity_val[channel->ch_num]);

		if (((channel->ch_num) % 2) != 0) {
			chn_range <<= CHANNEL_RANGE_MSK_OFFSET;
		}

		if (ad7606_spi_write_mask(device,
					  AD7606_REG_RANGE_CH_ADDR(channel->ch_num),
					  AD7606_RANGE_CH_MSK(channel->ch_num),
					  chn_range) == SUCCESS) {
			return len;
		}
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel offset attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_chn_offset(void *device,
			      char *buf,
			      size_t len,
			      const struct iio_ch_info *channel,
			      intptr_t id)
{
	uint8_t chn_offset_value;

	if (ad7606_spi_reg_read(device, AD7606_REG_OFFSET_CH(channel->ch_num),
				&chn_offset_value) == SUCCESS) {
		return (ssize_t)sprintf(buf, "%d", chn_offset_value);
	}

	return -EINVAL;
}

static ssize_t set_chn_offset(void *device,
			      char *buf,
			      size_t len,
			      const struct iio_ch_info *channel,
			      intptr_t id)
{
	uint8_t chn_offset_value = 0;

	(void)sscanf(buf, "%d", &chn_offset_value);

	if (ad7606_set_ch_offset(device, channel->ch_num,
				 chn_offset_value) == SUCCESS) {
		return len;
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel pahse offset attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_chn_phase_offset(void *device,
				    char *buf,
				    size_t len,
				    const struct iio_ch_info *channel,
				    intptr_t id)
{
	uint8_t chn_phase_offset_value;

	if (ad7606_spi_reg_read(device,
				AD7606_REG_PHASE_CH(channel->ch_num),
				&chn_phase_offset_value) == SUCCESS) {
		return (ssize_t)sprintf(buf, "%d", chn_phase_offset_value);
	}

	return -EINVAL;
}

static ssize_t set_chn_phase_offset(void *device,
				    char *buf,
				    size_t len,
				    const struct iio_ch_info *channel,
				    intptr_t id)
{
	uint8_t chn_phase_offset_value = 0;

	(void)sscanf(buf, "%d", &chn_phase_offset_value);

	if (ad7606_set_ch_phase(device, channel->ch_num,
				chn_phase_offset_value) == SUCCESS) {
		return len;
	}

	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel temperature attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_chn_temperature(void *device,
				   char *buf,
				   size_t len,
				   const struct iio_ch_info *channel,
				   intptr_t id)
{
	int32_t adc_data_raw = 0;
	float temperature;
	float voltage;

	/* Configure the channel multiplexer to select temperature read */
	if (ad7606_spi_write_mask(device,
				  AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
						  TEMPERATURE_MUX)) == SUCCESS) {

		/* Allow to settle Mux channel */
		udelay(100);

		/* Sample the channel and read conversion result */
		read_single_sample(channel->ch_num, (uint32_t *)&adc_data_raw);

		/* Convert ADC data into equivalent voltage */
		voltage = convert_adc_raw_to_voltage(adc_data_raw,
						     attr_scale_val[channel->ch_num]);

		/* Obtain the temperature using equation specified in device datasheet */
		temperature = ((voltage - 0.69068) / 0.019328) + 25.0;

		/* Change channel mux back to analog input */
		(void)ad7606_spi_write_mask(device,
					    AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
							    ANALOG_INPUT_MUX));

		return (ssize_t)sprintf(buf, "%f", temperature);
	}

	return -EINVAL;
}

static ssize_t set_chn_temperature(void *device,
				   char *buf,
				   size_t len,
				   const struct iio_ch_info *channel,
				   intptr_t id)
{
	// NA- Can't set temperature
	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel Vref attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_chn_vref(void *device,
			    char *buf,
			    size_t len,
			    const struct iio_ch_info *channel,
			    intptr_t id)
{
	float vref_voltge;
	int32_t adc_data_raw;

	/* Configure the channel multiplexer to select Vref read */
	if (ad7606_spi_write_mask(device,
				  AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
						  VREF_MUX)) == SUCCESS) {

		/* Allow to settle Mux channel */
		udelay(100);

		/* Sample the channel and read conversion result */
		read_single_sample(channel->ch_num, (uint32_t *)&adc_data_raw);

		/* Convert ADC data into equivalent voltage */
		vref_voltge = convert_adc_raw_to_voltage(adc_data_raw,
				attr_scale_val[channel->ch_num]);

		/* Divide by 4 since Vref Mux has 4x multiplier on it */
		vref_voltge /= VREF_MUX_MULTIPLIER;

		/* Change channel mux back to analog input */
		(void)ad7606_spi_write_mask(device,
					    AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
							    ANALOG_INPUT_MUX));

		return (ssize_t)sprintf(buf, "%f", vref_voltge);
	}

	return -EINVAL;
}

static ssize_t set_chn_vref(void *device,
			    char *buf,
			    size_t len,
			    const struct iio_ch_info *channel,
			    intptr_t id)
{
	// NA- Can't set Vref
	return - EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel Vdrive attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_chn_vdrive(void *device,
			      char *buf,
			      size_t len,
			      const struct iio_ch_info *channel,
			      intptr_t id)
{
	float vdrive_voltge;
	int32_t adc_data_raw;

	/* Configure the channel multiplexer to select Vdrive read */
	if (ad7606_spi_write_mask(device,
				  AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
						  VDRIVE_MUX)) == SUCCESS) {

		/* Allow to settle Mux channel */
		udelay(100);

		/* Sample the channel and read conversion result */
		read_single_sample(channel->ch_num, (uint32_t *)&adc_data_raw);

		/* Convert ADC data into equivalent voltage */
		vdrive_voltge = convert_adc_raw_to_voltage(adc_data_raw,
				attr_scale_val[channel->ch_num]);

		/* Change channel mux back to analog input */
		(void)ad7606_spi_write_mask(device,
					    AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
							    ANALOG_INPUT_MUX));

		return (ssize_t)sprintf(buf, "%f", vdrive_voltge);
	}

	return -EINVAL;
}

static ssize_t set_chn_vdrive(void *device,
			      char *buf,
			      size_t len,
			      const struct iio_ch_info *channel,
			      intptr_t id)
{
	// NA- Can't set Vdrive
	return - EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel ALDO attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_chn_aldo(void *device,
			    char *buf,
			    size_t len,
			    const struct iio_ch_info *channel,
			    intptr_t id)
{
	float aldo_voltge;
	int32_t adc_data_raw;

	/* Configure the channel multiplexer to select ALDO read */
	if (ad7606_spi_write_mask(device,
				  AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
						  ALDO_MUX)) == SUCCESS) {

		/* Allow to settle Mux channel */
		udelay(100);

		/* Sample the channel and read conversion result */
		read_single_sample(channel->ch_num, (uint32_t *)&adc_data_raw);

		/* Convert ADC data into equivalent voltage */
		aldo_voltge = convert_adc_raw_to_voltage(adc_data_raw,
				attr_scale_val[channel->ch_num]);

		/* Divide by 4 since ALDO Mux has 4x multiplier on it */
		aldo_voltge /= VREF_MUX_MULTIPLIER;

		/* Change channel mux back to analog input */
		(void)ad7606_spi_write_mask(device,
					    AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
							    ANALOG_INPUT_MUX));

		return (ssize_t)sprintf(buf, "%f", aldo_voltge);
	}

	return -EINVAL;
}

static ssize_t set_chn_aldo(void *device,
			    char *buf,
			    size_t len,
			    const struct iio_ch_info *channel,
			    intptr_t id)
{
	// NA- Can't set ALDO
	return - EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel DLDO attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 * @note	Available only for AD7606B and AD7606C
 */
static ssize_t get_chn_dldo(void *device,
			    char *buf,
			    size_t len,
			    const struct iio_ch_info *channel,
			    intptr_t id)
{
	float dldo_voltge;
	int32_t adc_data_raw;

	/* Configure the channel multiplexer to select DLDO read */
	if (ad7606_spi_write_mask(device,
				  AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
				  AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
						  DLDO_MUX)) == SUCCESS) {

		/* Allow to settle Mux channel */
		udelay(100);

		/* Sample the channel and read conversion result */
		read_single_sample(channel->ch_num, (uint32_t *)&adc_data_raw);

		/* Convert ADC data into equivalent voltage */
		dldo_voltge = convert_adc_raw_to_voltage(adc_data_raw,
				attr_scale_val[channel->ch_num]);

		/* Divide by 4 since ALDO Mux has 4x multiplier on it */
		dldo_voltge /= VREF_MUX_MULTIPLIER;

		/* Change channel mux back to analog input */
		(void)ad7606_spi_write_mask(device,
					    AD7606_REG_DIAGNOSTIC_MUX_CH(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_MSK(channel->ch_num),
					    AD7606_DIAGN_MUX_CH_VAL((channel->ch_num),
							    ANALOG_INPUT_MUX));

		return (ssize_t)sprintf(buf, "%f", dldo_voltge);
	}

	return -EINVAL;
}

static ssize_t set_chn_dldo(void *device,
			    char *buf,
			    size_t len,
			    const struct iio_ch_info *channel,
			    intptr_t id)
{
	// NA- Can't set DLDO
	return - EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel open circuit detect manual attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 */
static ssize_t get_chn_open_circuit_detect_manual(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	int32_t prev_adc_code, curr_adc_code;
	bool open_detect_flag = false;
	bool open_detect_done = false;
	uint8_t cnt;

	/* Enter into manual open circuit detection mode */
	do {
		if (ad7606_spi_reg_write(device, AD7606_REG_OPEN_DETECT_QUEUE, 1) == SUCCESS) {
			/* Read the ADC on selected chnnel (first reading post open circuit detection start) */
			read_single_sample(channel->ch_num, (uint32_t *)&prev_adc_code);

			/* Perform N conversions and monitor the code delta */
			for (cnt = 0; cnt < MANUAL_OPEN_DETECT_CONV_CNTS; cnt++) {
				/* Check if code is within 350LSB (nearest ZS code) */
				if (prev_adc_code >= 0 && prev_adc_code < MANUAL_OPEN_DETECT_ENTRY_TRHLD) {
					/* Perform next conversion and read the result */
					read_single_sample(channel->ch_num, (uint32_t *)&curr_adc_code);

					/* Check if delta b/w current and previus reading is within 10 LSB code */
					if (abs(curr_adc_code - prev_adc_code) > MANUAL_OPEN_DETECT_CONV_TRSHLD) {
						open_detect_done = true;
						break;
					}

					/* Get the previous code */
					prev_adc_code = curr_adc_code;
				} else {
					open_detect_done = true;
					break;
				}
			}

			/* Break if open circuit detection aborted (in case above conditions not met) */
			if (open_detect_done)
				break;

			/* Set common mode high (enabling open circuit detect on selected channel) */
			if (ad7606_spi_reg_write(device,
						 AD7606_REG_OPEN_DETECT_ENABLE,
						 (1 << (channel->ch_num))) == SUCCESS) {

				/* Perform next conversions (~2-3) and read the result (with common mode set high) */
				for (cnt = 0; cnt < MANUAL_OPEN_DETECT_CM_CNV_CNT; cnt++) {
					udelay(100);
					read_single_sample(channel->ch_num, (uint32_t *)&curr_adc_code);
				}

				/* Check if delta b/w common mode high code and previous N conversion code is > threshold */
				if ((curr_adc_code - prev_adc_code) < MANUAL_OPEN_DETECT_THRESHOLD_RPD50K) {
					open_detect_done = true;
					break;
				}
			} else {
				return -EINVAL;
			}

			/* Break if open circuit detection aborted (in case above conditions not met) */
			if (open_detect_done)
				break;

			/* Set common mode low (disabling open circuit detect on channel) */
			if (ad7606_spi_reg_write(device,
						 AD7606_REG_OPEN_DETECT_ENABLE,
						 0) == SUCCESS) {
				/* Perform next conversion and read the result (with common mode set low) */
				read_single_sample(channel->ch_num, (uint32_t *)&curr_adc_code);

				/* Check if delta b/w common mode low code and previous N conversion code is < threshold */
				if (abs(curr_adc_code - prev_adc_code) < MANUAL_OPEN_DETECT_THRESHOLD_RPD50K) {
					open_detect_flag = true;
					open_detect_done = true;
				}
			} else {
				return -EINVAL;
			}
		} else {
			return -EINVAL;
		}
	} while (0);

	/* Disable open detect mode */
	(void)ad7606_spi_reg_write(device, AD7606_REG_OPEN_DETECT_QUEUE, 0);

	if (open_detect_done) {
		if (open_detect_flag) {
			strcpy(buf, "Open Circuit Detected");
		} else {
			strcpy(buf, "Open Circuit Not Detected");
		}

		return len;
	}

	return -EINVAL;
}

static ssize_t set_chn_open_circuit_detect_manual(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	// NA- Can't set open circuit detect
	return - EINVAL;
}


/*!
 * @brief	Getter/Setter for the channel open circuit detect auto attribute value
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 */
static ssize_t get_chn_open_circuit_detect_auto(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	if (open_circuit_detect_read_done) {
		open_circuit_detect_read_done = false;

		if (open_circuit_detection_error) {
			strcpy(buf, "Error!!");
		}

		if (open_circuit_detection_done) {
			strcpy(buf, "Open Circuit Detected");
		} else {
			strcpy(buf, "Open Circuit Not Detected");
		}

		return len;
	}

	return (ssize_t)sprintf(buf, "OPEN_DETECT_QUEUE: %d",
				open_detect_queue_cnts[channel->ch_num]);
}

static ssize_t set_chn_open_circuit_detect_auto(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	uint8_t data;
	uint8_t open_detect_flag = false;
	int32_t rw_status = FAILURE;
	uint16_t conv_cnts;

	(void)sscanf(buf, "%d", &data);
	open_circuit_detection_error = false;

	if ((data > 1 && data <= AUTO_OPEN_DETECT_QUEUE_MAX_CNT) && (buf[0] >= '0'
			&& buf[0] <= '9')) {
		open_detect_queue_cnts[channel->ch_num] = data;

		/* Enter into open circuit auto open detect mode */
		if (ad7606_spi_reg_write(device,
					 AD7606_REG_OPEN_DETECT_QUEUE,
					 open_detect_queue_cnts[channel->ch_num]) == SUCCESS) {
			/* Enable open circuit detection on selected channel */
			if (ad7606_spi_reg_write(device,
						 AD7606_REG_OPEN_DETECT_ENABLE,
						 (1 << (channel->ch_num))) == SUCCESS) {
				/* Monitor the open detect flag for max N+15 (open detect queue count) conversions.
				 * Note: In ideal scenario, the open detect flash should be monitored continuously while
				 * background N conversions are in progress */
				for (conv_cnts = 0;
				     conv_cnts < (open_detect_queue_cnts[channel->ch_num] +
						  AUTO_OPEN_DETECT_QUEUE_EXTRA_CONV_CNT);
				     conv_cnts++) {
					if (ad7606_convst(device) == SUCCESS) {
						udelay(100);

						/* Monitor the open detect flag */
						if (ad7606_spi_reg_read(device,
									AD7606_REG_OPEN_DETECTED,
									&open_detect_flag) == SUCCESS) {
							open_detect_flag >>= (channel->ch_num);
							open_detect_flag &= 0x1;

							rw_status = SUCCESS;
							if (open_detect_flag) {
								break;
							}
						} else {
							rw_status = FAILURE;
							break;
						}
					} else {
						rw_status = FAILURE;
						break;
					}
				}
			}
		}

		/* Disable open detect mode and clear open detect flag */
		(void)ad7606_spi_reg_write(device, AD7606_REG_OPEN_DETECT_QUEUE, 0);
		(void)ad7606_spi_reg_write(device, AD7606_REG_OPEN_DETECTED, 0xFF);

		open_detect_queue_cnts[channel->ch_num] = 0;

		if (rw_status == SUCCESS) {
			if (open_detect_flag) {
				open_circuit_detection_done = true;
			} else {
				open_circuit_detection_done = false;
			}

			open_circuit_detect_read_done = true;
			return len;
		}
	}

	open_circuit_detection_error = true;
	return -EINVAL;
}


/*!
 * @brief	Getter/Setter for the adc offset calibration
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 */
static ssize_t get_chn_calibrate_adc_offset(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	float lsb_voltage;
	float adc_voltage;
	polarity_e polarity = attr_polarity_val[channel->ch_num];
	int32_t adc_raw_data;
	int8_t chn_offset;

	/* Perform the system offset calibration */

	if (polarity == UNIPOLAR) {
		lsb_voltage = attr_chn_range[channel->ch_num] / ADC_MAX_COUNT_UNIPOLAR;
	} else {
		lsb_voltage = attr_chn_range[channel->ch_num] / ADC_MAX_COUNT_BIPOLAR;
	}

	/* Sample and read the ADC channel */
	read_single_sample(channel->ch_num, (uint32_t *)&adc_raw_data);

	/* Get an equivalent ADC voltage */
	adc_voltage = convert_adc_raw_to_voltage(adc_raw_data,
			attr_scale_val[channel->ch_num]);

	/* Calculate the channel offset and write it to offset register */
	chn_offset = -(adc_voltage / lsb_voltage / OFFSET_REG_RESOLUTION);

	if (ad7606_set_ch_offset(device, channel->ch_num,
				 chn_offset) == SUCCESS) {
		return sprintf(buf, "%s", "ADC Calibration Done");
	}

	return -EINVAL;
}

static ssize_t set_chn_calibrate_adc_offset(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	// NA- Can't set open circuit detect
	return - EINVAL;
}


/*!
 * @brief	Getter/Setter for the adc gain calibration
 * @param	device- pointer to IIO device structure
 * @param	buf- pointer to buffer holding attribute value
 * @param	len- length of buffer string data
 * @param	channel- pointer to IIO channel structure
 * @return	Number of characters read/written
 */
static ssize_t get_chn_calibrate_adc_gain(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	uint8_t read_val;

	if (gain_calibration_done) {
		/* Get calibration status for previous gain value write event */
		gain_calibration_done = false;
		return sprintf(buf, "Calibration Done (Rfilter=%d K)",
			       gain_calibration_reg_val[channel->ch_num]);
	}

	/* Return gain value when normal read event is triggered */
	if (ad7606_spi_reg_read(device,
				AD7606_REG_GAIN_CH(channel->ch_num),
				&read_val) == SUCCESS) {
		gain_calibration_reg_val[channel->ch_num] = (read_val & AD7606_GAIN_MSK);
		return sprintf(buf, "Rfilter= %d K",
			       gain_calibration_reg_val[channel->ch_num]);
	}

	return -EINVAL;
}

static ssize_t set_chn_calibrate_adc_gain(void *device,
		char *buf,
		size_t len,
		const struct iio_ch_info *channel,
		intptr_t id)
{
	float data;

	if (buf[0] >= '0' && buf[0] <= '9') {
		(void)sscanf(buf, "%f", &data);

		if (data >= 0 && data < ADC_CALIBRATION_GAIN_MAX) {
			/* Get the nearest value of unsigned integer */
			gain_calibration_reg_val[channel->ch_num] = (uint8_t)(round(data));

			/* Perform the gain calibration by writing gain value into gain register */
			if (ad7606_set_ch_gain(device,
					       channel->ch_num,
					       gain_calibration_reg_val[channel->ch_num]) == SUCCESS) {
				gain_calibration_done = true;
				return len;
			}
		}
	}

	return -EINVAL;
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
	/* Read the data from device */
	if (reg <= NUM_OF_REGISTERS) {
		if ((ad7606_spi_reg_read(dev, reg, (uint8_t *)readval) == SUCCESS)) {
			return SUCCESS;
		}
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
	if (reg <= NUM_OF_REGISTERS) {
		if ((ad7606_spi_reg_write(dev, reg, (uint8_t)writeval) == SUCCESS)) {
			save_local_attributes();
			return SUCCESS;
		}
	}

	return FAILURE;
}


/**
 * @brief	Read buffer data corresponding to AD7606 IIO device
 * @param	dev_instance[in] - IIO device instance
 * @param	pbuf[out] - Pointer to output data buffer
 * @return	SUCCESS in case of success or negative value otherwise
 */
static ssize_t iio_ad7606_read_data(void *dev_instance,
				    char *pbuf,
				    size_t offset,
				    size_t bytes_count,
				    uint32_t ch_mask)
{
	if (adc_data_capture_started == false) {
		start_data_capture(ch_mask, AD7606X_ADC_CHANNELS);
		adc_data_capture_started = true;
	}

	/* Read the buffered data */
	return (ssize_t)read_buffered_data(pbuf,
					   bytes_count,
					   offset,
					   ch_mask,
					   BYTES_PER_SAMPLE);
}


/**
 * @brief	Transfer the device data into memory (optional)
 * @param	dev_instance[in] - IIO device instance
 * @param	bytes_count[in] - Number of bytes to read
 * @param	ch_mask[in] - Channels select mask
 * @return	SUCCESS in case of success or negative value otherwise
 */
static ssize_t iio_ad7606_transfer_dev_data(void *dev_instance,
		size_t bytes_count,
		uint32_t ch_mask)
{
	/* The function insures that data is first read into memory from the device.
	 * This function doesn't do any sort of data transfer but it make sure data
	 * read and it's transfer to memory from device is happening in application through
	 * iio_ad7606_read_data() function */

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
static int32_t iio_ad7606_start_transfer(void *dev_instance, uint32_t ch_mask)
{
	return SUCCESS;
}


/**
 * @brief	Perform tasks before end of current data transfer
 * @param	dev_instance[in] - IIO device instance
 * @return	SUCCESS in case of success or negative value otherwise
 */
static int32_t iio_ad7606_stop_transfer(void *dev)
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
		.show = get_chn_raw,
		.store = set_chn_raw,
	},
	{
		.name = "scale",
		.show = get_chn_scale,
		.store = set_chn_scale,
	},
#if defined(DEV_AD7606B) || defined(DEV_AD7606C_18) || defined(DEV_AD7606C_16)
	{
		.name = "chn_range",
		.show = get_chn_range,
		.store = set_chn_range,
	},
	{
		.name = "offset",
		.show = get_chn_offset,
		.store = set_chn_offset,
	},
	{
		.name = "chn_phase offset",
		.show = get_chn_phase_offset,
		.store = set_chn_phase_offset,
	},
	{
		.name = "temperature",
		.show = get_chn_temperature,
		.store = set_chn_temperature,
	},
	{
		.name = "vref",
		.show = get_chn_vref,
		.store = set_chn_vref,
	},
	{
		.name = "vdrive",
		.show = get_chn_vdrive,
		.store = set_chn_vdrive,
	},
	{
		.name = "ALDO",
		.show = get_chn_aldo,
		.store = set_chn_aldo,
	},
	{
		.name = "DLDO",
		.show = get_chn_dldo,
		.store = set_chn_dldo,
	},
	{
		.name = "open_circuit_detect_manual",
		.show = get_chn_open_circuit_detect_manual,
		.store = set_chn_open_circuit_detect_manual,
	},
	{
		.name = "open_circuit_detect_auto",
		.show = get_chn_open_circuit_detect_auto,
		.store = set_chn_open_circuit_detect_auto,
	},
	{
		.name = "calibrate_adc_offset",
		.show = get_chn_calibrate_adc_offset,
		.store = set_chn_calibrate_adc_offset,
	},
	{
		.name = "calibrate_adc_gain",
		.show = get_chn_calibrate_adc_gain,
		.store = set_chn_calibrate_adc_gain,
	},
#if defined(DEV_AD7606C_18) || defined(DEV_AD7606C_16)
	{
		.name = "bandwidth",
		.show = get_bandwidth,
		.store = set_bandwidth,
	},
#endif
#endif

	END_ATTRIBUTES_ARRAY
};

/* IIOD device (global) attributes list */
static struct iio_attribute global_attributes[] = {
#if defined(DEV_AD7606B) || defined(DEV_AD7606C_18) || defined(DEV_AD7606C_16)
	{
		.name = "operating_mode",
		.show = get_operating_mode,
		.store = set_operating_mode,
	},
	{
		.name = "oversampling_ratio",
		.show = get_oversampling,
		.store = set_oversampling,
	},
#else
	{
		.name = "power_down_mode",
		.show = get_power_down_mode,
		.store = set_power_down_mode,
	},
	{
		.name = "dev_range",
		.show = get_range,
		.store = set_range,
	},
#endif
	{
		.name = "sampling_frequency",
		.show = get_sampling_frequency,
		.store = set_sampling_frequency,
	},

	END_ATTRIBUTES_ARRAY
};

/* IIOD debug attributes list */
static struct iio_attribute debug_attributes[] = {
#if defined(DEV_AD7606B) || defined(DEV_AD7606C_18) || defined(DEV_AD7606C_16)
	{
		.name = "direct_reg_access",
		.show = NULL,
		.store = NULL,
	},
#endif

	END_ATTRIBUTES_ARRAY
};

/* IIOD channels configurations */
struct scan_type chn_scan = {
	.sign = 's',
	.realbits = CHN_STORAGE_BITS,
	.storagebits = CHN_STORAGE_BITS,
	.shift = 0,
	.is_big_endian = false
};

static struct iio_channel iio_ad7606_channels[] = {
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
	}
};


/**
 * @brief	Init for reading/writing and parameterization of a
 * 			ad7606 IIO device
 * @param 	desc[in,out] - IIO device descriptor
 * @param	init[in] - Configuration structure
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
static int32_t iio_ad7606_init(struct iio_device **desc)
{
	struct iio_device *iio_ad7606_inst;

	iio_ad7606_inst = calloc(1, sizeof(struct iio_device));
	if (!iio_ad7606_inst) {
		return FAILURE;
	}

	iio_ad7606_inst->num_ch = sizeof(iio_ad7606_channels) / sizeof(
					  iio_ad7606_channels[0]);
	iio_ad7606_inst->channels = iio_ad7606_channels;
	iio_ad7606_inst->attributes = global_attributes;
	iio_ad7606_inst->debug_attributes = debug_attributes;

	iio_ad7606_inst->transfer_dev_to_mem = iio_ad7606_transfer_dev_data;
	iio_ad7606_inst->transfer_mem_to_dev = NULL;
	iio_ad7606_inst->read_data = iio_ad7606_read_data;
	iio_ad7606_inst->write_data = NULL;
	iio_ad7606_inst->prepare_transfer = iio_ad7606_start_transfer;
	iio_ad7606_inst->end_transfer = iio_ad7606_stop_transfer;
	iio_ad7606_inst->debug_reg_read = debug_reg_read;
	iio_ad7606_inst->debug_reg_write = debug_reg_write;

	*desc = iio_ad7606_inst;

	return SUCCESS;
}


/**
 * @brief Release resources allocated for IIO device
 * @param desc[in] - IIO device descriptor
 * @return SUCCESS in case of success, FAILURE otherwise
 */
static int32_t iio_ad7606_remove(struct iio_desc *desc)
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
 * @brief	Get scale factor for adc data to voltage conversion for IIO client
 * @param	chn_range[in] - Current channel voltage range
 * @param	chn_range_bits[in] - Channel range register bits
 * @return	scale
 * @details	This function samples and capture the new data when previous data
 * 			is transmitted to IIO client
 */
static float get_vltg_conv_scale_factor(float chn_range, polarity_e polarity)
{
	float scale;

	/* Get the scale factor for voltage conversion from range */
	if (polarity == UNIPOLAR) {
		scale = (chn_range / ADC_MAX_COUNT_UNIPOLAR) * 1000;
	} else {
		scale = (chn_range / ADC_MAX_COUNT_BIPOLAR) * 1000;
	}

	return scale;
}


/**
 * @brief 	Save local variables
 * @return	none
 * @details	This function saves the local parameters with updated device values
 */
static void save_local_attributes(void)
{
	char buf[50];
	struct iio_ch_info channel;

	for (uint8_t chn = 0; chn < AD7606X_ADC_CHANNELS; chn++) {
		channel.ch_num = chn;

		/* Get channel range */
		(void)get_chn_range(p_ad7606_dev_inst, buf, 0, &channel, 0);

		/* Get scale */
		attr_scale_val[channel.ch_num] = get_vltg_conv_scale_factor(
				attr_chn_range[channel.ch_num],
				attr_polarity_val[channel.ch_num]);
	}
}


/**
 * @brief	Initialize the IIO interface for AD7606 IIO device
 * @return	none
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t ad7606_iio_initialize(void)
{
	int32_t init_status;

	/* IIO device descriptor */
	struct iio_device *p_iio_ad7606_dev;

	/**
	* IIO interface init parameters
	*/
	struct iio_init_param iio_init_params = {
		.phy_type = USE_UART,
		{
			&uart_init_params
		}
	};

	/* Initialize AD7606 device and peripheral interface */
	init_status = ad7606_init(&p_ad7606_dev_inst, &ad7606_init_str);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Initialize the IIO interface */
	init_status = iio_init(&p_ad7606_iio_desc, &iio_init_params);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Initialize the AD7606 IIO application interface */
	init_status = iio_ad7606_init(&p_iio_ad7606_dev);
	if (init_status != SUCCESS) {
		return init_status;
	}

	/* Register AD7606 IIO interface */
	init_status = iio_register(p_ad7606_iio_desc,
				   p_iio_ad7606_dev,
				   (char *)dev_name,
				   p_ad7606_dev_inst,
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
 * @brief 	Run the AD7606 IIO event handler
 * @return	none
 * @details	This function monitors the new IIO client event
 */
void ad7606_iio_event_handler(void)
{
	while (1) {
		(void)iio_step(p_ad7606_iio_desc);
	}
}
