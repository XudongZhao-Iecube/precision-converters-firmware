/*************************************************************************//**
 *   @file   ad4170_user_config.c
 *   @brief  Default user configurations file for AD4170 device
******************************************************************************
* Copyright (c) 2021 Analog Devices, Inc.
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
#include "ad4170_user_config.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

#define	AD4170_OFFSET_RESET_VAL		0x0
#define AD4170_GAIN_RESET_VAL		0x555555

/* FIR coeffients */
#define AD4170_FIR_MODE			AD4170_FIR_DEFAULT		/* FIR mode */
#define AD4170_FIR_LENGTH		72						/* Number of programmable coefficients */
#define AD4170_COEF_SET			AD4170_FIR_COEFF_SET0	/* FIR Coefficient set (0/1) */

/* FIR coefficient values (max 72) */
static int32_t fir_coefficients[AD4170_FIR_LENGTH] = {
	-8, 28, 46, -46, -130, 44, 286, 30, -518, -256, 802,
		734, -1046, -1562, 1080, 2798, -646, -4404, -586, 6192, 2968,
		-7754, -6788, 8454, 12154, -7434, -18858, 3696, 26242, 3752,
		-33122, -15692, 37742, 32442, -37850, -53622, 30836, 77916, -13954
		-102932, -15458, 125088, 59824, - 139536, -121452, 139872, 203270,
		-117090, -311224, 55864, 462102, 80366, -717918, -437946, 1492796, 3494708
	};

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

static struct gpio_init_param gpio_init_ldac_n = {
	.number = DIG_AUX_2,
	.extra = NULL
};

static struct gpio_init_param gpio_init_rdy = {
	.number = DIG_AUX_1,
	.extra = NULL
};

static struct gpio_init_param gpio_init_sync_inb = {
	.number = SYNC_INB,
	.extra = NULL
};

/* Initialize the AD4170 device structure */
struct ad4170_init_param ad4170_user_config_params = {
	/* Note: Max supported SPI frequency can vary from one platform to other */
	.spi_init = {
		.max_speed_hz = 10000000,	// Max SPI Speed
		.chip_select = SPI_CSB,		// Chip Select
		.mode = SPI_MODE_3,			// CPOL = 1, CPHA = 1
		.extra = &spi_extra_init_params // SPI extra configurations
	},

	.spi_settings = {
		.short_instruction = false,		// 14-bit instruction mode to access full register range
		.crc_enabled = false,			// CRC Disabled for faster data access
		/* Use during 3-wire Isolated SPI mode (no CSB) -Not supported with firmware */
		.sync_loss_detect = false
	},

	.rdy_conv_timeout = 10000000,

	.config = {
		.pin_muxing =  {
			.chan_to_gpio = AD4170_CHANNEL_NOT_TO_GPIO,
			.dig_aux2_ctrl = AD4170_DIG_AUX2_LDAC,	// Used as h/w LDACB
			.dig_aux1_ctrl = AD4170_DIG_AUX1_RDY,	// Used as RDY (end of conversion)
			.sync_ctrl = AD4170_SYNC_STANDARD,		// sync_ctrl pin must be high (deasserted)
			.dig_out_str = AD4170_DIG_STR_DEFAULT,
			.sdo_rdby_dly = AD4170_SDO_RDY_SCLK
		},

		/* Note: MCLCK is default set to 16Mhz using below configs. Changing MCLK
		 * from default value can result into a failure of data capture through IIO client */
		.clock_ctrl = {
			.dclk_divide = AD4170_DCLKDIVBY1,
			.clockdiv = AD4170_CLKDIVBY1,
			.clocksel = AD4170_INTERNAL_OSC
		},

		.standby_ctrl = 0,	// All blocks inactive during standby
		.powerdown_sw = 0,
		.error_en = 0,

		.adc_ctrl = {
			.parallel_filt_en = false,
			.multi_data_reg_sel = true,		// Data register shared b/w all channels
			.cont_read_status_en = false,
			.cont_read = AD4170_CONT_READ_OFF,
			.mode = AD4170_CONT_CONV_MODE_CONFIG
		},

		/* Enabled all channels (all channels must be enabled to apply
		 * configurations on them such as setup, pin mapping, etc)
		 **/
		.channel_en = AD4170_CHANNEL(0) | AD4170_CHANNEL(1) | AD4170_CHANNEL(2) |
		AD4170_CHANNEL(3) | AD4170_CHANNEL(4) | AD4170_CHANNEL(5) |
		AD4170_CHANNEL(6) | AD4170_CHANNEL(7) | AD4170_CHANNEL(8) |
		AD4170_CHANNEL(9) | AD4170_CHANNEL(10) | AD4170_CHANNEL(11) |
		AD4170_CHANNEL(12) | AD4170_CHANNEL(13) | AD4170_CHANNEL(14) |
		AD4170_CHANNEL(15),

		/* Channel setup */
		.setup = {
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 },
			{ .repeat_n = 0, .delay_n = AD4170_DLY_0, .setup_n = 0 }
		},

		/* Channel configurations */
		.map = {
			{ . ainp = AD4170_AIN0, .ainm = AD4170_AIN1 },
			{ . ainp = AD4170_AIN2, .ainm = AD4170_AIN3 },
			{ . ainp = AD4170_AIN4, .ainm = AD4170_AIN5 },
			{ . ainp = AD4170_AIN6, .ainm = AD4170_AIN7 },
			{ . ainp = AD4170_AIN8, .ainm = AD4170_AIN9 },
			{ . ainp = AD4170_AIN10, .ainm = AD4170_AIN11 },
			{ . ainp = AD4170_AIN12, .ainm = AD4170_AIN13 },
			{ . ainp = AD4170_AIN14, .ainm = AD4170_AIN15 },
			{ . ainp = AD4170_TEMP_SENSOR_P, .ainm = AD4170_TEMP_SENSOR_N },
			{ . ainp = AD4170_REFIN1_P, .ainm = AD4170_REFIN1_N },
			{ . ainp = AD4170_REFIN2_P, .ainm = AD4170_REFIN2_N },
			{ . ainp = AD4170_AVDD_AVSS_P, .ainm = AD4170_AVDD_AVSS_N },
			{ . ainp = AD4170_ALDO, .ainm = AD4170_DGND },
			{ . ainp = AD4170_DLDO, .ainm = AD4170_DGND },
			{ . ainp = AD4170_DAC, .ainm = AD4170_DGND }
		},

		/* Setup configurations */
		.setups = {
			{
				.misc = {.chop_iexc = AD4170_CHOP_IEXC_OFF, .chop_adc = AD4170_CHOP_OFF, .burnout = AD4170_BURNOUT_OFF },
				.afe = {
					.ref_buf_m = AD4170_REF_BUF_PRE, .ref_buf_p = AD4170_REF_BUF_PRE,
					.ref_select = AD4170_REFIN_REFOUT, .bipolar = AD4170_BIPOLAR_MODE, .pga_gain = AD4170_PGA_CONFIG
				},
				.filter = {.post_filter_sel = AD4170_POST_FILTER_NONE, .filter_type = AD4170_FILT_SINC5_AVG},
				.filter_fs = AD4170_FS_CONFIG, .offset = AD4170_OFFSET_RESET_VAL, .gain = AD4170_GAIN_RESET_VAL,
			},
			{
				.misc = {.chop_iexc = AD4170_CHOP_IEXC_OFF, .chop_adc = AD4170_CHOP_OFF, .burnout = AD4170_BURNOUT_OFF },
				.afe = {
					.ref_buf_m = AD4170_REF_BUF_PRE, .ref_buf_p = AD4170_REF_BUF_PRE,
					.ref_select = AD4170_REFIN_REFOUT, .bipolar = AD4170_BIPOLAR_MODE, .pga_gain = AD4170_PGA_CONFIG
				},
				.filter = {.post_filter_sel = AD4170_POST_FILTER_NONE, .filter_type = AD4170_FILT_SINC5_AVG},
				.filter_fs = AD4170_FS_CONFIG, .offset = AD4170_OFFSET_RESET_VAL, .gain = AD4170_GAIN_RESET_VAL,
			},
			{
				.misc = {.chop_iexc = AD4170_CHOP_IEXC_OFF, .chop_adc = AD4170_CHOP_OFF, .burnout = AD4170_BURNOUT_OFF },
				.afe = {
					.ref_buf_m = AD4170_REF_BUF_PRE, .ref_buf_p = AD4170_REF_BUF_PRE,
					.ref_select = AD4170_REFIN_REFOUT, .bipolar = AD4170_BIPOLAR_MODE, .pga_gain = AD4170_PGA_CONFIG
				},
				.filter = {.post_filter_sel = AD4170_POST_FILTER_NONE, .filter_type = AD4170_FILT_SINC5_AVG},
				.filter_fs = AD4170_FS_CONFIG, .offset = AD4170_OFFSET_RESET_VAL, .gain = AD4170_GAIN_RESET_VAL,
			},
			{
				.misc = {.chop_iexc = AD4170_CHOP_IEXC_OFF, .chop_adc = AD4170_CHOP_OFF, .burnout = AD4170_BURNOUT_OFF },
				.afe = {
					.ref_buf_m = AD4170_REF_BUF_PRE, .ref_buf_p = AD4170_REF_BUF_PRE,
					.ref_select = AD4170_REFIN_REFOUT, .bipolar = AD4170_BIPOLAR_MODE, .pga_gain = AD4170_PGA_CONFIG
				},
				.filter_fs = AD4170_FS_CONFIG, .offset = AD4170_OFFSET_RESET_VAL, .gain = AD4170_GAIN_RESET_VAL,
			},
			{
				.misc = {.chop_iexc = AD4170_CHOP_IEXC_OFF, .chop_adc = AD4170_CHOP_OFF, .burnout = AD4170_BURNOUT_OFF },
				.afe = {
					.ref_buf_m = AD4170_REF_BUF_PRE, .ref_buf_p = AD4170_REF_BUF_PRE,
					.ref_select = AD4170_REFIN_REFOUT, .bipolar = AD4170_BIPOLAR_MODE, .pga_gain = AD4170_PGA_CONFIG
				},
				.filter = {.post_filter_sel = AD4170_POST_FILTER_NONE, .filter_type = AD4170_FILT_SINC5_AVG},
				.filter_fs = AD4170_FS_CONFIG, .offset = AD4170_OFFSET_RESET_VAL, .gain = AD4170_GAIN_RESET_VAL,
			},
			{
				.misc = {.chop_iexc = AD4170_CHOP_IEXC_OFF, .chop_adc = AD4170_CHOP_OFF, .burnout = AD4170_BURNOUT_OFF },
				.afe = {
					.ref_buf_m = AD4170_REF_BUF_PRE, .ref_buf_p = AD4170_REF_BUF_PRE,
					.ref_select = AD4170_REFIN_REFOUT, .bipolar = AD4170_BIPOLAR_MODE, .pga_gain = AD4170_PGA_CONFIG
				},
				.filter = {.post_filter_sel = AD4170_POST_FILTER_NONE, .filter_type = AD4170_FILT_SINC5_AVG},
				.filter_fs = AD4170_FS_CONFIG, .offset = AD4170_OFFSET_RESET_VAL, .gain = AD4170_GAIN_RESET_VAL,
			},
			{
				.misc = {.chop_iexc = AD4170_CHOP_IEXC_OFF, .chop_adc = AD4170_CHOP_OFF, .burnout = AD4170_BURNOUT_OFF },
				.afe = {
					.ref_buf_m = AD4170_REF_BUF_PRE, .ref_buf_p = AD4170_REF_BUF_PRE,
					.ref_select = AD4170_REFIN_REFOUT, .bipolar = AD4170_BIPOLAR_MODE, .pga_gain = AD4170_PGA_CONFIG
				},
				.filter = {.post_filter_sel = AD4170_POST_FILTER_NONE, .filter_type = AD4170_FILT_SINC5_AVG},
				.filter_fs = AD4170_FS_CONFIG, .offset = AD4170_OFFSET_RESET_VAL, .gain = AD4170_GAIN_RESET_VAL,
			},
			{
				.misc = {.chop_iexc = AD4170_CHOP_IEXC_OFF, .chop_adc = AD4170_CHOP_OFF, .burnout = AD4170_BURNOUT_OFF },
				.afe = {
					.ref_buf_m = AD4170_REF_BUF_PRE, .ref_buf_p = AD4170_REF_BUF_PRE,
					.ref_select = AD4170_REFIN_REFOUT, .bipolar = AD4170_BIPOLAR_MODE, .pga_gain = AD4170_PGA_CONFIG
				},
				.filter = {.post_filter_sel = AD4170_POST_FILTER_NONE, .filter_type = AD4170_FILT_SINC5_AVG},
				.filter_fs = AD4170_FS_CONFIG, .offset = AD4170_OFFSET_RESET_VAL, .gain = AD4170_GAIN_RESET_VAL,
			}
		},

		{ .ref_en = true },		// Enable internal reference
		.v_bias = 0,			// No Vbias enabled on any input
		.i_pullup = 0,			// No pull-up enabled on any input

		.current_source = {
			{ .i_out_pin = AD4170_I_OUT_AIN0, .i_out_val = AD4170_I_OUT_0UA },
			{ .i_out_pin = AD4170_I_OUT_AIN0, .i_out_val = AD4170_I_OUT_0UA },
			{ .i_out_pin = AD4170_I_OUT_AIN0, .i_out_val = AD4170_I_OUT_0UA },
			{ .i_out_pin = AD4170_I_OUT_AIN0, .i_out_val = AD4170_I_OUT_0UA }
		},
		.fir_control = {
			.fir_mode = AD4170_FIR_MODE,
			.coeff_set = AD4170_COEF_SET,
			.fir_length = AD4170_FIR_LENGTH,
			.fir_coefficients = fir_coefficients
		},
		.dac = {
			.enabled = false,
			.gain = AD4170_DAC_GAIN_1,
			.hw_toggle = false,
			.hw_ldac = false
		}
	},

	&gpio_init_sync_inb,
	&gpio_init_rdy,		// DIAG_AUX1
	&gpio_init_ldac_n	// DIAG_AUX2
};
