/***************************************************************************//**
* @file   ad3552r_iio.h
* @brief  Header file for AD3552R IIO interface
********************************************************************************
* Copyright (c) 2021 Analog Devices, Inc.
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*******************************************************************************/
#ifndef _AD3552R_IIO_H_
#define _AD3552R_IIO_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio.h"
#include "iio_types.h"
#include "ad3552r.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

/* AD3552R global device instance for accessing device specific APIs */
extern struct ad3552r_desc *p_ad3552r_dev_inst;

/* Init the IIO interface */
int32_t ad3552r_iio_initialize(void);

/* Run the IIO event handler */
void ad3552r_iio_event_handler(void);

#endif /* _AD3552R_IIO_H_ */
