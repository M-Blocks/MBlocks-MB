/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef TWI_MASTER_CONFIG
#define TWI_MASTER_CONFIG

#include "nrf51.h"
#include "pins.h"

#define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER	(I2C_SCL_PIN_NO)
#define TWI_MASTER_CONFIG_DATA_PIN_NUMBER	(I2C_SDA_PIN_NO)

#define	TWI_MASTER_PPI_CHANNEL				(6)
#define TWI_MASTER_PPI_CHENSET_MASK			PPI_CHENSET_CH6_Msk
#define TWI_MASTER_PPI_CHENCLR_MASK			PPI_CHENCLR_CH6_Msk
#define TWI_MASTER_PPI_CHEN_MASK			PPI_CHEN_CH6_Msk

#endif
