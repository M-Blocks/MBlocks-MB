/*
 * adc.c
 *
 *  Created on: Oct 18, 2013
 *      Author: Kyle Gilpin
 */


#include <stdint.h>
#include <stdlib.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"

#include "app_error.h"
#include "app_util.h"

#include "global.h"
#include "pins.h"

#include "adc.h"

bool adc_init() {
	/* Make the VINSENSE, FRAMETEMP, LIPROVBATOUT, and ICHARGE pins inputs with
	 * their input buffers disabled so that they do not consume current when
	 * the pins are at a voltage somewhere between 0 and VCC. */

    NRF_GPIO->PIN_CNF[VINSENSE_PIN_NO] =
    		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
    		(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
    		(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
    		(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
    		(GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[LIPROVBATOUT_PIN_NO] =
    		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
    		(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
    		(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
    		(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
    		(GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[ICHARGE_PIN_NO] =
    		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
    		(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
    		(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
    		(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
    		(GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

	return true;
}

void adc_deinit() {
	/* Nothing to do here */
	;
}

uint16_t adc_read_mV(uint8_t channel) {
    uint32_t raw, mV;

    /* If the channel is invalid, return now */
    if (channel >= 8) {
    	return 0;
    }

    /* Disable the ADC before changing its configuration */
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled << ADC_ENABLE_ENABLE_Pos;

    /* Disable ADC interrupt */
    NRF_ADC->INTENCLR = ADC_INTENSET_END_Msk;

    /* Configure the ADC parameters */
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) | /* 10-bit result */
    		(ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) | /* Scale positive input by 1/3 */
    		(ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) | /* Use 1.2V bandgap reference voltage */
    		((0x01 << channel) << ADC_CONFIG_PSEL_Pos) | /* Select the specified channel */
    		(ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos); /* Disable external reference pins */


    /* Clear the conversion complete flag */
    NRF_ADC->EVENTS_END = 0;

    /* Enable the ADC */
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos;

    /* Start the conversion */
    NRF_ADC->TASKS_START = 1;

    /* Wait for the conversion to complete */
    while (NRF_ADC->BUSY);

    /* The actual voltage is found by
     * 1) Multiplying the raw ADC value by 3 to account for the input being
     *    divided by 3 before conversion
     * 2) Multiplying by the 1.2V bandgap reference voltage, now expressed in
     *    millivolts
     * 3) Dividing by the resolution of the conversion expressed as the maximum
     *    number of counts.
     */
    raw = NRF_ADC->RESULT;
    mV = (3 * raw * 1200) / 1023;

    /* Explicitly stop the ADC to lower current consumption.  This is a work-
     * around for PAN 028 rev 1.5m anomaly 1. */
    NRF_ADC->TASKS_STOP = 1;

    /* Disable the ADC to reduce power consumption */
    NRF_ADC->ENABLE = (ADC_ENABLE_ENABLE_Disabled << ADC_ENABLE_ENABLE_Pos);

    return mV;
}

uint16_t adc_avg_mV(uint8_t channel, uint8_t nsamples) {
	uint32_t accum = 0;
	int i;

	for (i=0; i<nsamples; i++) {
		accum += adc_read_mV(channel);
	}

	return (uint16_t)(accum / nsamples);
}
