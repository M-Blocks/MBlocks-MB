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

#include "adc.h"

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
