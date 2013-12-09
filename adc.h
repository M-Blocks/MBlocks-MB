/*
 * adc.h
 *
 *  Created on: Oct 18, 2013
 *      Author: Kyle Gilpin
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

uint16_t adc_read_mV(uint8_t channel);
uint16_t adc_avg_mV(uint8_t channel, uint8_t nsamples);


#endif /* ADC_H_ */
