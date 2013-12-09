/*
 * bldc.h
 *
 *  Created on: Nov 25, 2013
 *      Author: kwgilpin
 */

#ifndef BLDC_H_
#define BLDC_H_

#include <stdint.h>
#include <stdbool.h>

bool bldc_init(void);
bool bldc_on(void);
bool bldc_off(void);
bool bldc_isOn(void);
bool bldc_config(void);
bool bldc_run(bool reverse);
bool bldc_stop(bool brake);
bool bldc_setMaxCurrent_mA(uint16_t mA);
bool bldc_setSpeed(uint16_t speed_rpm, bool reverse);
bool bldc_setKP(int32_t numerator, int32_t denominator);
bool bldc_getKP(int32_t *numerator, int32_t *denominator);
bool bldc_setKI(int32_t numerator, int32_t denominator);
bool bldc_getKI(int32_t *numerator, int32_t *denominator);
bool bldc_isStable(void);

#endif /* BLDC_H_ */
