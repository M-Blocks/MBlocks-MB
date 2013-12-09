/*
 * pwm.h
 *
 *  Created on: Nov 18, 2013
 *      Author: kwgilpin
 */

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <stdbool.h>

#define PRECHRGEN_PWM_CHNL	0
#define SMAIREF_PWM_CHNL	1
#define BLDCIREF_PWM_CHNL	2

void pwm_init(void);
bool pwm_setOnPeriod(unsigned int channel, uint32_t onPeriod);
uint32_t pwm_getOnPeriod(unsigned int channel);
uint32_t pwm_getPeriod(void);

#endif /* PWM_H_ */
