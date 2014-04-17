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

#include "app_scheduler.h"

bool bldc_init(void);
void bldc_deinit(void);

bool bldc_setMaxCurrent_mA(uint16_t mA);

bool bldc_setSpeed(uint16_t speed_rpm, bool reverse, bool brake, app_sched_event_handler_t bldcEventHandler);
bool bldc_setAccel(uint16_t accel_mA, uint16_t time_ms, bool reverse, app_sched_event_handler_t bldcEventHandler);

void bldc_setReverseDirections(bool reverse);
bool bldc_getReverseDirections(void);

bool bldc_setKP(int32_t numerator, int32_t denominator);
bool bldc_getKP(int32_t *numerator, int32_t *denominator);
bool bldc_setKI(int32_t numerator, int32_t denominator);
bool bldc_getKI(int32_t *numerator, int32_t *denominator);
bool bldc_isStable(void);

#endif /* BLDC_H_ */
