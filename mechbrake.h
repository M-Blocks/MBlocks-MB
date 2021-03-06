/*
 * mechbrake.h
 *
 *  Created on: Feb 9, 2014
 *      Author: kwgilpin
 */

#ifndef MECHBRAKE_H_
#define MECHBRAKE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	int16_t current_mA;
	uint16_t time_ms;
} coilCurrentStep_t;


bool mechbrake_init(void);
void mechbrake_deinit(void);
bool mechbrake_actuate(uint8_t stepCount, const coilCurrentStep_t *steps, app_sched_event_handler_t brakeCompleteEventHandler);
void mechbrake_setReverseDirections(bool reverse);
bool mechbrake_getReverseDirections(void);

#endif /* MECHBRAKE_H_ */
