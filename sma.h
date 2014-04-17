/*
 * sma.h
 *
 *  Created on: Dec 5, 2013
 *      Author: kwgilpin
 */

#ifndef SMA_H_
#define SMA_H_

#include <stdint.h>
#include <stdbool.h>

#include "app_scheduler.h"

typedef enum {
	SMA_STATE_EXTENDED,
	SMA_STATE_RETRACTING,
	SMA_STATE_HOLDING,
	SMA_STATE_EXTENDING
} smaState_t;


bool sma_setRetractCurrent_mA(uint16_t current_mA);
uint16_t sma_getRetractCurrent_mA(void);
bool sma_setHoldCurrent_mA(uint16_t current_mA);
uint16_t sma_getHoldCurrent_mA(void);
bool sma_setRetractTime_ms(uint16_t time_ms);
uint16_t sma_getRetractTime_ms(void);

smaState_t sma_getState(void);

bool sma_retract(uint16_t holdTime_ms, app_sched_event_handler_t smaEventHandler);
bool sma_extend(app_sched_event_handler_t smaEventHandler);


#endif /* SMA_H_ */
