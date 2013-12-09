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

typedef enum {
	SMA_STATE_EXTENDED,
	SMA_STATE_RETRACTING,
	SMA_STATE_RETRACTED,
	SMA_STATE_EXTENDING
} smaState_t;

bool sma_init(void);
bool sma_retractTime_ms(uint16_t time_ms);
bool sma_retractTimeCurrent_ms_mA(uint16_t time_ms, uint16_t current_mA);
bool sma_extend(void);
smaState_t sma_getState(void);
bool sma_setCurrent_mA(uint32_t current_mA);

#endif /* SMA_H_ */
