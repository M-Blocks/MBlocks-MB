/*
 * led.h
 *
 *  Created on: Jan 29, 2014
 *      Author: kwgilpin
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>
#include <stdbool.h>

#include "app_timer.h"

#define LED_COUNT	3

#define LED_RED		0
#define LED_GREEN	1
#define LED_BLUE	2

typedef enum {
	LED_STATE_OFF = 0,
	LED_STATE_ON,
	LED_STATE_SLOW_FLASH,
	LED_STATE_FAST_FLASH,
	LED_STATE_SINGLE_BLINK,
	LED_STATE_DOUBLE_BLINK
} led_state_t;

extern app_timer_id_t led_timer_id;

void led_init(void);
void led_setAllOff(void);
void led_setAllOn(void);
void led_setState(uint8_t led, led_state_t state);
void led_timer_handler(void *p_context);

#endif /* LED_H_ */
