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

#define LED_STATE_IS_VALID(s)	((s == LED_STATE_OFF) || (s == LED_STATE_ON) || \
								 (s == LED_STATE_SLOW_FLASH) || (s == LED_STATE_FAST_FLASH) || \
								 (s == LED_STATE_SINGLE_BLINK) || (s == LED_STATE_DOUBLE_BLINK))

void led_init(void);
void led_deinit(void);
void led_setAllOff(void);
void led_setAllOn(void);
bool led_setState(uint8_t led, led_state_t state);
led_state_t led_getState(uint8_t led);
uint32_t led_getDutyCycle_percent(uint8_t led);

#endif /* LED_H_ */
