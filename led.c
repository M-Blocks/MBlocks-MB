/*
 * led.c
 *
 *  Created on: Jan 29, 2014
 *      Author: kwgilpin
 */

#include "nrf_gpio.h"

#include "pins.h"
#include "led.h"

#define COUNT_PERIOD	20

static uint8_t led_pin_number[LED_COUNT] = {LED_RED_PIN_NO, LED_GREEN_PIN_NO, LED_BLUE_PIN_NO};
static led_state_t led_state[LED_COUNT];

void led_init() {
	uint32_t led;

	for (led=0; led<LED_COUNT; led++) {
		GPIO_LED_CONFIG(led_pin_number[led]);
		nrf_gpio_pin_clear(led_pin_number[led]);
		led_state[led] = LED_STATE_OFF;
	}
}

void led_setAllOff() {
	uint32_t i;

	for (i=0; i<LED_COUNT; i++) {
		led_state[i] = LED_STATE_OFF;
	}
}

void led_setAllOn() {
	uint32_t i;

	for (i=0; i<LED_COUNT; i++) {
		led_state[i] = LED_STATE_ON;
	}
}

void led_setState(uint8_t led, led_state_t state) {
	if (led < LED_COUNT) {
		led_state[led] = state;
	}
}

void led_timer_handler(void *p_context) {
	static uint32_t count = 0;
	uint32_t adjCount;
	uint32_t led;

	count = (count + 1) % COUNT_PERIOD;

	for (led=0; led<LED_COUNT; led++) {
		adjCount = (count + 3*led) % COUNT_PERIOD;

		if (led_state[led] == LED_STATE_OFF) {
			nrf_gpio_pin_clear(led_pin_number[led]);
		} else if (led_state[led] == LED_STATE_ON) {
			nrf_gpio_pin_set(led_pin_number[led]);
		} else if (led_state[led] == LED_STATE_SLOW_FLASH) {
			if ((adjCount >> 1) < (COUNT_PERIOD / 4)) {
				nrf_gpio_pin_set(led_pin_number[led]);
			} else {
				nrf_gpio_pin_clear(led_pin_number[led]);
			}
		} else if (led_state[led] == LED_STATE_FAST_FLASH) {
			if (adjCount >> 1 & 0x01) {
				nrf_gpio_pin_set(led_pin_number[led]);
			} else {
				nrf_gpio_pin_clear(led_pin_number[led]);
			}
		} else if (led_state[led] == LED_STATE_SINGLE_BLINK) {
			if (adjCount % (COUNT_PERIOD / 2) == 0) {
				nrf_gpio_pin_set(led_pin_number[led]);
			} else {
				nrf_gpio_pin_clear(led_pin_number[led]);
			}
		} else if (led_state[led] == LED_STATE_DOUBLE_BLINK) {
			if ((adjCount % (COUNT_PERIOD / 2) == 0) || (adjCount % (COUNT_PERIOD / 2) == 2)) {
				nrf_gpio_pin_set(led_pin_number[led]);
			} else {
				nrf_gpio_pin_clear(led_pin_number[led]);
			}
		}
	}
}


