/*
 * led.c
 *
 *  Created on: Jan 29, 2014
 *      Author: kwgilpin, sclaici
 */

#include "nrf_gpio.h"
#include "nrf_assert.h"


#include "app_timer.h"

#include "global.h"
#include "pins.h"
#include "led.h"

#define COUNT_PERIOD	20

APP_TIMER_DEF(led_timer_id); 

static uint8_t led_pin_number[LED_COUNT] = {LED_RED_PIN_NO, LED_GREEN_PIN_NO, LED_BLUE_PIN_NO};
static led_state_t led_state[LED_COUNT];
static bool initialized = false;

static void led_timer_handler(void *p_context);

void led_init() {
	uint32_t err_code;

    /* Create a timer which will be used to flash the LEDs */
	err_code = app_timer_create(&led_timer_id, APP_TIMER_MODE_REPEATED, led_timer_handler);
	APP_ERROR_CHECK(err_code);
	/* Start the timer which controls the LEDs */
	err_code = app_timer_start(led_timer_id, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

	initialized = true;
}

void led_deinit(void) {
	uint32_t err_code;
	uint32_t led;

	if (!initialized) {
		return;
	}

	err_code = app_timer_stop(led_timer_id);
	APP_ERROR_CHECK(err_code);
		
	for (led=0; led<LED_COUNT; led++) {
		led_state[led] = LED_STATE_OFF;
		nrf_gpio_pin_clear(led_pin_number[led]);
	}

	initialized = false;
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

bool led_setState(uint8_t led, led_state_t state) {
	if ((led < LED_COUNT) && (LED_STATE_IS_VALID(state))) {
		led_state[led] = state;
		return true;
	}

	return false;
}

led_state_t led_getState(uint8_t led) {
	if (led < LED_COUNT) {
		return led_state[led];
	}

	return LED_STATE_OFF;
}


uint32_t led_getDutyCycle_percent(uint8_t led) {
	if (led < LED_COUNT) {
		if (led_state[led] == LED_STATE_OFF) {
			return 0;
		} else if (led_state[led] == LED_STATE_ON) {
			return 100;
		} else if (led_state[led] == LED_STATE_SLOW_FLASH) {
			return 50;
		} else if (led_state[led] == LED_STATE_FAST_FLASH) {
			return 50;
		} else if (led_state[led] == LED_STATE_SINGLE_BLINK) {
			return 10;
		} else if (led_state[led] == LED_STATE_DOUBLE_BLINK) {
			return 20;
		} else {
			return 0;
		}
	}

	return 0;
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
		} else {
			/* If the state is unknown, set the state to OFF */
			led_state[led] = LED_STATE_OFF;
			nrf_gpio_pin_clear(led_pin_number[led]);
		}
	}
}


