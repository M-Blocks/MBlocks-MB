/**@file led.c
 * @brief LED handler.
 *
 * @author Kyle W. Gilpin
 * @author Sebastian Claici
 * @bug No known bugs.
 */

#include "app_timer.h"
#include "app_error.h"

#include "nrf_gpio.h"

#include "global.h"
#include "pins.h"
#include "led.h"

static app_timer_id_t m_led_timer_id;

static uint8_t led_pin_number[LED_COUNT] = {LED_RED_PIN_NO, LED_GREEN_PIN_NO, LED_BLUE_PIN_NO};
static led_state_t led_state[LED_COUNT];


/**@brief Function for handling the LED timeout
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information from the
 *                          app_start_timer() call to the timeout handler
 */
static void led_timeout_handler(void *p_context) {
    static int cycle_count = 0;
    
    for (int i = 0; i < LED_COUNT; ++i) {
	switch (led_state[i]) {
	case LED_STATE_OFF:
	    nrf_gpio_pin_clear(led_pin_number[i]);
	    break;
	case LED_STATE_ON:
	    nrf_gpio_pin_set(led_pin_number[i]);
	    break;
	case LED_STATE_SLOW_FLASH:
	    if (cycle_count % 3 == 0) {
		nrf_gpio_pin_set(led_pin_number[i]);
	    } else if (cycle_count % 3 == 1) {
		nrf_gpio_pin_clear(led_pin_number[i]);
	    }
	    break;
	default:
	    break;
	}
    }
    cycle_count = cycle_count + 1;
}


void led_timers_start(void) {
    uint32_t err_code;
    err_code = app_timer_create(&m_led_timer_id,
				APP_TIMER_MODE_REPEATED,
				led_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_led_timer_id, LED_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


void led_timers_stop(void) {
    app_timer_stop(m_led_timer_id);
}


void led_set_state(led_t led_number, led_state_t state) {
    if (led_number >= LED_COUNT || state >= LED_STATE_NUM_TYPES) {
	return;
    }
    led_state[led_number] = state;
}


void led_set_all_on() {
    for (int i = 0; i < LED_COUNT; ++i) {
	led_state[i] = LED_STATE_ON;
    }
}


void led_set_all_off() {
    for (int i = 0; i < LED_COUNT; ++i) {
	led_state[i] = LED_STATE_OFF;
    }
}
