/**@file led.h
 * @brief Function prototypes for LED handler.
 *
 * @author Kyle W. Gilpin
 * @author Sebastian Claici
 * @bug No known bugs.
 */
#ifndef LED_HEADER_
#define LED_HEADER_

#define LED_MEAS_INTERVAL       APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

typedef enum {
    LED_STATE_OFF = 0,       /**< LED is off. */
    LED_STATE_ON,            /**< LED is on. */
    LED_STATE_SLOW_FLASH,    /**< LED flashes once every three cycles. */
    LED_STATE_NUM_TYPES
} led_state_t;

typedef enum {
    LED_RED = 0,
    LED_GREEN,
    LED_BLUE,
    LED_COUNT
} led_t;

/**@brief Function for starting the LED timers.
 */
void led_timers_start(void);

/**@brief Function for stopping the LED timers.
 */
void led_timers_stop(void);

/**@brief Function for setting the state of a LED.
 */
void led_set_state(led_t led_number, led_state_t led_state);

/**@brief Function for setting all LEDs on.
 */
void led_set_all_on(void);

/**@brief Function for setting all LEDs off.
 */
void led_set_all_off(void);

#endif
