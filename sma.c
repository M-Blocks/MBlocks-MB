/*
 * sma.c
 *
 *  Created on: Dec 5, 2013
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nordic_common.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_timer.h"

#include "global.h"
#include "pins.h"
#include "util.h"
#include "power.h"
#include "motionEvent.h"
#include "sma.h"

#define SMA_DEBUG	0

#define MAX_CURRENT_MA					1515

#define RETRACT_CURRENT_DEFAULT_MA		1200
#define HOLD_CURRENT_DEFAULT_MA			700

#define RETRACT_TIME_DEFAULT_MS			2000
#define RETRACT_TIME_MAX_MS				2500
#define	HOLD_TIME_MAX_MS				8000
#define RECOVERY_TIME_MS				3000

#define TIMER_PERIOD_MS					50

static bool initialized = false;
static smaState_t smaState;

static uint32_t retractStartTime_rtcTicks = 0;

static uint32_t retractCurrent_mA = RETRACT_CURRENT_DEFAULT_MA;
static uint32_t holdCurrent_mA = HOLD_CURRENT_DEFAULT_MA;

static uint32_t retractPWMOnTime_ms;
static uint32_t retractPWMOffTime_ms;
static uint32_t holdPWMOnTime_ms;
static uint32_t holdPWMOffTime_ms;

static uint32_t retractTime_ms = RETRACT_TIME_DEFAULT_MS;
static uint32_t holdTime_ms;

static app_sched_event_handler_t eventHandler;


static bool sma_init(void);
static void sma_deinit(void);

static app_timer_id_t sma_timerID = TIMER_NULL;
static void sma_timerHandler(void *p_context);

bool sma_init() {
	uint32_t err_code;

	if (initialized) {
		return true;
	}

	nrf_gpio_pin_clear(SMAIREF_PIN_NO);
    GPIO_PIN_CONFIG((SMAIREF_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

	if (sma_timerID == TIMER_NULL) {
		/* If the timer has not yet been created, create it now. At the moment,
		 * the SDK does not provide a way to destroy a timer, so we set the
		 * initial value of sma_timerID to TIMER_NULL in order to mark it
		 * invalid. */
		err_code = app_timer_create(&sma_timerID, APP_TIMER_MODE_SINGLE_SHOT, sma_timerHandler);
		APP_ERROR_CHECK(err_code);
	}

    smaState = SMA_STATE_EXTENDED;

    initialized = true;

	return true;
}

void sma_deinit() {
	if (!initialized) {
		return;
	}

	/* If the SMA timer exists, attempt to stop it.  If it is already stopped,
	 * there is no harm done. */
	if (sma_timerID != TIMER_NULL) {
		app_timer_stop(sma_timerID);
	}

	/* Disable the SMA controller */
	nrf_gpio_pin_clear(SMAIREF_PIN_NO);

	smaState = SMA_STATE_EXTENDED;

	initialized = false;
}

bool sma_setRetractCurrent_mA(uint16_t current_mA) {
	if (current_mA > MAX_CURRENT_MA) {
		return false;
	}

	retractCurrent_mA = current_mA;
	return true;
}

uint16_t sma_getRetractCurrent_mA() {
	return retractCurrent_mA;
}

bool sma_setHoldCurrent_mA(uint16_t current_mA) {
	if (current_mA > retractCurrent_mA) {
		return false;
	}

	holdCurrent_mA = current_mA;
	return true;
}

uint16_t sma_getHoldCurrent_mA() {
	return holdCurrent_mA;
}

bool sma_setRetractTime_ms(uint16_t time_ms) {
	if (retractTime_ms > RETRACT_TIME_MAX_MS) {
		return false;
	}

	retractTime_ms = time_ms;
	return true;
}

uint16_t sma_getRetractTime_ms() {
	return retractTime_ms;
}

smaState_t sma_getState(void) {
	return smaState;
}

bool sma_retract(uint16_t hold_ms, app_sched_event_handler_t smaEventHandler){
	uint32_t err_code;

	if (hold_ms > HOLD_TIME_MAX_MS) {
		return false;
	}

	if (smaState != SMA_STATE_EXTENDED) {
		return false;
	}

	/* We take care of automatically initializing the SMA controller when a
	 * caller attempts to use it. */
	sma_init();

	/* Supply power to the SMA circuitry */
	power_setVBATSWState(VBATSW_USER_SMA, true);

	/* Save the hold time and the event handler pointer. */
	holdTime_ms = hold_ms;
	eventHandler = smaEventHandler;

	/* Calculate the PWM on-times while both retracting and holding the SMA */
	retractPWMOnTime_ms = (retractCurrent_mA * TIMER_PERIOD_MS) / MAX_CURRENT_MA;
	retractPWMOffTime_ms = TIMER_PERIOD_MS - retractPWMOnTime_ms;

	holdPWMOnTime_ms = (holdCurrent_mA * TIMER_PERIOD_MS) / MAX_CURRENT_MA;
	holdPWMOffTime_ms = TIMER_PERIOD_MS - holdPWMOnTime_ms;

	smaState = SMA_STATE_RETRACTING;

	/* Record the time at which we start retracting the SMA */
	app_timer_cnt_get(&retractStartTime_rtcTicks);

	/* Start the timer which for the on-portion of the PWM period.  When it
	 * expired, we will turn the SMA off. */
	err_code = app_timer_start(sma_timerID,
			APP_TIMER_TICKS(retractPWMOnTime_ms, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

	/* Turn the SMA on */
	nrf_gpio_pin_set(SMAIREF_PIN_NO);

	return true;
}


bool sma_extend(app_sched_event_handler_t smaEventHandler) {
	uint32_t err_code;

	/* Save the event handler function pointer. */
	eventHandler = smaEventHandler;

	if (smaState == SMA_STATE_EXTENDED) {
		if (eventHandler != NULL) {
			app_sched_event_put((void *)&smaState, sizeof(smaState), eventHandler);
		}
		return true;
	} else if (smaState == SMA_STATE_EXTENDING) {
		/* If the SMA is already extending, we do not need to reset the
		 * recovery timer or do anything else before returning true because
		 * presumably everything is already setup correctly. */
		return true;
	}

	/* If we reach this point, the SMA must have been retracting or holding. */

	/* Update our state to reflect that we are now allowing the SMA to
	 * recover/extend. */
	smaState = SMA_STATE_EXTENDING;

	/* Until we stop the time, issuing a new start command will have no
	 * effect. */
	err_code = app_timer_stop(sma_timerID);
	APP_ERROR_CHECK(err_code);


	/* Re-start the timer after configuring it to expire after the SMA's
	 * recovery time. */
	err_code = app_timer_start(sma_timerID,
			APP_TIMER_TICKS(RECOVERY_TIME_MS, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

	/* Disable the SMA controller thereby allowing the SMA to cool down and
	 * extend to its resting length. */
	nrf_gpio_pin_clear(SMAIREF_PIN_NO);

	return true;
}

void sma_timerHandler(void *p_context) {
	UNUSED_PARAMETER(p_context);

	uint32_t err_code;
	uint32_t rtcTicks;
	uint32_t elapsedTime_ms;
	uint32_t pwmOnTime_ms, pwmOffTime_ms;
	motionPrimitive_t motionPrimitive;

	/* Calculate the time in milliseconds (by dividing RTC ticks by 32.378)
	 * since we began retracting the SMA. We mask the high-order byte of the
	 * subtraction result because app_timer_cnt_get only returns a 24-bit
	 * result. */
	app_timer_cnt_get(&rtcTicks);
	elapsedTime_ms = ((0x00FFFFFF & (rtcTicks - retractStartTime_rtcTicks)) * USEC_PER_APP_TIMER_TICK) / 1000;


	if ((smaState == SMA_STATE_RETRACTING) || (smaState == SMA_STATE_HOLDING)) {
		if (elapsedTime_ms >= retractTime_ms + holdTime_ms) {
			/* If we were holding but now the hold time has expired, we
			 * update our state and then queue an event that will be processed
			 * by the app_sched_execute() function in main() loop. */
			if (smaState == SMA_STATE_HOLDING) {
				smaState = SMA_STATE_EXTENDING;
				if (eventHandler != NULL) {
					motionPrimitive = MOTION_PRIMITIVE_SMA_EXTENDING;
					app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
				}
			}

			/* Restart the timer so that it will expire after the recovery
			 * period. */
			err_code = app_timer_start(sma_timerID,
					APP_TIMER_TICKS(RECOVERY_TIME_MS, APP_TIMER_PRESCALER), NULL);
			APP_ERROR_CHECK(err_code);

			/* Stop driving current through the SMA */
			nrf_gpio_pin_clear(SMAIREF_PIN_NO);
			return;
		} else if (elapsedTime_ms > retractTime_ms) {
			/* If we were retracting but now the retract time has expired, we
			 * update our state and then queue an event that will be processed
			 * by the app_sched_execute() function in main() loop. */
			if (smaState == SMA_STATE_RETRACTING) {
				smaState = SMA_STATE_HOLDING;
				if (eventHandler != NULL) {
					motionPrimitive = MOTION_PRIMITIVE_SMA_RETRACTED;
					app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
				}
			}

			/* Switch to using the holding duty cycle to PWM the SMA */
			pwmOnTime_ms = holdPWMOnTime_ms;
			pwmOffTime_ms = holdPWMOffTime_ms;
		} else {
			/* If are still retracting, continue to use the retracting duty
			 * cycle to PWM the SMA. */
			pwmOnTime_ms = retractPWMOnTime_ms;
			pwmOffTime_ms = retractPWMOffTime_ms;
		}

		/* If we reach this point, we know that we need to continue, PWM-ing
		 * the SMA. */

		if (nrf_gpio_pin_read(SMAIREF_PIN_NO)) {
			/* If the PWM output is currently high, we just finished the on-
			 * interval, so now we need to disable the SMA driver for the
			 * remainder of this period. */
			if (pwmOffTime_ms > 0) {
				/* If the PWM duty cycle is not 100%, we set the PWM timer to
				 * expire after the off-interval. */
				err_code = app_timer_start(sma_timerID,
						APP_TIMER_TICKS(pwmOffTime_ms, APP_TIMER_PRESCALER), NULL);
				APP_ERROR_CHECK(err_code);

				/* Finally, we turn off the SMA controller */
				nrf_gpio_pin_clear(SMAIREF_PIN_NO);
			} else {
				/* Otherwise, the PWM duty cycle is 100%, so we restart the
				 * timer with the same on-time as the interval that just
				 * expired (which is the same as the overall PWM period) and
				 * we leave the SMA on. */
				err_code = app_timer_start(sma_timerID,
						APP_TIMER_TICKS(pwmOnTime_ms, APP_TIMER_PRESCALER), NULL);
				APP_ERROR_CHECK(err_code);
			}
		} else {
			/* If the SMA is currently off, we are starting a new PWM
			 * period so we need to turn the SMA on for the calculated
			 * on-interval. */
			err_code = app_timer_start(sma_timerID,
					APP_TIMER_TICKS(pwmOnTime_ms, APP_TIMER_PRESCALER), NULL);
			APP_ERROR_CHECK(err_code);

			nrf_gpio_pin_set(SMAIREF_PIN_NO);
		}
	} else if (smaState == SMA_STATE_EXTENDING) {
		/* When setting the SMA state to EXTENDING above, we configured the
		 * timer to expire when the recovery time had elapsed.  Presumably,
		 * that has brought us here, which means that the SMA is fully
		 * extended. Since the SMA is already off, all we need to do is to
		 * update the state of the controller. */
		smaState = SMA_STATE_EXTENDED;
		/* Just to be safe, we also stop the timer. */
		app_timer_stop(sma_timerID);

		/* Withdraw our request to keep the VBATSW supply powered.  If nothing
		 * else is using the VBASTSW supply, this will turn it off.  */
		power_setVBATSWState(VBATSW_USER_SMA, false);

		/* Now that the SMA has completely extended, we disable the SMA
		 * controller.  It will automatically be re-initializing the next
		 * time the sma_retract() function is called.  */
		sma_deinit();

		/* If the caller has setup a callback to be executed once the SMA is
		 * fully extended, we execute it here. */
		if (eventHandler != NULL) {
			motionPrimitive = MOTION_PRIMITIVE_SMA_EXTENDED;
			app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
		}
	}

#if (SMA_DEBUG)
	if (smaState == SMA_STATE_EXTENDED) {
		app_uart_put_string("SMA EXTENDED\r\n");
	} else if (smaState == SMA_STATE_RETRACTING) {
		app_uart_put_string("SMA RETRACTING\r\n");
	} else if (smaState == SMA_STATE_HOLDING) {
		app_uart_put_string("SMA HOLDING\r\n");
	} else if (smaState == SMA_STATE_EXTENDING) {
		app_uart_put_string("SMA EXTENDING\r\n");
	} else {
		app_uart_put_string("SMA STATE UNKNOWN\r\n");
	}
#endif

}


