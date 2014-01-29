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
#include "app_timer.h"

#include "global.h"
#include "pins.h"
#include "util.h"
#include "adc.h"
#include "pwm.h"
#include "sma.h"

#define SMA_DEBUG	1

#define SMA_MAX_TIME_MS			10000
#define SMA_TIMER_PERIOD_MS		50
#define SMA_EXTENSION_TIME_MS	3000
#define SMA_COOLING_TIME_MS		10000

#define RETRACTED_VOLTAGE_CHANGE_PCT	20

static bool initialized = false;
static bool firstRetraction = true;
static smaState_t smaState;
static uint16_t maxTime_ms;
static uint32_t referenceVoltage_mV;
static uint32_t retractStartTime_rtcTicks = 0;
static uint32_t retractStopTime_rtcTicks = 0;

static app_timer_id_t sma_timerID;

static void sma_timerHandler(void *p_context);

bool sma_init() {
	uint32_t err_code;

    err_code = app_timer_create(&sma_timerID, APP_TIMER_MODE_REPEATED, sma_timerHandler);
    APP_ERROR_CHECK(err_code);

	initialized = true;

	return true;
}

bool sma_retractTime_ms(uint16_t time_ms) {
	return sma_retractTimeCurrent_ms_mA(time_ms, 500);
}

bool sma_retractTimeCurrent_ms_mA(uint16_t time_ms, uint16_t current_mA) {
	uint32_t err_code;
	uint32_t offTime_ms;

	if (!initialized) {
		return false;
	}

	if ((smaState == SMA_STATE_RETRACTED) || (smaState == SMA_STATE_RETRACTING)) {
		/* If the SMA is already retracting or is fully retracted, there is
		 * nothing to do here.  We still return true because nothing went
		 * wrong. */
		return true;
	}

	if (maxTime_ms > SMA_MAX_TIME_MS) {
		maxTime_ms = SMA_MAX_TIME_MS;
	}

	/* Record the maximum time that the SMA should be actuated */
	maxTime_ms = time_ms;

	/* Record the time (expressed as ticks of the 32.768kHz RTC) at which we
	 * started to apply current to the SMA. */
	app_timer_cnt_get(&retractStartTime_rtcTicks);

	/* Calculate the time in milliseconds (by dividing RTC ticks by 32.378)
	 * the SMA was last energized. We mask the high-order byte of the
	 * subtraction result because app_timer_cnt_get only returns a 24-bit
	 * result. */
	offTime_ms = (0x00FFFFFF & (retractStartTime_rtcTicks - retractStopTime_rtcTicks)) / 33;

	/* Apply current to start the retraction */
	sma_setCurrent_mA(current_mA);
	smaState = SMA_STATE_RETRACTING;

	/* Allow the low-passed PWM out time to reach a steady-state analog
	 * voltage. */
	nrf_delay_ms(100);

	/* Before energizing the SMA above, if it had remained off for sufficient
	 * time to completely cool down, we measure the voltage across the SMA now.
	 * We will look for a decrease in the voltage (corresponding to a decrease
	 * in the SMA's resistance as a result of the phase transition from
	 * martensite to austenite) to determine when the SMA is fully retracted.
	 * If the SMA has not been turn off for a sufficient amount of time, we will
	 * use the previous reference value.*/
	if ((offTime_ms > SMA_COOLING_TIME_MS) || firstRetraction) {
		referenceVoltage_mV = adc_avg_mV(SMAVOLTS_ADC_CHNL, 8);
	}

	/* Start the timer which we use to both monitor the resistance of the SMA
	 * and to stop energizing it once the maximum timeout has expired. */
	err_code = app_timer_start(sma_timerID,
			APP_TIMER_TICKS(SMA_TIMER_PERIOD_MS, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

	return true;
}

bool sma_extend() {
	if ((smaState != SMA_STATE_RETRACTED) && (smaState != SMA_STATE_RETRACTING)) {
		/* If the SMA is not retracting or already retracted, there's nothing
		 * to do here, but we still return true to indicate nothing went
		 * wrong. */
		return true;
	}

	/* Turn off the current to the SMA */
	sma_setCurrent_mA(0);

	/* Record the time at which we turned off the current */
	app_timer_cnt_get(&retractStopTime_rtcTicks);

	smaState = SMA_STATE_EXTENDING;

	firstRetraction = false;

	return true;
}

smaState_t sma_getState () {
	return smaState;
}

bool sma_setCurrent_mA(uint32_t current_mA) {

	uint32_t onPeriod;

	if (current_mA > 1250) {
		current_mA = 1250;
	}

	onPeriod = (current_mA * pwm_getPeriod()) / 1250;

	pwm_setOnPeriod(SMAIREF_PWM_CHNL, onPeriod);

	return true;
}

void sma_timerHandler(void *p_context) {
	UNUSED_PARAMETER(p_context);

	uint32_t rtcTicks;
	uint32_t elapsedTime_ms;
	uint32_t smaVoltage_mV, deltaV_mV;

#if (SMA_DEBUG)
	char str[150];
#endif

	app_timer_cnt_get(&rtcTicks);

	if ((smaState == SMA_STATE_RETRACTING) || (smaState == SMA_STATE_RETRACTED)) {
		/* Calculate the time in milliseconds (by dividing RTC ticks by 32.378)
		 * since we began energizing the SMA. We mask the high-order byte of the
		 * subtraction result because app_timer_cnt_get only returns a 24-bit
		 * result. */
		elapsedTime_ms = (0x00FFFFFF & (rtcTicks - retractStartTime_rtcTicks)) / 33;

		if (elapsedTime_ms > maxTime_ms) {
			/* Turn off the current to the SMA */
			sma_setCurrent_mA(0);

			/* Record the time at which we turned off the current */
			retractStopTime_rtcTicks = rtcTicks;

			smaState = SMA_STATE_EXTENDING;
			firstRetraction = false;
		} else if (smaState == SMA_STATE_RETRACTING) {
			/* If the SMA is still retracting, we measure the voltage across it
			 * to determine whether the voltage has decreased (relative to the
			 * reference voltage) by enough to say that the SMA is fully
			 * retracted. */
			smaVoltage_mV = adc_avg_mV(SMAVOLTS_ADC_CHNL, 8);
			deltaV_mV = referenceVoltage_mV - smaVoltage_mV;

#if (SMA_DEBUG)
			snprintf(str, sizeof(str), "SMA: Voltage: %lumV, RefV: %lumV, Time: %lums\r\n",
					smaVoltage_mV, referenceVoltage_mV, elapsedTime_ms);
			app_uart_put_string(str);
#endif

			if (100 * deltaV_mV > RETRACTED_VOLTAGE_CHANGE_PCT * referenceVoltage_mV) {
				/* If the voltage across the SMA has decreased by
				 * RETRACTED_VOLTAGE_CHANGE_PCT percent, the SMA is now fully
				 * retracted, so we update the state variable. */
				//smaState = SMA_STATE_RETRACTED;
			}
		}
	} else if (smaState == SMA_STATE_EXTENDING) {
		elapsedTime_ms = (0x00FFFFFF & (rtcTicks - retractStopTime_rtcTicks)) / 33;

		if (elapsedTime_ms > SMA_EXTENSION_TIME_MS) {
			/* Now that the SMA is fully extended, there is no need to waste
			 * overhead running the state update timer, so we turn it off. */
			app_timer_stop(sma_timerID);

			smaState = SMA_STATE_EXTENDED;
		}
	} else if (smaState == SMA_STATE_EXTENDED) {
		/* Once the SMA is fully extended, there is no need to waste overhead
		 * running the state update timer, so we turn it off.  Note: the timer
		 * should have previously been turned off when we transitioned from the
		 * EXTENDEDING to EXTENDED phase. */
		app_timer_stop(sma_timerID);
	}

#if (SMA_DEBUG)
	if (smaState == SMA_STATE_EXTENDED) {
		app_uart_put_string("SMA EXTENDED\r\n");
	} else if (smaState == SMA_STATE_RETRACTING) {
		app_uart_put_string("SMA RETRACTING\r\n");
	} else if (smaState == SMA_STATE_RETRACTED) {
		app_uart_put_string("SMA RETRACTED\r\n");
	} else if (smaState == SMA_STATE_EXTENDING) {
		app_uart_put_string("SMA EXTENDING\r\n");
	} else {
		app_uart_put_string("SMA STATE UNKNOWN\r\n");
	}
#endif

}
