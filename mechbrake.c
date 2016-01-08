/*
 * mechbrake.c
 *
 *  Created on: Feb 9, 2014
 *      Author: kwgilpin
 */


#include <stdint.h>
#include <stdbool.h>

#include "app_timer.h"

#include "global.h"
#include "util.h"
#include "twi_master.h"
#include "motionEvent.h"
#include "db.h"
#include "mechbrake.h"

#define MECHBRAKE_POLL_INTERVAL_MS	5

static bool initialized = false;
static bool directionsReversed = false;
static bool brakeActive = false;

static app_sched_event_handler_t eventHandler = NULL;

static uint32_t startTime_rtcTicks;
static uint32_t timeoutInterval_ms;

APP_TIMER_DEF(mechbrake_timerID); 

static void mechbrake_timerHandler(void *p_context);

bool mechbrake_init() {
	uint32_t err_code;

	err_code = app_timer_create(&mechbrake_timerID, APP_TIMER_MODE_REPEATED, mechbrake_timerHandler);
	APP_ERROR_CHECK(err_code);

	initialized = true;

	return true;
}

void mechbrake_deinit() {
	uint32_t err_code;

	err_code = app_timer_stop(mechbrake_timerID);
	APP_ERROR_CHECK(err_code);

	initialized = false;
}

bool mechbrake_actuate(uint8_t stepCount, const coilCurrentStep_t *steps, app_sched_event_handler_t brakeCompleteEventHandler) {
	uint32_t err_code;
	uint8_t twiBuf[34];
	uint8_t i;
	uint16_t current_mA;

	if (brakeActive) {
		return false;
	}

	if (!initialized) {
		mechbrake_init();
	}

	/* If the number of steps provided by the user will over-run the buffer,
	 * we truncate the list of steps. */
	if (2 + (4*stepCount) > sizeof(twiBuf)) {
		stepCount = (sizeof(twiBuf) - 2) / 4;
	}

	twiBuf[0] = DB_BRAKE_CMD;
	twiBuf[1] = stepCount;

	timeoutInterval_ms = 0;

	for (i=0; i<stepCount; i++) {
		if (mechbrake_getReverseDirections()) {
			current_mA = -steps[i].current_mA;
		} else {
			current_mA = steps[i].current_mA;
		}

		twiBuf[(4*i) + 2] = (current_mA >> 0) & 0xFF;
		twiBuf[(4*i) + 3] = (current_mA >> 8) & 0xFF;
		twiBuf[(4*i) + 4] = (steps[i].time_ms >> 0) & 0xFF;
		twiBuf[(4*i) + 5] = (steps[i].time_ms >> 8) & 0xFF;

		timeoutInterval_ms += steps[i].time_ms;
	}

	/* To account for some imprecision (in both the timer system and the
	 * daughterboard processor) we allow an extra 25ms for the mechanical
	 * braking action to run to completion. */
	timeoutInterval_ms += 100;

	eventHandler = brakeCompleteEventHandler;

	twi_master_init();

	if (!twi_master_transfer((DB_TWI_ADDR << 1), twiBuf, 2 + (4*stepCount), true)) {
		twi_master_deinit();
		return false;
	}

	err_code = app_timer_start(mechbrake_timerID, APP_TIMER_TICKS(MECHBRAKE_POLL_INTERVAL_MS, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_cnt_get(&startTime_rtcTicks);
	APP_ERROR_CHECK(err_code);

	brakeActive = true;

	twi_master_deinit();
	return true;
}

void mechbrake_timerHandler(void *p_context) {
	uint32_t err_code;
	bool rxSuccess;
	uint8_t twiBuf[2];
	uint32_t currentTime_rtcTicks;
	uint32_t elapsedTime_ms;
	motionPrimitive_t motionPrimitive;

	twi_master_init();

	rxSuccess = twi_master_transfer((DB_TWI_ADDR << 1) | TWI_READ_BIT, twiBuf, 2, true);
	if (rxSuccess && (twiBuf[0] == DB_BRAKE_CMD) && (twiBuf[1] == 0x01)) {
		app_timer_stop(mechbrake_timerID);

		if (eventHandler != NULL) {
			motionPrimitive = MOTION_PRIMITIVE_MECHBRAKE_SUCCESS;
			err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		brakeActive = false;
	} else if (rxSuccess && (twiBuf[0] == DB_BRAKE_CMD) && (twiBuf[1] != 0x01)) {
		app_timer_stop(mechbrake_timerID);

		if (eventHandler != NULL) {
			motionPrimitive = MOTION_PRIMITIVE_MECHBRAKE_FAILURE;
			err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		brakeActive = false;
	} else {
		app_timer_cnt_get(&currentTime_rtcTicks);
		elapsedTime_ms = ((0x00FFFFFF & (currentTime_rtcTicks - startTime_rtcTicks)) * USEC_PER_APP_TIMER_TICK) / 1000;

		if (elapsedTime_ms > timeoutInterval_ms) {
			app_timer_stop(mechbrake_timerID);

			if (eventHandler != NULL) {
				motionPrimitive = MOTION_PRIMITIVE_MECHBRAKE_TIMEOUT;
				err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
				APP_ERROR_CHECK(err_code);
			}
			brakeActive = false;
		}
	}

	twi_master_deinit();
}

void mechbrake_setReverseDirections(bool reverse) {
	directionsReversed = reverse;
}

bool mechbrake_getReverseDirections() {
	return directionsReversed;
}

