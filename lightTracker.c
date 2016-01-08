/*
 * motionPrimitive.c
 *
 *  Created on: Apr 14, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "app_scheduler.h"
#include "app_timer.h"

#include "global.h"
#include "mpu6050.h"
#include "sma.h"
#include "mechbrake.h"
#include "bldc.h"
#include "imu.h"
#include "util.h"
#include "fb.h"
#include "motionEvent.h"

#include "lightTracker.h"

#define DEBUG_EVENTS			1

enum {
	FORWARD = 3,
	BACKWARD,
	TOP,
	BOTTOM,
	LEFT,
	RIGHT
};

static const int alignment[12][9] = {
	{1, 1, 0, 6, 5, 2, 4, 1, 3},
	{1, -1, 0, 2, 4, 5, 6, 1, 3},
	{-1, -1, 0, 5, 6, 4, 2, 1, 3},
	{-1, 1, 0, 4, 2, 6, 5, 1, 3},
	{1, 1, 0, 3, 1, 5, 6, 2, 4},
	{1, -1, 0, 5, 6, 1, 3, 2, 4},
	{-1, -1, 0, 1, 3, 6, 5, 2, 4},
	{-1, 1, 0, 6, 5, 3, 1, 2, 4},
	{1, 1, 0, 4, 2, 1, 3, 5, 6},
	{1, -1, 0, 1, 3, 2, 4, 5, 6},
	{-1, -1, 0, 2, 4, 3, 1, 5, 6},
	{-1, 1, 0, 3, 1, 4, 2, 5, 6}
};

static bool initialized = false;

APP_TIMER_DEF(timerID); 

static app_sched_event_handler_t timeoutHandler = NULL;

static void lightTracker_timeoutHandler(void *p_context);
static void lightTracker_delay(uint16_t delay_ms, app_sched_event_handler_t delayTimeoutHandler);
static void lightTrackerPrimitiveHandler(void *p_event_data, uint16_t event_size);
static void lightTracker_motionEventHandler(void *p_event_data, uint16_t event_size);

static uint16_t inertialActuationSpeed_rpm;
static uint16_t inertialActuationBrakeCurrent_mA;
static uint16_t inertialActuationBrakeTime_ms;
static uint16_t lightThreshold;

static app_sched_event_handler_t eventHandler;

bool lightTracker_init() {
	uint32_t err_code;

	err_code = app_timer_create(&timerID, APP_TIMER_MODE_SINGLE_SHOT, lightTracker_timeoutHandler);
	APP_ERROR_CHECK(err_code);

	initialized = true;

	return true;
}

void lightTracker_timeoutHandler(void *p_context) {
	uint32_t err_code;
	trackerPrimitive_t trackerPrimitive;

	if (timeoutHandler != NULL) {
		trackerPrimitive = TRACKER_PRIMITIVE_TIMER_EXPIRED;
		err_code = app_sched_event_put(&trackerPrimitive, sizeof(trackerPrimitive), timeoutHandler);
		APP_ERROR_CHECK(err_code);
	}
}

void lightTracker_delay(uint16_t delay_ms, app_sched_event_handler_t delayTimeoutHandler) {
	uint32_t err_code;

	if (!initialized) {
		lightTracker_init();
	}

	timeoutHandler = delayTimeoutHandler;
	err_code = app_timer_start(timerID, APP_TIMER_TICKS(delay_ms, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
}

bool lightTracker_startLightTracker(uint16_t bldcSpeed_rpm,
			uint16_t brakeCurrent_mA, uint16_t brakeTime_ms,
			uint16_t threshold,
			app_sched_event_handler_t lightTrackerEventHandler) {
	uint32_t err_code;
	inertialActuationSpeed_rpm = bldcSpeed_rpm;
	inertialActuationBrakeCurrent_mA = brakeCurrent_mA;
	inertialActuationBrakeTime_ms = brakeTime_ms;
	lightThreshold = threshold; 

	eventHandler = lightTrackerEventHandler;

	fb_setRxEnable(0, true);
	mpu6050_setAddress(MPU6050_I2C_ADDR_CENTRAL);

	trackerPrimitive_t trackerPrimitive = TRACKER_PRIMITIVE_START_SEQUENCE;
	err_code = app_sched_event_put(&trackerPrimitive, sizeof(trackerPrimitive), lightTrackerPrimitiveHandler);
	APP_ERROR_CHECK(err_code);

	return true;
}

void lightTrackerPrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	static vectorFloat_t gravityVector;
	static int ambientReadings[6];
	static int config;
	
	int max_signal = 0, max_face = 0;

	trackerPrimitive_t trackerPrimitive;
	trackerPrimitive = *(trackerPrimitive_t *)p_event_data;

	if (!imu_getGravityFloat(&gravityVector)) {
		app_uart_put_string("Failed to read gravity vector.\r\n");
		return;
	}

	switch (trackerPrimitive) {
	case TRACKER_PRIMITIVE_START_SEQUENCE:
		lightTracker_getConfiguration(&config, gravityVector);
		lightTracker_readAmbientLight(ambientReadings);
		lightTracker_delay(50, lightTrackerPrimitiveHandler);
		break;
	case TRACKER_PRIMITIVE_TIMER_EXPIRED:
		for (int i = 1; i <= 6; i++) {
			if (alignment[config][TOP] == i || alignment[config][BOTTOM] == i)
				continue;

			if (ambientReadings[i] > max_signal) {
				max_signal = ambientReadings[i];
				max_face = i;
			}
		}
		if (max_signal < lightThreshold) {
			if (eventHandler != NULL) {
				trackerEvent_t trackerEvent = LIGHT_TRACKER_EVENT_COMPLETE;
				err_code = app_sched_event_put(&trackerEvent, sizeof(trackerEvent), eventHandler);
				APP_ERROR_CHECK(err_code);
			}
			return;
		}

		if (alignment[config][FORWARD] == max_face) {
			if (motionEvent_startInertialActuation(inertialActuationSpeed_rpm, 
					inertialActuationBrakeCurrent_mA, inertialActuationBrakeTime_ms,
					false, false, false, 0, false, lightTracker_motionEventHandler)) {
				app_uart_put_string("Starting inertial actuation forward...\r\n");
			}

		} else if (alignment[config][BACKWARD] == max_face) {
			if (motionEvent_startInertialActuation(inertialActuationSpeed_rpm, 
					inertialActuationBrakeCurrent_mA, inertialActuationBrakeTime_ms,
					true, false, false, 0, false, lightTracker_motionEventHandler)) {
				app_uart_put_string("Starting inertial actuation forward...\r\n");
			}
		} else {
			bool reverse = false;
			if (alignment[config][0] == alignment[config][1]) {
				reverse = true;
			}
			if (motionEvent_startEBrakePlaneChange(4000, 40, 0, 0, reverse,
					lightTracker_motionEventHandler)) {
				app_uart_put_string("Starting e-brake based plane change...\r\n");
			}
		}
		break;
	case TRACKER_PRIMITIVE_EVENT_COMPLETE:
		lightTracker_getConfiguration(&config, gravityVector);
		lightTracker_readAmbientLight(ambientReadings);
		lightTracker_delay(50, lightTrackerPrimitiveHandler);
		break;
	case TRACKER_PRIMITIVE_EVENT_FAILURE:
		break;
	default:
		break;
	}
}

void lightTracker_motionEventHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;

	motionEvent_t motionEvent;
	motionEvent = *(motionEvent_t *) p_event_data;

	trackerPrimitive_t trackerPrimitive = TRACKER_PRIMITIVE_EVENT_FAILURE;

	switch (motionEvent) {
	case MOTION_EVENT_PLANE_CHANGE_SUCCESS:
		trackerPrimitive = TRACKER_PRIMITIVE_EVENT_COMPLETE;
		app_uart_put_string("Successfully changed planes\r\n");
		break;
	case MOTION_EVENT_PLANE_CHANGE_FAILURE:
		app_uart_put_string("Failed to change planes\r\n");
		break;
	case MOTION_EVENT_INERTIAL_ACTUATION_COMPLETE:
		trackerPrimitive = TRACKER_PRIMITIVE_EVENT_COMPLETE;
		app_uart_put_string("Inertial actuation complete\r\n");
		break;
	case MOTION_EVENT_INERTIAL_ACTUATION_FAILURE:
		app_uart_put_string("Inertial actuation failure\r\n");
		break;
	default:
		app_uart_put_string("Motion event not recognized\r\n");
	}

	err_code = app_sched_event_put(&trackerPrimitive, sizeof(trackerPrimitive), lightTrackerPrimitiveHandler);
	APP_ERROR_CHECK(err_code);
}

bool lightTracker_readAmbientLight(int *ambientReadings) {
	for (int face = 1; face <= 6; face++) {
		ambientReadings[face - 1] = fb_getAmbientLight(face);
	}

	return true;
}

bool lightTracker_getConfiguration(int *config, vectorFloat_t gravity) {
	double EPS = 1e-3;
	int LIGHTEPS = 5;

	int ambientReadings[6];
	lightTracker_readAmbientLight(ambientReadings);

	for (int i = 0; i < 12; i++) {
		int ambientBottom = ambientReadings[alignment[i][BOTTOM]];

		double cross = sqrt(2) - (alignment[i][0] * gravity.x + alignment[i][1] * gravity.y + alignment[i][2] * gravity.z);
		if (cross < EPS && ambientBottom < LIGHTEPS) {
			*config = i;
			return true;
		}
	}

	return false;
}