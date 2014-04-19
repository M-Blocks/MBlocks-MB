/*
 * motionPrimitive.c
 *
 *  Created on: Apr 14, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "app_scheduler.h"
#include "app_timer.h"

#include "global.h"
#include "sma.h"
#include "mechbrake.h"
#include "bldc.h"
#include "imu.h"
#include "motionEvent.h"

static bool initialized = false;

static app_timer_id_t timerID = TIMER_NULL;
static app_sched_event_handler_t timeoutHandler = NULL;
static void motionEvent_timeoutHandler(void *p_context);
static void motionEvent_delay(uint16_t delay_ms, app_sched_event_handler_t delayTimeoutHandler);

static app_sched_event_handler_t eventHandler;

/* These module-level  variables must be set when changing planes using motor acceleration*/
static uint16_t accelPlaneChangeSMAHoldTime_ms;
static uint16_t accelPlaneChangeAccel_mA;
static uint16_t accelPlaneChangeTime_ms;
static bool accelPlaneChangeReverse;

/* These module-level variables must be set when changing planes using the electronic brake*/
static uint16_t ebrakePlaneChangeSMAHoldTime_ms;
static uint16_t ebrakePlaneChangeBLDCSpeed_rpm;
static uint16_t ebrakePlaneChangeEBrakeTime_ms;
static bool ebrakePlaneChangeReverse;

/* These module variables must be set when performing a simple inertial actuation */
static uint16_t inertialActuationSpeed_rpm;
static uint16_t inertialActuationBrakeCurrent_mA;
static uint16_t inertialActuationBrakeTime_ms;
static bool inertialActuationReverse;

static void accelPlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size);
static void ebrakePlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size);
static void inertialActuationPrimitiveHandler(void *p_event_data, uint16_t event_size);

bool motionEvent_init() {
	uint32_t err_code;

	if (timerID == TIMER_NULL) {
		err_code = app_timer_create(&timerID, APP_TIMER_MODE_SINGLE_SHOT, motionEvent_timeoutHandler);
		APP_ERROR_CHECK(err_code);
	}

	initialized = true;

	return true;
}

void motionEvent_timeoutHandler(void *p_context) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;

	if (timeoutHandler != NULL) {
		motionPrimitive = MOTION_PRIMITIVE_TIMER_EXPIRED;
		err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), timeoutHandler);
		APP_ERROR_CHECK(err_code);
	}
}

void motionEvent_delay(uint16_t delay_ms, app_sched_event_handler_t delayTimeoutHandler) {
	uint32_t err_code;

	if (!initialized) {
		motionEvent_init();
	}

	timeoutHandler = delayTimeoutHandler;
	err_code = app_timer_start(timerID, APP_TIMER_TICKS(delay_ms, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
}

bool motionEvent_startAccelPlaneChange(uint16_t accelCurrent_mA, uint16_t accelTime_ms, bool reverse, app_sched_event_handler_t motionEventHandler) {
	accelPlaneChangeSMAHoldTime_ms = 1000;
	accelPlaneChangeAccel_mA = accelCurrent_mA;
	accelPlaneChangeTime_ms = accelTime_ms;
	accelPlaneChangeReverse = reverse;

	eventHandler = motionEventHandler;

	sma_retract(accelPlaneChangeSMAHoldTime_ms, accelPlaneChangePrimitiveHandler);

	return true;
}

void accelPlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;
	motionEvent_t motionEvent;

	motionPrimitive = *(motionPrimitive_t *)p_event_data;

	switch(motionPrimitive) {
	case MOTION_PRIMITIVE_SMA_RETRACTED:
		bldc_setAccel(accelPlaneChangeAccel_mA, accelPlaneChangeTime_ms, accelPlaneChangeReverse, accelPlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE:
		motionEvent_delay(750, accelPlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_TIMER_EXPIRED:
		sma_extend(accelPlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_SMA_EXTENDED:
		bldc_setSpeed(0, false, BLDC_EBRAKE_COMPLETE_STOP_TIME_MS, accelPlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_COASTING:
		if (eventHandler != NULL) {
			motionEvent = MOTION_EVENT_PLANE_CHANGE_SUCCESS;
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
	default:
		break;
	}
}

bool motionEvent_startEBrakePlaneChange(uint16_t bldcSpeed_rpm, uint16_t ebrakeTime_ms, bool reverse, app_sched_event_handler_t motionEventHandler) {
	ebrakePlaneChangeSMAHoldTime_ms = 1000;
	ebrakePlaneChangeBLDCSpeed_rpm = bldcSpeed_rpm;
	ebrakePlaneChangeEBrakeTime_ms = ebrakeTime_ms;
	ebrakePlaneChangeReverse = reverse;

	eventHandler = motionEventHandler;

	bldc_setSpeed(ebrakePlaneChangeBLDCSpeed_rpm, ebrakePlaneChangeReverse, 0, ebrakePlaneChangePrimitiveHandler);

	return true;
}

void ebrakePlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;
	motionEvent_t motionEvent;

	motionPrimitive = *(motionPrimitive_t *)p_event_data;

	switch(motionPrimitive) {
	case MOTION_PRIMITIVE_BLDC_STABLE:
		sma_retract(ebrakePlaneChangeSMAHoldTime_ms, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_SMA_RETRACTED:
		bldc_setSpeed(0, false, ebrakePlaneChangeEBrakeTime_ms, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_COASTING:
		motionEvent_delay(750, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_TIMER_EXPIRED:
		sma_extend(ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_SMA_EXTENDED:
		if (eventHandler != NULL) {
			motionEvent = MOTION_EVENT_PLANE_CHANGE_SUCCESS;
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
	default:
		break;
	}
}

bool motionEvent_startInertialActuation(uint16_t bldcSpeed_rpm,
			uint16_t brakeCurrent_mA, uint16_t brakeTime_ms, bool reverse,
			app_sched_event_handler_t motionEventHandler) {
	inertialActuationSpeed_rpm = bldcSpeed_rpm;
	inertialActuationBrakeCurrent_mA = brakeCurrent_mA;
	inertialActuationBrakeTime_ms = brakeTime_ms;
	inertialActuationReverse = reverse;

	eventHandler = motionEventHandler;

	bldc_setSpeed(inertialActuationSpeed_rpm, inertialActuationReverse, 0,
			inertialActuationPrimitiveHandler);

	return true;
}

void inertialActuationPrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;
	motionEvent_t motionEvent;
	coilCurrentStep_t coilCurrentStep;

	motionPrimitive = *(motionPrimitive_t *)p_event_data;

	switch(motionPrimitive) {
	case MOTION_PRIMITIVE_BLDC_STABLE:
		if (inertialActuationReverse) {
			coilCurrentStep.current_mA = -inertialActuationBrakeCurrent_mA;
		} else {
			coilCurrentStep.current_mA = inertialActuationBrakeCurrent_mA;
		}
		coilCurrentStep.time_ms = inertialActuationBrakeTime_ms;
		bldc_setSpeed(0, false, 0, NULL);
		mechbrake_actuate(1, &coilCurrentStep, inertialActuationPrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_TIMEOUT:
		bldc_setSpeed(0, false, 0, NULL);
		if (eventHandler != NULL) {
			motionEvent = MOTION_EVENT_INERTIAL_ACTUATION_FAILURE;
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
	case MOTION_PRIMITIVE_MECHBRAKE_SUCCESS:
		if (eventHandler != NULL) {
			motionEvent = MOTION_EVENT_INERTIAL_ACTUATION_COMPLETE;
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
	default:
		break;
	}
}
