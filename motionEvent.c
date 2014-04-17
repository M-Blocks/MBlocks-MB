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

#include "sma.h"
#include "mechbrake.h"
#include "bldc.h"
#include "imu.h"
#include "motionEvent.h"


app_sched_event_handler_t eventHandler;

/* These module variables must be set when changing planes */
static uint16_t planeChangeSMAHoldTime_ms;
static uint16_t planeChangeAccel_mA;
static uint16_t planeChangeTime_ms;
static bool planeChangeReverse;

/* These module variables must be set when performing a simple inertial actuation */
static uint16_t inertialActuationSpeed_rpm;
static uint16_t inertialActuationBrakeCurrent_mA;
static uint16_t inertialActuationBrakeTime_ms;
static bool inertialActuationReverse;

static void planeChangePrimitiveHandler(void *p_event_data, uint16_t event_size);
static void inertialActuationPrimitiveHandler(void *p_event_data, uint16_t event_size);

bool motionEvent_startPlaneChange(uint16_t accelCurrent_mA, uint16_t accelTime_ms, bool reverse, app_sched_event_handler_t motionEventHandler) {
	planeChangeSMAHoldTime_ms = 1000;
	planeChangeAccel_mA = accelCurrent_mA;
	planeChangeTime_ms = accelTime_ms;
	planeChangeReverse = reverse;

	eventHandler = motionEventHandler;

	sma_retract(planeChangeSMAHoldTime_ms, planeChangePrimitiveHandler);

	return true;
}

void planeChangePrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;
	motionEvent_t motionEvent;

	motionPrimitive = *(motionPrimitive_t *)p_event_data;

	switch(motionPrimitive) {
	case MOTION_PRIMITIVE_SMA_RETRACTED:
		bldc_setAccel(planeChangeAccel_mA, planeChangeTime_ms, planeChangeReverse, planeChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE:
		sma_extend(planeChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_SMA_EXTENDED:
		bldc_setSpeed(0, false, true, planeChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_STOPPED:
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

	bldc_setSpeed(inertialActuationSpeed_rpm, inertialActuationReverse, false,
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
		bldc_setSpeed(0, false, false, NULL);
		mechbrake_actuate(1, &coilCurrentStep, inertialActuationPrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_TIMEOUT:
		bldc_setSpeed(0, false, false, NULL);
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
