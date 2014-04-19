/*
 * motionEvent.h
 *
 *  Created on: Apr 14, 2014
 *      Author: kwgilpin
 */

#ifndef MOTIONEVENT_H_
#define MOTIONEVENT_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	MOTION_PRIMITIVE_TIMER_EXPIRED,
	MOTION_PRIMITIVE_MECHBRAKE_SUCCESS,
	MOTION_PRIMITIVE_MECHBRAKE_FAILURE,
	MOTION_PRIMITIVE_MECHBRAKE_TIMEOUT,
	MOTION_PRIMITIVE_SMA_RETRACTED,
	MOTION_PRIMITIVE_SMA_EXTENDING,
	MOTION_PRIMITIVE_SMA_EXTENDED,
	MOTION_PRIMITIVE_BLDC_STABLE,
	MOTION_PRIMITIVE_BLDC_TIMEOUT,
	MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE,
	MOTION_PRIMITIVE_BLDC_COASTING,
} motionPrimitive_t;

typedef enum {
	MOTION_EVENT_PLANE_CHANGE_SUCCESS,
	MOTION_EVENT_PLANE_CHANGE_FAILURE,
	MOTION_EVENT_INERTIAL_ACTUATION_COMPLETE,
	MOTION_EVENT_INERTIAL_ACTUATION_FAILURE
} motionEvent_t;

bool motionEvent_init(void);
bool motionEvent_startPlaneChange(uint16_t accelCurrent_mA, uint16_t accelTime_ms, bool reverse, app_sched_event_handler_t motionEventHandler);
bool motionEvent_startInertialActuation(uint16_t bldcSpeed_rpm, uint16_t brakeCurrent_mA, uint16_t brakeTime_ms, bool reverse, app_sched_event_handler_t motionEventHandler);

#endif /* MOTIONEVENT_H_ */
