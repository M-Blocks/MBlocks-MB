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
#include "sma.h"
#include "mechbrake.h"
#include "bldc.h"
#include "imu.h"
#include "util.h"
#include "motionEvent.h"

#define DEBUG_MOTION_EVENTS			1

#define AXIS_ALIGNMENT_ERROR_DEG	10.00f

static bool initialized = false;

static app_timer_id_t timerID = TIMER_NULL;
static app_sched_event_handler_t timeoutHandler = NULL;
static void motionEvent_timeoutHandler(void *p_context);
static void motionEvent_delay(uint16_t delay_ms, app_sched_event_handler_t delayTimeoutHandler);

static app_sched_event_handler_t eventHandler;

/* These module-level  variables must be set when changing planes using motor acceleration*/
static uint16_t accelPlaneChangeSMAHoldTime_ms;
static uint16_t accelPlaneChangeAccelCurrent_mA;
static uint16_t accelPlaneChangeAccelTime_ms;
static bool accelPlaneChangeReverse;

/* These module-level variable must be set when changing planes using acceleration followed by the electronic brake */
static uint16_t accelBrakePlaneChangeAccelCurrent_mA;
static uint16_t accelBrakePlaneChangeAccelTime_ms;
static uint16_t accelBrakePlaneChangeCoastTime_ms;
static uint16_t accelBrakePlaneChangeBrakeTime_ms;
static uint16_t accelBrakePlaneChangeReverse;

/* These module-level variables must be set when changing planes using the electronic brake*/
static uint16_t ebrakePlaneChangeSMAHoldTime_ms;
static uint16_t ebrakePlaneChangeBLDCSpeed_rpm;
static uint16_t ebrakePlaneChangeEBrakeTime_ms;
static int16_t ebrakePlaneChangeBLDCSpeedChange_rpm;
static int16_t ebrakePlaneChangeEBrakeTimeChange_ms;
static uint16_t ebrakePlaneChangePostBrakeAccelCurrent_mA;
static uint16_t ebrakePlaneChangePostBrakeAccelTime_ms;
static bool ebrakePlaneChangeReverse;

/* These module variables must be set when performing a simple inertial actuation */
static uint16_t inertialActuationSpeed_rpm;
static uint16_t inertialActuationBrakeCurrent_mA;
static uint16_t inertialActuationBrakeTime_ms;
static bool inertialActuationReverse;

static void accelPlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size);
static void accelBrakePlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size);
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
	accelPlaneChangeAccelCurrent_mA = accelCurrent_mA;
	accelPlaneChangeAccelTime_ms = accelTime_ms;
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
		bldc_setAccel(accelPlaneChangeAccelCurrent_mA, accelPlaneChangeAccelTime_ms, accelPlaneChangeReverse, accelPlaneChangePrimitiveHandler);
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

bool motionEvent_startAccelBrakePlaneChange(uint16_t accelCurrent_mA, uint16_t accelTime_ms, uint16_t coastTime_ms, uint16_t brakeTime_ms, bool reverse, app_sched_event_handler_t motionEventHandler) {
	accelBrakePlaneChangeAccelCurrent_mA = accelCurrent_mA;
	accelBrakePlaneChangeAccelTime_ms = accelTime_ms;
	accelBrakePlaneChangeCoastTime_ms = coastTime_ms;
	accelBrakePlaneChangeBrakeTime_ms = brakeTime_ms;
	accelBrakePlaneChangeReverse = reverse;

	eventHandler = motionEventHandler;

	sma_retract(3000, accelBrakePlaneChangePrimitiveHandler);

	return true;
}

void accelBrakePlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;
	motionEvent_t motionEvent;

	motionPrimitive = *(motionPrimitive_t *)p_event_data;

	switch(motionPrimitive) {
	case MOTION_PRIMITIVE_SMA_RETRACTED:
		bldc_setAccel(accelBrakePlaneChangeAccelCurrent_mA, accelBrakePlaneChangeAccelTime_ms, accelBrakePlaneChangeReverse, accelBrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE:
		motionEvent_delay(accelBrakePlaneChangeCoastTime_ms, accelBrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_TIMER_EXPIRED:
		bldc_setSpeed(0, false, accelBrakePlaneChangeBrakeTime_ms, accelBrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_COASTING:
		//sma_extend(accelBrakePlaneChangePrimitiveHandler);
		//break;
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

bool motionEvent_startEBrakePlaneChange(uint16_t bldcSpeed_rpm, uint16_t ebrakeTime_ms,
		uint16_t postBrakeAccelCurrent_ma, uint16_t postBrakeAccelTime_ms,
		bool reverse, app_sched_event_handler_t motionEventHandler) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;

	ebrakePlaneChangeSMAHoldTime_ms = 6000;
	ebrakePlaneChangeBLDCSpeed_rpm = bldcSpeed_rpm;
	ebrakePlaneChangeEBrakeTime_ms = ebrakeTime_ms;

	ebrakePlaneChangePostBrakeAccelCurrent_mA = postBrakeAccelCurrent_ma;
	ebrakePlaneChangePostBrakeAccelTime_ms = postBrakeAccelTime_ms;

	ebrakePlaneChangeBLDCSpeedChange_rpm = 0;
	ebrakePlaneChangeEBrakeTimeChange_ms = 0;

	ebrakePlaneChangeReverse = reverse;

	eventHandler = motionEventHandler;

	motionPrimitive = MOTION_PRIMITIVE_START_SEQUENCE;
	err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), ebrakePlaneChangePrimitiveHandler);
	APP_ERROR_CHECK(err_code);

	return true;
}

#if (0)
void ebrakePlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;
	motionEvent_t motionEvent;
	vectorFloat_t gyrosRates;
	//vectorFloat_t gravityNew;
	bool flywheelFrameAligned;
	unsigned int alignmentAxisIndex;
	bool reverseAcceleration;
	float gyroMag;

	unsigned int i;
	float axisAngles[3];

	char str[128];

	static bool flywheelFrameAlignedInitial;
	static unsigned int alignmentAxisIndexInitial, alignmentAxisIndexDesired, alignmentAxisIndexUnwanted;
	static bool success = false;

	motionPrimitive = *(motionPrimitive_t *)p_event_data;

	switch(motionPrimitive) {
	case MOTION_PRIMITIVE_START_SEQUENCE:
		motionEvent_getFlywheelFrameAligned(&flywheelFrameAlignedInitial, &alignmentAxisIndexInitial);
		if (!ebrakePlaneChangeReverse) {
			alignmentAxisIndexDesired = (alignmentAxisIndexInitial + 1) % 3;
		} else {
			alignmentAxisIndexDesired = ((alignmentAxisIndexInitial + 3) - 1) % 3;
		}


#if (DEBUG_MOTION_EVENTS == 1)
		app_uart_put_string("Executing e-brake based plane change\r\n");
		if (flywheelFrameAlignedInitial) {
			app_uart_put_string("Flywheel axis-aligned with frame: true\r\n");
			snprintf(str, sizeof(str), "Initial frame axis index: %u ([%f %f %f])\r\n",
					alignmentAxisIndexInitial,
					frameAlignmentVectorsFloat[alignmentAxisIndexInitial].x,
					frameAlignmentVectorsFloat[alignmentAxisIndexInitial].y,
					frameAlignmentVectorsFloat[alignmentAxisIndexInitial].z);
			app_uart_put_string(str);
			snprintf(str, sizeof(str), "Desired frame axis index: %u ([%f %f %f])\r\n",
					alignmentAxisIndexDesired,
					frameAlignmentVectorsFloat[alignmentAxisIndexDesired].x,
					frameAlignmentVectorsFloat[alignmentAxisIndexDesired].y,
					frameAlignmentVectorsFloat[alignmentAxisIndexDesired].z);
			app_uart_put_string(str);
		} else {
			app_uart_put_string("Flywheel axis-aligned with frame: false\r\n");
		}

		if (!ebrakePlaneChangeReverse) {
			app_uart_put_string("Starting flywheel spinning forward\r\n");
		} else {
			app_uart_put_string("Starting flywheel spinning in reverse\r\n");
		}
#endif

		bldc_setSpeed(ebrakePlaneChangeBLDCSpeed_rpm, ebrakePlaneChangeReverse, 0, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_STABLE:
		/* Once the motor's speed has stabilized, begin to retract the pin. */
		app_uart_put_debug("Flywheel speed stabilized\r\n", DEBUG_MOTION_EVENTS);
		app_uart_put_debug("Retracting SMA pin\r\n", DEBUG_MOTION_EVENTS);
		sma_retract(ebrakePlaneChangeSMAHoldTime_ms, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_SMA_RETRACTED:
		/* Once the pin is retracted, apply the electronic brake for the
		 * specified duration. */
		app_uart_put_debug("SMA pin retracted\r\n", DEBUG_MOTION_EVENTS);
		app_uart_put_debug("Applying e-brake to flywheel\r\n", DEBUG_MOTION_EVENTS);
		bldc_setSpeed(0, false, ebrakePlaneChangeEBrakeTime_ms, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_COASTING:
	case MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE:
		/* After applying the electronic brake to the flywheel, or after the
		 * flywheel finishes accelerating, we pause briefly to allow the
		 * central actuator to stop rotating and stabilize before we check
		 * whether the central actuator has rotated into the correct position. */
		app_uart_put_debug("Waiting for central actuator to stabilize\r\n", DEBUG_MOTION_EVENTS);
		motionEvent_delay(250, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_TIMER_EXPIRED:
		/* Check for angular acceleration.  If negligible, the central actuator
		 * has come to stop, so we then verify whether the actuator is aligned
		 * with one of the cube's faces. */
		imu_getGyrosFloat(&gyrosRates);
		gyroMag = imu_getVectorFloatMagnitude(&gyrosRates);

		if (gyroMag < 0.000488f) {
			/* Central actuator is not moving, so we read the gravity vector
			 * from the IMU and check whether it is 1) aligned with one of the
			 * cube's faces, and 2) aligned with a the correct face. */
			snprintf(str, sizeof(str), "Central actuator has stabilized (gyroscope magnitude: %f)\r\n", gyroMag);
			app_uart_put_debug(str, DEBUG_MOTION_EVENTS);

			if (!motionEvent_getFlywheelFrameAligned(&flywheelFrameAligned, &alignmentAxisIndex)) {
				app_uart_put_debug("Failed to determine whether flywheel and frame are aligned\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				sma_extend(ebrakePlaneChangePrimitiveHandler);
			} else if (flywheelFrameAligned && !flywheelFrameAlignedInitial) {
				app_uart_put_debug("Previously unaligned central actuator is now aligned with a frame axis\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				success = true;
				sma_extend(ebrakePlaneChangePrimitiveHandler);
			} else if (flywheelFrameAligned && (alignmentAxisIndex == alignmentAxisIndexDesired)) {
				app_uart_put_debug("Central actuator is aligned with desired frame axis\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				success = true;
				sma_extend(ebrakePlaneChangePrimitiveHandler);
			} else if (flywheelFrameAligned && (sma_getHoldTimeRemaining_ms() > 500)) {
				if (alignmentAxisIndex == alignmentAxisIndexInitial) {
					/* If the central actuator is in the same position that it
					 * started in, we need to accelerate the motor in the
					 * opposite direction that we did when using the e-brake in
					 * order to create a torque in the same direction using
					 * pure acceleration. */
					reverseAcceleration = !ebrakePlaneChangeReverse;
					app_uart_put_debug("Central actuator is aligned, but with the initial, instead of desired, axis\r\n", DEBUG_MOTION_EVENTS);
				} else {
					/* At this point, we can deduce that the central actuator
					 * is aligned with the third frame axis, that is not the
					 * initial axis with which it was aligned, and not the
					 * desired axis.  The desired axis is most easily reached
					 * by applying a torque in the opposite direction of the
					 * torque applied by the initial e-braking event, so we
					 * need to accelerate the flywheel in the same direction
					 * as we spun the motor before applying the e-brake. */
					reverseAcceleration = ebrakePlaneChangeReverse;
					app_uart_put_debug("Central actuator is aligned, but with the undesired, instead of desired, axis\r\n", DEBUG_MOTION_EVENTS);
				}

				if (!reverseAcceleration) {
					app_uart_put_debug("Accelerating flywheel in forward direction\r\n", DEBUG_MOTION_EVENTS);
				} else {
					app_uart_put_debug("Accelerating flywheel in reverse\r\n", DEBUG_MOTION_EVENTS);
				}

				bldc_setAccel(ebrakePlaneChangeAccelCurrent_mA, ebrakePlaneChangeAccelTime_ms,
						reverseAcceleration, ebrakePlaneChangePrimitiveHandler);
			} else if (sma_getHoldTimeRemaining_ms() > 500) {
				app_uart_put_debug("Central actuator is not aligned with any frame axis\r\n", DEBUG_MOTION_EVENTS);

				vectorFloat_t gravity;
				bool upsideDown = false;

				imu_getGravityFloat(&gravity);
				for (i=0; i<3; i++) {
					axisAngles[i] = imu_getVectorFloatAngle(&gravity, &frameAlignmentVectorsFloat[i]);
					if (axisAngles[i] > 120.0f) {
						upsideDown = true;
					}
				}

				if (upsideDown) {
					for (i=0; i<3; i++) {
						axisAngles[i] = 180.0f - axisAngles[i];
					}
				}

				unsigned int alignmentAxisIndexPrecedingDesired = ((alignmentAxisIndexDesired + 3) - 1) % 3;
				unsigned int alignmentAxisIndexFollowingDesired = (alignmentAxisIndexDesired + 1) % 3;

				snprintf(str, sizeof(str), "Angle to axis (%u) preceding desired axis (%u): %f degrees\r\n",
						alignmentAxisIndexPrecedingDesired, alignmentAxisIndexDesired,
						axisAngles[alignmentAxisIndexPrecedingDesired]);
				app_uart_put_debug(str, DEBUG_MOTION_EVENTS);

				snprintf(str, sizeof(str), "Angle to axis (%u) following desired axis (%u): %f degrees\r\n",
						alignmentAxisIndexFollowingDesired, alignmentAxisIndexDesired,
						axisAngles[alignmentAxisIndexFollowingDesired]);
				app_uart_put_debug(str, DEBUG_MOTION_EVENTS);

				if (axisAngles[alignmentAxisIndexPrecedingDesired] <= axisAngles[alignmentAxisIndexFollowingDesired]) {
					/* If the central actuator is closer to the frame axis in
					 * the reverse direction, we need to move the actuator
					 * 'forward', which means that we must accelerate in
					 * reverse. */
					reverseAcceleration = true;
					app_uart_put_debug("Central actuator closer to preceding axis, need to move central actuator forward\r\n", DEBUG_MOTION_EVENTS);
				} else {
					reverseAcceleration = false;
					app_uart_put_debug("Central actuator closer to following axis, need to move central actuator in reverse\r\n", DEBUG_MOTION_EVENTS);
				}


				if (!reverseAcceleration) {
					app_uart_put_debug("Accelerating flywheel in forward direction\r\n", DEBUG_MOTION_EVENTS);
				} else {
					app_uart_put_debug("Accelerating flywheel in reverse\r\n", DEBUG_MOTION_EVENTS);
				}

				bldc_setAccel(3000, 100, reverseAcceleration, ebrakePlaneChangePrimitiveHandler);
			} else {
				app_uart_put_debug("Central actuator is not aligned with frame, but SMA hold time has expired\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				success = false;
				sma_extend(ebrakePlaneChangePrimitiveHandler);
			}
		} else  {
			/* If the central actuator is still spinning, we delay for longer
			 * knowing that the central actuator will eventually come to
			 * rest.*/
			snprintf(str, sizeof(str), "Central actuator is still rotating (gyroscope magnitude: %f)\r\n", gyroMag);
			app_uart_put_debug(str, DEBUG_MOTION_EVENTS);
			motionEvent_delay(250, ebrakePlaneChangePrimitiveHandler);
		}
		break;
	case MOTION_PRIMITIVE_SMA_EXTENDED:
		app_uart_put_debug("SMA pin is fully extended\r\n", DEBUG_MOTION_EVENTS);
		if (eventHandler != NULL) {
			if (success) {
				motionEvent = MOTION_EVENT_PLANE_CHANGE_SUCCESS;
			} else {
				motionEvent = MOTION_EVENT_PLANE_CHANGE_FAILURE;
			}
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
	default:
		break;
	}
}
#else
void ebrakePlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;
	motionEvent_t motionEvent;
	vectorFloat_t gyrosRates;
	//vectorFloat_t gravityNew;
	bool flywheelFrameAligned;
	unsigned int alignmentAxisIndex;
	bool reverseAcceleration;
	float gyroMag;

	unsigned int i;
	float axisAngles[3];

	char str[128];

	static bool flywheelFrameAlignedInitial;
	static unsigned int alignmentAxisIndexInitial, alignmentAxisIndexDesired, alignmentAxisIndexUnwanted;
	static bool success = false;

	motionPrimitive = *(motionPrimitive_t *)p_event_data;

	switch(motionPrimitive) {
	case MOTION_PRIMITIVE_START_SEQUENCE:
		motionEvent_getFlywheelFrameAligned(&flywheelFrameAlignedInitial, &alignmentAxisIndexInitial);
		if (!ebrakePlaneChangeReverse) {
			alignmentAxisIndexDesired = (alignmentAxisIndexInitial + 1) % 3;
		} else {
			alignmentAxisIndexDesired = ((alignmentAxisIndexInitial + 3) - 1) % 3;
		}


#if (DEBUG_MOTION_EVENTS == 1)
		app_uart_put_string("Executing e-brake based plane change\r\n");
		if (flywheelFrameAlignedInitial) {
			app_uart_put_string("Flywheel axis-aligned with frame: true\r\n");
			snprintf(str, sizeof(str), "Initial frame axis index: %u ([%f %f %f])\r\n",
					alignmentAxisIndexInitial,
					frameAlignmentVectorsFloat[alignmentAxisIndexInitial].x,
					frameAlignmentVectorsFloat[alignmentAxisIndexInitial].y,
					frameAlignmentVectorsFloat[alignmentAxisIndexInitial].z);
			app_uart_put_string(str);
			snprintf(str, sizeof(str), "Desired frame axis index: %u ([%f %f %f])\r\n",
					alignmentAxisIndexDesired,
					frameAlignmentVectorsFloat[alignmentAxisIndexDesired].x,
					frameAlignmentVectorsFloat[alignmentAxisIndexDesired].y,
					frameAlignmentVectorsFloat[alignmentAxisIndexDesired].z);
			app_uart_put_string(str);
		} else {
			app_uart_put_string("Flywheel axis-aligned with frame: false\r\n");
		}

		if (!ebrakePlaneChangeReverse) {
			app_uart_put_string("Starting flywheel spinning forward\r\n");
		} else {
			app_uart_put_string("Starting flywheel spinning in reverse\r\n");
		}
#endif

		bldc_setSpeed(ebrakePlaneChangeBLDCSpeed_rpm, ebrakePlaneChangeReverse, 0, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_STABLE:
		/* Once the motor's speed has stabilized, begin to retract the pin. */
		app_uart_put_debug("Flywheel speed stabilized\r\n", DEBUG_MOTION_EVENTS);

		/* If the SMA pin is not already retracted, retract it now */
		if (sma_getState() != SMA_STATE_HOLDING) {
			app_uart_put_debug("Retracting SMA pin\r\n", DEBUG_MOTION_EVENTS);
			sma_retract(ebrakePlaneChangeSMAHoldTime_ms, ebrakePlaneChangePrimitiveHandler);
			break;
		}

		/* At this point, we know that the SMA is already retracted, so we must
		 * be making the second+ attempt at changing planes.  If the central
		 * actuator is not moving after bring the flywheel up to speed, we check
		 * whether bringing the motor up to speed caused they central actuator to
		 * accidently fall into the desired position. */
		imu_getGyrosFloat(&gyrosRates);
		gyroMag = imu_getVectorFloatMagnitude(&gyrosRates);

		if (gyroMag < 0.000488f) {
			/* Central actuator is not moving, so we read the gravity vector
			 * from the IMU and check whether it is 1) aligned with one of the
			 * cube's faces, and 2) aligned with a the correct face. */

			if (!motionEvent_getFlywheelFrameAligned(&flywheelFrameAligned, &alignmentAxisIndex)) {
				app_uart_put_debug("Failed to determine whether flywheel and frame are aligned\r\n", DEBUG_MOTION_EVENTS);
			} else if (flywheelFrameAligned && (alignmentAxisIndex == alignmentAxisIndexDesired)) {
				app_uart_put_debug("Bringing flywheel up to speed accidently aligned central actuator with desired frame axis\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				success = true;
				sma_extend(ebrakePlaneChangePrimitiveHandler);
				break;
			} else {
				app_uart_put_debug("Brining flywheel up to speed did not bump central actuator into the desired position\r\n", DEBUG_MOTION_EVENTS);
			}
		}

		/* If bringing the flywheel up to speed did not happen to bring the
		 * central actuator into alignment with the desired frame axis, we
		 * proceed to actuate the e-brake. */
		app_uart_put_debug("Applying e-brake to flywheel\r\n", DEBUG_MOTION_EVENTS);
		bldc_setSpeed(0, false, ebrakePlaneChangeEBrakeTime_ms, ebrakePlaneChangePrimitiveHandler);
		ebrakePlaneChangeEBrakeTime_ms += ebrakePlaneChangeEBrakeTimeChange_ms;
		break;
	case MOTION_PRIMITIVE_SMA_RETRACTED:
		/* Once the pin is retracted, apply the electronic brake for the
		 * specified duration. */
		app_uart_put_debug("SMA pin retracted\r\n", DEBUG_MOTION_EVENTS);
		app_uart_put_debug("Applying e-brake to flywheel\r\n", DEBUG_MOTION_EVENTS);
		bldc_setSpeed(0, false, ebrakePlaneChangeEBrakeTime_ms, ebrakePlaneChangePrimitiveHandler);
		ebrakePlaneChangeEBrakeTime_ms += ebrakePlaneChangeEBrakeTimeChange_ms;
		break;
	case MOTION_PRIMITIVE_BLDC_COASTING:
		app_uart_put_debug("E-brake released\r\n", DEBUG_MOTION_EVENTS);

		if ((ebrakePlaneChangePostBrakeAccelCurrent_mA == 0) || (ebrakePlaneChangePostBrakeAccelTime_ms == 0)) {
			app_uart_put_debug("Post-e-brake acceleration time or current is set to 0, skipping acceleration\r\n", DEBUG_MOTION_EVENTS);
			app_uart_put_debug("Waiting for central actuator to stabilize\r\n", DEBUG_MOTION_EVENTS);
			motionEvent_delay(1000, ebrakePlaneChangePrimitiveHandler);
			break;
		}

		reverseAcceleration = ebrakePlaneChangeReverse;
		if (!reverseAcceleration) {
			app_uart_put_debug("Accelerating flywheel forward to counteract e-brake torque\r\n", DEBUG_MOTION_EVENTS);
		} else {
			app_uart_put_debug("Accelerating flywheel in reverse to counteract e-brake torque\r\n", DEBUG_MOTION_EVENTS);
		}

		bldc_setAccel(ebrakePlaneChangePostBrakeAccelCurrent_mA, ebrakePlaneChangePostBrakeAccelTime_ms, reverseAcceleration, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE:
		/* After the acceleration is complete, we pause briefly to allow the
		 * central actuator to stop rotating and stabilize before we check
		 * whether the central actuator has rotated into the correct position.
		 */
		app_uart_put_debug("Waiting for central actuator to stabilize\r\n", DEBUG_MOTION_EVENTS);
		motionEvent_delay(1000, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_TIMER_EXPIRED:
		/* Check for angular acceleration.  If negligible, the central actuator
		 * has come to stop, so we then verify whether the actuator is aligned
		 * with one of the cube's faces. */
		imu_getGyrosFloat(&gyrosRates);
		gyroMag = imu_getVectorFloatMagnitude(&gyrosRates);

		if (gyroMag < 0.000488f) {
			/* Central actuator is not moving, so we read the gravity vector
			 * from the IMU and check whether it is 1) aligned with one of the
			 * cube's faces, and 2) aligned with a the correct face. */
			snprintf(str, sizeof(str), "Central actuator has stabilized (gyroscope magnitude: %f)\r\n", gyroMag);
			app_uart_put_debug(str, DEBUG_MOTION_EVENTS);

			if (!motionEvent_getFlywheelFrameAligned(&flywheelFrameAligned, &alignmentAxisIndex)) {
				app_uart_put_debug("Failed to determine whether flywheel and frame are aligned\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				sma_extend(ebrakePlaneChangePrimitiveHandler);
			} else if (flywheelFrameAligned && !flywheelFrameAlignedInitial) {
				app_uart_put_debug("Previously unaligned central actuator is now aligned with a frame axis\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				success = true;
				sma_extend(ebrakePlaneChangePrimitiveHandler);
			} else if (flywheelFrameAligned && (alignmentAxisIndex == alignmentAxisIndexDesired)) {
				app_uart_put_debug("Central actuator is aligned with desired frame axis\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				success = true;
				sma_extend(ebrakePlaneChangePrimitiveHandler);
			} else if (flywheelFrameAligned && (sma_getHoldTimeRemaining_ms() > 1000)) {
				if (alignmentAxisIndex == alignmentAxisIndexInitial) {
					/* If the central actuator is in the same position that it
					 * started in, we do not want to change the direction of
					 * the torque applied when we next actuate the e-brake. */
					app_uart_put_debug("Central actuator is aligned, but with the initial, instead of desired, axis\r\n", DEBUG_MOTION_EVENTS);

					if (!ebrakePlaneChangeReverse) {
						app_uart_put_string("Starting flywheel spinning forward\r\n");
					} else {
						app_uart_put_string("Starting flywheel spinning in reverse\r\n");
					}

					ebrakePlaneChangeBLDCSpeed_rpm += ebrakePlaneChangeBLDCSpeedChange_rpm;
					bldc_setSpeed(ebrakePlaneChangeBLDCSpeed_rpm, ebrakePlaneChangeReverse, 0, ebrakePlaneChangePrimitiveHandler);
				} else {
					/* At this point, we can deduce that the central actuator
					 * is aligned with the third frame axis, that is not the
					 * initial axis with which it was aligned, and not the
					 * desired axis.  The desired axis is most easily reached
					 * by applying a torque in the opposite direction of the
					 * torque applied by the initial e-braking event, so we
					 * need to spin the flywheel in the opposite direction
					 * as we spun the motor initially.  */
					app_uart_put_debug("Central actuator is aligned, but with the undesired, instead of desired, axis\r\n", DEBUG_MOTION_EVENTS);

					if (ebrakePlaneChangeReverse) {
						app_uart_put_string("Starting flywheel spinning forward\r\n");
					} else {
						app_uart_put_string("Starting flywheel spinning in reverse\r\n");
					}

					ebrakePlaneChangeBLDCSpeed_rpm += ebrakePlaneChangeBLDCSpeedChange_rpm;
					bldc_setSpeed(ebrakePlaneChangeBLDCSpeed_rpm, !ebrakePlaneChangeReverse, 0, ebrakePlaneChangePrimitiveHandler);
				}
			} else if (sma_getHoldTimeRemaining_ms() > 500) {
				app_uart_put_debug("Central actuator is not aligned with any frame axis\r\n", DEBUG_MOTION_EVENTS);

				vectorFloat_t gravity;
				bool upsideDown = false;

				imu_getGravityFloat(&gravity);
				for (i=0; i<3; i++) {
					axisAngles[i] = imu_getVectorFloatAngle(&gravity, &frameAlignmentVectorsFloat[i]);
					if (axisAngles[i] > 120.0f) {
						upsideDown = true;
					}
				}

				if (upsideDown) {
					for (i=0; i<3; i++) {
						axisAngles[i] = 180.0f - axisAngles[i];
					}
				}

				unsigned int alignmentAxisIndexPrecedingDesired = ((alignmentAxisIndexDesired + 3) - 1) % 3;
				unsigned int alignmentAxisIndexFollowingDesired = (alignmentAxisIndexDesired + 1) % 3;

				snprintf(str, sizeof(str), "Angle to axis (%u) preceding desired axis (%u): %f degrees\r\n",
						alignmentAxisIndexPrecedingDesired, alignmentAxisIndexDesired,
						axisAngles[alignmentAxisIndexPrecedingDesired]);
				app_uart_put_debug(str, DEBUG_MOTION_EVENTS);

				snprintf(str, sizeof(str), "Angle to axis (%u) following desired axis (%u): %f degrees\r\n",
						alignmentAxisIndexFollowingDesired, alignmentAxisIndexDesired,
						axisAngles[alignmentAxisIndexFollowingDesired]);
				app_uart_put_debug(str, DEBUG_MOTION_EVENTS);

				if (axisAngles[alignmentAxisIndexPrecedingDesired] <= axisAngles[alignmentAxisIndexFollowingDesired]) {
					/* If the central actuator is closer to the frame axis in
					 * the reverse direction, we need to move the actuator
					 * 'forward' by spinning the motor in its initial direction
					 * and then applying the e-brake. */
					app_uart_put_debug("Central actuator closer to preceding axis, need to move central actuator forward\r\n", DEBUG_MOTION_EVENTS);

					if (!ebrakePlaneChangeReverse) {
						app_uart_put_string("Starting flywheel spinning forward\r\n");
					} else {
						app_uart_put_string("Starting flywheel spinning in reverse\r\n");
					}

					ebrakePlaneChangeBLDCSpeed_rpm += ebrakePlaneChangeBLDCSpeedChange_rpm;
					bldc_setSpeed(ebrakePlaneChangeBLDCSpeed_rpm, ebrakePlaneChangeReverse, 0, ebrakePlaneChangePrimitiveHandler);
				} else {
					app_uart_put_debug("Central actuator closer to following axis, need to move central actuator in reverse\r\n", DEBUG_MOTION_EVENTS);

					if (ebrakePlaneChangeReverse) {
						app_uart_put_string("Starting flywheel spinning forward\r\n");
					} else {
						app_uart_put_string("Starting flywheel spinning in reverse\r\n");
					}

					ebrakePlaneChangeBLDCSpeed_rpm += ebrakePlaneChangeBLDCSpeedChange_rpm;
					bldc_setSpeed(ebrakePlaneChangeBLDCSpeed_rpm, !ebrakePlaneChangeReverse, 0, ebrakePlaneChangePrimitiveHandler);
				}
			} else {
				app_uart_put_debug("Central actuator is not aligned with frame, but SMA hold time has expired\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
				success = false;
				sma_extend(ebrakePlaneChangePrimitiveHandler);
			}
		} else  {
			/* If the central actuator is still spinning, we delay for longer
			 * knowing that the central actuator will eventually come to
			 * rest.*/
			snprintf(str, sizeof(str), "Central actuator is still rotating (gyroscope magnitude: %f)\r\n", gyroMag);
			app_uart_put_debug(str, DEBUG_MOTION_EVENTS);
			motionEvent_delay(500, ebrakePlaneChangePrimitiveHandler);
		}
		break;
	case MOTION_PRIMITIVE_SMA_EXTENDED:
		app_uart_put_debug("SMA pin is fully extended\r\n", DEBUG_MOTION_EVENTS);

		/* Stop the flywheel in case it is still spinning */
		bldc_setSpeed(0, false, 0, NULL);

		if (eventHandler != NULL) {
			if (success) {
				motionEvent = MOTION_EVENT_PLANE_CHANGE_SUCCESS;
			} else {
				motionEvent = MOTION_EVENT_PLANE_CHANGE_FAILURE;
			}
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
	default:
		break;
	}
}
#endif

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

bool motionEvent_getFlywheelFrameAligned(bool *flywheelFrameAligned, unsigned int *axisIndex) {
	uint8_t i;
	vectorFloat_t gravity;
	float angleDiff;
	float minAngle = 90.0f;

	if (!imu_getGravityFloat(&gravity)) {
		return false;
	}

	*flywheelFrameAligned = false;

	for (i=0; i<3; i++) {
		angleDiff = fabs(imu_getVectorFloatAngle(&gravity, &frameAlignmentVectorsFloat[i]));

		if (angleDiff < AXIS_ALIGNMENT_ERROR_DEG) {
			*flywheelFrameAligned = true;
			*axisIndex = i;
		} else if (180.0f - angleDiff < AXIS_ALIGNMENT_ERROR_DEG) {
			*flywheelFrameAligned = true;
			*axisIndex = i;
		}

		if (angleDiff < minAngle) {
			minAngle = angleDiff;
		} else if (180.0f - angleDiff < minAngle) {
			minAngle = 180.0f - angleDiff;
		}
	}

#if (DEBUG_MOTION_EVENTS == 1)
	char str[128];
	snprintf(str, sizeof(str), "Gravity to frame axis minimum angle: %f degrees\r\n", minAngle);
	app_uart_put_string(str);
#endif

	return true;
}
