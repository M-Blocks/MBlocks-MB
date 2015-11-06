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
static bool inertialActuationEBrake;
static bool inertialActuationAccel;
static uint16_t inertialActuationEBrakeAccelStartDelay_ms;
static bool inertialActuationAccelReverse;

/* These module-level variables must be set to track a light source */
static bool lt_type;
static unsigned int lt_bldcSpeed_rpm_f, lt_brakeCurrent_mA_f, lt_brakeTime_ms_f;
static unsigned int lt_bldcSpeed_rpm_r, lt_brakeCurrent_mA_r, lt_brakeTime_ms_r;
static unsigned int lt_threshold;

/* These module-level variables are used to check for actuator stabilization */
static vectorFloat_t gravityCurrent;
static vectorFloat_t gravityNew;

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

	mpu6050_setAddress(MPU6050_I2C_ADDR_CENTRAL);

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

	mpu6050_setAddress(MPU6050_I2C_ADDR_CENTRAL);

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
		uint16_t postBrakeAccelCurrent_ma, uint16_t postBrakeAccelTime_ms, bool reverse, 
		app_sched_event_handler_t motionEventHandler) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;

	ebrakePlaneChangeSMAHoldTime_ms = 8000;
	ebrakePlaneChangeBLDCSpeed_rpm = bldcSpeed_rpm;
	ebrakePlaneChangeEBrakeTime_ms = ebrakeTime_ms;

	ebrakePlaneChangePostBrakeAccelCurrent_mA = postBrakeAccelCurrent_ma;
	ebrakePlaneChangePostBrakeAccelTime_ms = postBrakeAccelTime_ms;

	ebrakePlaneChangeBLDCSpeedChange_rpm = 0;
	ebrakePlaneChangeEBrakeTimeChange_ms = 0;

	ebrakePlaneChangeReverse = reverse;

	eventHandler = motionEventHandler;

	mpu6050_setAddress(MPU6050_I2C_ADDR_CENTRAL);

	motionPrimitive = MOTION_PRIMITIVE_START_SEQUENCE;
	err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), ebrakePlaneChangePrimitiveHandler);
	APP_ERROR_CHECK(err_code);

	return true;
}

void ebrakePlaneChangePrimitiveHandler(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;
	motionEvent_t motionEvent;
	vectorFloat_t gyrosRates;
	bool flywheelFrameAligned;
	unsigned int alignmentAxisIndex;
	float gyroMag;

	char str[100];
	float axisAngles[3];

	static bool tapBreak = false;
	static int tapCount = 0;
	int ebrakeTapSpeed_rpm = 3000;
	int ebrakeTapBrake_ms = 5;

	static bool flywheelFrameAlignedInitial;
	static unsigned int alignmentAxisIndexInitial, alignmentAxisIndexDesired;

	motionPrimitive = *(motionPrimitive_t *)p_event_data;

	switch(motionPrimitive) {
	case MOTION_PRIMITIVE_START_SEQUENCE:
		tapBreak = false;
		tapCount = 0;

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
				sma_extend(ebrakePlaneChangePrimitiveHandler);
				break;
			} else {
				app_uart_put_debug("Bringing flywheel up to speed did not bump central actuator into the desired position\r\n", DEBUG_MOTION_EVENTS);
			}
		}

		/* If bringing the flywheel up to speed did not happen to bring the
		 * central actuator into alignment with the desired frame axis, we
		 * proceed to actuate the e-brake. */
		app_uart_put_debug("Applying e-brake tap to flywheel\r\n", DEBUG_MOTION_EVENTS);
		tapBreak = true;
		tapCount = 1;
		bldc_setSpeed(0, false, ebrakeTapBrake_ms, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_SMA_RETRACTED:
		/* Once the pin is retracted, apply the electronic brake for the
		 * specified duration. */
		app_uart_put_debug("SMA pin retracted\r\n", DEBUG_MOTION_EVENTS);
		app_uart_put_debug("Applying e-brake to flywheel\r\n", DEBUG_MOTION_EVENTS);
		bldc_setSpeed(0, false, ebrakePlaneChangeEBrakeTime_ms, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_BLDC_COASTING:
		app_uart_put_debug("E-brake released\r\n", DEBUG_MOTION_EVENTS);

		if (tapBreak && tapCount < 3) {
			motionEvent_delay(250, ebrakePlaneChangePrimitiveHandler);
			break;
		}

		if ((ebrakePlaneChangePostBrakeAccelCurrent_mA == 0) || (ebrakePlaneChangePostBrakeAccelTime_ms == 0)) {
			app_uart_put_debug("Post-e-brake acceleration time or current is set to 0, skipping acceleration\r\n", DEBUG_MOTION_EVENTS);
			app_uart_put_debug("Waiting for central actuator to stabilize\r\n", DEBUG_MOTION_EVENTS);
			motionEvent_delay(250, ebrakePlaneChangePrimitiveHandler);
			break;
		}
	case MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE:
		/* After the acceleration is complete, we pause briefly to allow the
		 * central actuator to stop rotating and stabilize before we check
		 * whether the central actuator has rotated into the correct position.
		 */
		app_uart_put_debug("Waiting for central actuator to stabilize\r\n", DEBUG_MOTION_EVENTS);
		motionEvent_delay(250, ebrakePlaneChangePrimitiveHandler);
		break;
	case MOTION_PRIMITIVE_TIMER_EXPIRED:
		/* Check if we need to tap the break again. */
		if (tapBreak && tapCount < 3) {
			app_uart_put_debug("Applying e-brake tap to flywheel\r\n", DEBUG_MOTION_EVENTS);
			bldc_setSpeed(0, false, ebrakeTapBrake_ms, ebrakePlaneChangePrimitiveHandler);
			tapCount++;
			break;
		}

		/* Check that the accelerometer readings have stabilized. */
		imu_getGravityFloat(&gravityNew);
		if (fabs(gravityNew.x - gravityCurrent.x) < 0.02 &&
			fabs(gravityNew.y - gravityCurrent.y) < 0.02 &&
			fabs(gravityNew.z - gravityCurrent.z) < 0.02) {	
			/* Central actuator is not moving, so we read the gravity vector
			 * from the IMU and check whether it is 1) aligned with one of the
			 * cube's faces, and 2) aligned with a the correct face. */
			app_uart_put_debug("Central actuator has stabilized\r\n", DEBUG_MOTION_EVENTS);
			
			if (!motionEvent_getFlywheelFrameAligned(&flywheelFrameAligned, &alignmentAxisIndex)) {
				app_uart_put_debug("Failed to determine whether flywheel and frame are aligned\r\n", DEBUG_MOTION_EVENTS);
				app_uart_put_debug("Trying to accelerate flywheel.\r\n", DEBUG_MOTION_EVENTS);
				bldc_setSpeed(ebrakeTapSpeed_rpm, ebrakePlaneChangeReverse, 0, ebrakePlaneChangePrimitiveHandler);
				break;
			} else if (!flywheelFrameAligned && sma_getHoldTimeRemaining_ms() > 1000) {
				app_uart_put_debug("Central actuator is not aligned with any frame axis\r\n", DEBUG_MOTION_EVENTS);
				vectorFloat_t gravity;
				bool upsideDown = false;

				imu_getGravityFloat(&gravity);
				for (int i = 0; i < 3; i++) {
					axisAngles[i] = imu_getVectorFloatAngle(&gravity, &frameAlignmentVectorsFloat[i]);
					if (axisAngles[i] > 120.0f) {
						upsideDown = true;
					}
				}
				if (upsideDown) {
					for (int i = 0; i < 3; i++) {
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

				bool reverseSpeed;
				if (axisAngles[alignmentAxisIndexPrecedingDesired] <= axisAngles[alignmentAxisIndexFollowingDesired]) {
					/* If the central actuator is closer to the frame axis in
					 * the reverse direction, we need to move the actuator
					 * 'forward', which means that we must accelerate in
					 * reverse. */
					reverseSpeed = true;
					app_uart_put_debug("Central actuator closer to preceding axis, need to move central actuator forward\r\n", DEBUG_MOTION_EVENTS);
				} else {
					reverseSpeed = false;
					app_uart_put_debug("Central actuator closer to following axis, need to move central actuator in reverse\r\n", DEBUG_MOTION_EVENTS);
				}

				app_uart_put_debug("Trying to accelerate flywheel.\r\n", DEBUG_MOTION_EVENTS);
				bldc_setSpeed(ebrakeTapSpeed_rpm, reverseSpeed, 0, ebrakePlaneChangePrimitiveHandler);
				break;
			}

			app_uart_put_debug("Extending SMA pin\r\n", DEBUG_MOTION_EVENTS);
			sma_extend(ebrakePlaneChangePrimitiveHandler);
		} else  {	
			snprintf(str, sizeof(str), "Previous accelerometer readings: [%f %f %f]\r\n", 
				gravityCurrent.x, gravityCurrent.y, gravityCurrent.z);
			app_uart_put_debug(str, DEBUG_MOTION_EVENTS);
			snprintf(str, sizeof(str), "Current accelerometer readings: [%f %f %f]\r\n", 
				gravityNew.x, gravityNew.y, gravityNew.z);
			app_uart_put_debug(str, DEBUG_MOTION_EVENTS);

			/* Set current accelerometer readings to previous */
			gravityCurrent.x = gravityNew.x;
			gravityCurrent.y = gravityNew.y;
			gravityCurrent.z = gravityNew.z;

			/* If the central actuator is still spinning, we delay for longer
			 * knowing that the central actuator will eventually come to
			 * rest.*/
			app_uart_put_debug("Central actuator is still rotating\r\n", DEBUG_MOTION_EVENTS);
			motionEvent_delay(500, ebrakePlaneChangePrimitiveHandler);
		}
		break;
	case MOTION_PRIMITIVE_SMA_EXTENDED:
		app_uart_put_debug("SMA pin is fully extended\r\n", DEBUG_MOTION_EVENTS);

		/* Stop the flywheel in case it is still spinning */
		bldc_setSpeed(0, false, 0, NULL);

		bool success = false;
		if (!motionEvent_getFlywheelFrameAligned(&flywheelFrameAligned, &alignmentAxisIndex)) {
			app_uart_put_debug("Failed to determine whether flywheel is aligned.\r\n", DEBUG_MOTION_EVENTS);
		} else if (flywheelFrameAligned && (alignmentAxisIndex == alignmentAxisIndexDesired)) {
			success = true;
		}

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

bool motionEvent_startInertialActuation(uint16_t bldcSpeed_rpm,
			uint16_t brakeCurrent_mA, uint16_t brakeTime_ms, bool reverse,
			bool eBrake, bool accel, uint16_t eBrakeAccelStartDelay_ms, bool accelReverse,
			app_sched_event_handler_t motionEventHandler) {
	inertialActuationSpeed_rpm = bldcSpeed_rpm;
	inertialActuationBrakeCurrent_mA = brakeCurrent_mA;
	inertialActuationBrakeTime_ms = brakeTime_ms;
	inertialActuationReverse = reverse;
	inertialActuationEBrake = eBrake;
	inertialActuationAccel = accel;
	inertialActuationEBrakeAccelStartDelay_ms = eBrakeAccelStartDelay_ms;
	inertialActuationAccelReverse = accelReverse;

	eventHandler = motionEventHandler;

	/* Use the IMU on the central actuator */
	mpu6050_setAddress(MPU6050_I2C_ADDR_CENTRAL);

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
	case MOTION_PRIMITIVE_BLDC_TIMEOUT:
		if (inertialActuationReverse) {
			coilCurrentStep.current_mA = -inertialActuationBrakeCurrent_mA;
		} else {
			coilCurrentStep.current_mA = inertialActuationBrakeCurrent_mA;
		}
		coilCurrentStep.time_ms = inertialActuationBrakeTime_ms;
		bldc_setSpeed(0, false, 0, NULL);
		mechbrake_actuate(1, &coilCurrentStep, inertialActuationPrimitiveHandler);
		break;
#if (0)
	case MOTION_PRIMITIVE_BLDC_TIMEOUT:
		bldc_setSpeed(0, false, 0, NULL);
		if (eventHandler != NULL) {
			motionEvent = MOTION_EVENT_INERTIAL_ACTUATION_FAILURE;
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
#endif
	case MOTION_PRIMITIVE_MECHBRAKE_SUCCESS:
		if (inertialActuationEBrakeAccelStartDelay_ms > 0) {
			motionEvent_delay(inertialActuationEBrakeAccelStartDelay_ms, inertialActuationPrimitiveHandler);
		} else if (inertialActuationEBrake) {
			bldc_setSpeed(0, false, 500,  inertialActuationPrimitiveHandler);
		} else if (inertialActuationAccel) {
			bldc_setAccel(BLDC_ACCEL_CURRENT_MAX_MA, 500, inertialActuationAccelReverse, inertialActuationPrimitiveHandler);
		} else if (eventHandler != NULL) {
			motionEvent = MOTION_EVENT_INERTIAL_ACTUATION_COMPLETE;
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
	case MOTION_PRIMITIVE_TIMER_EXPIRED:
		if (inertialActuationEBrake) {
			bldc_setSpeed(0, false, 500,  inertialActuationPrimitiveHandler);
		} else if (inertialActuationAccel) {
			bldc_setAccel(BLDC_ACCEL_CURRENT_MAX_MA, 500, inertialActuationAccelReverse, inertialActuationPrimitiveHandler);
		} else if (eventHandler != NULL) {
			motionEvent = MOTION_EVENT_INERTIAL_ACTUATION_COMPLETE;
			err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		break;
	case MOTION_PRIMITIVE_BLDC_COASTING:
	case MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE:
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

// TODO
static const enum Face {
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

void lightTrackerPrimitiveHandler(void *p_event_data, uint16_t event_size) {
	char str[100];

	motionEvent_t motionEvent;
	motionEvent = *(motionEvent_t *)p_event_data;
	switch (motionEvent) {
	case MOTION_EVENT_INERTIAL_ACTUATION_COMPLETE:
	case MOTION_EVENT_INERTIAL_ACTUATION_FAILURE:
		delay_ms(500);
		break;
	default:
		break;
	}

	app_uart_put_debug("Calling tracker function.\r\n", DEBUG_MOTION_EVENTS);
	vectorFloat_t vf;
	if (!imu_getGravityFloat(&vf)) {
		snprintf(str, sizeof(str), "Failed to read gravity vector from %s IMU\r\n", mpu6050_getName());
		app_uart_put_string(str);
		return;
	}

	int config = -1;
	double EPS = 0.01;
	int LIGHTEPS = 5;
	for (int i = 0; i < 12; i++) {
		if (fb_setRxEnable(alignment[i][BOTTOM], true)) {
			delay_ms(50);
			int16_t ambientLightBottom = fb_getAmbientLight(alignment[i][BOTTOM]);
			fb_setRxEnable(alignment[i][BOTTOM], false);
			
			double cross = sqrt(2) - (alignment[i][0] * vf.x + alignment[i][1] * vf.y + alignment[i][2] * vf.z);

			if (cross < EPS && ambientLightBottom < LIGHTEPS) {
				config = i;
				break;
			}
		}
	}

	/* Check to see if we're in a valid configuration */
	if (config == -1) {
		app_uart_put_string("Failed to find configuration of the cube. Trying to change plane.\r\n");
		if (motionEvent_startEBrakePlaneChange(4000, 40, 0, 0, false,
				lightTrackerPrimitiveHandler)) {
			app_uart_put_string("Starting e-brake based plane change...\r\n");			
		}
		return;
	}
	snprintf(str, sizeof(str), "Found configuration. TOP: %u; LEFT: %u; FORWARD: %u\r\n", 
		alignment[config][TOP], alignment[config][LEFT], alignment[config][FORWARD]);
	app_uart_put_string(str);

	/* Find strongest light signal */
	int max_signal = 0;
	int best_face = -1;
	for (int i = 1; i <= 6; i++) {
		// exclude top and bottom faces since we deal with 2D motion
		if (alignment[config][TOP] == i || alignment[config][BOTTOM] == i)
			continue;

		if (fb_setRxEnable(i, true)) {
			delay_ms(50);
			int16_t ambientLight = fb_getAmbientLight(i);
			fb_setRxEnable(i, false);
			// if ambient light reading is 0, then we are connected and done
			if (ambientLight < LIGHTEPS) {
				app_uart_put_string("Connected to aggregate.\r\n");
				return;
			}

			if (ambientLight > max_signal) {
				max_signal = ambientLight;
				best_face = i;
			}
		}
	}

	if (lt_type == 1 && max_signal < lt_threshold) {
		app_uart_put_string("Maximum signal is below allowed threshold.\r\n");
		return;
	}
	/* Move cube forward or backward if possible */
	if (alignment[config][FORWARD] == best_face) {
		if (motionEvent_startInertialActuation(lt_bldcSpeed_rpm_f, lt_brakeCurrent_mA_f,
			lt_brakeTime_ms_f, false, false, false, 0, false,
			lightTrackerPrimitiveHandler)) {
			app_uart_put_string("Starting inertial actuation forward...\r\n");
		}
	}
	else if (alignment[config][BACKWARD] == best_face) {
		if (motionEvent_startInertialActuation(lt_bldcSpeed_rpm_r, lt_brakeCurrent_mA_r,
			lt_brakeTime_ms_r, true, false, false, 0, false, 
			lightTrackerPrimitiveHandler)) {
			app_uart_put_string("Starting inertial actuation backward...\r\n");
		}
	}
	/* Change plane to orient flywheel towards the right direction */
	else {
		bool reverse = false;
		if (alignment[config][0] == alignment[config][1]) {
			reverse = true;
		}
		if (motionEvent_startEBrakePlaneChange(4000, 40, 0, 0, reverse,
				lightTrackerPrimitiveHandler)) {
			app_uart_put_string("Starting e-brake based plane change...\r\n");
		}
	}
}

bool motionEvent_startLightTracker(bool type, 
	uint16_t bldcSpeed_rpm_f, uint16_t brakeCurrent_mA_f, uint16_t brakeTime_ms_f,
	uint16_t bldcSpeed_rpm_r, uint16_t brakeCurrent_mA_r, uint16_t brakeTime_ms_r,
	uint16_t threshold) {
	uint32_t err_code;

	lt_type = type;
	lt_bldcSpeed_rpm_f = bldcSpeed_rpm_f;
	lt_brakeCurrent_mA_f = brakeCurrent_mA_f;
	lt_brakeTime_ms_f = brakeTime_ms_f;
	lt_bldcSpeed_rpm_r = bldcSpeed_rpm_r;
	lt_brakeCurrent_mA_r = brakeCurrent_mA_r;
	lt_brakeTime_ms_r = brakeTime_ms_r;
	lt_threshold = threshold;

	motionEvent_t motionEvent = MOTION_EVENT_START_TRACKER;
	err_code = app_sched_event_put(&motionEvent, sizeof(motionEvent), lightTrackerPrimitiveHandler);
	APP_ERROR_CHECK(err_code);

	return true;
}