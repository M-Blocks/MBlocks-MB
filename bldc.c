/*
 * bldc.c
 *
 *  Created on: Nov 25, 2013
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"

#include "global.h"
#include "pins.h"
#include "util.h"
#include "pwm.h"
#include "a4960.h"
#include "freqcntr.h"
#include "power.h"
#include "motionEvent.h"
#include "bldc.h"

#define BLDC_DEBUG			0

#define RSENSE_MILLIOHMS	10
#define ISENSE_GAIN			10

/* Resistors used in divider which sets maximum controller IC current */
#define R30					10000
//#define R29				1000
#define R29					2000

#define ACCEL_CURRENT_MAX_MA	((((3300 * R29) / (R30 + R29)) * 1000) / (RSENSE_MILLIOHMS * ISENSE_GAIN))

/* Number of points used in the variance calculation that determines whether
 * the motor has reached a steady state speed. */
#define STABLILITY_SAMPLE_SIZE	20

#define COAST_TIME_MAX_MS 		12000

/* Closed loop control constants */
#if (0)
#define K_P_NUMERATOR		30
#define K_P_DENOMINATOR		10000
#define K_I_NUMERATOR		5
#define K_I_DENOMINATOR		50000
#endif

typedef enum {
	BLDC_MODE_OFF,
	BLDC_MODE_CONSTANT_SPEED,
	BLDC_MODE_ACCEL,
	BLDC_MODE_EBRAKE,
	BLDC_MODE_COAST
} bldcMode_t;

static bldcMode_t bldcModeConstSpeed = BLDC_MODE_CONSTANT_SPEED;
static bldcMode_t bldcModeAccel = BLDC_MODE_ACCEL;
static bldcMode_t bldcModeEBrake = BLDC_MODE_EBRAKE;
static bldcMode_t bldcModeCoast = BLDC_MODE_COAST;

static bldcMode_t bldcModeCurrent = BLDC_MODE_OFF;

static int32_t kp_numerator = 3;
static int32_t kp_denominator = 1;
static int32_t ki_numerator = 16;
static int32_t ki_denominator = 100000;

static bool directionsReversed = false;

static bool initialized = false;
static bool configured = false;
static bool stable = false;
static bool needToReconfigure = false;

static uint32_t startTime_rtcTicks;

static uint16_t setSpeed_rpm;
//static bool bldcRunning = false;
//static bool accelModeActive = false;
//static bool brakeModeActive = false;
//static bool coastModeActive = false;
static app_timer_id_t bldc_speedControlTimerID = TIMER_NULL;

static app_sched_event_handler_t eventHandler;

static bool bldc_config(void);
static bool bldc_start(bool reverse);
static bool bldc_stop(uint16_t brakeTime_ms);

static bool bldc_translateDirection(bool reverse);

static bool bldc_setIRefRatio(uint8_t refRatio);
static void bldc_speedControlTimerHandler(void *p_context);
static void bldc_speedControlLoop(bool reinit, bldcMode_t bldcModeTimerStart);
static uint32_t bldc_getSpeedTime_ms(uint32_t rpm);
static bool bldc_updateStable(int32_t newestError);

bool bldc_init() {
	uint32_t err_code;

	if (bldc_speedControlTimerID == TIMER_NULL) {
		err_code = app_timer_create(&bldc_speedControlTimerID, APP_TIMER_MODE_REPEATED, bldc_speedControlTimerHandler);
		APP_ERROR_CHECK(err_code);
	}

	/* The BLDCSPEED is a PWM input to the A4960 which can be used to modulate
	 * speed, but we use the IREF input instead.  To keep the A4960 from
	 * stopping the motor, the BLDCSPEED pin must be tied high. */
    nrf_gpio_pin_set(BLDCSPEED_PIN_NO);
    GPIO_PIN_CONFIG((BLDCSPEED_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    /* If the charging voltage is removed while the MBlock is charging, it
     * causes a dip in the 3.3V supply.  This can reset the BLDC controller, so
     * after this happens, we need to reconfigure the controller. */
    power_registerImproperTerminationFlag(&needToReconfigure);
    power_registerVBATSWTurnedOffFlag(&needToReconfigure);

    initialized = true;

    return true;
}

void bldc_deinit() {
	if (!initialized) {
		return;
	}

    /* The A4960 consumes about 50uA additional current when the BLDCSPEED pin
     * is high, so we drive it low after de-initializing the BLDC controller. */
    nrf_gpio_pin_clear(BLDCSPEED_PIN_NO);
    GPIO_PIN_CONFIG((BLDCSPEED_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    initialized = false;
}

bool bldc_config() {
	if (!initialized || !power_getVBATSWState()) {
		return false;
	}

	/* Pre-emptively clear the flag which indicates whether we need to
	 * configure the BLDC controller as the result of the charging
	 * voltage being removed while the module is still charging. */
	needToReconfigure = false;

	/* 50us commutation blank time, 1.6us blank time, 0.3us dead time */
	if (!a4960_writeReg(A4960_CONFIG0_REG_ADDR,
			(0x00 << A4960_COMMBLANKTIME_POSN) |
			(0x04 << A4960_BLANKTIME_POSN) |
			(0x06 << A4960_DEADTIME_POSN),
			NULL)) {
		return false;
	}

	/* 6.25% reference ratio, 0.8V VDS threshold */
	if (!a4960_writeReg(A4960_CONFIG1_REG_ADDR,
			(0x00 << A4960_REFERENCERATIO_POSN) |
			(0x20 << A4960_VDSTHRESHOLD_POSN),
			NULL)) {
		return false;
	}

	/* 18.0us PWM fixed off time */
	if (!a4960_writeReg(A4960_CONFIG2_REG_ADDR,
			(0x05 << A4960_PWMOFFTIME_POSN),
			NULL)) {
		return false;
	}

	/* Duty-cycle limited start-up torque, 6.25% hold torque limit, 2ms hold
	 * time */
	if (!a4960_writeReg(A4960_CONFIG3_REG_ADDR,
			(0x01 << A4960_TORQUECONTROL_POSN) |
			(0x00 << A4960_HOLDTORQUE_POSN) |
			(0x07 << A4960_HOLDTIME_POSN),
			NULL)) {
		return false;
	}

	/* 104ms starting commutation time, 1.0ms ending commutation time */
	if (!a4960_writeReg(A4960_CONFIG4_REG_ADDR,
			(0x04 << A4960_ENDCOMM_POSN) |
			(0x0C << A4960_STARTCOMM_POSN),
			NULL)) {
		return false;
	}

	/* 28.125 degree phase advance, 12.5% ramp torque, -3.0ms ramp rate */
	if (!a4960_writeReg(A4960_CONFIG5_REG_ADDR,
			(0x0F << A4960_PHASEADV_POSN) |
			(0x01 << A4960_RAMPTORQUE_POSN) |
			(0x0E << A4960_RAMPRATE_POSN),
			NULL)) {
		return false;
	}

	/* Auto high start/low run BEMF hysteresis, 6.4us BEMF window, stop on
	 * fail, fault-low true on diagnostic pin, no restart after loss of sync,
	 * brake off, forward direction, disable outputs */
	if (!a4960_writeReg(A4960_RUN_REG_ADDR,
			(0x00 << A4960_BEMFHYSTERESIS_POSN) |
			(0x04 << A4960_BEMFWINDOW_POSN) |
			(0x01 << A4960_ENABLESTOPFAIL_POSN) |
			(0x00 << A4960_DIAGSIGNAL_POSN) |
			(0x00 << A4960_RESTART_POSN) |
			(0x00 << A4960_BRAKE_POSN) |
			(0x00 << A4960_DIRECTION_POSN) |
			(0x00 << A4960_RUN_POSN),
			NULL)) {
		return false;
	}

	/* Enable detection of all faults */
	if (!a4960_writeReg(A4960_MASK_REG_ADDR,
			(0x00 << A4960_HIGHTEMP_POSN) |
			(0x00 << A4960_OVERTEMP_POSN) |
			(0x00 << A4960_BEMFLOST_POSN) |
			(0x00 << A4960_PHASEABOOTCAP_POSN) |
			(0x00 << A4960_PHASEBBOOTCAP_POSN) |
			(0x00 << A4960_PHASECBOOTCAP_POSN) |
			(0x00 << A4960_PHASEAHIGH_POSN) |
			(0x00 << A4960_PHASEALOW_POSN) |
			(0x00 << A4960_PHASEBHIGH_POSN) |
			(0x00 << A4960_PHASEBLOW_POSN) |
			(0x00 << A4960_PHASECHIGH_POSN) |
			(0x00 << A4960_PHASECLOW_POSN),
			NULL)) {
		return false;
	}

	configured = true;

	return true;
}

bool bldc_start(bool reverse) {
	uint16_t runReg;

	/* If the controller has not been initialized or its battery supply is off,
	 * we return false. */
	if (!initialized || !power_getVBATSWState()) {
		return false;
	}

	/* Depending on how the 3 BLDC wires are connected, the direction in which
	 * the motor spins will change. */
	reverse = bldc_translateDirection(reverse);


	/* Read the current value of the run register */
	if (!a4960_readReg(A4960_RUN_REG_ADDR, &runReg)) {
		return false;
	}

	/* Clear the run bit.  It will still be set if the motor was stopped with
	 * the electronic brake. */
	runReg &= ~(0x01 << A4960_RUN_POSN);
	if (!a4960_writeReg(A4960_RUN_REG_ADDR, runReg, NULL)) {
		return false;
	}

	/* Set the reverse bit iif requested by the caller */
	if (reverse) {
		runReg |= (0x01 << A4960_DIRECTION_POSN);
	} else {
		runReg &= ~(0x01 << A4960_DIRECTION_POSN);
	}

	/* Clear the brake bit (in case it was set last time we stopped the
	 * motor). */
	runReg &= ~(0x01 << A4960_BRAKE_POSN);

	/* Before starting the motor with the run bit, write the modified run
	 * register back to the A4960.  If the direction bit is set at the
	 * same time as the run bit, it will be ignored. */
	if (!a4960_writeReg(A4960_RUN_REG_ADDR, runReg, NULL)) {
		return false;
	}

	/* Set the run bit */
	runReg |= (0x01 << A4960_RUN_POSN);

	/* Re-write the modified run register back to the A4960, now with the run
	 * bit set. */
	if (!a4960_writeReg(A4960_RUN_REG_ADDR, runReg, NULL)) {
		return false;
	}

	//bldcRunning = true;

	return true;
}

bool bldc_stop(uint16_t brakeTime_ms) {
	uint32_t err_code;
	uint16_t runReg;

	/* If the controller has not been initialized or its battery supply is off,
	 * we return false. */
	if (!initialized || !power_getVBATSWState()) {
		//bldcRunning = false;
		bldcModeCurrent = BLDC_MODE_OFF;
		return false;
	}

	/* Read the current value of the run register */
	if (!a4960_readReg(A4960_RUN_REG_ADDR, &runReg)) {
		return false;
	}

	if (brakeTime_ms > 0) {
		/* Set the brake bit if requested by the caller.  For the brake to be
	     * effective, we have to leave the run bit set.  We assume that the run
	     * bit is already set, but if it is not the motor is already stopped,
	     * so the state of the brake bit does not matter. */
		runReg |= (0x01 << A4960_BRAKE_POSN);
		/* When using the brake, the brake bit is ignored unless the run bit is
		 * also set.  If we are trying to brake the motor as it is coasting to
		 * a stop, the run bit has already been cleared, so we must set it
		 * again here. */
		runReg |= (0x01 << A4960_RUN_POSN);

		/* Write the modified run register back to the A4960 */
		if (!a4960_writeReg(A4960_RUN_REG_ADDR, runReg, NULL)) {
			return false;
		}

		/* Stop and restart the speed control timer so that it expires at the
		 * moment when we need to release the electronic brake. */
		err_code = app_timer_stop(bldc_speedControlTimerID);
		APP_ERROR_CHECK(err_code);

		/* Indicate to the speed control timer handler that when the timer next
		 * expires that the motor should be stopped. */
		bldcModeCurrent = BLDC_MODE_EBRAKE;

		/* Re-start the timer, indicating that when it expires that the eBrake
		 * should be released. */
		err_code = app_timer_start(bldc_speedControlTimerID,
				APP_TIMER_TICKS(brakeTime_ms, APP_TIMER_PRESCALER), &bldcModeEBrake);
		APP_ERROR_CHECK(err_code);
	} else {
		/* If we are allowing the motor to coast to a stop without using the
		 * electronic break, we clear the run bit.  The state of the brake bit
		 * does not matter once the run bit is cleared. */
		runReg &= ~(0x01 << A4960_RUN_POSN);

		/* Write the modified run register back to the A4960 */
		if (!a4960_writeReg(A4960_RUN_REG_ADDR, runReg, NULL)) {
			return false;
		}

		/* Stop the speed control timer before restarting to so that it expires
		 * next when we know that the motor has coasted to a stop. */
		err_code = app_timer_stop(bldc_speedControlTimerID);
		APP_ERROR_CHECK(err_code);

		bldcModeCurrent = BLDC_MODE_COAST;

		/* Re-start the timer, indicating that when it expires that the motor
		 * has coasted to a stop. */
		err_code = app_timer_start(bldc_speedControlTimerID,
				APP_TIMER_TICKS(COAST_TIME_MAX_MS, APP_TIMER_PRESCALER), &bldcModeCoast);
		APP_ERROR_CHECK(err_code);

	}

	/* Stop the frequency counter as we do not need to keep it running if the
	 * motor is off.  */
	freqcntr_deinit();

	setSpeed_rpm = 0;
	stable = false;

	return true;
}

bool bldc_setMaxCurrent_mA(uint16_t iLimit_mA) {
	uint32_t vref_mV, bldciref_mV;
	uint32_t onPeriod;

	/* Calculate the voltage that we want at the A4960's REF pin.  See page 18
	 * of the A4960 datasheet for an explanation. */
	vref_mV = (iLimit_mA * ISENSE_GAIN * RSENSE_MILLIOHMS) / 1000;

	/* We resistor-divide and low-pass filter a PWM output to generate a DC
	 * level at the REF pin.  Here we back-calculate the DC voltage that we'd
	 * want to apply to the input of the resistor divider. */
	bldciref_mV = ((R30 + R29) * vref_mV) / R29;

	/* From the desired DC voltage at the input of the resistor divider, we
	 * calculate the necessary on period knowing the total period of the
	 * PWM generator and that the reference voltage is 3.3V.*/
	onPeriod = (bldciref_mV * pwm_getPeriod()) / 3300;

	if (onPeriod > pwm_getPeriod()) {
		return false;
	}

	if (!pwm_setOnPeriod(BLDCIREF_PWM_CHNL, onPeriod)) {
		return false;
	}

	return true;
}

bool bldc_setSpeed(uint16_t speed_rpm, bool reverse, uint16_t brakeTime_ms, app_sched_event_handler_t bldcEventHandler) {
	uint32_t err_code;
	motionPrimitive_t motionPrimitive;

	if (!initialized) {
		return false;
	}

	/* If the caller passed a speed of 0, we brake the motor in order to
	 * decelerate it as rapidly as possible. */
	if (speed_rpm == 0) {
		bldc_stop(brakeTime_ms);

		if ((brakeTime_ms == 0) && (bldcEventHandler != NULL)) {
			motionPrimitive = MOTION_PRIMITIVE_BLDC_COASTING;
			err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), bldcEventHandler);
			APP_ERROR_CHECK(err_code);
		}

		return true;
	}

	eventHandler = bldcEventHandler;

	/* Turn on the BLDC controller IC and configure (or reconfigure) it if
	 * necessary. */
	power_setVBATSWState(VBATSW_USER_BLDC, true);
	if (!configured || needToReconfigure) {
		bldc_config();
	}

	/* Limit the speed to about 20000 RPM.  With fresh batteries, we can hit
	 * ~25 KRPM, but 22 KRPM is more conservative. */
	if (speed_rpm > 20000) {
		speed_rpm = 20000;
	} else if (speed_rpm < 3000) {
		speed_rpm = 3000;
	}

	/* Save the set point speed.  This will be used by the speed control
	 * loop. */
	setSpeed_rpm = speed_rpm;

	/* At lower speeds, controller is unstable unless the control constant
	 * are reduced because the output current levels are so low it is easy
	 * to overshoot the correct current level.*/
	if (setSpeed_rpm < 6000) {
		bldc_setKP(3, 1);
		bldc_setKI(3, 100000);
	} else {
		bldc_setKP(3, 1);
		bldc_setKI(16, 100000);
	}

	/* Start the frequency counter which we will use to measure the motor's
	 * RPM. */
	freqcntr_init();

	/* Start the motor running */
	bldc_start(reverse);

	bldcModeCurrent = BLDC_MODE_CONSTANT_SPEED;

	app_timer_cnt_get(&startTime_rtcTicks);

	/* Stop the speed control timer before restarting it in case it is already
	 * running. */
	err_code = app_timer_stop(bldc_speedControlTimerID);
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_start(bldc_speedControlTimerID,
			APP_TIMER_TICKS(20, APP_TIMER_PRESCALER), &bldcModeConstSpeed);
	APP_ERROR_CHECK(err_code);

	/* Manually call the control function immediately so that we do not have to
	 * wait for the time to expire before sending a control signal to the
	 * controller.  By passing 'true' to the control loop function, we are
	 * instructing it to reinitialize. */
	bldc_speedControlLoop(true, BLDC_MODE_CONSTANT_SPEED);

	return true;
}

bool bldc_setAccel(uint16_t accel_mA, uint16_t time_ms, bool reverse, app_sched_event_handler_t bldcEventHandler) {
	uint32_t err_code;

	if (!initialized) {
		return false;
	}

	if (time_ms > 1000) {
		time_ms = 1000;
	}

	if (accel_mA > ACCEL_CURRENT_MAX_MA) {
		accel_mA = ACCEL_CURRENT_MAX_MA;
	}

	eventHandler = bldcEventHandler;

	/* Turn on the BLDC controller IC and configure (or reconfigure) it if
	 * necessary. */
	power_setVBATSWState(VBATSW_USER_BLDC, true);
	if (!configured || needToReconfigure) {
		bldc_config();
	}

	/* Set the BLDC controller chip's software-controlled current limit to its
	 * maximum value.  We will control acceleration solely though the voltage
	 * on the IREF pin. */
	bldc_setIRefRatio(0x0F);

	/* Start driving voltage on the IREF pin of the A4960 (and allow it to
	 * stabilize if the motor is not already being driven at constant speed).
	 * This stabilization should lead to more constant acceleration. */
	bldc_setMaxCurrent_mA(accel_mA);
	if (bldcModeCurrent != BLDC_MODE_CONSTANT_SPEED) {
		delay_ms(25);
	}

	/* Start the motor spinning */
	bldc_start(reverse);

	bldcModeCurrent = BLDC_MODE_ACCEL;

	/* The speed control timer may already be running if the motor is already
	 * spinning.  Once we set the accelModeActive flag, the next time the timer
	 * handler is called, it will put the motor into coast mode.  To avoid this
	 * from happening before the acceleration is complete, we stop the timer
	 * before setting the accelModeActive flag and restarting the timer.
	 * Unfortunately, calling app_timer_stop only schedules the timer to be
	 * stopped.  It is possible that the timer handler is already in the queue
	 * to be executed and that it will be executed once more before the timer
	 * is actually stopped.  To avoid this scenario, we make use of the
	 * constSpeedModeActive flag in the timer handler.  */
	err_code = app_timer_stop(bldc_speedControlTimerID);
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_start(bldc_speedControlTimerID,
			APP_TIMER_TICKS(time_ms, APP_TIMER_PRESCALER), &bldcModeAccel);
	APP_ERROR_CHECK(err_code);

	return true;
}

void bldc_setReverseDirections(bool reverse) {
	directionsReversed = reverse;
}

bool bldc_getReverseDirections() {
	return directionsReversed;
}

bool bldc_translateDirection(bool reverse) {
	if (directionsReversed) {
		return (!reverse);
	} else {
		return reverse;
	}
}

bool bldc_setKP(int32_t numerator, int32_t denominator) {
	if ((numerator < 0) || (denominator < 0)) {
		return false;
	}

	kp_numerator = numerator;
	kp_denominator = denominator;

	return true;
}

bool bldc_getKP(int32_t *numerator, int32_t *denominator) {
	*numerator = kp_numerator;
	*denominator = kp_denominator;
	return true;
}

bool bldc_setKI(int32_t numerator, int32_t denominator) {
	if ((numerator < 0) || (denominator < 0)) {
		return false;
	}

	ki_numerator = numerator;
	ki_denominator = denominator;

	return true;
}

bool bldc_getKI(int32_t *numerator, int32_t *denominator) {
	*numerator = ki_numerator;
	*denominator = ki_denominator;
	return true;
}

void bldc_speedControlTimerHandler(void *p_context) {
	UNUSED_PARAMETER(p_context);

	/* Call the speed control loop with 'false' to avoid reinitializing
	 * the controller's state. We also pass the operating mode (constant speed,
	 * accelerate, e-brake, cost, off) at the time when the timer was started.
	 * This mode may be different than the current mode if there was a pending
	 * timer event in the event queue before the timer was restarted. */
	bldcMode_t bldcModeTimerStart = *(bldcMode_t *)p_context;
	bldc_speedControlLoop(false, bldcModeTimerStart);
}

bool bldc_setIRefRatio(uint8_t refRatio) {
	uint16_t config1;

	/* If the BLDC system has not been initialized or the BLDC controller does
	 * not have power, return false as we cannot set the IREF ratio. */
	if (!initialized || !power_getVBATSWState()) {
		return false;
	}

	/* If we need to reconfigure the BLDC controller as the result of a power
	 * glitch, we do so now before changing the IREF ratio.  If we did not
	 * re-configure now, we would inevitably reconfigure later and over-write
	 * the IREF ratio we're setting here with the default value. */
	if (!configured || needToReconfigure) {
		bldc_config();
	}

	/* The reference ratio is 6.25% * (refRatio + 1) where
	 * 0 <= refRatio <= 15. */
	if (refRatio > 0x0F) {
		return false;
	}

	/* Read the current value of the CONFIG1 register */
	if (!a4960_readReg(A4960_CONFIG1_REG_ADDR, &config1)) {
		return false;
	}

	/* Clear the old reference ratio bits and then write the new onces in their
	 * place. */
	config1 &= ~A4960_REFERENCERATIO_MASK;
	config1 |= refRatio << A4960_REFERENCERATIO_POSN;

	/* Write the modified register back to the controller */
	if (!a4960_writeReg(A4960_CONFIG1_REG_ADDR, config1, NULL)) {
		return false;
	}

	return true;
}

void bldc_speedControlLoop(bool reinit, bldcMode_t bldcModeTimerStart) {
	int32_t actual_rpm, error_rpm;
	uint16_t controlCurrent;
	uint32_t rtcTicks, elapsedTicks;
	uint32_t elapsedTime_us;
	int32_t pError;
	motionPrimitive_t motionPrimitive;
	uint32_t err_code;
	uint32_t elapsedTime_msec;

#if (BLDC_DEBUG)
	char str[125];
#endif

	static uint32_t lastCallRTCTicks = 0;
	static int32_t intError = 0;
	static unsigned int posSaturationCount = 0;
	static unsigned int negSaturationCount = 0;
	static uint8_t iRefRatio = 15;
	static bool stabilized = false;
	static bool timeoutPrimitiveSent = false;

	/* If there is something wrong, either the BLDC system has not been
	 * initialized, the BLDC IC does not have battery power, the IC has not
	 * been configured, the IC needs to be reconfigured as the result of a
	 * power glitch, or the motor is not running, there is no reason for the
	 * control loop to be executing.  Consequently, we stop the timer that
	 * triggers the control loop and then we make sure that the motor is
	 * stopped before returning. */
	if (!initialized || !power_getVBATSWState() ||
			!configured || needToReconfigure) {
		app_timer_stop(bldc_speedControlTimerID);
		bldc_stop(0);
		return;
	}

	if (bldcModeTimerStart != bldcModeCurrent) {
		/* If the BLDC operating mode under which the timer which generated
		 * this event was started does not match the current BLDC operating
		 * mode, we return as the we do not want to process a timer event
		 * from a mode that is no longer active.  This can happen when there
		 * is already a timer event pending in the events queue when we stop
		 * and restarted the timer in a new mode. */
		return;
	}

	if (bldcModeCurrent == BLDC_MODE_ACCEL) {
		/* If we have just finished accelerating, we allow the motor to coast
		 * to a stop.
		 * By calling the bldc_stop function with an argument of 0, we will
		 * restart the speed control loop timer.  When it next expires, the
		 * motor will have spun down to a stop, at which point we can remove
		 * power. */
		bldc_stop(0);

		/* For now, queue an event to indicate that the motor is now
		 * coasting. */
		if (eventHandler != NULL) {
			motionPrimitive = MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE;
			err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		return;
	} else if (bldcModeCurrent == BLDC_MODE_EBRAKE) {
		/* If we have just finished applying the e-brake for the specified
		 * amount of time we allow the motor to coast to a stop, (if it is
		 * not already stopped as a consequence of actuating the electronic
		 * break for enough time). By calling the bldc_stop function with an
		 * argument of 0, we will restart the speed control loop timer.
		 * When it next expires, the motor will have spun down to a stop,
		 * at which point we can remove power. */
		bldc_stop(0);

		/* For now, queue an event to indicate that the motor is now
		 * coasting. */
		if (eventHandler != NULL) {
			motionPrimitive = MOTION_PRIMITIVE_BLDC_COASTING;
			err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
		return;
	} else if ((bldcModeCurrent == BLDC_MODE_COAST) || (bldcModeCurrent == BLDC_MODE_OFF)) {
		/* When the current operation mode is COAST and the timer expires, it
		 * indicates that enough time has expired for the motor to completely
		 * spin down to its stopped state.  As such, we disable the speed
		 * control timer and turn off the BLDC controller. */
		app_timer_stop(bldc_speedControlTimerID);

		/* Withdraw our request to keep the VBATSW supply powered.  If nothing
		 * else is using the VBATSW supply, this will turn it off. */
		power_setVBATSWState(VBATSW_USER_BLDC, false);

		bldcModeCurrent = BLDC_MODE_OFF;

		/* Finally, queue an event to indicate that the motor is definitely
		 * stopped and power potentially removed. */
		if (eventHandler != NULL) {
			motionPrimitive = MOTION_PRIMITIVE_BLDC_STOPPED;
			err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
	}

	/* At this point, bldcModeCurrent = BLDC_MODE_CONSTANT_SPEED */

	/* Get the current time expressed as ticks of the 32.768kHz RTC */
	app_timer_cnt_get(&rtcTicks);

	/* The RTC is only a 24-bit counter, so when subtracting the two
	 * tick counts to find the elapsed number of ticks, we must mask out the
	 * 8 most significant bits. */
	elapsedTicks = 0x00FFFFFF & (rtcTicks - lastCallRTCTicks);
	elapsedTime_us = elapsedTicks * USEC_PER_APP_TIMER_TICK;

	/* If the caller wants to reinitialize the controller we reset the
	 * integrator and assume that the control loop has not been previously
	 * executed so that the elapsed time is 0. */
	if (reinit) {
		intError = 0;
		elapsedTime_us = 0;
		posSaturationCount = negSaturationCount = 0;

		iRefRatio = 0;
		bldc_setIRefRatio(0);

		stabilized = false;
		timeoutPrimitiveSent = false;
	}

	/* The first thing we do is recalculate the frequency of the BLDC
	 * controller's TACH output. */
	freqcntr_updateFreq();

	/* Then we translate this frequency into the RPM of the motor */
	actual_rpm = (freqcntr_getFreq_Hz() * 60) / 42;

	/* Subtract the actual speed from the set point speed to find the raw RPM
	 * error. */
	error_rpm = setSpeed_rpm - actual_rpm;

	/* Calculate the proportional error */
	pError = (kp_numerator * error_rpm) / kp_denominator;

	/* Calculate the integrated error */
	intError += (ki_numerator * error_rpm * (int32_t)elapsedTime_us) / ki_denominator;

	/* We limit the point at which the integrator saturates to the maximum
	 * allowable control input (3000mA).  */
	if (intError < -3000) {
		intError = -3000;
	} else if (intError > 3000) {
		intError = 3000;
	}

	/* Sum the proportional and integral errors with the restriction that the
	 * control current can never be set to less than 0 and it can never be set
	 * to greater than 3000mA. */
	if (pError + intError < 0) {
		controlCurrent = 0;
	} else if (pError + intError > 3000) {
		controlCurrent = 3000;
	} else {
		controlCurrent = pError + intError;
	}

	/* If the control current is saturating, we attempt to change the A4960's
	 * internal current reference ratio which effectively scales the control
	 * current input. */
	if (controlCurrent == 0) {
		negSaturationCount++;

		/* The length of time over which negative saturation must occur before
		 * changing the current reference ratio is much greater than threshold
		 * for positive saturating because the flywheel only decelerates due to
		 * friction, where as the flywheel can be actively accelerated by
		 * applying more current. */
		if ((negSaturationCount > 50) && (iRefRatio > 0)) {
			bldc_setIRefRatio(--iRefRatio);
			negSaturationCount = 0;
		}
	} else if (controlCurrent == 3000) {
		posSaturationCount++;

		if ((posSaturationCount > 10) && (iRefRatio < 15)) {
			bldc_setIRefRatio(++iRefRatio);
			posSaturationCount = 0;
		}
	} else {
		posSaturationCount = negSaturationCount = 0;
	}

	/* Update the PWM output which is low-pass filtered to produce the analog
	 * level which controls motor current. */
	bldc_setMaxCurrent_mA(controlCurrent);

	/* Save the current tick count for the next iteration of the loop */
	lastCallRTCTicks = rtcTicks;

#if (BLDC_DEBUG)
	snprintf(str, sizeof(str), "Time [us]: %lu, Speed: %lu, Error: %ld, pError: %ld, intError: %ld, iRef: %u\r\n", elapsedTime_us, actual_rpm, error_rpm, pError, intError, iRefRatio);
	app_uart_put_string(str);
#endif

	/* Check whether the motor's speed has stabilized.   */
	bldc_updateStable(error_rpm);

	/* If the motor's speed _just_ stabilized, we set the stabilized flag.  We
	 * will not clear this flag until we reinitialize the speed controller. */
	if (bldc_isStable() && !stabilized) {
		stabilized = true;
		/* If the motor's speed just stabilized and there is a callback which
		 * needs to be executed, we add the callback to the scheduler's
		 * queue now. */
		if (eventHandler != NULL) {
			motionPrimitive = MOTION_PRIMITIVE_BLDC_STABLE;
			err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
			APP_ERROR_CHECK(err_code);
		}
	}

	/* Compute the amount of time that has elapsed since we turned the motor
	 * on.*/
	elapsedTime_msec = ((0x00FFFFFF & (rtcTicks - startTime_rtcTicks)) * USEC_PER_APP_TIMER_TICK) / 1000;
	/* If the elapsed time exceeds the maximum amount of time which is should
	 * take to reach the desired speed, and if there is a valid callback
	 * function, we add the callback to the scheduler and with an argument
	 * indicating that the motor did not reach its desired speed within a
	 * reasonable amount of time. */
	if (!stabilized && (elapsedTime_msec > bldc_getSpeedTime_ms(setSpeed_rpm)) && (eventHandler != NULL) && !timeoutPrimitiveSent) {
		motionPrimitive = MOTION_PRIMITIVE_BLDC_TIMEOUT;
		err_code = app_sched_event_put(&motionPrimitive, sizeof(motionPrimitive), eventHandler);
		APP_ERROR_CHECK(err_code);
		timeoutPrimitiveSent = true;
	}
}

uint32_t bldc_getSpeedTime_ms(uint32_t rpm) {
	uint32_t time_ms;

	/* This computes the maximum number of milliseconds it should take the BLDC
	 * motor to reach the desired RPM.  This equation was experimentally
	 * determined. */
	time_ms = (rpm / 3) + 2000;
	return time_ms;
}

bool bldc_updateStable(int32_t newestError) {
	int32_t meanError, meanErrorSquare;
	int32_t varianceError;

#if (BLDC_DEBUG)
	char str[150];
#endif

	static int validSamples = 0;
	static int oldestSample = 0;

	static int32_t sumError = 0;
	static int32_t sumErrorSquare = 0;
	static int32_t error[STABLILITY_SAMPLE_SIZE];
	static int32_t errorSquare[STABLILITY_SAMPLE_SIZE];

	if (validSamples == STABLILITY_SAMPLE_SIZE) {
		/* If the error sample buffers full, we subtract the oldest error value
		 * from the sum of the errors, we then replace that spot in the buffer
		 * with value with the newest error value, and then add the newest
		 * error to the running sum. */
		sumError -= error[oldestSample];
		sumError += error[oldestSample] = newestError;
		/* We do the exact same thing for the squared error buffer */
		sumErrorSquare -= errorSquare[oldestSample];
		sumErrorSquare += errorSquare[oldestSample] = newestError * newestError;
		/* Update the pointer to the oldest sample so that the next time this
		 * function is called we remove the oldest sample instead of the sample
		 * that we just added. */
		oldestSample = (oldestSample + 1) % STABLILITY_SAMPLE_SIZE;
	} else {
		/* If the error buffer is not yet full, we insert the newest value
		 * into the first empty position, add the value to the running sum of
		 * errors */
		sumError += error[validSamples] = newestError;
		/* We do the exact same thing of the squared error buffer. */
		sumErrorSquare += errorSquare[validSamples] = newestError * newestError;
		/* Increment the number of valid samples. The oldest sample remains
		 * unchanged because nothing in the buffer was overwritten. */
		validSamples++;
	}

	if (validSamples < STABLILITY_SAMPLE_SIZE) {
		stable = false;
		return false;
	}

	meanError = sumError / STABLILITY_SAMPLE_SIZE;
	meanErrorSquare = sumErrorSquare / STABLILITY_SAMPLE_SIZE;
	varianceError = meanErrorSquare - (meanError * meanError);

#if (BLDC_DEBUG)
	snprintf(str, sizeof(str), "Error Mean: %ld, Variance: %ld\r\n", meanError, varianceError);
	app_uart_put_string(str);
#endif

	/* Take the absolute value of the mean error so that we can determine
	 * whether it is within a certain percentage of the set point speed. */
	if (meanError < 0) {
		meanError = -meanError;
	}

	if ((setSpeed_rpm >= 6000) && (meanError < setSpeed_rpm / 100) && (varianceError <= 2500)) {
		/* To claim that the motor has reached a stable speed, we require that
		 * there be less than 1% error in the speed and the variance of the
		 * error be less than an experimentally chosen value. */
		stable = true;
#if (BLDC_DEBUG)
		snprintf(str, sizeof(str), "STABILIZED\r\n");
		app_uart_put_string(str);
#endif
	} else if ((setSpeed_rpm < 6000) && (meanError < setSpeed_rpm / 100)) {
		/* For motor speeds less than 6000 RPM, the measured speed gets too
		 * noisy to enforce a variance requirement on the speed error. */
		stable = true;
#if (BLDC_DEBUG)
		snprintf(str, sizeof(str), "STABILIZED\r\n");
		app_uart_put_string(str);
#endif
	} else {
		stable = false;
#if (BLDC_DEBUG)
		snprintf(str, sizeof(str), "NOT STABLE\r\n");
		app_uart_put_string(str);
#endif
	}

	return stable;
}

bool bldc_isStable() {
	/* If the controller does not have battery power or the motor is not in
	 * constant speed mode, we say, by definition, that the motor speed is not
	 * stable. */
	if (!power_getVBATSWState() || (bldcModeCurrent != BLDC_MODE_CONSTANT_SPEED)) {
		return false;
	}

	return stable;
}
