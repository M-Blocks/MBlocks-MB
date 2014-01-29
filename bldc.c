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
#include "bldc.h"

#define BLDC_DEBUG			0

#define RSENSE_MILLIOHMS	10
#define ISENSE_GAIN			10

/* Resistors used in divider which sets maximum controller IC current */
#define R30					10000
#define R29					1000

/* Number of points used in the variance calculation that determines whether
 * the motor has reached a steady state speed. */
#define STABLILITY_SAMPLE_SIZE	20

/* Closed loop control constants */
#if (0)
#define K_P_NUMERATOR		30
#define K_P_DENOMINATOR		10000
#define K_I_NUMERATOR		5
#define K_I_DENOMINATOR		50000
#endif

static int32_t kp_numerator = 3;
static int32_t kp_denominator = 1;
static int32_t ki_numerator = 5;
static int32_t ki_denominator = 1000;


static bool initialized = false;
static bool bldcOn = false;
static bool stable = false;

static bool bldcRunning = false;
static uint16_t setSpeed_rpm;

static app_timer_id_t bldc_speedControlTimerID;

static bool bldc_setIRefRatio(uint8_t refRatio);
static void bldc_speedControlTimerHandler(void *p_context);
static void bldc_speedControlLoop(bool reinit);
static void bldc_updateStable(int32_t newestError);

bool bldc_init() {
	uint32_t err_code;

    err_code = app_timer_create(&bldc_speedControlTimerID, APP_TIMER_MODE_REPEATED, bldc_speedControlTimerHandler);
    APP_ERROR_CHECK(err_code);

    initialized = true;

    return true;
}

bool bldc_on() {
	if (!initialized) {
		return false;
	}

	/* The BLDC controller IC's reset line also switches on the power stage of
	 * the driver. */
	nrf_gpio_pin_set(BLDCRESETN_PIN_NO);
	bldcOn = true;

	nrf_delay_ms(25);

	if (!bldc_config()) {
		/* If we fail to configure the controller's parameters, we turn it
		 * off before returning failure. */
		bldc_off();
		return false;
	}

	return true;
}

bool bldc_off() {
	if (!initialized) {
		return false;
	}

	/* Stop the speed control timer to remove the control loop overhead */
	app_timer_stop(bldc_speedControlTimerID);

	/* Remove power from the BLDC controller */
	nrf_gpio_pin_clear(BLDCRESETN_PIN_NO);

	bldcOn = false;
	setSpeed_rpm = 0;
	stable = false;

	return true;
}

bool bldc_isOn() {
	return bldcOn;
}

bool bldc_config() {
	if (!initialized || !bldcOn) {
		return false;
	}

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

	/* Duty-cycle limited start-up torque, 6.25% hold torque limit, 58ms hold
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

	return true;
}

bool bldc_run(bool reverse) {
	uint16_t runReg;

	if (!initialized || !bldcOn) {
		return false;
	}

	/* Read the current value of the run register */
	if (!a4960_readReg(A4960_RUN_REG_ADDR, &runReg)) {
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

	/* Set the run bit */
	runReg |= (0x01 << A4960_RUN_POSN);

	/* Write the modified run register back to the A4960 */
	if (!a4960_writeReg(A4960_RUN_REG_ADDR, runReg, NULL)) {
		return false;
	}

	bldcRunning = true;

	return true;
}

bool bldc_stop(bool brake) {
	uint16_t runReg;

	if (!initialized || !bldcOn) {
		return false;
	}

	/* Read the current value of the run register */
	if (!a4960_readReg(A4960_RUN_REG_ADDR, &runReg)) {
		return false;
	}

	/* Stop the speed control timer to remove the control loop overhead */
	app_timer_stop(bldc_speedControlTimerID);

	if (brake) {
		/* Set the brake bit if requested by the caller.  For the brake to be
	     * effective, we have to leave the run bit set.  We assume that the run
	     * bit is already set, but it it is not the motor is already stopped,
	     * so the state of the brake bit does not matter. */
		runReg |= (0x01 << A4960_BRAKE_POSN);
	} else {
		/* If we are allowing the motor to coast to a stop, we clear the run
		 * bit.  The state of the brake bit does not matter once the run bit is
		 * cleared. */
		runReg &= ~(0x01 << A4960_RUN_POSN);
	}

	/* Write the modified run register back to the A4960 */
	if (!a4960_writeReg(A4960_RUN_REG_ADDR, runReg, NULL)) {
		return false;
	}

	setSpeed_rpm = 0;
	bldcRunning = false;
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
	 * want to aplpy to the input of the resistor divider. */
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

bool bldc_setSpeed(uint16_t speed_rpm, bool reverse) {
	uint32_t err_code;

	if (!initialized) {
		return false;
	}

	/* If the caller passed a speed of 0, we brake the motor in order to
	 * decelerate it as rapidly as possible. */
	if (speed_rpm == 0) {
		bldc_stop(true);
		return true;
	}

	/* Limit the speed to about 22000 RPM.  With fresh batteries, we can hit
	 * ~25 KRPM, but 22 KRPM is more conservative. */
	if (speed_rpm > 20000) {
		speed_rpm = 20000;
	} else if (speed_rpm < 3000) {
		speed_rpm = 3000;
	}

	/* Turn on the BLDC controller IC */
	if (!bldc_isOn()) {
		bldc_on();
	}

	/* Save the set point speed.  This will be used by the speed control
	 * loop. */
	setSpeed_rpm = speed_rpm;


	/* At lower speeds, controller is unstable unless the control constant
	 * are reduced because the output current levels are so low it is easy
	 * to overshoot the correct current level.*/
	if (setSpeed_rpm < 6000) {
		bldc_setKP(3, 1);
		bldc_setKI(1, 1000);
	} else {
		bldc_setKP(3, 1);
		bldc_setKI(5, 1000);
	}

	/* Start the timer that will drive the control loop.  If the timer is
	 * already running, this will be ignored and no harm will result. */
	err_code = app_timer_start(bldc_speedControlTimerID, APP_TIMER_TICKS(20, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

	/* Start the motor running */
	bldc_run(reverse);

	/* Manually call the control function immediately so that we do not have to
	 * wait for the time to expire before sending a control signal to the
	 * controller.  By passing 'true' to the control loop function, we are
	 * instructing it to reinitialize. */
	bldc_speedControlLoop(true);

	return true;
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
	 * the controller's state. */
	bldc_speedControlLoop(false);
}

bool bldc_setIRefRatio(uint8_t refRatio) {
	uint16_t config1;

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

void bldc_speedControlLoop(bool reinit) {
	int32_t actual_rpm, error_rpm;
	uint16_t controlCurrent;
	uint32_t rtcTicks, elapsedTicks;
	int32_t pError;

#if (BLDC_DEBUG)
	char str[125];
#endif

	static uint32_t lastCallRTCTicks = 0;
	static int32_t intError = 0;
	static unsigned int posSaturationCount = 0;
	static unsigned int negSaturationCount = 0;
	static uint8_t iRefRatio = 15;


	if (!initialized || !bldcOn || !bldcRunning) {
		return;
	}

	/* Get the current time expressed as ticks of the 32.768kHz RTC */
	app_timer_cnt_get(&rtcTicks);

	/* The RTC is only a 24-bit counter, so when subtracting the two
	 * tick counts to find the elapsed number of ticks, we must mask out the
	 * 8 most significant bits. */
	elapsedTicks = 0x00FFFFFF & (rtcTicks - lastCallRTCTicks);

	/* If the caller wants to reinitialize the controller we reset the
	 * integrator and assume that the control loop has not been previously
	 * executed so that the elapsed time is 0. */
	if (reinit) {
		intError = 0;
		elapsedTicks = 0;
		posSaturationCount = negSaturationCount = 0;

		iRefRatio = 0;
		bldc_setIRefRatio(0);
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
	intError += (ki_numerator * error_rpm * (int32_t)elapsedTicks) / ki_denominator;

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
		 * for positive saturing because the flywheel only decelerates due to
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
	snprintf(str, sizeof(str), "Ticks: %lu, Speed: %lu, Error: %ld, pError: %ld, intError: %ld, iRef: %u\r\n", elapsedTicks, actual_rpm, error_rpm, pError, intError, iRefRatio);
	app_uart_put_string(str);
#endif

	bldc_updateStable(error_rpm);
}

void bldc_updateStable(int32_t newestError) {
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
		return;
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

}

bool bldc_isStable() {
	if (!bldcOn || !bldcRunning) {
		return false;
	}

	return stable;
}
