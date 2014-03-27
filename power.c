/*
 * power.c
 *
 *  Created on: Nov 12, 2013
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "global.h"
#include "pins.h"
#include "util.h"
#include "adc.h"
#include "pwm.h"
#include "led.h"
#include "bleApp.h"
#include "power.h"

#define CELL_CHARGED_THRESHOLD_MV			4200	/* Stop charging as soon as all cells rise to this voltage, really they should never rise over 4.2V and we should halt charging when the current falls sufficiently. */

#define CELL_DISCHARGED_THRESHOLD_MV		3000	/* Stop discharging as soon as all cells fall to this voltage */

#define CELL_RECHARGE_THRESHOLD_MV			3900	/* Start charging as soon as any cell falls below this voltage */
#define CELL_PRECHARGE_THRESHOLD_MV			2800	/* Cell voltage below which we charge at a 0.1C rate */

#define CELL_UNDERVOLTAGE_THRESHOLD_MV		2500	/* Cell voltage under which we assume something is wrong */
#define CELL_OVERVOLTAGE_THRESHOLD_MV		4300	/* Cell voltage over which we assume something is wrong */

#define MAXIMUM_IMBALANCE_MV				15		/* Maximum allowed imbalance between cells, over which we activate the cell's self-discharge switch */

#define CHARGER_INPUT_VOLTAGE_START_THRESHOLD_MV	4300	/* Minimum input voltage above which we start charging. */
#define CHARGER_INPUT_VOLTAGE_STOP_THRESHOLD_MV		4100	/* Maximum input voltage below which we stop charging. */

#define PRECHARGE_CURRENT_MA				12		/* Precharge current, ~0.1C */
#define CHARGE_CURRENT_1C_MA				125		/* Normal 1C charge rate */
#define CHARGE_CURRENT_MAX_MA				80		/* Current limit imposed by the charge itself */
#define CHARGE_CURRENT_CUTOFF_THRESHOLD_MA	12		/* Once in constant voltage mode, current below which we stop the charging process */

#define CHARGE_TIME_MAXIMUM_MS				10800000 /* Maximum time (3 hours) we run the charger before assuming something is wrong */

#define RTC_TICKS_PER_MS					33		/* 32.768 ticks of the RTS per millisecond (32.768kHz RTC) */

#define RUN_MODE_CURRENT_UA					12100
#define BLE_ADVERTISING_CURRENT_UA			1500
#define BLE_CONNECTED_CURRENT_UA			200
#define LED0_CURRENT_UA						1800
#define LED1_CURRENT_UA						2600
#define LED2_CURRENT_UA						1200

app_timer_id_t power_timer_id;

//static power_chargeState_t chargeState = POWER_CHARGESTATE_OFF;
static power_chargeState_t chargeState = POWER_CHARGESTATE_STANDBY;
static power_chargeError_t chargeError = POWER_CHARGEERROR_NOERROR;
static bool chargeTimerRunning = false;
static uint32_t chargingTime_rtcTicks = 0;
static bool debug = false;
static uint8_t dischargeSwitchState = 0x00;
static uint16_t chargeCurrentLimit_mA = 0;

static bool VBATSWEnabled = false;


/* These flags are set when VIN is removed while the charger is still active.
 * When this happens, the 3.3V rail can dip sufficiently to reset the BLDC
 * controller and possibly the IMU.  */
#define IMPROP_TERM_FLAG_COUNT	4
bool *impropTermFlags[IMPROP_TERM_FLAG_COUNT] = {NULL, NULL, NULL, NULL};

static void power_setImproperTerminationFlags(void);

bool power_registerImproperTerminationFlag(bool *flag) {
	uint8_t i;

	for (i=0; i<IMPROP_TERM_FLAG_COUNT; i++) {
		if (impropTermFlags[i] == NULL) {
			impropTermFlags[i] = flag;
			return true;
		}
	}
	return false;
}

bool power_unregisterImproperTerminationFlag(bool *flag) {
	uint8_t i;

	for (i=0; i<IMPROP_TERM_FLAG_COUNT; i++) {
		if (impropTermFlags[i] == flag) {
			impropTermFlags[i] = NULL;
			return true;
		}
	}
	return false;
}

void power_setImproperTerminationFlags() {
	uint8_t i;

	for (i=0; i<IMPROP_TERM_FLAG_COUNT; i++) {
		if (impropTermFlags[i] != NULL) {
			*impropTermFlags[i] = true;
		}
	}
}

void power_setVBATSWState(bool enabled) {
	/* The BLDC controller IC's reset line also switches on the power stage of
	 * the driver. */
	if (enabled) {
		nrf_gpio_pin_set(BLDCRESETN_PIN_NO);
		VBATSWEnabled = true;
	} else {
		nrf_gpio_pin_clear(BLDCRESETN_PIN_NO);
		VBATSWEnabled = false;
	}
}

bool power_getVBATSWState() {
	return VBATSWEnabled;
}

bool power_setDischargeSwitches(uint8_t switches) {
	if (switches & 0xF0) {
		return false;
	}

	if (switches & 0x01) {
		nrf_gpio_pin_set(BAT1DISCHRG_PIN_NO);
	} else {
		nrf_gpio_pin_clear(BAT1DISCHRG_PIN_NO);
	}

	if (switches & 0x02) {
		nrf_gpio_pin_set(BAT2DISCHRG_PIN_NO);
	} else {
		nrf_gpio_pin_clear(BAT2DISCHRG_PIN_NO);
	}

	if (switches & 0x04) {
		nrf_gpio_pin_set(BAT3DISCHRG_PIN_NO);
	} else {
		nrf_gpio_pin_clear(BAT3DISCHRG_PIN_NO);
	}

	if (switches & 0x08) {
		nrf_gpio_pin_set(BAT4DISCHRG_PIN_NO);
	} else {
		nrf_gpio_pin_clear(BAT4DISCHRG_PIN_NO);
	}

	dischargeSwitchState = switches;

	return true;
}

uint8_t power_getDischargeSwitches() {
	return dischargeSwitchState;
}

bool power_setChargerCurrentLimit_mA (uint16_t iLimit_mA) {
	uint32_t viadj_mV_x16;
	uint32_t pwmAvg_mV_x16;
	uint32_t onCounts;

	/* The maximum allowable charge current is 150mA.  This guarantees that all
	 * of the following math works correctly. */
	if (iLimit_mA > 100) {
		return false;
	}

	/* The maximum charging current is set according to
	 * ICHRG = ((1.263 - 0.8 * VIADJ) / 25) / RSENSE
	 * where RSENSE is 0.33 Ohms and VIADJ is the voltage at the IAJD pin of
	 * the LT1618 charger IC.  Solving this for VIADJ, we arrive at
	 * VIADJ = 1.25 * (1.263 - 8.25 * ICHRG).  To use integer math, we will
	 * multiply each side of the equation by 16 initially and then divide the
	 * final result by 16.  Additionally, we express the current in mA and the
	 * voltage in mV, so we need to multiply 1.263 by 1000. */
	viadj_mV_x16 = 5 * (5052 - 33 * iLimit_mA);

	/* There is a resistor divider between the processor and the charger IC
	 * which attenuates the average value of the PWM signal by a factor of
	 * 12.7 / (16.2 + 12.7) = 0.4394.  To find the desired average value of the
	 * PWM signal, we must divide VIADJ by this factor.  To increase the
	 * precision of the calculation, we will pre-multiply by 1000. */
	pwmAvg_mV_x16 = (viadj_mV_x16 * 2276) / 1000;

	/* Finally, we need to convert this average PWM value into a number of
	 * on-counts of the complete PWM period.  To do so, we divide the desired
	 * average PWM value by the PWM on-voltage (3.3V) and then multiply by
	 * the PWM period.  To preserve accuracy, we actually pre-multiply by the
	 * period. */
	onCounts = (pwm_getPeriod() * pwmAvg_mV_x16) / (3300 * 16);

	if (onCounts > pwm_getPeriod()) {
		/* Very low charge currents are not obtainable because they require
		 * average PWM values above 3.3V. */
		return false;
	}

	if (!pwm_setOnPeriod(PRECHRGEN_PWM_CHNL, onCounts)) {
		return false;
	}

	chargeCurrentLimit_mA = iLimit_mA;

	return true;
}

uint16_t power_getChargerCurrentLimit_mA() {
	return chargeCurrentLimit_mA;
}

bool power_enableCharger() {
	nrf_gpio_pin_set(CHRGEN_PIN_NO);
	return true;
}

bool power_disableCharger() {
	nrf_gpio_pin_clear(CHRGEN_PIN_NO);
	return true;
}

void power_setChargeState(power_chargeState_t newState) {
	/* If we are changing from some other state to the DISCHARGE state, we
	 * reset the charging timer because we may be moving directly from the
	 * PRECHARGING or CHARGING state thereby skipping the STANDBY or OFF
	 * states where the charging timer is normally reset. */
	if ((chargeState != POWER_CHARGESTATE_DISCHARGE) &&
			(newState == POWER_CHARGESTATE_DISCHARGE)) {
		chargingTime_rtcTicks = 0;
	}

	/* Likewise, if we are changing from some state other than CHARGING to
	 * PRECHARGE, we reset the charging timer because we may be moving directly
	 * from the DISCHARGE state thereby skipping the STANDBY or OFF states
	 * where the charging timer is normally reset. */
	if ((chargeState != POWER_CHARGESTATE_PRECHARGE) &&
			(chargeState != POWER_CHARGESTATE_CHARGING) &&
			(newState == POWER_CHARGESTATE_PRECHARGE)) {
		chargingTime_rtcTicks = 0;
	}

	chargeState = newState;
	power_updateChargeState();
}

power_chargeState_t power_getChargeState() {
	return chargeState;
}

void power_setDebug(bool enable) {
	debug = enable;
}

bool power_getDebug() {
	return debug;
}

uint16_t power_getVIn_mV() {
	/* VIN is divided by 2 with a resistor divider pair, so we multiply by 2 in
	 * order to return the correct voltage. */
	return 2 * adc_avg_mV(VINSENSE_ADC_CHNL, 32);
}

uint16_t power_getBatteryVoltage_mV(uint8_t batNum) {
	uint16_t offset_mV;
	uint16_t offsetPlusFifthActual_mV;
	uint16_t actual_mV;

	/* Return 0 if the battery number is invalid.  Valid numbers are 1...4. */
	if ((batNum <= 0) || (batNum >= 5)) {
		return 0;
	}

	/* Note: the way that the S-8243B numbers the batteries is exactly opposite
	 * what the schematic calls them, i.e. battery 1 according to the S-8243B
	 * is labeled battery 4 on the schematic. */

	if (batNum == 1) {
		/* CTL3 Low, CTL4 High */
		nrf_gpio_cfg_output(LIPROCTL3_PIN_NO);
		nrf_gpio_pin_clear(LIPROCTL3_PIN_NO);
		nrf_gpio_cfg_output(LIPROCTL4_PIN_NO);
		nrf_gpio_pin_set(LIPROCTL4_PIN_NO);
	} else if (batNum == 2) {
		/* CTL3 Open, CTL4 Open */
		nrf_gpio_cfg_input(LIPROCTL3_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_input(LIPROCTL4_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);
	} else if (batNum == 3) {
		/* CTL3 High, CTL4 Low */
		nrf_gpio_cfg_output(LIPROCTL3_PIN_NO);
		nrf_gpio_pin_set(LIPROCTL3_PIN_NO);
		nrf_gpio_cfg_output(LIPROCTL4_PIN_NO);
		nrf_gpio_pin_clear(LIPROCTL4_PIN_NO);
	} else if (batNum == 4) {
		/* CTL3 High, CTL4 High */
		nrf_gpio_cfg_output(LIPROCTL3_PIN_NO);
		nrf_gpio_pin_set(LIPROCTL3_PIN_NO);
		nrf_gpio_cfg_output(LIPROCTL4_PIN_NO);
		nrf_gpio_pin_set(LIPROCTL4_PIN_NO);
	}

	nrf_delay_ms(1);

	offset_mV = adc_avg_mV(LIPROVBATOUT_ADC_CHNL, 32);

	if (batNum == 1) {
		/* CTL3 Low, CTL4 Open */
		nrf_gpio_cfg_output(LIPROCTL3_PIN_NO);
		nrf_gpio_pin_clear(LIPROCTL3_PIN_NO);
		nrf_gpio_cfg_input(LIPROCTL4_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);
	} else if (batNum == 2) {
		/* CTL3 Open, CTL4 Low */
		nrf_gpio_cfg_input(LIPROCTL3_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_output(LIPROCTL4_PIN_NO);
		nrf_gpio_pin_clear(LIPROCTL4_PIN_NO);
	} else if (batNum == 3) {
		/* CTL3 Open, CTL4 High */
		nrf_gpio_cfg_input(LIPROCTL3_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_output(LIPROCTL4_PIN_NO);
		nrf_gpio_pin_set(LIPROCTL4_PIN_NO);
	} else if (batNum == 4) {
		/* CTL3 High, CTL4 Open */
		nrf_gpio_cfg_output(LIPROCTL3_PIN_NO);
		nrf_gpio_pin_set(LIPROCTL3_PIN_NO);
		nrf_gpio_cfg_input(LIPROCTL4_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);
	}

	nrf_delay_ms(1);

	offsetPlusFifthActual_mV = adc_avg_mV(LIPROVBATOUT_ADC_CHNL, 32);

	/* Return both control lines to their high impedance state */
	nrf_gpio_cfg_input(LIPROCTL3_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);
	nrf_gpio_cfg_input(LIPROCTL4_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);

	/* Subtract the offset voltage and multiply by five to arrive at the actual
	 * battery voltage. This comes from the Seiko datasheet. */
	actual_mV = (offsetPlusFifthActual_mV - offset_mV) * 5;

	return actual_mV;
}

uint16_t power_getBatteryVoltageMin_mV() {
	uint8_t batNum;
	uint16_t mV;
	uint16_t minVoltage_mV = UINT16_MAX;

	for (batNum = 1; batNum <= 4; batNum++) {
		mV = power_getBatteryVoltage_mV(batNum);
		if (mV < minVoltage_mV) {
			minVoltage_mV = mV;
		}
	}

	return minVoltage_mV;
}

uint16_t power_getBatteryVoltageMax_mV() {
	uint8_t batNum;
	uint16_t mV;
	uint16_t maxVoltage_mV = 0;

	for (batNum = 1; batNum <= 4; batNum++) {
		mV = power_getBatteryVoltage_mV(batNum);
		if (mV > maxVoltage_mV) {
			maxVoltage_mV = mV;
		}
	}

	return maxVoltage_mV;
}

uint16_t power_getBatteryPackVoltage_mV() {
	int i;
	uint16_t accumVoltage = 0;

	for (i=1; i<=4; i++) {
		accumVoltage += power_getBatteryVoltage_mV(i);
	}

	return accumVoltage;
}

uint16_t power_getChargeCurrent_mA() {
	uint16_t rawVoltage_mV;

	rawVoltage_mV = adc_avg_mV(ICHARGE_ADC_CHNL, 8);

	/* The charge current is sensed by measuring the voltage across a 0.33 Ohm
	 * resistor.  The current sensor has a gain of 50.  To convert the sensor's
	 * output to milliamps, we divide by 0.33 (i.e. multiply by 3) and then
	 * divide by the sensor's gain. */
	return (3 * rawVoltage_mV) / 50;
}

uint16_t power_getEstimatedCurrentConsumption_mA() {
	uint32_t ledCurrent_uA[3] = {LED0_CURRENT_UA, LED1_CURRENT_UA, LED2_CURRENT_UA};
	uint8_t led;
	uint32_t uA = 0;

	uA += RUN_MODE_CURRENT_UA;

	if (bleApp_isConnected()) {
		uA += BLE_CONNECTED_CURRENT_UA;
	} else if (bleApp_isAdvertisingEnabled()) {
		uA += BLE_ADVERTISING_CURRENT_UA;
	}

	for (led = 0; led < 3; led++) {
		uA += (led_getDutyCycle_percent(led) * ledCurrent_uA[led]) / 100;
	}

	return uA / 1000;
}

bool power_isCellUndervoltage() {
	uint16_t minimum_mV;

	minimum_mV = power_getBatteryVoltageMax_mV();

	if (minimum_mV < CELL_UNDERVOLTAGE_THRESHOLD_MV) {
		return true;
	}

	return false;
}

bool power_isCellOvervoltage() {
	uint16_t maximum_mV;

	maximum_mV = power_getBatteryVoltageMax_mV();

	if (maximum_mV > CELL_OVERVOLTAGE_THRESHOLD_MV) {
		return true;
	}

	return false;
}

bool power_isUndertemp() {
	return false;
}

bool power_isOvertemp() {
	return false;
}

void power_timerHandler(void *p_contex) {
		power_updateChargeState();
}

void power_updateChargeState() {
	uint16_t vin_mV;
	bool vinAbsent = false;
	uint8_t shorts = 0x00;
	uint8_t batNum;
	uint16_t minVoltage_mV, maxVoltage_mV;
	bool updateAgain = false;
	uint32_t currentTime_rtcTicks, elapsedTime_rtcTicks;

	static uint32_t lastStateUpdateTime_rtcTicks = 0;

	/* After the Lipo protection IC trips, or when the batteries are first
	 * connected, it will appear to the processor that all batteries are
	 * disconnected because all will read 0mV.  While normally, we consider
	 * such low voltage an error severe enough to stop charging, we ignore
	 * 0mV readings until we have at least attempted to activate the charger.
	 * To accomplish this, we use the chargingAttempted flag, which essentially
	 * masks low battery voltages until it has been set. */
	static bool chargingAttempted = false;

	/* Once we enter the ERROR state, the only way to leave the state is for
	 * the user to remove the charging voltage.  We use this flag to keep track
	 * of whether that has happened yet. */
	static bool chargingVoltageRemoved = false;

	/* Get the current time expressed as ticks of the 32.768kHz RTC */
	app_timer_cnt_get(&currentTime_rtcTicks);

	/* The RTC is only a 24-bit counter, so when subtracting the two
	 * tick counts to find the elapsed number of ticks, we must mask out the
	 * 8 most significant bits. */
	elapsedTime_rtcTicks = 0x00FFFFFF & (currentTime_rtcTicks - lastStateUpdateTime_rtcTicks);

	if (((chargeState == POWER_CHARGESTATE_PRECHARGE) ||
			(chargeState == POWER_CHARGESTATE_CHARGING)) &&
			((vin_mV = power_getVIn_mV()) <= CHARGER_INPUT_VOLTAGE_STOP_THRESHOLD_MV)) {
		/* Every time this function is executed, we check whether VIN has
		 * fallen below the charging input threshold.  If it has, it indicates
		 * that the external charger is no longer connected, so we immediately
		 * stop trying to charge and then proceed to update the state machine.
		 * If the charging voltage is suddenly removed, the charger will try to
		 * charge the batteries from the 3.3V rail (which is supplied by the
		 * batteries themselves).  In addition to just being inefficient, this
		 * is particularly bad because the 3.3V regulator cannot supply enough
		 * current to actually run the charger, and the 3.3V regulator over-
		 * heats and the 3.3V rail browns-out. */
		vinAbsent = true;
		power_disableCharger();
		power_setImproperTerminationFlags();
	} else if (elapsedTime_rtcTicks >= APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)) {
		/* If 10 seconds have elapsed since we last updated the charge state
		 * machine, we will proceed to update the charge state again.  First,
		 * we record the current time so that we'll know when to next update
		 * the charge state. */
		lastStateUpdateTime_rtcTicks = currentTime_rtcTicks;
	} else {
		/* Otherwise, if it is not yet time to update the charge state, we
		 * return without doing anything else. */
		return;
	}

	/* Unless the updateAgain flag is set, we only execute this do loop once.
	 * We typically set the updateAgain flag when switching states so that the
	 * actions of the new state are carried about before returning to the
	 * caller. */
	do {
		/* If this is not the first iteration of the loop, indicate that no
		 * additional time has elapsed since the first iteration. */
		if (updateAgain) {
			elapsedTime_rtcTicks = 0;
		}

		/* Assume that this will be the last iteration of the loop.  If the
		 * state is changed, we will re-set this flag. */
		updateAgain = false;

		/* Find the lowest and highest voltages among all four cells. */
		minVoltage_mV = power_getBatteryVoltageMin_mV();
		maxVoltage_mV = power_getBatteryVoltageMax_mV();

		/* Check for errors and jump to the ERROR state if any are found. */
		if (power_isCellOvervoltage() &&
				chargingAttempted && (chargingTime_rtcTicks / RTC_TICKS_PER_MS > 1000)) {
			/* If we have at least attempted to charge the batteries for 1sec,
			 * (in order to reset the Lipo protection IC) and one or more still
			 * has a voltage above the over-voltage threshold we enter the
			 * ERROR state. */
			chargeState = POWER_CHARGESTATE_ERROR;
			chargeError = POWER_CHARGEERROR_OVERVOLTAGE;
		} else if  (power_isCellUndervoltage() &&
				chargingAttempted && (chargingTime_rtcTicks / RTC_TICKS_PER_MS > 1000)) {
			/* If we have at least attempted to charge the batteries for 1sec,
			 * (in order to reset the Lipo protection IC) and one or more still
			 * has a voltage below the under-voltage threshold we enter the
			 * ERROR state. */
			chargeState = POWER_CHARGESTATE_ERROR;
			chargeError = POWER_CHARGEERROR_UNDERVOLTAGE;
		} else if (power_isOvertemp()) {
			chargeState = POWER_CHARGESTATE_ERROR;
			chargeError = POWER_CHARGEERROR_OVERTEMP;
		} else if (power_isUndertemp()) {
			chargeState = POWER_CHARGESTATE_ERROR;
			chargeError = POWER_CHARGEERROR_UNDERTEMP;
		} else if (chargeTimerRunning &&
				(chargingTime_rtcTicks / RTC_TICKS_PER_MS >= CHARGE_TIME_MAXIMUM_MS)) {
			/* If the charger has not reached the STANDBY state within a
			 * specified time limit, we assume something has gone wrong. */
			chargeState = POWER_CHARGESTATE_ERROR;
			chargeError = POWER_CHARGEERROR_TIMEOUT;
		}

		if (chargeState == POWER_CHARGESTATE_OFF) {
			/* Turn off the charger and open all battery discharge switches so
			 * that we do not drain current through the shorting resistors. */
			power_disableCharger();
			power_setDischargeSwitches(0x00);

			/* Stop and reset the charge timer */
			chargeTimerRunning = 0;
			chargingTime_rtcTicks = 0;

			/* When entering the off state, clear any errors which have
			 * previously occurred. */
			chargeError = POWER_CHARGEERROR_NOERROR;

			/* Turn off the green LED. */
			led_setState(LED_GREEN, LED_STATE_OFF);
		} else if (chargeState == POWER_CHARGESTATE_MANUAL) {
			/* In the MANUAL state, we leave it up to the user to set the charge
			 * current and the states of the battery discharge switches and to
			 * enable or disable the charger IC. */

			/* When entering this state, we clear any error, but that is not to
			 * say that error detection is turned off. Errors can still be
			 * detected above and will still move the FSM to the ERROR state.*/
			chargeError = POWER_CHARGEERROR_NOERROR;

			/* Start the charge timer if it is not already running.  If it is
			 * already running, update it with the amount of elapsed time since
			 * the last call. */
			if (!chargeTimerRunning) {
				chargingTime_rtcTicks = 0;
				chargeTimerRunning = true;
			} else {
				chargingTime_rtcTicks += elapsedTime_rtcTicks;
			}

			/* Turn the green LED on solid to indicate that we are in the ON
			 * state and that the user has control of the current and discharge
			 * switches. */
			led_setState(LED_GREEN, LED_STATE_ON);
		} else if ((chargeState == POWER_CHARGESTATE_PRECHARGE) ||
				(chargeState == POWER_CHARGESTATE_CHARGING)) {
			/* If the charger loses its external input voltage, we revert to
			 * the STANDBY state. */
			if (vinAbsent) {
				chargeState = POWER_CHARGESTATE_STANDBY;
				updateAgain = true;
				break;
			}

			/* If we are currently in the PRECHARGE state but the the minimum
			 * cell voltage is above the threshold which necessitates a reduced
			 * charging current, we skip the PRECHARGE state and go directly to
			 * the CHARGING state. */
			if ((chargeState == POWER_CHARGESTATE_PRECHARGE) &&
					(minVoltage_mV > CELL_PRECHARGE_THRESHOLD_MV)) {
				chargeState = POWER_CHARGESTATE_CHARGING;
			}

			/* If we are CHARGING state and all cell voltages are above the
			 * fully charged threshold voltage and the charging current has
			 * fallen below the current cutoff threshold (adjusted for
			 * whatever current the microprocessor and other circuitry is
			 * consuming, we assume that charging is complete. */
			if ((chargeState == POWER_CHARGESTATE_CHARGING) &&
					(minVoltage_mV >= CELL_CHARGED_THRESHOLD_MV) &&
					(power_getChargeCurrent_mA() <
							CHARGE_CURRENT_CUTOFF_THRESHOLD_MA +
							power_getEstimatedCurrentConsumption_mA())) {
				chargeState = POWER_CHARGESTATE_STANDBY;
				updateAgain = true;
				break;
			}

			/* Start the charge timer if it is not already running.  If it is
			 * already running, update it with the amount of elapsed time since
			 * the last call. */
			if (!chargeTimerRunning) {
				chargingTime_rtcTicks = 0;
				chargeTimerRunning = true;
			} else {
				chargingTime_rtcTicks += elapsedTime_rtcTicks;
			}

			/* Single-flash the LED to indicate that charging is underway */
			led_setState(LED_GREEN, LED_STATE_SINGLE_BLINK);

			if (chargeState == POWER_CHARGESTATE_PRECHARGE) {
				/* Set the charger's output current limit to 0.1*C plus
				 * whatever we estimate the processor and other circuitry to be
				 * consuming.*/
				power_setChargerCurrentLimit_mA(PRECHARGE_CURRENT_MA +
						power_getEstimatedCurrentConsumption_mA());
			} else {
				/* If we are not precharging, we set the current to the maximum
				 * value which does not harm the charger.  This limit is lower
				 * than the approved charge rate for the batteries. */
				power_setChargerCurrentLimit_mA(CHARGE_CURRENT_MAX_MA);
			}

			/* Turning on the charger IC starts the flow of current. */
			power_enableCharger();

			/* Indicate that we have attempted to charge the batteries.  This
			 * flag is only ever cleared when the unit loses power completely.
			 * Until this flag is set, we ignore under-voltage conditions
			 * because it could be that the Lipo protection IC has tripped
			 * making it impossible to measure the battery voltages. */
			chargingAttempted = true;

			/* If we have not been charging the batteries for at least 1sec,
			 * we do not connect any of the battery shorting resistors.  This
			 * allows the voltages to stabilize so that we can make a better
			 * decision about which batteries to short. */
			if (chargingTime_rtcTicks / RTC_TICKS_PER_MS < 1000) {
				power_setDischargeSwitches(0x00);
				break;
			}

			/* We only reach this point if we have been charging for at least
			 * 1 sec.  Here we decide which of the discharge switches to close
			 * in order to keep the batteries balanced as they charge. */

			/* Check whether each of the 4 battery voltages is greater than the
			 * voltage of the most discharged cell by more than the maximum
			 * imbalance threshold. If one is, we will short a resistor across its
			 * terminals to reduce its charge rate. */
			for (batNum = 1; batNum <= 4; batNum++) {
				if (power_getBatteryVoltage_mV(batNum) > minVoltage_mV + MAXIMUM_IMBALANCE_MV) {
					shorts |= (0x01 << (batNum-1));
				}
			}

			/* If the fact that we resampled all battery voltages resulted in
			 * enough variation to short all discharge switches, keep them all
			 * open. */
			if (shorts == 0x0F) {
				shorts = 0x00;
			}

			power_setDischargeSwitches(shorts);
		} else if (chargeState == POWER_CHARGESTATE_STANDBY) {
			/* If there is an old error when we enter this state, jump to the
			 * ERROR state.  This is necessary because the user may try to
			 * force us into the STANDBY state to start charging, but we do
			 * not want to start charging if there is some type of error. */
			if (chargeError != POWER_CHARGEERROR_NOERROR) {
				chargeState = POWER_CHARGESTATE_ERROR;
				updateAgain = true;
				break;
			}

			/* If a single cell is below the recharge threshold voltage, and
			 * there is sufficient input voltage to the charger, we start the
			 * charging process.  We always jump to the PRECHARGE state, but
			 * from there we can immediately jump to the CHARGING state. */
			if ((minVoltage_mV < CELL_RECHARGE_THRESHOLD_MV) &&
					(power_getVIn_mV() >= CHARGER_INPUT_VOLTAGE_START_THRESHOLD_MV)) {
				chargeState = POWER_CHARGESTATE_PRECHARGE;
				updateAgain = true;
				break;
			}

			/* Now that charging is complete, we can stop the charge timer, but
			 * we do not reset the time.  We turn off the green LED and also
			 * disable the charger and disconnect all of the cell-shorting
			 * resistors.*/
			chargeTimerRunning = false;
			led_setState(LED_GREEN, LED_STATE_OFF);
			power_disableCharger();
			power_setDischargeSwitches(0x00);
		} else if (chargeState == POWER_CHARGESTATE_DISCHARGE) {
			/* If all cells are below the nominal discharge voltage, we are
			 * done discharging.  Consequently, proceed to the OFF state, which
			 * will stop discharging and open all of the switches.  From there,
			 * the user will have to restart the charging process manually. */
			if (maxVoltage_mV < CELL_DISCHARGED_THRESHOLD_MV) {
				chargeState = POWER_CHARGESTATE_OFF;
				updateAgain = true;
				break;
			}

			/* Start/update the charge timer */
			if (!chargeTimerRunning) {
				chargingTime_rtcTicks = 0;
				chargeTimerRunning = true;
			} else {
				chargingTime_rtcTicks += elapsedTime_rtcTicks;
			}

			/* When discharging, flash the green LED */
			led_setState(LED_GREEN, LED_STATE_SLOW_FLASH);

			/* Check whether each of the 4 battery voltages is greater than the
			 * voltage of the most discharged cell by more than the maximum
			 * imbalance threshold. If one is, we will short a resistor across its
			 * terminals to speed it discharge. */
			for (batNum = 1; batNum <= 4; batNum++) {
				if (power_getBatteryVoltage_mV(batNum) > minVoltage_mV + MAXIMUM_IMBALANCE_MV) {
					shorts |= (0x01 << (batNum-1));
				}
			}

			/* If the cell voltages are all balanced, we turn on all discharged
			 * switches so that there are resistors shorted across all four cells.
			 */
			if (shorts == 0x00) {
				shorts = 0x0F;
			}

			/* Disable charging and close the appropriate discharge switches so
			 * that we drain current from the batteries. */
			power_disableCharger();
			power_setDischargeSwitches(shorts);
		} else if (chargeState == POWER_CHARGESTATE_ERROR) {
			/* When we encounter an error, we stop charging, open all of the
			 * discharge switches, stop the charger timer, and reset it to 0.*/
			power_disableCharger();
			power_setDischargeSwitches(0x00);
			chargeTimerRunning = false;
			chargingTime_rtcTicks = 0;

			/* Fast-flash the LED to indicate a problem */
			led_setState(LED_GREEN, LED_STATE_FAST_FLASH);

			/* In the error state, we wait for the supply voltage to the
			 * charger IC to be removed.  Once it has been, we set the
			 * chargingVoltageRemoved flag.  We use this way as a way to
			 * force the charging voltage to be removed and then re-applied
			 * in order to exit the ERROR state. */
			if (vinAbsent) {
				chargingVoltageRemoved = true;
				break;
			}

			/* At this point, the supply voltage to the charger IC must be
			 * enough to trigger charging, so if the voltage has previously
			 * been removed, we start the charging process and thereby exit
			 * the ERROR state. */
			if (chargingVoltageRemoved) {
				chargingVoltageRemoved = false;
				/* Once in the STANDBY state, the code will re-evaluate
				 * whether the actual battery voltage is low enough to start
				 * a new charge cycle. */
				chargeState = POWER_CHARGESTATE_STANDBY;
				/* When leaving the ERROR state we clear the error code. */
				chargeError = POWER_CHARGEERROR_NOERROR;
				updateAgain = true;
				break;
			}
		}
	} while (updateAgain);

	if (debug) {
		power_printDebugInfo();
	}
}


void power_printDebugInfo() {
	char stateStr[20];
	char errStr[20];

	unsigned int hours, minutes, seconds;

	unsigned int chargerCurrent_mA, estShuntCurrent_mA, batteryCurrent_mA;

	unsigned int cell1_mV, cell2_mV, cell3_mV, cell4_mV, pack_mV, input_mV;
	char cell1ShortedStr[4], cell2ShortedStr[4], cell3ShortedStr[4], cell4ShortedStr[4];

	char line[100];

	if (chargeState == POWER_CHARGESTATE_OFF) {
		strncpy(stateStr, "Off       ", sizeof(stateStr));
	} else if (chargeState == POWER_CHARGESTATE_MANUAL) {
		strncpy(stateStr, "Manual    ", sizeof(stateStr));
	} else if (chargeState == POWER_CHARGESTATE_PRECHARGE) {
		strncpy(stateStr, "Pre-charge", sizeof(stateStr));
	} else if (chargeState == POWER_CHARGESTATE_CHARGING) {
		strncpy(stateStr, "Charging  ", sizeof(stateStr));
	} else if (chargeState == POWER_CHARGESTATE_STANDBY) {
		strncpy(stateStr, "Standby   ", sizeof(stateStr));
	} else if (chargeState == POWER_CHARGESTATE_DISCHARGE) {
		strncpy(stateStr, "Discharge ", sizeof(stateStr));
	} else if (chargeState == POWER_CHARGESTATE_ERROR) {
		strncpy(stateStr, "Error     ", sizeof(stateStr));
	} else {
		strncpy(stateStr, "Unknown   ", sizeof(stateStr));
	}

	if (chargeError == POWER_CHARGEERROR_NOERROR) {
		strncpy(errStr, "-                ", sizeof(errStr));
	} else if (chargeError == POWER_CHARGEERROR_OVERVOLTAGE) {
		strncpy(errStr, "Over-voltage     ", sizeof(errStr));
	} else if (chargeError == POWER_CHARGEERROR_UNDERVOLTAGE) {
		strncpy(errStr, "Under-voltage    ", sizeof(errStr));
	} else if (chargeError == POWER_CHARGEERROR_OVERTEMP) {
		strncpy(errStr, "Over-temperature ", sizeof(errStr));
	} else if (chargeError == POWER_CHARGEERROR_UNDERTEMP) {
		strncpy(errStr, "Under-temperature", sizeof(errStr));
	} else if (chargeError == POWER_CHARGEERROR_TIMEOUT) {
		strncpy(errStr, "Time-out         ", sizeof(errStr));
	} else {
		strncpy(errStr, "Unknown          ", sizeof(errStr));
	}

	seconds = (unsigned int)(chargingTime_rtcTicks / (RTC_TICKS_PER_MS * 1000));

	hours = seconds / 3600;
	seconds -= hours * 3600;

	minutes = seconds / 60;
	seconds -= minutes * 60;

	chargerCurrent_mA = power_getChargeCurrent_mA();
	estShuntCurrent_mA = power_getEstimatedCurrentConsumption_mA();
	if (chargerCurrent_mA > estShuntCurrent_mA) {
		batteryCurrent_mA = chargerCurrent_mA - estShuntCurrent_mA;
	} else {
		batteryCurrent_mA = 0;
	}

	cell1_mV = power_getBatteryVoltage_mV(1);
	cell2_mV = power_getBatteryVoltage_mV(2);
	cell3_mV = power_getBatteryVoltage_mV(3);
	cell4_mV = power_getBatteryVoltage_mV(4);
	pack_mV = power_getBatteryPackVoltage_mV();
	input_mV = power_getVIn_mV();

	if (power_getDischargeSwitches() & 0x01) {
		strncpy(cell1ShortedStr, "(s)", sizeof(cell1ShortedStr));
	} else {
		strncpy(cell1ShortedStr, "   ", sizeof(cell1ShortedStr));
	}

	if (power_getDischargeSwitches() & 0x02) {
		strncpy(cell2ShortedStr, "(s)", sizeof(cell2ShortedStr));
	} else {
		strncpy(cell2ShortedStr, "   ", sizeof(cell2ShortedStr));
	}

	if (power_getDischargeSwitches() & 0x04) {
		strncpy(cell3ShortedStr, "(s)", sizeof(cell3ShortedStr));
	} else {
		strncpy(cell3ShortedStr, "   ", sizeof(cell3ShortedStr));
	}

	if (power_getDischargeSwitches() & 0x08) {
		strncpy(cell4ShortedStr, "(s)", sizeof(cell4ShortedStr));
	} else {
		strncpy(cell4ShortedStr, "   ", sizeof(cell4ShortedStr));
	}

	app_uart_put_string("\r\n");
	snprintf(line, sizeof(line), "Charger State: %s   Error: %s              Time: %02u:%02u:%02u\r\n", stateStr, errStr, hours, minutes, seconds);
	app_uart_put_string(line);
	snprintf(line, sizeof(line), "Charger Current: %3umA      Est. Shunt Current: %3umA     Battery Current: %3umA\r\n", chargerCurrent_mA, estShuntCurrent_mA, batteryCurrent_mA);
	app_uart_put_string(line);
	app_uart_put_string("Cell#1         Cell#2         Cell#3         Cell#4         Pack          Input \r\n");
	snprintf(line, sizeof(line), "%4umV %s     %4umV %s     %4umV %s     %4umV %s     %5umV       %4umV\r\n",
			cell1_mV, cell1ShortedStr, cell2_mV, cell2ShortedStr, cell3_mV, cell3ShortedStr, cell4_mV, cell4ShortedStr, pack_mV, input_mV);
	app_uart_put_string(line);
	app_uart_put_string("\r\n");

}

void power_printDischargeSwitchState() {
	uint8_t dischargeState;
	int i;
	char str[100];

	dischargeState = power_getDischargeSwitches();

	for (i=1; i<=4; i++) {
		if (dischargeState & (0x01 << (i-1))) {
			snprintf(str, sizeof(str), "Battery %u discharge switch: shorted (discharging)\r\n", i);
		} else {
			snprintf(str, sizeof(str), "Battery %u discharge switch: open\r\n", i);
		}

		app_uart_put_string(str);
	}
}

void power_printVIn() {
	uint16_t mV;
	char str[30];

	mV = power_getVIn_mV();
	snprintf(str, sizeof(str), "Input: %umV\r\n", mV);
	app_uart_put_string(str);
}

void power_printBatteryVoltages() {
	int i;
	uint16_t mV;
	char str[30];

	for (i=1; i<=4; i++) {
		mV = power_getBatteryVoltage_mV(i);
		snprintf(str, sizeof(str), "Battery %u: %dmV\r\n", i, mV);
		app_uart_put_string(str);
	}
}

void power_printChargeCurrent() {
	uint16_t mA;
	char str[30];

	mA = power_getChargeCurrent_mA();
	snprintf(str, sizeof(str), "Charging current: %umA\r\n", mA);
	app_uart_put_string(str);
}

