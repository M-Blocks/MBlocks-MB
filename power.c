/*
 * power.c
 *
 *  Created on: Nov 12, 2013
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "pins.h"
#include "util.h"
#include "adc.h"
#include "pwm.h"
#include "power.h"

static uint8_t dischargeSwitchState = 0x00;
static uint16_t chargeCurrentLimit_mA = 0;


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

uint16_t power_getVIn_mV() {
	/* VIN is divided by 2 with a resistor divider pair, so we multiply by 2 in
	 * order to return the correct voltage. */
	return 2 * adc_avg_mV(VINSENSE_ADC_CHNL, 8);
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

	offset_mV = adc_avg_mV(LIPROVBATOUT_ADC_CHNL, 8);

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

	offsetPlusFifthActual_mV = adc_avg_mV(LIPROVBATOUT_ADC_CHNL, 8);

	/* Return both control lines to their high impedance state */
	nrf_gpio_cfg_input(LIPROCTL3_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);
	nrf_gpio_cfg_input(LIPROCTL4_PIN_NO, GPIO_PIN_CNF_PULL_Disabled);

	/* Subtract the offset voltage and multiply by five to arrive at the actual
	 * battery voltage. This comes from the Seiko datasheet. */
	actual_mV = (offsetPlusFifthActual_mV - offset_mV) * 5;

	return actual_mV;
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
