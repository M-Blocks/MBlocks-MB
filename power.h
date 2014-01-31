/*
 * power.h
 *
 *  Created on: Nov 12, 2013
 *      Author: kwgilpin
 */

#ifndef POWER_H_
#define POWER_H_

#include <stdint.h>
#include <stdbool.h>

#include "app_timer.h"

typedef enum {
	POWER_CHARGESTATE_OFF,
	POWER_CHARGESTATE_MANUAL,
	POWER_CHARGESTATE_PRECHARGE,
	POWER_CHARGESTATE_CHARGING,
	POWER_CHARGESTATE_STANDBY,
	POWER_CHARGESTATE_DISCHARGE,
	POWER_CHARGESTATE_ERROR
} power_chargeState_t;

typedef enum {
	POWER_CHARGEERROR_NOERROR,
	POWER_CHARGEERROR_UNDERVOLTAGE,
	POWER_CHARGEERROR_OVERVOLTAGE,
	POWER_CHARGEERROR_UNDERTEMP,
	POWER_CHARGEERROR_OVERTEMP,
	POWER_CHARGEERROR_TIMEOUT
} power_chargeError_t;

extern app_timer_id_t power_timer_id;

bool power_setDischargeSwitches(uint8_t switches);
uint8_t power_getDischargeSwitches(void);

bool power_setChargerCurrentLimit_mA (uint16_t iLimit);
uint16_t power_getChargerCurrentLimit_mA(void);

bool power_enableCharger(void);
bool power_disableCharger(void);

void power_setChargeState(power_chargeState_t state);

void power_setDebug(bool enable);
bool power_getDebug(void);

uint16_t power_getVIn_mV(void);
uint16_t power_getBatteryVoltage_mV(uint8_t batNum);
uint16_t power_getBatteryVoltageMin_mV(void);
uint16_t power_getBatteryVoltageMax_mV(void);
uint16_t power_getBatteryPackVoltage_mV(void);

uint16_t power_getChargeCurrent_mA(void);
uint16_t power_getEstimatedCurrentConsumption_mA(void);

bool power_isCellUndervoltage(void);
bool power_isCellOvervoltage(void);
bool power_isUndertemp(void);
bool power_isOvertemp(void);

void power_printDebugInfo(void);
void power_printDischargeSwitchState(void);
void power_printVIn(void);
void power_printBatteryVoltages(void);
void power_printChargeCurrent(void);

void power_updateChargeState(void);

void power_timerHandler(void *p_context);

#endif /* POWER_H_ */
