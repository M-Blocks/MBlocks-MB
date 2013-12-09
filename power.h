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



bool power_setDischargeSwitches(uint8_t switches);
uint8_t power_getDischargeSwitches(void);

bool power_setChargerCurrentLimit_mA (uint16_t iLimit);
uint16_t power_getChargerCurrentLimit_mA(void);

bool power_enableCharger(void);
bool power_disableCharger(void);

uint16_t power_getVIn_mV(void);
uint16_t power_getBatteryVoltage_mV(uint8_t batNum);
uint16_t power_getBatteryPackVoltage_mV(void);
uint16_t power_getChargeCurrent_mA(void);

void power_printDischargeSwitchState(void);
void power_printVIn(void);
void power_printBatteryVoltages(void);
void power_printChargeCurrent(void);


#endif /* POWER_H_ */
