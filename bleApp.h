/*
 * ble.h
 *
 *  Created on: Jan 31, 2014
 *      Author: kwgilpin
 */

#ifndef BLE_H_
#define BLE_H_

#include <stdint.h>
#include <stdbool.h>

#include "ble_vns.h"
#include "ble_sps.h"

extern ble_vns_t m_vns;
extern ble_sps_t m_sps;

void bleApp_stackInit(void);
void bleApp_gapParamsInit(void);
void bleApp_servicesInit(void);
void bleApp_secParamsInit(void);
void bleApp_connParamsInit(void);

bool bleApp_isConnected(void);

void bleApp_setAdvertisingEnabled(bool advertise);
bool bleApp_isAdvertisingEnabled(void);
void bleApp_advertisingInit(void);
void bleApp_advertisingStart(void);
void bleApp_advertisingStop(void);

#endif /* BLE_H_ */
