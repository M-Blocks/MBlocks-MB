/*
 * mpu6050.c
 *
 *  Created on: Mar 27, 2014
 *      Author: kwgilpin
 */

#include <stdbool.h>
#include <stdint.h>

#include "twi_master.h"
#include "mpu6050.h"

static uint8_t mpu6050Address;

void mpu6050_setAddress(uint8_t address) {
	mpu6050Address = address;
}

bool mpu6050_writeReg(uint8_t addr, uint8_t data) {
	uint8_t packet[2];

	/* The caller must initialize the TWI interface */
	if (!twi_master_get_init()) {
		return false;
	}

	packet[0] = addr;
	packet[1] = data;
	return twi_master_transfer((mpu6050Address<<1), packet, sizeof(packet), TWI_ISSUE_STOP);
}

bool mpu6050_readReg(uint8_t addr, uint8_t *data, uint8_t nBytes) {
	bool success = true;

	/* The caller must initialize the TWI interface */
	if (!twi_master_get_init()) {
		return false;
	}

	success &= twi_master_transfer((mpu6050Address<<1), &addr, 1, TWI_DONT_ISSUE_STOP);
	success &= twi_master_transfer((mpu6050Address<<1) | TWI_READ_BIT, data, nBytes, TWI_ISSUE_STOP);
	return success;
}

