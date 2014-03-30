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
static bool initialized = false;

bool mpu6050_init(uint8_t address) {
	bool success = true;
	uint8_t data;

	mpu6050Address = address;

	success &= mpu6050_readReg(MPU6050_WHO_AM_I_ADDR, &data, 1);
	if (!success || (data != 0x68)) {
		return false;
	}

	/* Reset the gyro, accelerometer, and temperature signal paths */
	success &= mpu6050_writeReg(MPU6050_SIGNAL_PATH_RESET_ADDR, 0x04 | 0x02 | 0x01);
	/* Perform device reset */
	success &= mpu6050_writeReg(MPU6050_PWR_MGMT_1_ADDR, 0x80);

	if (success) {
		initialized = true;
	} else {
		initialized = false;
	}

	return success;
}

bool mpu6050_writeReg(uint8_t addr, uint8_t data) {
	uint8_t packet[2];

	packet[0] = addr;
	packet[1] = data;
	return twi_master_transfer((mpu6050Address<<1), packet, sizeof(packet), TWI_ISSUE_STOP);
}

bool mpu6050_readReg(uint8_t addr, uint8_t *data, uint8_t nBytes) {
	bool success = true;
	success &= twi_master_transfer((mpu6050Address<<1), &addr, 1, TWI_DONT_ISSUE_STOP);
	success &= twi_master_transfer((mpu6050Address<<1) | TWI_READ_BIT, data, nBytes, TWI_ISSUE_STOP);
	return success;
}

