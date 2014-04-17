/*
 * imu.c
 *
 *  Created on: Apr 3, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>

#include <app_timer.h>

#include "global.h"
#include "util.h"
#include "twi_master.h"
#include "mpu6050.h"
#include "imu.h"

static bool initialized = false;

bool imu_init(uint8_t address) {
	bool success = true;
	uint8_t data;

	mpu6050_setAddress(address);

	twi_master_init();

	success &= mpu6050_readReg(MPU6050_WHO_AM_I_REG_ADDR, &data, 1);
	if (!success || (data != address)) {
		return false;
	}

	/* Reset the gyro, accelerometer, and temperature signal paths */
	success &= mpu6050_writeReg(MPU6050_SIGNAL_PATH_RESET_REG_ADDR,
			(1<<MPU6050_GYRO_RESET_POSN) | (1<<MPU6050_ACCEL_RESET_POSN) | (1<<MPU6050_TEMP_RESET_POSN));
	/* Perform device reset */
	success &= mpu6050_writeReg(MPU6050_PWR_MGMT_1_REG_ADDR,
			(1<<MPU6050_DEVICE_RESET_POSN));

	if (success) {
		initialized = true;
	} else {
		initialized = false;
	}

	twi_master_deinit();

	return success;
}

bool imu_enableMotionDetection(bool enable) {
	uint32_t err_code;
	uint8_t data;
	bool success = true;

	if (!initialized) {
		return false;
	}

	twi_master_init();

	/* Clear the sleep and cycle bits in order to keep the IMU awake while we
	 * configure motion detection. */
	success &= mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &data, 1);
	data &= ~(MPU6050_SLEEP_MASK | MPU6050_CYCLE_MASK);
	success &= mpu6050_writeReg(MPU6050_PWR_MGMT_1_REG_ADDR, data);

	/* Configure the wake-up frequency to 1.25Hz, put all 3 gyros into standby,
	 * but keep all 3 accelerometers enabled. */
	success &= mpu6050_writeReg(MPU6050_PWR_MGMT_2_REG_ADDR,
			(0<<MPU6050_LP_WAKE_CTRL_POSN) |
			(1<<MPU6050_STBY_XG_POSN) |
			(1<<MPU6050_STBY_YG_POSN) |
			(1<<MPU6050_STBY_ZG_POSN));

	/* Reset the accelerometers' digital high pass filter (and stop/disable the
	 * accelerometers self tests) while leaving the accelerometer full scale
	 * value unchanged. */
	success &= mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &data, 1);
	data &= ~(MPU6050_ACCEL_HPF_MASK | MPU6050_XA_ST_MASK | MPU6050_YA_ST_MASK | MPU6050_ZA_ST_MASK);
	success &= mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, 0x00);

	/* Set the digital low-pass filter to 260 Hz, (and disable the FSYNC pin).
	 */
	success &= mpu6050_writeReg(MPU6050_CONFIG_REG_ADDR, 0x00);

	/* Enable only motion detection interrupts and no other interrupt */
	success &= mpu6050_writeReg(MPU6050_INT_ENABLE_REG_ADDR, (1<<MPU6050_MOT_EN_POSN));

	/* Set the motion detection duration threshold to 1 sample */
	success &= mpu6050_writeReg(MPU6050_MOT_DUR_REG_ADDR, 0x01);

	/* Set the motion threshold in terms of LSBs, where 1 LSB is 32mg. */
	success &= mpu6050_writeReg(MPU6050_MOT_THR_REG_ADDR, 20);

	/* Accumulate some accelerometer samples so that we have a reference value
	 * against which to compare when sensing motion. */
	delay_ms(5);

	/* Set the accelerometers' digital high pass filter to hold its current
	 * sample.  All future output samples will be the difference between the
	 * input sample and the held sample. Leaving the accelerometer full scale
	 * value unchanged. */
	success &= mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &data, 1);
	data |= (7 << MPU6050_ACCEL_HPF_POSN);
	success &= mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, 0x00);

	/* Clear any pending interrupt flag */
	success &= mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &data, 1);

	/* Set the CYCLE bit so that the IMU goes to sleep and routinely wakes up
	 * to sample the accelerometers and determine whether motion has occurred.
	 */
	success &= mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &data, 1);
	data |= (1 << MPU6050_CYCLE_POSN);
	success &= mpu6050_writeReg(MPU6050_PWR_MGMT_1_REG_ADDR, data);

	twi_master_deinit();

	return success;
}

bool imu_checkForMotion() {
	uint8_t data;
	bool motion = false;

	if (!initialized) {
		return false;
	}

	twi_master_init();

	mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &data, 1);
	if (data & MPU6050_MOT_INT_MASK) {
		motion = true;
	}

	twi_master_deinit();
	return motion;
}

void imu_resetMotionFlag() {
	uint8_t data;

	if (!initialized) {
		return;
	}

	twi_master_init();
	mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &data, 1);
	twi_master_deinit();
}
