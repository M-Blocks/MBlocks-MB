/*
 * imu.c
 *
 *  Created on: Apr 3, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>

#include "mpu6050.h"
#include "imu.h"

bool imu_enableMotionDetection() {
	uint8_t data;

	/* Clear the sleep and cycle bits in order to keep the IMU awake while we
	 * configure motion detection. */
	mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &data, 1);
	data &= ~(MPU6050_SLEEP_MASK | MPU6050_CYCLE_MASK);
	mpu6050_writeReg(MPU6050_PWR_MGMT_1_REG_ADDR, data);

	/* Configure the wake-up frequency to 2.5Hz, put all 3 gyros into standby,
	 * but keep all 3 accelerometers enabled. */
	mpu6050_writeReg(MPU6050_PWR_MGMT_2_REG_ADDR,
			(1<<MPU6050_LP_WAKE_CTRL_POSN) |
			(1<<MPU6050_STBY_XG_POSN) |
			(1<<MPU6050_STBY_YG_POSN) |
			(1<<MPU6050_STBY_ZG_POSN));

	/* Reset the accelerometers' digital high pass filter (and stop/disable the
	 * accelerometers self tests) while leaving the accelerometer full scale
	 * value unchanged. */
	mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &data, 1);
	data &= ~(MPU6050_ACCEL_HPF_MASK | MPU6050_XA_ST_MASK | MPU6050_YA_ST_MASK | MPU6050_ZA_ST_MASK);
	mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, 0x00);

	/* Set the digital low-pass filter to 260 Hz, (and disable the FSYNC pin).
	 */
	mpu6050_writeReg(MPU6050_CONFIG_REG_ADDR, 0x00);

	/* Enable only motion detection interrupts and no other interrupt */
	mpu6050_writeReg(MPU6050_INT_ENABLE_REG_ADDR, (1<<MPU6050_MOT_EN_POSN));

	/* Set the motion detection duration threshold to 1 sample */
	mpu6050_writeReg(MPU6050_MOT_DUR_REG_ADDR, 0x01);

	/* Set the motion threshold in terms of LSBs, where 1 LSB is 32mg. */
	mpu6050_writeReg(MPU6050_MOT_THR_REG_ADDR, 20);

	/* Accumulate some accelerometer samples so that we have a reference value
	 * against which to compare when sensing motion. */
	delay_ms(5);

	/* Set the accelerometers' digital high pass filter to hold its current
	 * sample.  All future output samples will be the difference between the
	 * input sample and the held sample. Leaving the accelerometer full scale
	 * value unchanged. */
	mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &data, 1);
	data |= (7 << MPU6050_ACCEL_HPF_POSN);
	mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, 0x00);

	/* Clear any pending interrupt flag */
	mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &data, 1);

	/* Set the CYCLE bit so that the IMU goes to sleep and routinely wakes up
	 * to sample the accelerometers and determine whether motion has occurred.
	 */
	mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &data, 1);
	data |= (1 << MPU6050_CYCLE_MASK);
	mpu6050_writeReg(MPU6050_PWR_MGMT_1_REG_ADDR, data);

	return true;
}

bool imu_checkForMotion() {
	uint8_t data;

	if (!mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &data, 1)) {
		return false;
	}

	if (data & MPU6050_MOT_INT_MASK) {
		return true;
	}

	return false;
}
