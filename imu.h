/*
 * imu.h
 *
 *  Created on: Apr 3, 2014
 *      Author: kwgilpin
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>

bool imu_init(uint8_t address);
bool imu_enableMotionDetection(bool enable);
bool imu_checkForMotion(void);
void imu_resetMotionFlag(void);

bool imu_initDMP(void);

#endif /* IMU_H_ */
