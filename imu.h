/*
 * imu.h
 *
 *  Created on: Apr 3, 2014
 *      Author: kwgilpin
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	float w;
	float x;
	float y;
	float z;
} quaternion_t;

typedef struct {
	float x;
	float y;
	float z;
} vectorFloat_t;

extern const vectorFloat_t cornerVectors[3];

bool imu_init(uint8_t address);
bool imu_initDMP(void);

bool imu_enableMotionDetection(bool enable);
bool imu_enableDMP(void);
bool imu_enableSleepMode(void);

bool imu_checkForMotion(bool *motionDetected);
bool imu_getLatestFIFOPacket(uint8_t *packet);

bool imu_getQuaternionFromPacket(quaternion_t *q, const uint8_t *packet);
bool imu_getGravityFromQuaternion(vectorFloat_t *v, const quaternion_t *q);
float imu_getVectorAngle(const vectorFloat_t *v, const vectorFloat_t *u);

void imu_testDMPLoop(void);

#endif /* IMU_H_ */
