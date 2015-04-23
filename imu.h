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

#define IMU_FIFO_PACKET_SIZE		42

typedef uint8_t imuFIFOPacket_t[IMU_FIFO_PACKET_SIZE];

typedef struct {
	int16_t w;
	int16_t x;
	int16_t y;
	int16_t z;
} quaternion16_t;

typedef struct {
	float w;
	float x;
	float y;
	float z;
} quaternionFloat_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} vector16_t;

typedef struct {
	float x;
	float y;
	float z;
} vectorFloat_t;

extern const vectorFloat_t frameAlignmentVectorsFloat[3];

bool imu_init();
bool imu_initDMP(void);

bool imu_enableMotionDetection(bool enable);
bool imu_enableDMP(void);
bool imu_enableSleepMode(void);

bool imu_checkForMotion(bool *motionDetected);
bool imu_getLatestFIFOPacket(uint8_t *packet);

bool imu_getGyros16(vector16_t *v16);
bool imu_getGyros16FromPacket(vector16_t *v16, const uint8_t *packet);
bool imu_getGyrosFloat(vectorFloat_t *vf);
bool imu_getGyrosFloatFromPacket(vectorFloat_t *vf, const uint8_t *packet);

bool imu_getUnitQuaternion16(quaternion16_t *q16);
bool imu_getUnitQuaternion16FromPacket(quaternion16_t *q16, const uint8_t *packet);
bool imu_getUnitQuaternionFloat(quaternionFloat_t *qf);
bool imu_getUnitQuaternionFloatFromPacket(quaternionFloat_t *qf, const uint8_t *packet);

bool imu_getGravity16(vector16_t *v16);
bool imu_getGravity16FromPacket(vector16_t *v16, const uint8_t *packet);
bool imu_getGravityFloat(vectorFloat_t *vf);
bool imu_getGravityFloatFromPacket(vectorFloat_t *vf, const uint8_t *packet);

float imu_getVectorFloatAngle(const vectorFloat_t *v, const vectorFloat_t *u);
float imu_getVectorFloatMagnitude(const vectorFloat_t *v);

void imu_testDMPLoop(void);

#endif /* IMU_H_ */
