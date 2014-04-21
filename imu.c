/*
 * imu.c
 *
 *  Created on: Apr 3, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "nrf_delay.h"

#include "app_timer.h"

#include "util.h"

#include "global.h"
#include "util.h"
#include "twi_master.h"
#include "mpu6050.h"
#include "imu.h"

#define DEBUG_IMU 0

#define IMU_FIFO_PACKET_SIZE 42

#if (0)
typedef struct {
	union {
		struct data16 {
			uint16_t quatw;
			uint8_t reserved0[2];
			uint16_t quatx;
			uint8_t reserved0[2];
			uint16_t quaty;
			uint8_t reserved0[2];
			uint16_t quatz;
			uint8_t reserved0[2];
			uint16_t gyrox;
			uint8_t reserved0[2];
			uint16_t gyroy;
			uint8_t reserved0[2];
			uint16_t gyroz;
			uint16_t accelx;
			uint16_t accely;
			uint16_t accelz;
			uint8_t reserved0[2];
		};
		struct data32 {
			uint32_t quatw;
			uint32_t quatx;
			uint32_t quaty;
			uint32_t quatz;
			uint32_t gyrox;
			uint32_t gyroy;
			uint32_t gyroz;
			uint32_t accelx;
			uint32_t accely;
			uint32_t accelz;
			uint8_t reserved0[2];
		};
		uint8_t raw[];
	};
} dmpFIFOPacket_t;
#endif

#define MPU6050_DMP_CODE_SIZE 1929 // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE 192 // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE 47 // dmpUpdates[]

static const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] = {
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 138 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

static const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE] = {
// BANK OFFSET LENGTH [DATA]
    0x03, 0x7B, 0x03, 0x4C, 0xCD, 0x6C, // FCFG_1 inv_set_gyro_calibration
    0x03, 0xAB, 0x03, 0x36, 0x56, 0x76, // FCFG_3 inv_set_gyro_calibration
    0x00, 0x68, 0x04, 0x02, 0xCB, 0x47, 0xA2, // D_0_104 inv_set_gyro_calibration
    0x02, 0x18, 0x04, 0x00, 0x05, 0x8B, 0xC1, // D_0_24 inv_set_gyro_calibration
    0x01, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00, // D_1_152 inv_set_accel_calibration
    0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
    0x03, 0x89, 0x03, 0x26, 0x46, 0x66, // FCFG_7 inv_set_accel_calibration
    0x00, 0x6C, 0x02, 0x20, 0x00, // D_0_108 inv_set_accel_calibration
    0x02, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_00 inv_set_compass_calibration
    0x02, 0x44, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_01
    0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_02
    0x02, 0x4C, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_10
    0x02, 0x50, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_11
    0x02, 0x54, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_12
    0x02, 0x58, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_20
    0x02, 0x5C, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_21
    0x02, 0xBC, 0x04, 0x00, 0x00, 0x00, 0x00, // CPASS_MTX_22
    0x01, 0xEC, 0x04, 0x00, 0x00, 0x40, 0x00, // D_1_236 inv_apply_endian_accel
    0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04, 0x02, 0x03, 0x0D, 0x35, 0x5D, // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x04, 0x09, 0x04, 0x87, 0x2D, 0x35, 0x3D, // FCFG_5 inv_set_bias_update
    0x00, 0xA3, 0x01, 0x00, // D_0_163 inv_set_dead_zone
                 // SPECIAL 0x01 = enable interrupts
    0x00, 0x00, 0x00, 0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07, 0x86, 0x01, 0xFE, // CFG_6 inv_set_fifo_interupt
    0x07, 0x41, 0x05, 0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07, 0x7E, 0x01, 0x30, // CFG_16 inv_set_footer
    0x07, 0x46, 0x01, 0x9A, // CFG_GYRO_SOURCE inv_send_gyro
    0x07, 0x47, 0x04, 0xF1, 0x28, 0x30, 0x38, // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07, 0x6C, 0x04, 0xF1, 0x28, 0x30, 0x38, // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02, 0x16, 0x02, 0x00, 0x01 // D_0_22 inv_set_fifo_rate

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};


typedef struct {
	uint8_t bank;
	uint8_t address;
	uint8_t size;
	uint8_t data[];
} dmpUpdate_t;

static const dmpUpdate_t dmpUpdate0 = {0x01, 0xB2, 0x02, {0xFF, 0xFF}};
static const dmpUpdate_t dmpUpdate1 = {0x01, 0x90, 0x04, {0x09, 0x23, 0xA1, 0x35}};
static const dmpUpdate_t dmpUpdate2 = {0x01, 0x6A, 0x02, {0x06, 0x00}};
static const dmpUpdate_t dmpUpdate3 = {0x01, 0x60, 0x08, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static const dmpUpdate_t dmpUpdate4 = {0x00, 0x60, 0x04, {0x40, 0x00, 0x00, 0x00}};
static const dmpUpdate_t dmpUpdate5 = {0x01, 0x62, 0x02, {0x00, 0x00}};
static const dmpUpdate_t dmpUpdate6 = {0x00, 0x60, 0x04, {0x00, 0x40, 0x00, 0x00}};

static const dmpUpdate_t *dmpUpdates[7] = {
		&dmpUpdate0,
		&dmpUpdate1,
		&dmpUpdate2,
		&dmpUpdate3,
		&dmpUpdate4,
		&dmpUpdate5,
		&dmpUpdate6
};

static bool initialized = false;


bool imu_init(uint8_t address) {
	bool success = true;
	uint8_t data;

	mpu6050_setAddress(address);

	twi_master_init();

	success &= mpu6050_readReg(MPU6050_WHO_AM_I_REG_ADDR, &data);
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
	uint8_t data;
	bool success = true;

	if (!initialized) {
		return false;
	}

	twi_master_init();

	/* Clear the sleep and cycle bits in order to keep the IMU awake while we
	 * configure motion detection. */
	success &= mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &data);
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
	success &= mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &data);
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
	success &= mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &data);
	data |= (7 << MPU6050_ACCEL_HPF_POSN);
	success &= mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, 0x00);

	/* Clear any pending interrupt flag */
	success &= mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &data);

	/* Set the CYCLE bit so that the IMU goes to sleep and routinely wakes up
	 * to sample the accelerometers and determine whether motion has occurred.
	 */
	success &= mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &data);
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

	mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &data);
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
	mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &data);
	twi_master_deinit();
}

bool imu_reset() {
	return mpu6050_setBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_DEVICE_RESET_POSN));
}

bool imu_resetDMP() {
	return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_RESET_POSN));
}

bool imu_resetFIFO() {
	return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_RESET_POSN));
}

bool imu_resetI2CMaster() {
	return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_I2C_MST_RESET_POSN));
}

bool imu_getFIFOEnabled(bool *fifoEnabled) {
	uint8_t reg;

	if (!mpu6050_readReg(MPU6050_USER_CTRL_REG_ADDR, &reg)) {
		return false;
	}

	if (reg & (1<<MPU6050_FIFO_EN_POSN)) {
		*fifoEnabled = true;
	} else {
		*fifoEnabled = false;
	}

	return true;
}

bool imu_setFIFOEnabled(bool fifoEnabled) {
	if (fifoEnabled) {
		return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_EN_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_EN_POSN));
	}
}

bool imu_getDMPEnabled(bool *dmpEnabled) {
	uint8_t reg;

	if (!mpu6050_readReg(MPU6050_USER_CTRL_REG_ADDR, &reg)) {
		return false;
	}

	if (reg & (1<<MPU6050_DMP_EN_POSN)) {
		*dmpEnabled = true;
	} else {
		*dmpEnabled = false;
	}

	return true;
}

bool imu_setDMPEnabled(bool dmpEnabled) {
	if (dmpEnabled) {
		return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_EN_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_EN_POSN));
	}
}

bool imu_setClockSource(uint8_t clockSource) {
	uint8_t reg;
	bool success = true;

	clockSource <<= MPU6050_CLK_SEL_POSN;
	clockSource &= MPU6050_CLK_SEL_MASK;

	success &= mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &reg);
	reg &= ~(MPU6050_CLK_SEL_MASK);
	reg |= clockSource;
	success &= mpu6050_writeReg(MPU6050_PWR_MGMT_1_REG_ADDR, reg);

	return success;
}

bool imu_setExternalFrameSync(uint8_t externalSync) {
	uint8_t reg;
	bool success = true;

	externalSync <<= MPU6050_EXT_SYNC_SET_POSN;
	externalSync &= MPU6050_EXT_SYNC_SET_MASK;

	success &= mpu6050_readReg(MPU6050_CONFIG_REG_ADDR, &reg);
	reg &= ~(MPU6050_EXT_SYNC_SET_MASK);
	reg |= externalSync;
	success &= mpu6050_writeReg(MPU6050_CONFIG_REG_ADDR, reg);

	return success;
}

bool imu_setDLPFMode(uint8_t dlpfMode) {
	uint8_t reg;
	bool success = true;

	dlpfMode <<= MPU6050_DLPF_CFG_POSN;
	dlpfMode &= MPU6050_DLPF_CFG_MASK;

	success &= mpu6050_readReg(MPU6050_CONFIG_REG_ADDR, &reg);
	reg &= ~(MPU6050_DLPF_CFG_MASK);
	reg |= dlpfMode;
	success &= mpu6050_writeReg(MPU6050_CONFIG_REG_ADDR, reg);

	return success;
}

bool imu_setFullScaleGyroRange(uint8_t fsGyroRange) {
	uint8_t reg;
	bool success = true;

	fsGyroRange <<= MPU6050_FS_SEL_POSN;
	fsGyroRange &= MPU6050_FS_SEL_MASK;

	success &= mpu6050_readReg(MPU6050_GYRO_CONFIG_REG_ADDR, &reg);
	reg &= ~(MPU6050_FS_SEL_MASK);
	reg |= fsGyroRange;
	success |= mpu6050_writeReg(MPU6050_GYRO_CONFIG_REG_ADDR, reg);

	return success;
}

bool imu_getOTPBankValid(bool *otpBankValid) {
	uint8_t reg;
	bool success;

	success = mpu6050_readReg(MPU6050_AUX_VDDIO_REG_ADDR, &reg);
	if (reg & (1<<MPU6050_OTP_BANK_VLD_POSN)) {
		*otpBankValid = true;
	} else {
		*otpBankValid = false;
	}

	return success;
}

bool imu_setOTPBankValid(bool otpBankValid) {
	if (otpBankValid) {
		return mpu6050_setBits(MPU6050_AUX_VDDIO_REG_ADDR, (1<<MPU6050_OTP_BANK_VLD_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_AUX_VDDIO_REG_ADDR, (1<<MPU6050_OTP_BANK_VLD_POSN));
	}
}

bool imu_getXGyroOffsetTC(int8_t *offset) {
	uint8_t data;
	bool success;

	success = mpu6050_readReg(MPU6050_AUX_VDDIO_REG_ADDR, &data);
	data &= MPU6050_XG_OFFS_TC_MASK;
	*offset = data >> MPU6050_XG_OFFS_TC_POSN;

	return success;
}

bool imu_setXGyroOffsetTC(int8_t offset) {
	uint8_t data;

	data = offset << MPU6050_XG_OFFS_TC_POSN;
	data &= MPU6050_XG_OFFS_TC_MASK;

	return mpu6050_writeReg(MPU6050_AUX_VDDIO_REG_ADDR, data);
}

bool imu_getYGyroOffsetTC(int8_t *offset) {
	uint8_t data;
	bool success;

	success = mpu6050_readReg(MPU6050_YG_OFFS_TC_REG_ADDR, &data);
	data &= MPU6050_YG_OFFS_TC_MASK;
	*offset = data >> MPU6050_YG_OFFS_TC_POSN;

	return success;
}

bool imu_setYGyroOffsetTC(int8_t offset) {
	uint8_t data;

	data = offset << MPU6050_YG_OFFS_TC_POSN;
	data &= MPU6050_YG_OFFS_TC_MASK;

	return mpu6050_writeReg(MPU6050_YG_OFFS_TC_REG_ADDR, data);
}

bool imu_getZGyroOffsetTC(int8_t *offset) {
	uint8_t data;
	bool success;

	success = mpu6050_readReg(MPU6050_ZG_OFFS_TC_REG_ADDR, &data);
	data &= MPU6050_ZG_OFFS_TC_MASK;
	*offset = data >> MPU6050_ZG_OFFS_TC_POSN;

	return success;
}

bool imu_setZGyroOffsetTC(int8_t offset) {
	uint8_t data;

	data = offset << MPU6050_ZG_OFFS_TC_POSN;
	data &= MPU6050_ZG_OFFS_TC_MASK;

	return mpu6050_writeReg(MPU6050_ZG_OFFS_TC_REG_ADDR, data);
}

bool imu_setSleepEnabled(bool sleepEnabled) {
	if (sleepEnabled) {
		return mpu6050_setBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_SLEEP_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_SLEEP_POSN));
	}
}

bool imu_getFIFOCount(uint16_t *fifoCount) {
	bool success;
	uint8_t data[2];

	success = mpu6050_readBytes(MPU6050_FIFO_COUNTH_REG_ADDR, data, 2);
	*fifoCount = (((uint16_t)data[0]) << 8 | (uint16_t)data[1]);

	return success;
}

bool imu_getFIFOBytes(uint8_t *fifoBuffer, uint16_t fifoCount) {
	if (fifoCount == 0) {
		return true;
	}

	return mpu6050_readBytes(MPU6050_FIFO_R_W_REG_ADDR, fifoBuffer, fifoCount);
}

bool imu_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;

    if (userBank) {
    	bank |= 0x20;
    }

    if (prefetchEnabled) {
    	bank |= 0x40;
    }

    return mpu6050_writeReg(MPU6050_BANK_SEL_REG_ADDR, bank);
}

bool imu_setMemoryStartAddress(uint8_t address) {
	return mpu6050_writeReg(MPU6050_MEM_START_ADDR_REG_ADDR, address);
}

bool imu_readMemoryByte(uint8_t *data) {
	return mpu6050_readReg(MPU6050_MEM_R_W_REG_ADDR, data);
}

bool imu_writeMemoryByte(uint8_t data) {
	return mpu6050_writeReg(MPU6050_MEM_R_W_REG_ADDR, data);
}

bool imu_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
	bool success = true;
	uint8_t chunkSize;
	uint16_t i;

	imu_setMemoryBank(bank, false, false);
    imu_setMemoryStartAddress(address);

    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) {
        	chunkSize = dataSize - i;
        }

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) {
        	chunkSize = 256 - address;
        }

        // read the chunk of data as specified
        success &= mpu6050_readBytes(MPU6050_MEM_R_W_REG_ADDR, data + i, chunkSize);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) {
            	bank++;
            }
            success &= imu_setMemoryBank(bank, false, false);
            success &= imu_setMemoryStartAddress(address);
        }
    }

    return success;
}

bool imu_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
	bool success = true;
    uint8_t chunkSize;
    uint16_t i;
    uint8_t progBuffer[1+MPU6050_DMP_MEMORY_CHUNK_SIZE];
    uint8_t verifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];

    success &= imu_setMemoryBank(bank, false, false);
    success &= imu_setMemoryStartAddress(address);

    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) {
        	chunkSize = dataSize - i;
        }

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) {
        	chunkSize = 256 - address;
        }

        progBuffer[0] = MPU6050_MEM_R_W_REG_ADDR;
        memcpy(&progBuffer[1], data + i, chunkSize);
        mpu6050_writeBytes(progBuffer, chunkSize + 1);

        if (verify) {
        	success &= imu_setMemoryBank(bank, false, false);
        	success &= imu_setMemoryStartAddress(address);
        	success &= mpu6050_readBytes(MPU6050_MEM_R_W_REG_ADDR, verifyBuffer, chunkSize);
            if (memcmp(&progBuffer[1], verifyBuffer, chunkSize) != 0) {
                success = false;
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) {
            	bank++;
            }
            success &= imu_setMemoryBank(bank, false, false);
            success &= imu_setMemoryStartAddress(address);
        }
    }
    return success;
}

bool imu_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
    uint8_t *progBuffer, special;
    uint16_t i;
    uint8_t bank, offset, length;
    bool success = true;

    // config set data is a long string of blocks, each with the following structure:
    // {bank} {offset} {length} {byte[0], byte[1], ..., byte[length-1] | special}
    for (i = 0; i < dataSize;) {
		bank = data[i++];
		offset = data[i++];
		length = data[i++];

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
            progBuffer = (uint8_t *)data + i;
            success &= imu_writeMemoryBlock(progBuffer, length, bank, offset, true);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.

            special = data[i++];
            if (special == 0x01) {
                // enable DMP-related interrupts
            	success &= mpu6050_writeReg(MPU6050_INT_ENABLE_REG_ADDR,
            			(1<<MPU6050_ZMOT_EN_POSN) | (1<<MPU6050_FIFO_OFLOW_EN_POSN) | (1<<MPU6050_DMP_INT_EN_POSN));
            } else {
                success = false;
            }
        }
    }

    return success;
}


bool imu_initDMP() {
	char str[64];
	uint8_t hwRevision;
	bool otpValid;
	int8_t xgOffsetTC, ygOffsetTC, zgOffsetTC;
	uint16_t fifoCount;
	uint8_t fifoBuffer[128];
	uint8_t intStatus;
	bool success = true;

#if (DEBUG_IMU == 1)
	app_uart_put_string("Initializing IMU DMP\r\n");
#endif

	/* Reset the IMU */
	imu_reset();
	nrf_delay_ms(30);

	/* Bring the IMU out of sleep mode, which it enters automatically after
	 * reset. */
	success &= imu_setSleepEnabled(false);

	/* Read the hardware version from user bank 16, byte 6 */
	success &= imu_setMemoryBank(0x10, true, true);
	success &= imu_setMemoryStartAddress(0x06);
	success &= imu_readMemoryByte(&hwRevision);
	success &= imu_setMemoryBank(0x00, false, false);
#if (DEBUG_IMU == 1)
	snprintf(str, sizeof(str), "IMU hardware revision as per user[16][6] = %02X\r\n", hwRevision);
	app_uart_put_string(str);
#endif

	/* Check whether the OTB bank is valid */
	success &= imu_getOTPBankValid(&otpValid);
#if (DEBUG_IMU == 1)
	if (otpValid) {
		app_uart_put_string("IMU OTP bank is valid\r\n");
	} else {
		app_uart_put_string("IMU OPT bank is invalid\r\n");
	}
#endif

	/* Read the gyroscope offsets for all three axes */
	success &= imu_getXGyroOffsetTC(&xgOffsetTC);
	success &= imu_getYGyroOffsetTC(&ygOffsetTC);
	success &= imu_getZGyroOffsetTC(&zgOffsetTC);
	snprintf(str, sizeof(str), "IMU X gyro offset: %d\r\n", xgOffsetTC);
	app_uart_put_string(str);
	snprintf(str, sizeof(str), "IMU Y gyro offset: %d\r\n", ygOffsetTC);
	app_uart_put_string(str);
	snprintf(str, sizeof(str), "IMU Z gyro offset: %d\r\n", zgOffsetTC);
	app_uart_put_string(str);

	/* Setup weird I2C slave stuff.  The exact purpose of this is unknown. */

	/* Set slave 0 address to 0x7F */
	success &= mpu6050_writeReg(MPU6050_I2C_SLV0_ADDR_REG_ADDR, 0x7F);
	/* Disable I2C master mode */
	success &= mpu6050_clearBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_I2C_MST_EN_POSN));
	/* Set slave 0 address to own address */
	success &= mpu6050_writeReg(MPU6050_I2C_SLV0_ADDR_REG_ADDR, MPU6050_I2C_ADDR);
	/* Reset I2C master */
	success &= imu_resetI2CMaster();
	nrf_delay_ms(20);

	/* */
	if (imu_writeMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, true)) {
#if (DEBUG_IMU == 1)
		app_uart_put_string("IMU DMP code written and verified\r\n");
#endif
		if (imu_writeDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
#if (DEBUG_IMU == 1)
			app_uart_put_string("IMU DMP configuration written and verified\r\n");
#endif

			/* Set clock source to Z-axis gyroscope */
			success &= imu_setClockSource(MPU6050_CLK_SEL_PLL_ZGYRO);

			/* Enable only the DMP and FIFO overflow interrupts */
			success &= mpu6050_writeReg(MPU6050_INT_ENABLE_REG_ADDR,
					(1<<MPU6050_DMP_INT_EN_POSN) | (1<<MPU6050_FIFO_OFLOW_EN_POSN));

			/* Set the sample rate to 200Hz (1kHZ / (4+1) = 200Hz)*/
			success &= mpu6050_writeReg(MPU6050_SMPLRT_DIV_REG_ADDR, 4);

			/* Set the external frame sync to appear in the LSB of the temperature data*/
			success &= imu_setExternalFrameSync(MPU6050_EXT_SYNC_SET_TEMP_OUT_L);

			/* Set the digital low pass filter's bandwidth (44Hz for the
			 * accelerometers, 42Hz for the gyroscopes) */
			success &= imu_setDLPFMode(MPU6050_DLPF_CFG_44HZ_42HZ);

			/* Set the gyroscopes' full-scale range to +/- 2000 deg/sec */
			success &= imu_setFullScaleGyroRange(MPU6050_FS_SEL_2000DEGPERSEC);

			/* Set the DMP configuration bytes--the rationale behind this is
			 * unknown. */
			success &= mpu6050_writeReg(MPU6050_DMP_CFG_1_REG_ADDR, 0x03);
			success &= mpu6050_writeReg(MPU6050_DMP_CFG_2_REG_ADDR, 0x00);

			/* Clear the OTP bank valid flag */
			success &= imu_setOTPBankValid(false);

			/* Re-set the gyro offsets from the values read from the same
			 * registers earlier. */
			success &= imu_setXGyroOffsetTC(xgOffsetTC);
			success &= imu_setYGyroOffsetTC(ygOffsetTC);
			success &= imu_setZGyroOffsetTC(zgOffsetTC);

			success &= imu_writeMemoryBlock(dmpUpdates[0]->data, dmpUpdates[0]->size, dmpUpdates[0]->bank, dmpUpdates[0]->address, true);
			success &= imu_writeMemoryBlock(dmpUpdates[1]->data, dmpUpdates[1]->size, dmpUpdates[1]->bank, dmpUpdates[1]->address, true);

			success &= imu_resetFIFO();

			success &= imu_getFIFOCount(&fifoCount);
#if (DEBUG_IMU == 1)
			snprintf(str, sizeof(str), "IMU FIFO count after FIFO reset: %u\r\n", fifoCount);
			app_uart_put_string(str);
#endif

			success &= imu_getFIFOBytes(fifoBuffer, fifoCount);

			success &= mpu6050_writeReg(MPU6050_MOT_THR_REG_ADDR, 2);
			success &= mpu6050_writeReg(MPU6050_ZRMOT_THR_REG_ADDR, 156);
			success &= mpu6050_writeReg(MPU6050_MOT_DUR_REG_ADDR, 80);
			success &= mpu6050_writeReg(MPU6050_ZRMOT_DUR_REG_ADDR, 0);

			success &= imu_resetFIFO();

			success &= imu_setFIFOEnabled(true);

			success &= imu_setDMPEnabled(true);

			success &= imu_resetDMP();

			success &= imu_writeMemoryBlock(dmpUpdates[2]->data, dmpUpdates[2]->size, dmpUpdates[2]->bank, dmpUpdates[2]->address, true);
			success &= imu_writeMemoryBlock(dmpUpdates[3]->data, dmpUpdates[3]->size, dmpUpdates[3]->bank, dmpUpdates[3]->address, true);
			success &= imu_writeMemoryBlock(dmpUpdates[4]->data, dmpUpdates[4]->size, dmpUpdates[4]->bank, dmpUpdates[4]->address, true);

#if (DEBUG_IMU == 1)
			app_uart_put_string("IMU waiting for FIFO count > 2\r\n");
#endif
			do {
				success &= imu_getFIFOCount(&fifoCount);
			} while (fifoCount < 3);
			success &= imu_getFIFOBytes(fifoBuffer, fifoCount);

#if (DEBUG_IMU == 1)
			snprintf(str, sizeof(str), "IMU FIFO count after loading updates 3-5: %u\r\n", fifoCount);
			app_uart_put_string(str);
#endif

			success &= mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &intStatus);
#if (DEBUG_IMU == 1)
			snprintf(str, sizeof(str), "IMU interrupt status: 0x%02x\r\n", intStatus);
			app_uart_put_string(str);
#endif

			uint8_t dummyData[2];
			success &= imu_readMemoryBlock(dummyData, dmpUpdates[5]->size, dmpUpdates[5]->bank, dmpUpdates[5]->address);

#if (DEBUG_IMU == 1)
			app_uart_put_string("IMU waiting for FIFO count > 2\r\n");
#endif
			do {
				success &= imu_getFIFOCount(&fifoCount);
			} while (fifoCount < 3);
			success &= imu_getFIFOBytes(fifoBuffer, fifoCount);

#if (DEBUG_IMU == 1)
			snprintf(str, sizeof(str), "IMU FIFO count after reading(?) update 6: %u\r\n", fifoCount);
			app_uart_put_string(str);
#endif

			success &= mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &intStatus);
#if (DEBUG_IMU == 1)
			snprintf(str, sizeof(str), "IMU interrupt status: 0x%02x\r\n", intStatus);
			app_uart_put_string(str);
#endif

			success &= imu_writeMemoryBlock(dmpUpdates[6]->data, dmpUpdates[6]->size, dmpUpdates[6]->bank, dmpUpdates[6]->address, true);

			/* Disable DMP for now, it can be re-enabled later */
			success &= imu_setDMPEnabled(false);

			success &= imu_resetFIFO();
			success &= mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &intStatus);

		} else {
			success = false;
#if (DEBUG_IMU == 1)
			app_uart_put_string("IMP DMP configuration verification failed\r\n");
#endif
		}

	} else {
		success = false;
#if (DEBUG_IMU == 1)
		app_uart_put_string("IMP DMP code verification failed\r\n");
#endif
	}

#if (DEBUG_IMU == 1)
	if (success) {
		app_uart_put_string("IMU DMP initialization complete\r\n");
	} else {
		app_uart_put_string("IMU DMP initialization failed\r\n");
	}
#endif

	return success;
}

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

bool imu_getQuaternion(quaternion_t *q, const uint8_t *packet) {
	int16_t w_raw, x_raw, y_raw, z_raw;

    w_raw = ((int16_t)packet[0] << 8) + (int16_t)packet[1];
    x_raw = ((int16_t)packet[4] << 8) + (int16_t)packet[5];
    y_raw = ((int16_t)packet[8] << 8) + (int16_t)packet[9];
    z_raw = ((int16_t)packet[12] << 8) + (int16_t)packet[13];

    q->w = (float)w_raw / 16384.0f;
    q->x = (float)x_raw / 16384.0f;
    q->y = (float)y_raw / 16384.0f;
    q->z = (float)z_raw / 16384.0f;

    return true;
}

bool imu_getGravityFromQuaternion(vectorFloat_t *v, const quaternion_t *q) {
	if ((q == NULL) || (v == NULL)) {
		return false;
	}

    v->x = 2 * (q->x * q->z - q->w*q->y);
    v->y = 2 * (q->w * q->x + q->y * q->z);
    v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    return true;
}

float imu_getVectorAngle(const vectorFloat_t *v, const vectorFloat_t *u) {
	float dotProduct;

	dotProduct = (v->x * u->x) + (v->y * u->y) + (v->z * u->z);
	return (180.0 / M_PI) * acosf(dotProduct);
}


bool imu_getLatestFIFOPacket(uint8_t *packet) {
	bool success = true;
	bool dmpEnabled, fifoEnabled;
	uint16_t fifoCount;
	uint8_t intStatus;

	/* Return indicating failure if the DMP is not running or the FIFO is not
	 * enabled. */
	success &= imu_getDMPEnabled(&dmpEnabled);
	success &= imu_getFIFOEnabled(&fifoEnabled);
	if (!success || !dmpEnabled || !fifoEnabled) {
		return false;
	}

	/* Read the interrupt flags and the number of bytes in the FIFO so that we
	 * can check for FIFO overflow. */
	success &= mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &intStatus);
	success &= imu_getFIFOCount(&fifoCount);

	/* If we fail to read the interrupt status register or the number of bytes
	 * in the FIFO, we return failure without providing the caller with the
	 * newest packet. */
	if (!success) {
		return false;
	}

	/* If the FIFO has overflowed, reset it */
	if ((intStatus & (1<<MPU6050_FIFO_OFLOW_INT_POSN)) || (fifoCount >= 1024)) {
		imu_resetFIFO();
		fifoCount = 0;
	}

	/* Wait until the FIFO contains at least a single complete packet. */
	while (fifoCount < IMU_FIFO_PACKET_SIZE) {
		/* If we fail to read the number of bytes in the FIFO, return false
		 * without providing the caller with the most recent FIFO packet. */
		if (!imu_getFIFOCount(&fifoCount)) {
			return false;
		}
	}

	/* Read packets from the FIFO until there are no complete packets
	 * remaining. */
	do {
		/* Read the oldest packet from the FIFO.  If we fail to do so, return
		 * false whithout providing the caller with the most recent packet. */
		if (!imu_getFIFOBytes(packet, IMU_FIFO_PACKET_SIZE)) {
			return false;
		}

		/* Having read a packet, decrement the number of bytes in the FIFO by
		 * the packet's size.  We will read another packet if the new count is
		 * still equal to or larger than the size of a single packet. */
		fifoCount -= IMU_FIFO_PACKET_SIZE;
	} while (fifoCount >= IMU_FIFO_PACKET_SIZE);

	return true;
}

const vectorFloat_t cornerVectors[3] = {
		{0.0f, 0.0f, 1.0f},
		{0.707107f, 0.707107f, 0.0f},
		{0.707107f, -0.707107f, 0.0f},
};

void imu_testDMPLoop() {
	char str[64];
	uint16_t fifoCount;
	uint8_t intStatus;
	uint8_t packetBuffer[IMU_FIFO_PACKET_SIZE];
	quaternion_t q;
	vectorFloat_t gravity;

	bool success = true;

	success = imu_setDMPEnabled(true);

	while (success) {
		success &= imu_getLatestFIFOPacket(packetBuffer);
		success &= imu_getQuaternion(&q, packetBuffer);

		success &= imu_getGravityFromQuaternion(&gravity, &q);

		snprintf(str, sizeof(str), "Gravity: %5.4f %5.4f %5.4f\r\n", gravity.x, gravity.y, gravity.z);
		app_uart_put_string(str);

		snprintf(str, sizeof(str), "Alignment angles: %4.1f %4.1f %4.1f\r\n",
				imu_getVectorAngle(&gravity, &cornerVectors[0]),
				imu_getVectorAngle(&gravity, &cornerVectors[1]),
				imu_getVectorAngle(&gravity, &cornerVectors[2])
				);
		app_uart_put_string(str);

		nrf_delay_ms(250);
	}


	while (success) {
		success &= mpu6050_readReg(MPU6050_INT_STATUS_REG_ADDR, &intStatus);
		success &= imu_getFIFOCount(&fifoCount);

		if ((intStatus & (1<<MPU6050_FIFO_OFLOW_INT_POSN)) || (fifoCount == 1024)) {
			app_uart_put_string("FIFO overflow\r\n");
			imu_resetFIFO();
		} else {
			while (fifoCount < IMU_FIFO_PACKET_SIZE) {
				success &= imu_getFIFOCount(&fifoCount);
			}

			success &= imu_getFIFOBytes(packetBuffer, sizeof(packetBuffer));
			fifoCount -= sizeof(packetBuffer);

			success &= imu_getQuaternion(&q, packetBuffer);

			//snprintf(str, sizeof(str), "Quaternion: %5.4f %5.4f %5.4f %5.4f\r\n", q.w, q.x, q.y, q.z);
			//app_uart_put_string(str);

			success &= imu_getGravityFromQuaternion(&gravity, &q);

			snprintf(str, sizeof(str), "Gravity: %5.4f %5.4f %5.4f\r\n", gravity.x, gravity.y, gravity.z);
			app_uart_put_string(str);
		}

	}
}
