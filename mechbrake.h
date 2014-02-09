/*
 * mechbrake.h
 *
 *  Created on: Feb 9, 2014
 *      Author: kwgilpin
 */

#ifndef MECHBRAKE_H_
#define MECHBRAKE_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	MECHBRAKE_DIR_CW = 0,
	MECHBRAKE_DIR_CCW = 1
} mechbrake_dir_t;

bool mechbrake_actuate(mechbrake_dir_t direction, uint16_t current_mA, uint8_t time_ms);

#endif /* MECHBRAKE_H_ */
