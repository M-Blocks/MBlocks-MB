/*
 * mechbrake.c
 *
 *  Created on: Feb 9, 2014
 *      Author: kwgilpin
 */


#include <stdint.h>
#include <stdbool.h>

#include "twi_master.h"
#include "db.h"
#include "mechbrake.h"

bool mechbrake_actuate(mechbrake_dir_t direction, uint16_t current_mA, uint8_t time_ms) {
	uint8_t twiBuf[5];

	twiBuf[0] = DB_BRAKE_CMD;

	if (direction == MECHBRAKE_DIR_CW) {
		twiBuf[1] = 0x00;
	} else if (direction == MECHBRAKE_DIR_CCW) {
		twiBuf[1] = 0x01;
	} else {
		return false;
	}

	twiBuf[2] = (current_mA >> 0) & 0xFF;
	twiBuf[3] = (current_mA >> 8) & 0xFF;

	twiBuf[4] = time_ms;

	return twi_master_transfer((DB_TWI_ADDR << 1), twiBuf, sizeof(twiBuf), true);
}
