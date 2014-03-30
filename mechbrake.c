/*
 * mechbrake.c
 *
 *  Created on: Feb 9, 2014
 *      Author: kwgilpin
 */


#include <stdint.h>
#include <stdbool.h>

#include "util.h"
#include "twi_master.h"
#include "db.h"
#include "mechbrake.h"

static bool directionsReversed = false;

bool mechbrake_actuate(uint8_t stepCount, const coilCurrentStep_t *steps) {
	uint8_t twiBuf[34];
	uint8_t i;
	uint32_t time_ms;
	uint16_t current_mA;
	bool rxSuccess;

	/* If the number of steps provided by the user will over-run the buffer,
	 * we truncate the list of steps. */
	if (2 + (4*stepCount) > sizeof(twiBuf)) {
		stepCount = (sizeof(twiBuf) - 2) / 4;
	}

	twiBuf[0] = DB_BRAKE_CMD;
	twiBuf[1] = stepCount;

	for (i=0; i<stepCount; i++) {
		if (mechbrake_getReverseDirections()) {
			current_mA = -steps[i].current_mA;
		} else {
			current_mA = steps[i].current_mA;
		}

		twiBuf[(4*i) + 2] = (current_mA >> 0) & 0xFF;
		twiBuf[(4*i) + 3] = (current_mA >> 8) & 0xFF;
		twiBuf[(4*i) + 4] = (steps[i].time_ms >> 0) & 0xFF;
		twiBuf[(4*i) + 5] = (steps[i].time_ms >> 8) & 0xFF;
	}

	if (!twi_master_transfer((DB_TWI_ADDR << 1), twiBuf, 2 + (4*stepCount), true)) {
		return false;
	}

	delay_ms(DB_POLL_INTERVAL_MS);

	for (time_ms = 0; time_ms < DB_BRAKE_TIMEOUT_MS; time_ms += DB_POLL_INTERVAL_MS) {
		rxSuccess = twi_master_transfer((DB_TWI_ADDR << 1) | TWI_READ_BIT, twiBuf, 2, true);
		if (!rxSuccess) {
			return false;
		}

		if (rxSuccess && (twiBuf[0] == DB_BRAKE_CMD) && (twiBuf[1] == 0x01)) {
			return true;
		}

		delay_ms(DB_POLL_INTERVAL_MS);
	}

	return false;
}

void mechbrake_setReverseDirections(bool reverse) {
	directionsReversed = reverse;
}

bool mechbrake_getReverseDirections() {
	return directionsReversed;
}

