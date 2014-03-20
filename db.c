/*
 * db.c
 *
 *  Created on: Mar 20, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "util.h"
#include "twi_master.h"
#include "db.h"

bool db_getVersion(char *verStr, uint8_t verStrSize) {
	uint8_t twiBuf[34];
	uint32_t time_ms;
	char *strPtr;

	twiBuf[0] = DB_VERSION_CMD;

	if (!twi_master_transfer((DB_TWI_ADDR << 1), twiBuf, 1, true)) {
		return false;
	}

	delay_ms(DB_POLL_INTERVAL_MS);

	for (time_ms = 0; time_ms < DB_VERSION_TIMEOUT_MS; time_ms += DB_POLL_INTERVAL_MS) {
		if (!twi_master_transfer((DB_TWI_ADDR << 1) | TWI_READ_BIT, twiBuf, sizeof(twiBuf), true)) {
			/* If the daughterboard fails to respond to the I2C master read,
			 * something is wrong, so we return failure. */
			return false;
		}

		/* If the daughterboard responds and echoes the command code in the the
		 * first byte, and the second byte is 0x01, the version command was
		 * successful. */
		if ((twiBuf[0] == DB_VERSION_CMD) && (twiBuf[1] == 0x01)) {
			/* The following bytes contain a human-readable version string. */
			strPtr = (char *)&twiBuf[2];
			/* If the version string will fit into the buffer provided by the
			 * caller, we copy it.  Otherwise, we copy whatever will fit and
			 * add a null-terminator. */
			if (strlen(strPtr) < verStrSize) {
				strcpy(verStr, strPtr);
			} else {
				strncpy(verStr, strPtr, verStrSize - 1);
				verStr[verStrSize-1] = '\0';
			}

			return true;
		}

		/* If the daughterboard responded but has not yet processed the command
		 * we delay and then try to read from the daughterboard again. */
		delay_ms(DB_POLL_INTERVAL_MS);
	}

	return false;
}
