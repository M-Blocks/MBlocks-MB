/*
 * fb.c
 *
 *  Created on: Apr 17, 2015
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "twi_master_config.h"
#include "twi_master.h"

#include "fb.h"

bool fb_getVersion(uint8_t faceNum, char *verStr, uint8_t verStrSize) {
	uint8_t twiBuf[2];
	bool success = true;

	if ((faceNum < 1) || (faceNum > 6)) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_VERSION_STRING;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, (uint8_t *)verStr, verStrSize, true);

	twi_master_deinit();

	/* Ensure that the version string is correctly terminated. */
	verStr[verStrSize-1] = '\0';

	return success;
}


bool fb_setTopLEDs(uint8_t faceNum, bool redOn, bool greenOn, bool blueOn) {
	uint8_t twiBuf[2];
	bool success = true;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_LEDS_TOP;

	twiBuf[1]  = 0x00;
	twiBuf[1] |= redOn ? 0x01 : 0x00;
	twiBuf[1] |= greenOn ? 0x02 : 0x00;
	twiBuf[1] |= blueOn ? 0x04 : 0x00;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 2, true);

	twi_master_deinit();

	return success;
}

bool fb_getTopLEDs(uint8_t faceNum, bool *redOn, bool *greenOn, bool *blueOn) {
	uint8_t twiBuf[2];
	bool success = true;

	if ((faceNum < 1) || (faceNum > 6)) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_LEDS_TOP;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, twiBuf, 1, true);

	twi_master_deinit();

	if (success) {
		*redOn = (twiBuf[0] & 0x01) ? true : false;
		*greenOn = (twiBuf[0] & 0x02) ? true : false;
		*blueOn = (twiBuf[0] & 0x04) ? true : false;
	}

	return success;
}

bool fb_setBottomLEDs(uint8_t faceNum, bool redOn, bool greenOn, bool blueOn) {
	uint8_t twiBuf[2];
	bool success = true;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_LEDS_BOTTOM;

	twiBuf[1]  = 0x00;
	twiBuf[1] |= redOn ? 0x01 : 0x00;
	twiBuf[1] |= greenOn ? 0x02 : 0x00;
	twiBuf[1] |= blueOn ? 0x04 : 0x00;

	success = twi_master_transfer((faceNum << 1), twiBuf, 2, true);

	twi_master_deinit();

	return success;
}

bool fb_getBottomLEDs(uint8_t faceNum, bool *redOn, bool *greenOn, bool *blueOn) {
	uint8_t twiBuf[2];
	bool success = true;

	if ((faceNum < 1) || (faceNum > 6)) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_LEDS_BOTTOM;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, twiBuf, 1, true);

	twi_master_deinit();

	if (success) {
		*redOn = (twiBuf[0] & 0x01) ? true : false;
		*greenOn = (twiBuf[0] & 0x02) ? true : false;
		*blueOn = (twiBuf[0] & 0x04) ? true : false;
	}

	return success;
}

int16_t fb_getAmbientLight(uint8_t faceNum) {
	uint8_t twiBuf[2];
	bool success = true;
	int16_t ambientLight;

	if ((faceNum < 1) || (faceNum > 6)) {
		return -1;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_AMBIENT_LIGHT;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, twiBuf, 2, true);

	twi_master_deinit();

	/* The 10-bit result is returned left-shifted so that it is possible to
	 * read just one byte and still get most of the resolution (even though
	 * we still read both bytes). */
	ambientLight  = twiBuf[0] << 2;
	ambientLight |= twiBuf[1] >> 6;

	if (!success) {
		ambientLight = -1;
	}

	return ambientLight;
}

bool fb_setIRManualLEDs(uint8_t faceNum, bool led1, bool led2, bool led3, bool led4) {
	uint8_t twiBuf[2];
	bool success = true;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0]  = FB_REGISTER_ADDR_IR_LEDS_MANUAL_CONTROL;
	twiBuf[1]  = led1 ? 0x01 : 0x00;
	twiBuf[1] |= led2 ? 0x02 : 0x00;
	twiBuf[1] |= led3 ? 0x04 : 0x00;
	twiBuf[1] |= led4 ? 0x08 : 0x00;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 2, true);

	twi_master_deinit();

	return success;
}

bool fb_getIRManualLEDs(uint8_t faceNum, bool *led1, bool *led2, bool *led3, bool *led4) {
	uint8_t twiBuf[2];
	bool success = true;

	if ((faceNum < 1) || (faceNum > 6)) {
		return -1;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_IR_LEDS_MANUAL_CONTROL;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, twiBuf, 1, true);

	twi_master_deinit();

	if (success) {
		*led1 = (twiBuf[0] & 0x01) ? true : false;
		*led2 = (twiBuf[0] & 0x02) ? true : false;
		*led3 = (twiBuf[0] & 0x04) ? true : false;
		*led4 = (twiBuf[0] & 0x08) ? true : false;
	}

	return success;
}

bool fb_sendToTxBuffer(uint8_t faceNum, uint8_t numBytes, const uint8_t *bytes) {
	uint8_t twiBuf[256];
	bool success = true;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0]  = FB_REGISTER_ADDR_TX_BUF;
	memcpy(&twiBuf[1], bytes, numBytes);

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1 + numBytes, true);

	twi_master_deinit();

	return success;
}

bool fb_sendMsgToTxBuffer(uint8_t faceNum, uint8_t numBytes, bool flash, const uint8_t *bytes) {
	uint8_t twiBuf[256];
	bool success = true;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_TX_MSG_CONTROL;
	twiBuf[1] = flash ? (0x01 | 0x02) : 0; 
	memcpy(&twiBuf[2], bytes, numBytes);

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1 + numBytes, true);

	twi_master_deinit();

	return success;
}

bool fb_getTxBufferAvailableCount(uint8_t faceNum, uint8_t *bytesAvailable) {
	uint8_t twiBuf[2];
	bool success = true;

	if ((faceNum < 1) || (faceNum > 6)) {
		return -1;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_TX_AVAILABLE_COUNT;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, twiBuf, 1, true);

	twi_master_deinit();

	if (success) {
		*bytesAvailable = twiBuf[0];
	}

	return success;
}

bool fb_setIRTxLEDs(uint8_t faceNum, bool led1, bool led2, bool led3, bool led4) {
	uint8_t twiBuf[2];
	bool success = true;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0]  = FB_REGISTER_ADDR_TX_LED_SELECT;
	twiBuf[1]  = led1 ? 0x01 : 0x00;
	twiBuf[1] |= led2 ? 0x02 : 0x00;
	twiBuf[1] |= led3 ? 0x04 : 0x00;
	twiBuf[1] |= led4 ? 0x08 : 0x00;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 2, true);

	twi_master_deinit();

	return success;
}

bool fb_getIRTxLEDs(uint8_t faceNum, bool *led1, bool *led2, bool *led3, bool *led4) {
	uint8_t twiBuf[2];
	bool success = true;

	if ((faceNum < 1) || (faceNum > 6)) {
		return -1;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_TX_LED_SELECT;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, twiBuf, 1, true);

	twi_master_deinit();

	if (success) {
		*led1 = (twiBuf[0] & 0x01) ? true : false;
		*led2 = (twiBuf[0] & 0x02) ? true : false;
		*led3 = (twiBuf[0] & 0x04) ? true : false;
		*led4 = (twiBuf[0] & 0x08) ? true : false;
	}

	return success;
}

bool fb_receiveFromRxBuffer(uint8_t faceNum, uint8_t numBytes, uint8_t *bytes) {
	uint8_t twiBuf[2];
	bool success = true;

	if ((faceNum < 1) || (faceNum > 6)) {
		return -1;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_RX_BUF;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, bytes, numBytes, true);

	twi_master_deinit();

	return success;
}

bool fb_getRxBufferConsumedCount(uint8_t faceNum, uint8_t *bytesConsumed) {
	uint8_t twiBuf[2];
	bool success = true;

	if ((faceNum < 1) || (faceNum > 6)) {
		return -1;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_RX_CONSUMED_COUNT;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, twiBuf, 1, true);

	twi_master_deinit();

	if (success) {
		*bytesConsumed = twiBuf[0];
	}

	return success;
}

bool fb_flushRxBuffer(uint8_t faceNum) {
	uint8_t twiBuf[2];
	bool success = true;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_RX_FLUSH;
	twiBuf[1] = 0x01;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 2, true);

	twi_master_deinit();

	return success;
}


bool fb_setRxEnable(uint8_t faceNum, bool rxEnable) {
	uint8_t twiBuf[2];
	bool success = true;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_RX_ENABLE;
	twiBuf[1] = rxEnable ? 0x01 : 0x00;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 2, true);

	twi_master_deinit();

	return success;
}

bool fb_getRxEnable(uint8_t faceNum, bool *rxEnabled) {
	uint8_t twiBuf[2];
	bool success = true;

	if (faceNum > 6) {
		return -1;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_RX_ENABLE;

	success &= twi_master_transfer((faceNum << 1), twiBuf, 1, true);
	success &= twi_master_transfer((faceNum << 1) | TWI_READ_BIT, twiBuf, 1, true);

	twi_master_deinit();

	if (twiBuf[0] & 0x01) {
		*rxEnabled = true;
	} else {
		*rxEnabled = false;
	}

	return success;
}

bool fb_sleep(uint8_t faceNum, bool sleepEnabled) {
	uint8_t twiBuf[2];
	bool success;

	if (faceNum > 6) {
		return false;
	}

	twi_master_init();

	twiBuf[0] = FB_REGISTER_ADDR_SLEEP;
	if (sleepEnabled) {
		twiBuf[1] = 0x01;
	} else {
		twiBuf[1] = 0x00;
	}

	success = twi_master_transfer((faceNum << 1), twiBuf, 2, true);

	/* After sending a sleep command, we do not attempt to read a response
	 * because doing so will wake-up the daughterboard processor. */
	twi_master_deinit();

	return success;
}
