/*
 * a4960.c
 *
 *  Created on: Nov 26, 2013
 *      Author: Kyle Gilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "a4960.h"

bool a4960_writeReg(uint8_t addr, uint16_t data, uint16_t *diag) {
	uint8_t writeData[2];
	uint8_t readData[2];

	if (addr & ~0x07) {
		/* The address must be only consume the three least-significant bytes
		 * of the addr argument. */
		return false;
	}

	if (data & ~0xFFF) {
		/* All registers hold a maximum of 12 bits */
		return false;
	}

	/* Each SPI transaction is 16-bits long, and data is transmitted MSB-first.
	 * A packet is structured as follows:
	 *   A2 A1 A0 WR D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
	 *   \__________   __________/ \_________   _________/
	 *              \ /                      \ /
	 *          writeData[0]             writeData[1]
	 */

	/* Move the address into position */
	writeData[0] = (addr << 5);

	/* Set the WR bit to indicate that we are writing to a register */
	writeData[0] |= (0x01 << 4);

	/* Move the 4 most significant bits of data into position */
	writeData[0] |= (data >> 8) & 0x0F;

	/* Place the 8 least significant bits of data into position */
	writeData[1] = data & 0xFF;

	/* Send the data and receive the A4960's response. */
	if (!spi_txRx(2, writeData, readData)) {
		return false;
	}

	/* Any time we write one of the A4960's registers, it returns the contents
	 * of its diagnostic register.  Assuming that the caller provided a
	 * destination for this data, we pass it back to the caller. */
	if (diag != NULL) {
		*diag = (readData[0] << 0xFF) | readData[1];
	}

	return true;
}

bool a4960_readReg(uint8_t addr, uint16_t *data) {
	uint8_t writeData[2];
	uint8_t readData[2];
	bool readSuccess;

	if (addr & ~0x07) {
		/* The address must be only consume the three least-significant bytes
		 * of the addr argument. */
		return false;
	}

	if (data == NULL) {
		return false;
	}

	/* Each SPI transaction is 16-bits long, and data is transmitted MSB-first.
	 * A packet is structured as follows:
	 *   A2 A1 A0 WR D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
	 *   \__________   __________/ \_________   _________/
	 *              \ /                      \ /
	 *          writeData[0]             writeData[1]
	 */

	/* Move the address into position.  This also implicitly clears the WR bit
	 * to indicate that we wish to read data. */
	writeData[0] = (addr << 5);

	/* Leave the 8 least significant bits of the 16-bit word set to 0. */
	writeData[1] = 0x00;

	/* Send the data and receive the A4960's response. */
	readSuccess = spi_txRx(2, writeData, readData);

	/* Combine the two bytes read from the A4960 into a 16-bit word being sure
	 * to mask the diagnostic bits that are always returned in
	 * readData[0][7:5].  Even if the spi_txRx function indicates failure, we
	 * still attempt to present the caller with the returned data. */
	*data  = (readData[0] & 0x0F) << 8;
	*data |= readData[1];

	if (readSuccess) {
		return true;
	}

	return false;
}

bool a4960_readDiag(uint16_t *diag) {
	uint16_t maskReg;

	/* To read the diagnostic register, we simply write to any register and the
	 * A4960 returns the diagnostic register's contents on the MISO line. There
	 * is no dummy register to write, so we first read the mask register and
	 * then write the same data back to the mask register.  We choose the mask
	 * register because an error is least likely to affect operation of the
	 * chip. */

	if (!a4960_readReg(A4960_MASK_REG_ADDR, &maskReg)) {
		return false;
	}

	return a4960_writeReg(A4960_MASK_REG_ADDR, maskReg, diag);
}
