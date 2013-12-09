/*
 * spi.c
 *
 *  Created on: Nov 25, 2013
 *      Author: kwgilpin
 */


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"

#include "spi_master.h"
#include "spi_master_config.h"

#include "spi.h"

static bool initialized = false;
static uint32_t *baseAddress = NULL;

void spi_init() {
	baseAddress = spi_master_init(0, SPI_MODE0, false);

	if (baseAddress == 0) {
		initialized = false;
		return;
	}

	initialized = true;
}

bool spi_txRx(uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data) {
	if (!initialized) {
		return false;
	}

	if (!spi_master_tx_rx(baseAddress, transfer_size, tx_data, rx_data)) {
		return false;
	}

	return true;
}
