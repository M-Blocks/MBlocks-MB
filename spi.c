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

#include "app_util.h"

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

void spi_deinit() {
	if (!initialized) {
		return;
	}

	NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos;

	/* Make the SCK, MOSI, and CS pins outputs with well defined states */
	nrf_gpio_pin_clear(SPI_SCK_PIN_NO);
    GPIO_PIN_CONFIG((SPI_SCK_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

	nrf_gpio_pin_clear(SPI_MOSI_PIN_NO);
    GPIO_PIN_CONFIG((SPI_MOSI_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

	nrf_gpio_pin_set(SPI_BLDCCS_PIN_NO);
    GPIO_PIN_CONFIG((SPI_BLDCCS_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

	/* Make sure the MISO pin has a pull-down so that it does not float and
	 * consume extra current. */
    GPIO_PIN_CONFIG((SPI_MISO_PIN_NO),
    		GPIO_PIN_CNF_DIR_Input,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Pulldown,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

	initialized = false;
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
