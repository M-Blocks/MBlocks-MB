/*
 * spi.c
 *
 *  Created on: Nov 25, 2013
 *      Author: kwgilpin, sclaici
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

#include "nrf_drv_spi.h"
#include "spi_master_config.h"

#include "spi.h"

#if (SPI0_ENABLED == 1)
static const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(0);
#elif (SPI1_ENABLED == 1)
static const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(1);
#elif (SPI2_ENABLED == 1)
static const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(2);
#else
#error "No SPI enabled."
#endif

static bool initialized = false;

void spi_init() {    
    nrf_drv_spi_config_t const config =
	{
	    .orc          = 0xCC,
	    .frequency    = NRF_DRV_SPI_FREQ_1M,
	    .mode         = NRF_DRV_SPI_MODE_0,
	    .bit_order    = NRF_DRV_SPI_BIT_ORDER_LSB_FIRST,
	};
    ret_code_t err_code = nrf_drv_spi_init(&m_spi_master, &config, NULL);
    APP_ERROR_CHECK(err_code);

    initialized = true;
}

void spi_deinit() {
    if (!initialized) {
	return;
    }

    nrf_drv_spi_uninit(&m_spi_master);

    /* Make the SCK, MOSI, and CS pins outputs with well defined states */
    nrf_gpio_pin_clear(SPI_SCK_PIN_NO);
    nrf_gpio_cfg_output(SPI_SCK_PIN_NO);

    nrf_gpio_pin_clear(SPI_MOSI_PIN_NO);
    nrf_gpio_cfg_output(SPI_MOSI_PIN_NO);

    nrf_gpio_pin_set(SPI_BLDCCS_PIN_NO);
    nrf_gpio_cfg_output(SPI_BLDCCS_PIN_NO);

    /* Make sure the MISO pin has a pull-down so that it does not float and
     * consume extra current. */
    nrf_gpio_cfg_input(SPI_MISO_PIN_NO, NRF_GPIO_PIN_PULLDOWN);

    initialized = false;
}

bool spi_txRx(uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data) {
    if (!initialized) {
	return false;
    }

    uint32_t err_code = nrf_drv_spi_transfer(&m_spi_master,
					     tx_data, transfer_size, rx_data, transfer_size);
    APP_ERROR_CHECK(err_code);

    return true;
}
