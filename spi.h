/*
 * spi.h
 *
 *  Created on: Nov 25, 2013
 *      Author: kwgilpin
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <stdbool.h>

void spi_init();
bool spi_txRx(uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data);

#endif /* SPI_H_ */
