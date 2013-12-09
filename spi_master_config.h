/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef SPI_MASTER_CONFIG_H
#define SPI_MASTER_CONFIG_H

#include "pins.h"

#define SPI_OPERATING_FREQUENCY  ( 0x02000000UL << (uint32_t)Freq_1Mbps )  /*!< Slave clock frequency. */

/*  SPI0 */
#define SPI_PSELSCK0              SPI_SCK_PIN_NO	/*!< GPIO pin number for SPI clock				*/
#define SPI_PSELMOSI0             SPI_MOSI_PIN_NO	/*!< GPIO pin number for Master Out Slave In	*/
#define SPI_PSELMISO0             SPI_MISO_PIN_NO 	/*!< GPIO pin number for Master In Slave Out	*/
#define SPI_PSELSS0               SPI_BLDCCS_PIN_NO	/*!< GPIO pin number for Slave Select			*/

/*  SPI1 */
#define SPI_PSELSCK1              SPI_SCK_PIN_NO	/*!< GPIO pin number for SPI clock				*/
#define SPI_PSELMOSI1             SPI_MOSI_PIN_NO	/*!< GPIO pin number for Master Out Slave In	*/
#define SPI_PSELMISO1             SPI_MISO_PIN_NO	/*!< GPIO pin number for Master In Slave Out	*/
#define SPI_PSELSS1               SPI_BLDCCS_PIN_NO	/*!< GPIO pin number for Slave Select			*/

#define TIMEOUT_COUNTER          0x3000UL  /*!< timeout for getting rx bytes from slave */


#endif /* SPI_MASTER_CONFIG_H */
