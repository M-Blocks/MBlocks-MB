/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 * @defgroup rng_example_main main.c
 * @{
 * @ingroup rng_example
 * @brief Random Number Generator Example Application main file.
 *
 * @details
 * This file contains the source code for a sample application using the Random Number Generator.
 * The following PAN workarounds must be taken into consideration (all but number 24 are demonstrated):
 * - PAN_028 rev2.0A anomaly 21 - RNG: Generated random value is reset when VALRDY event is cleared.
 * - PAN_028 rev2.0A anomaly 22 - RNG: RNG does not generate a new number after the current number generated
 * - PAN_028 rev2.0A anomaly 23 - RNG: STOP task clears the VALUE register
 * - PAN_028 rev2.0A anomaly 24 - RNG: The STOP task cannot be assigned to a PPI channel.
 *
 * In short the PANS add the following restrictions: 
 * - The random number must be read before VALRDY is cleared.
 * - EVENTS_VALRDY must be cleared to start generating a new value
 * - read VALUE before triggering STOP task.
 * - The random number generator has to be stopped by writing to the STOP task register.
 * 
 */


#include <stdio.h>
#include <stdint.h>
#include "bsp.h"
#include "nrf_delay.h"
#include "app_uart.h"
#include "app_gpiote.h"
#include "app_error.h"

#define UART_TX_BUF_SIZE 256                                                          /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                                                            /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/** @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;
    APP_GPIOTE_INIT(1);
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud38400
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);

    NRF_RNG->TASKS_START = 1; // start the RNG peripheral.

    while (true)
    {
        // Clear the VALRDY EVENT.
        NRF_RNG->EVENTS_VALRDY = 0;

        // Wait until the value ready event is generated.
        while (NRF_RNG->EVENTS_VALRDY == 0)
        {
            // Do nothing.
        }

        // Set output according to the random value.
        printf("New random number is %d\n\r",(int)(NRF_RNG->VALUE));
        nrf_delay_ms(100);
    }
}


/** @} */
