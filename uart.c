/*
 * uart.c
 *
 *  Created on: Apr 3, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "nrf_gpio.h"

#include "app_util.h"
#include "app_uart.h"

#include "pins.h"

static bool initialized = false;
static uint16_t app_uart_uid = 0;
static uint8_t rx_buf[64];
static uint8_t tx_buf[512];

static void on_uart_evt(app_uart_evt_t *p_app_uart_event);

void uart_init() {
	uint32_t err_code;
	app_uart_comm_params_t comm_params;
    app_uart_buffers_t buffers;

	comm_params.rx_pin_no = UART_RX_PIN_NO;
	comm_params.tx_pin_no = UART_TX_PIN_NO;
	comm_params.rts_pin_no = (uint8_t)UART_PIN_DISCONNECTED;
	comm_params.cts_pin_no = (uint8_t)UART_PIN_DISCONNECTED;
	comm_params.flow_control = APP_UART_FLOW_CONTROL_DISABLED;
	comm_params.use_parity = false;
	comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200;

    buffers.rx_buf = rx_buf;
    buffers.rx_buf_size = sizeof(rx_buf);
    buffers.tx_buf = tx_buf;
    buffers.tx_buf_size = sizeof(tx_buf);

	err_code = app_uart_init(&comm_params, &buffers, on_uart_evt, 3, &app_uart_uid);
	APP_ERROR_CHECK(err_code);

	initialized = true;
}


void uart_deinit() {
	if (!initialized) {
		return;
	}

	app_uart_close(app_uart_uid);
	app_uart_uid = 0;

	/* Make the TXD pin an output which drives high */
    nrf_gpio_pin_set(UART_TX_PIN_NO);
    GPIO_PIN_CONFIG((UART_TX_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    /* Make the RXD pin an input will a pull-up */
    nrf_gpio_pin_set(UART_RX_PIN_NO);
    GPIO_PIN_CONFIG((UART_RX_PIN_NO),
    		GPIO_PIN_CNF_DIR_Input,
    		GPIO_PIN_CNF_INPUT_Connect,
    		GPIO_PIN_CNF_PULL_Pullup,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

	initialized = false;
}

void on_uart_evt(app_uart_evt_t *p_app_uart_event) {
	;
}
