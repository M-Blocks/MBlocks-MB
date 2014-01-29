/*
 * util.c
 *
 *  Created on: Nov 13, 2013
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "nrf_error.h"
#include "app_timer.h"

#include "app_uart.h"

#include "ble_sps.h"

#include "util.h"

extern ble_sps_t m_sps;

uint32_t app_uart_put_string(char *str) {
	uint32_t i = 0;
	uint32_t err_code = NRF_SUCCESS;

	ble_sps_put_string(&m_sps, str);

	while (str[i] != '\0') {
		err_code = app_uart_put(str[i++]);
		if (err_code != NRF_SUCCESS) {
			return err_code;
		}
	}

	return err_code;
}

bool delay_ms(uint32_t ms) {
	uint32_t startTicks;
	uint32_t nowTicks;

	if (app_timer_cnt_get(&startTicks) != NRF_SUCCESS) {
		return false;
	}

	do {
		if (app_timer_cnt_get(&nowTicks) != NRF_SUCCESS) {
			return false;
		}

		if ((nowTicks - startTicks) * 1000 >= 32768) {
			return true;
		}
	} while (1);

	return false;
}
