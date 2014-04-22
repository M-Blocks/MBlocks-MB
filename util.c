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

#include "global.h"
#include "util.h"

extern ble_sps_t m_sps;

uint32_t app_uart_put_string(const char *str) {
	uint32_t i = 0;
	uint32_t err_code = NRF_SUCCESS;

#if (ENABLE_BLE_COMMANDS == 1)
	if (ble_sps_getInitialized()) {
		ble_sps_put_string(&m_sps, (const uint8_t *)str);
	}
#endif

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
    uint32_t elapsedTicks;
    uint32_t scheduleTicks;

    if (app_timer_cnt_get(&startTicks) != NRF_SUCCESS) {
        return false;
    }
    if (app_timer_cnt_get(&scheduleTicks) != NRF_SUCCESS) {
        return false;
    }

    do {
        if (app_timer_cnt_get(&nowTicks) != NRF_SUCCESS) {
            return false;
        }

        /* The RTC is only a 24-bit counter, so when subtracting the two
         * tick counts to find the elapsed number of ticks, we must mask out the
         * 8 most significant bits. */
        elapsedTicks = 0x00FFFFFF & (nowTicks - startTicks);

#if (0)
        uint32_t intermediaryTicks;
        intermediaryTicks = elapsedTicks - scheduleTicks;

        if (intermediaryTicks > APP_TIMER_TICKS(15, APP_TIMER_PRESCALER)) {
        	app_sched_execute();
            if (app_timer_cnt_get(&scheduleTicks) != NRF_SUCCESS) {
            	return false;
            }
        }
#endif

        if (elapsedTicks * USEC_PER_APP_TIMER_TICK >= 1000 * ms) {
            return true;
        }
    } while (1);

    return false;
}
