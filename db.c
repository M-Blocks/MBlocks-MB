/*
 * db.c
 *
 *  Created on: Mar 20, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nrf51.h"
#include "nrf51_bitfields.h"

#include "app_error.h"

#include "util.h"
#include "twi_master_config.h"
#include "nrf_drv_twi.h"
#include "db.h"

#if (TWI0_ENABLED == 1)
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);
#elif (TWI1_ENABLED == 1)
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(1);
#elif (TWI2_ENABLED == 1)
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(2);
#else
#error "No TWI enabled."
#endif

const nrf_drv_twi_config_t m_twi_master_config = {
    .scl                     = TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER,
    .sda                     = TWI_MASTER_CONFIG_DATA_PIN_NUMBER,
    .frequency               = NRF_TWI_FREQ_100K
};
    
void db_reset() {
    TWI_SCL_OUTPUT();
    TWI_SCL_LOW();
    delay_ms(30);
    TWI_SCL_HIGH();
    TWI_SCL_INPUT();

    /* Allow daughterboard time to boot */
    delay_ms(5);
}

bool db_sleep(bool sleepEnabled) {
    uint8_t twiBuf[2];
    ret_code_t err_code;
    
    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    
    twiBuf[0] = DB_SLEEP_CMD;
    if (sleepEnabled) {
	twiBuf[1] = 0x01;
    } else {
	twiBuf[1] = 0x00;
    }

    nrf_drv_twi_enable(&m_twi_master);
    err_code = nrf_drv_twi_tx(&m_twi_master, DB_TWI_ADDR, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS) {
	nrf_drv_twi_uninit(&m_twi_master);
	return false;
    }

    /* After sending a sleep command, we do not attempt to read a response
     * because doing so will wake-up the daughterboard processor. */
    nrf_drv_twi_uninit(&m_twi_master);
    return true;
}

bool db_getVersion(char *verStr, uint8_t verStrSize) {
    uint8_t twiBuf[34];
    uint32_t time_ms;
    char *strPtr;
    ret_code_t err_code;
    
    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = DB_VERSION_CMD;

    nrf_drv_twi_enable(&m_twi_master);
    err_code = nrf_drv_twi_tx(&m_twi_master, DB_TWI_ADDR, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS) {
	nrf_drv_twi_uninit(&m_twi_master);
	return false;
    }

    delay_ms(DB_POLL_FIRST_INTERVAL_MS);

    for (time_ms = 0; time_ms < DB_VERSION_TIMEOUT_MS; time_ms += DB_POLL_INTERVAL_MS) {
	err_code = nrf_drv_twi_rx(&m_twi_master, DB_TWI_ADDR, twiBuf, sizeof(twiBuf), true);
	if (err_code != NRF_SUCCESS) {
	    /* If the daughterboard fails to respond to the I2C master read,
	     * something is wrong, so we return failure. */
	    nrf_drv_twi_uninit(&m_twi_master);
	    return false;
	}

	/* If the daughterboard responds and echoes the command code in the the
	 * first byte, and the second byte is 0x01, the version command was
	 * successful. */
	if ((twiBuf[0] == DB_VERSION_CMD) && (twiBuf[1] == 0x01)) {
	    /* The following bytes contain a human-readable version string. */
	    strPtr = (char *)&twiBuf[2];
	    /* If the version string will fit into the buffer provided by the
	     * caller, we copy it.  Otherwise, we copy whatever will fit and
	     * add a null-terminator. */
	    if (strlen(strPtr) < verStrSize) {
		strcpy(verStr, strPtr);
	    } else {
		strncpy(verStr, strPtr, verStrSize - 1);
		verStr[verStrSize-1] = '\0';
	    }

	    nrf_drv_twi_uninit(&m_twi_master);
	    return true;
	}

	/* If the daughterboard responded but has not yet processed the command
	 * we delay and then try to read from the daughterboard again. */
	delay_ms(DB_POLL_INTERVAL_MS);
    }

    nrf_drv_twi_uninit(&m_twi_master);
    return false;
}

bool db_getTemp(int16_t *temperature_tenthDegC) {
    uint8_t twiBuf[4];
    uint32_t time_ms;
    ret_code_t err_code;

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = DB_TEMPERATURE_CMD;

    nrf_drv_twi_enable(&m_twi_master);
    err_code = nrf_drv_twi_tx(&m_twi_master, DB_TWI_ADDR, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS) {
	nrf_drv_twi_uninit(&m_twi_master);
	return false;
    }

    delay_ms(DB_POLL_FIRST_INTERVAL_MS);

    for (time_ms = 0; time_ms < DB_TEMPERATURE_TIMEOUT_MS; time_ms += DB_POLL_INTERVAL_MS) {
        err_code = nrf_drv_twi_rx(&m_twi_master, DB_TWI_ADDR, twiBuf, sizeof(twiBuf), true);
	if (err_code == NRF_SUCCESS) {
	    /* If the daughterboard responds and echoes the command code in the the
	     * first byte, and the second byte is 0x01, the temperature command was
	     * successful. */
	    if ((twiBuf[0] == DB_TEMPERATURE_CMD) && (twiBuf[1] == 0x01)) {
		/* The following two bytes contain the temperature in tenth of a
		 * degree Celsius. */
		*temperature_tenthDegC  = (twiBuf[2] << 0);
		*temperature_tenthDegC |= (twiBuf[3] << 8);

		nrf_drv_twi_uninit(&m_twi_master);
		return true;
	    }
	}

	/* If the daughterboard responded but has not yet processed the command
	 * we delay and then try to read from the daughterboard again. */
	delay_ms(DB_POLL_INTERVAL_MS);
    }

    nrf_drv_twi_uninit(&m_twi_master);
    return false;
}

bool db_setLEDs(bool redOn, bool greenOn, bool blueOn) {
    uint8_t twiBuf[2];
    uint32_t time_ms;
    ret_code_t err_code;

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = DB_LED_CMD;

    twiBuf[1] = 0x00;
    if (redOn) {
	twiBuf[1] |= 0x01;
    }

    if (greenOn) {
	twiBuf[1] |= 0x02;
    }

    if (blueOn) {
	twiBuf[1] |= 0x04;
    }

    err_code = nrf_drv_twi_tx(&m_twi_master, DB_TWI_ADDR, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS) {
	nrf_drv_twi_uninit(&m_twi_master);
	return false;
    }

    delay_ms(DB_POLL_FIRST_INTERVAL_MS);

    for (time_ms = 0; time_ms < DB_LED_TIMEOUT_MS; time_ms += DB_POLL_INTERVAL_MS) {
	err_code = nrf_drv_twi_rx(&m_twi_master, DB_TWI_ADDR, twiBuf, sizeof(twiBuf), true);
	if (err_code != NRF_SUCCESS) {
	    /* If the daughterboard fails to respond to the I2C master read,
	     * something is wrong, so we return failure. */
	    nrf_drv_twi_uninit(&m_twi_master);
	    return false;
	}

	/* If the daughterboard responds and echoes the command code in the the
	 * first byte, and the second byte is 0x01, the temperature command was
	 * successful. */
	if ((twiBuf[0] == DB_LED_CMD) && (twiBuf[1] == 0x01)) {
	    nrf_drv_twi_uninit(&m_twi_master);
	    return true;
	}

	/* If the daughterboard responded but has not yet processed the command
	 * we delay and then try to read from the daughterboard again. */
	delay_ms(DB_POLL_INTERVAL_MS);
    }

    nrf_drv_twi_uninit(&m_twi_master);
    return false;
}
