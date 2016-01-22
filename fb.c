/*
 * fb.c
 *
 *  Created on: Apr 17, 2015
 *      Author: kwgilpin, sclaici
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "twi_master_config.h"

#include "nrf_drv_twi.h"
#include "app_error.h"
#include "fb.h"

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

bool fb_getVersion(uint8_t faceNum, char *verStr, uint8_t verStrSize) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;
    
    if ((faceNum < 1) || (faceNum > 6)) {
	return false;
    }
    
    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_VERSION_STRING;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, (uint8_t *)verStr, verStrSize, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    /* Ensure that the version string is correctly terminated. */
    verStr[verStrSize-1] = '\0';

    return success;
}


bool fb_setTopLEDs(uint8_t faceNum, bool redOn, bool greenOn, bool blueOn) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);


    twiBuf[0] = FB_REGISTER_ADDR_LEDS_TOP;

    twiBuf[1]  = 0x00;
    twiBuf[1] |= redOn ? 0x01 : 0x00;
    twiBuf[1] |= greenOn ? 0x02 : 0x00;
    twiBuf[1] |= blueOn ? 0x04 : 0x00;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getTopLEDs(uint8_t faceNum, bool *redOn, bool *greenOn, bool *blueOn) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_LEDS_TOP;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    if (success) {
	*redOn = (twiBuf[0] & 0x01) ? true : false;
	*greenOn = (twiBuf[0] & 0x02) ? true : false;
	*blueOn = (twiBuf[0] & 0x04) ? true : false;
    }

    return success;
}

bool fb_setBottomLEDs(uint8_t faceNum, bool redOn, bool greenOn, bool blueOn) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_LEDS_BOTTOM;

    twiBuf[1]  = 0x00;
    twiBuf[1] |= redOn ? 0x01 : 0x00;
    twiBuf[1] |= greenOn ? 0x02 : 0x00;
    twiBuf[1] |= blueOn ? 0x04 : 0x00;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getBottomLEDs(uint8_t faceNum, bool *redOn, bool *greenOn, bool *blueOn) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_LEDS_BOTTOM;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    if (success) {
	*redOn = (twiBuf[0] & 0x01) ? true : false;
	*greenOn = (twiBuf[0] & 0x02) ? true : false;
	*blueOn = (twiBuf[0] & 0x04) ? true : false;
    }

    return success;
}

int16_t fb_getAmbientLight(uint8_t faceNum) {
    uint8_t twiBuf[2];
    bool success = true;
    int16_t ambientLight;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_AMBIENT_LIGHT;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    /* The 10-bit result is returned left-shifted so that it is possible to
     * read just one byte and still get most of the resolution (even though
     * we still read both bytes). */
    ambientLight  = twiBuf[0] << 2;
    ambientLight |= twiBuf[1] >> 6;

    if (!success) {
	ambientLight = -1;
    }

    return ambientLight;
}

bool fb_setIRManualLEDs(uint8_t faceNum, bool led1, bool led2, bool led3, bool led4) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0]  = FB_REGISTER_ADDR_IR_LEDS_MANUAL_CONTROL;
    twiBuf[1]  = led1 ? 0x01 : 0x00;
    twiBuf[1] |= led2 ? 0x02 : 0x00;
    twiBuf[1] |= led3 ? 0x04 : 0x00;
    twiBuf[1] |= led4 ? 0x08 : 0x00;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getIRManualLEDs(uint8_t faceNum, bool *led1, bool *led2, bool *led3, bool *led4) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_IR_LEDS_MANUAL_CONTROL;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    
    nrf_drv_twi_uninit(&m_twi_master);

    if (success) {
	*led1 = (twiBuf[0] & 0x01) ? true : false;
	*led2 = (twiBuf[0] & 0x02) ? true : false;
	*led3 = (twiBuf[0] & 0x04) ? true : false;
	*led4 = (twiBuf[0] & 0x08) ? true : false;
    }

    return success;
}

bool fb_sendToTxBuffer(uint8_t faceNum, uint8_t numBytes, const uint8_t *bytes) {
    uint8_t twiBuf[256];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0]  = FB_REGISTER_ADDR_TX_BUF;
    // pad message with 0xDE as there is a tendency to drop the first few characters
    twiBuf[1] = twiBuf[2] = twiBuf[3] = 0xB7;
    memcpy(&twiBuf[4], bytes, numBytes);

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 4 + numBytes, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_queueToTxBuffer(uint8_t faceNum, uint8_t numBytes, const uint8_t *bytes) {
    uint8_t twiBuf[256];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_TX_MSG_BUF;
    // pad message with 0xDE as there is a tendency to drop the first few characters
    twiBuf[1] = twiBuf[2] = twiBuf[3] = 0xB7;
    memcpy(&twiBuf[1], bytes, numBytes);

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 4 + numBytes, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_sendMsgToTxBuffer(uint8_t faceNum, bool flash) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_TX_MSG_CONTROL;
    twiBuf[1] = flash ? (0x01 | 0x02) : 0x00; 

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getTxBufferAvailableCount(uint8_t faceNum, uint8_t *bytesAvailable) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_TX_AVAILABLE_COUNT;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    if (success) {
	*bytesAvailable = twiBuf[0];
    }

    return success;
}

bool fb_setIRTxLEDs(uint8_t faceNum, bool led1, bool led2, bool led3, bool led4) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0]  = FB_REGISTER_ADDR_TX_LED_SELECT;
    twiBuf[1]  = led1 ? 0x01 : 0x00;
    twiBuf[1] |= led2 ? 0x02 : 0x00;
    twiBuf[1] |= led3 ? 0x04 : 0x00;
    twiBuf[1] |= led4 ? 0x08 : 0x00;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getIRTxLEDs(uint8_t faceNum, bool *led1, bool *led2, bool *led3, bool *led4) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_TX_LED_SELECT;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    if (success) {
	*led1 = (twiBuf[0] & 0x01) ? true : false;
	*led2 = (twiBuf[0] & 0x02) ? true : false;
	*led3 = (twiBuf[0] & 0x04) ? true : false;
	*led4 = (twiBuf[0] & 0x08) ? true : false;
    }

    return success;
}

bool fb_receiveFromRxBuffer(uint8_t faceNum, uint8_t numBytes, uint8_t *bytes) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_RX_BUF;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, numBytes, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    
    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getRxBufferConsumedCount(uint8_t faceNum, uint8_t *bytesConsumed) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_RX_CONSUMED_COUNT;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    if (success) {
	*bytesConsumed = twiBuf[0];
    }

    return success;
}

bool fb_flushRxBuffer(uint8_t faceNum) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_RX_FLUSH;
    twiBuf[1] = 0x01;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getRxAmbientLightBuffer(uint8_t faceNum, uint8_t numBytes, uint8_t *bytes) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_RX_AMBIENT_BUF;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, numBytes, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getRxAmbientBufferConsumedCount(uint8_t faceNum, uint8_t *bytesConsumed) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if ((faceNum < 1) || (faceNum > 6)) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_RX_AMBIENT_CONSUMED_COUNT;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    if (success) {
	*bytesConsumed = twiBuf[0];
    }

    return success;
}

bool fb_setRxEnable(uint8_t faceNum, bool rxEnable) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_RX_ENABLE;
    twiBuf[1] = rxEnable ? 0x01 : 0x00;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}

bool fb_getRxEnable(uint8_t faceNum, bool *rxEnabled) {
    uint8_t twiBuf[2];
    bool success = true;
    ret_code_t err_code;

    if (faceNum > 6) {
	return -1;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_RX_ENABLE;

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(&m_twi_master, faceNum, twiBuf, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    nrf_drv_twi_uninit(&m_twi_master);

    if (twiBuf[0] & 0x01) {
	*rxEnabled = true;
    } else {
	*rxEnabled = false;
    }

    return success;
}

bool fb_sleep(uint8_t faceNum, bool sleepEnabled) {
    uint8_t twiBuf[2];
    bool success;
    ret_code_t err_code;

    if (faceNum > 6) {
	return false;
    }

    err_code = nrf_drv_twi_init(&m_twi_master, &m_twi_master_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    twiBuf[0] = FB_REGISTER_ADDR_SLEEP;
    if (sleepEnabled) {
	twiBuf[1] = 0x01;
    } else {
	twiBuf[1] = 0x00;
    }

    err_code = nrf_drv_twi_tx(&m_twi_master, faceNum, twiBuf, 2, true);
    if (err_code != NRF_SUCCESS)
	success = false;
    
    /* After sending a sleep command, we do not attempt to read a response
     * because doing so will wake-up the daughterboard processor. */
    nrf_drv_twi_uninit(&m_twi_master);

    return success;
}
