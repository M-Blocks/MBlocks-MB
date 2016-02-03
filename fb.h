/*
 * fb.h
 *
 *  Created on: Apr 17, 2015
 *      Author: kwgilpin
 */

#ifndef FB_H_
#define FB_H_

#include <stdint.h>
#include <stdbool.h>

#define FB_REGISTER_ADDR_LEDS_TOP					0x00
#define FB_REGISTER_ADDR_LEDS_BOTTOM				0x01

#define FB_REGISTER_ADDR_AMBIENT_LIGHT				0x10

#define FB_REGISTER_ADDR_IR_LEDS_MANUAL_CONTROL		0x20

/* Multi-byte register to which data to be transmitted should be written */
#define FB_REGISTER_ADDR_TX_BUF						0x30
/* Read only register indicating how many bytes of the transmit buffer are available */
#define FB_REGISTER_ADDR_TX_AVAILABLE_COUNT			0x31
/* Select which IR LEDs are used for transmission */
#define FB_REGISTER_ADDR_TX_LED_SELECT				0x33
/* Transmit buffered message with LED flash post-transmission */
#define FB_REGISTER_ADDR_TX_MSG_CONTROL				0x34
#define FB_REGISTER_ADDR_TX_MSG_BUF					0x35

#define FB_REGISTER_ADDR_RX_BUF						0x40
#define FB_REGISTER_ADDR_RX_CONSUMED_COUNT			0x41
#define FB_REGISTER_ADDR_RX_FLUSH					0x42
#define FB_REGISTER_ADDR_RX_ENABLE					0x43

#define FB_REGISTER_ADDR_RX_AMBIENT_BUF				0x60
#define FB_REGISTER_ADDR_RX_AMBIENT_CONSUMED_COUNT	0x61


#define FB_REGISTER_ADDR_SLEEP						0x50

#define FB_REGISTER_ADDR_VERSION_STRING				0xFE
#define FB_REGISTER_ADDR_VERSION					0xFF


bool fb_getVersion(uint8_t faceNum, char *verStr, uint8_t verStrSize);

bool fb_setTopLEDs(uint8_t faceNum, bool redOn, bool greenOn, bool blueOn);
bool fb_getTopLEDs(uint8_t faceNum, bool *redOn, bool *greenOn, bool *blueOn);
bool fb_setBottomLEDs(uint8_t faceNum, bool redOn, bool greenOn, bool blueOn);
bool fb_getBottomLEDs(uint8_t faceNum, bool *redOn, bool *greenOn, bool *blueOn);

int16_t fb_getAmbientLight(uint8_t faceNum);

bool fb_setIRManualLEDs(uint8_t faceNum, bool led1, bool led2, bool led3, bool led4);
bool fb_getIRManualLEDs(uint8_t faceNum, bool *led1, bool *led2, bool *led3, bool *led4);

bool fb_sendToTxBuffer(uint8_t faceNum, uint8_t numBytes, const uint8_t *bytes);
bool fb_queueToTxBuffer(uint8_t faceNum, uint8_t numBytes, const uint8_t *bytes);
bool fb_sendMsgToTxBuffer(uint8_t faceNum, bool flash);
bool fb_getTxBufferAvailableCount(uint8_t faceNum, uint8_t *bytesAvailable);
bool fb_setIRTxLEDs(uint8_t faceNum, bool led1, bool led2, bool led3, bool led4);
bool fb_getIRTxLEDs(uint8_t faceNum, bool *led1, bool *led2, bool *led3, bool *led4);

bool fb_receiveFromRxBuffer(uint8_t faceNum, uint8_t numBytes, uint8_t *bytes);
bool fb_getRxBufferConsumedCount(uint8_t faceNum, uint8_t *bytesConsumed);
bool fb_flushRxBuffer(uint8_t faceNum);
bool fb_setRxEnable(uint8_t faceNum, bool rxEnable);
bool fb_getRxEnable(uint8_t faceNum, bool *rxEnabled);

bool fb_getRxAmbientBufferConsumedCount(uint8_t faceNum, uint8_t *bytesConsumed);
bool fb_getRxAmbientBuffer(uint8_t faceNum, uint8_t numBytes, uint8_t *bytes);

bool fb_sleep(uint8_t faceNum, bool sleepEnabled);

#endif /* FB_H_ */
