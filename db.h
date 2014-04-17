/*
 * db.h
 *
 *  Created on: Feb 9, 2014
 *      Author: kwgilpin
 */

#ifndef DB_H_
#define DB_H_

#define DB_TWI_ADDR					0x2A

#define DB_LED_CMD					0x62
#define DB_SLEEP_CMD				0x73
#define DB_VERSION_CMD				0x01
#define DB_BRAKE_CMD				0xC3
#define DB_TEMPERATURE_CMD			0x74

#define DB_POLL_FIRST_INTERVAL_MS	5
#define DB_POLL_INTERVAL_MS			5

#define DB_LED_TIMEOUT_MS			50
#define DB_SLEEP_TIMEOUT_MS			50
#define DB_VERSION_TIMEOUT_MS		50
#define DB_BRAKE_TIMEOUT_MS			1000
#define DB_TEMPERATURE_TIMEOUT_MS	50

void db_reset(void);
bool db_sleep(bool sleepEnabled);
bool db_getVersion(char *verStr, uint8_t verStrSize);
bool db_getTemp(int16_t *temperature_tenthDegC);
bool db_setLEDs(bool redOn, bool greenOn, bool blueOn);

#endif /* DB_H_ */
