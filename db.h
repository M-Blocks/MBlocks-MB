/*
 * db.h
 *
 *  Created on: Feb 9, 2014
 *      Author: kwgilpin
 */

#ifndef DB_H_
#define DB_H_

#define DB_TWI_ADDR				0x2A

#define DB_VERSION_CMD			0x01
#define DB_BRAKE_CMD			0xC3

#define DB_POLL_INTERVAL_MS		10

#define DB_VERSION_TIMEOUT_MS	20
#define DB_BRAKE_TIMEOUT_MS		1000

bool db_getVersion(char *verStr, uint8_t verStrSize);

#endif /* DB_H_ */
