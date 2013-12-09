/*
 * util.h
 *
 *  Created on: Nov 13, 2013
 *      Author: kwgilpin
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>
#include <stdbool.h>

uint32_t app_uart_put_string(char *str);
bool delay_ms(uint32_t ms);

#endif /* UTIL_H_ */
