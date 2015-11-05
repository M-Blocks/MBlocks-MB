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

uint32_t app_uart_put_string(const char *str);
uint32_t app_uart_put_debug(const char *str, bool debug);
bool delay_ms(uint32_t ms);
uint32_t curr_time();
int hash(const char *str);

#endif /* UTIL_H_ */
