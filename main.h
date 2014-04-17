/*
 * main.h
 *
 *  Created on: Apr 9, 2014
 *      Author: kwgilpin
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdbool.h>

void main_setSleepRequested(bool requested);
bool main_setSleepTime(uint32_t time_sec);
uint32_t main_getSleepTime(void);

#endif /* MAIN_H_ */
