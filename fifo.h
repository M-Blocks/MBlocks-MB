/*
 * fifo.h
 *
 *  Created on: Jan 26, 2014
 *      Author: kwgilpin
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>
#include <stdbool.h>

typedef uint16_t fifoSize_t;

typedef struct {
	uint8_t *p_data;
	fifoSize_t dataSize;
	fifoSize_t inPtr;
	fifoSize_t outPtr;
	bool empty;
	bool full;
} fifo_t;

void fifo_init(fifo_t *p_fifo, uint8_t *p_data, fifoSize_t dataSize);
fifoSize_t fifo_getUsedSpace(const fifo_t *p_fifo);
fifoSize_t fifo_getFreeSpace(const fifo_t *p_fifo);
bool fifo_push(fifo_t *p_fifo, const uint8_t *data, const fifoSize_t *dataLen);
bool fifo_pop(fifo_t *p_fifo, uint8_t *data, fifoSize_t *dataLen);
bool fifo_peek(const fifo_t *p_fifo, uint8_t *data, fifoSize_t *dataLen);
bool fifo_discard(fifo_t *p_fifo, fifoSize_t *dataLen);
bool fifo_purge(fifo_t *p_fifo);

#endif /* FIFO_H_ */
