/*
 * fifo.c
 *
 *  Created on: Jan 26, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>

#include "fifo.h"

void fifo_init(fifo_t *p_fifo, uint8_t *p_data, fifoSize_t dataSize) {
	p_fifo->p_data = p_data;
	p_fifo->dataSize = dataSize;
	p_fifo->inPtr = 0;
	p_fifo->outPtr = 0;
	p_fifo->empty = true;
	p_fifo->full = false;
}

fifoSize_t fifo_getUsedSpace(const fifo_t *p_fifo) {
	if (p_fifo->empty) {
		return 0;
	} else if (p_fifo->full) {
		return p_fifo->dataSize;
	}

	return (p_fifo->inPtr - p_fifo->outPtr);
}

fifoSize_t fifo_getFreeSpace(const fifo_t *p_fifo) {
	if (p_fifo->empty) {
		return p_fifo->dataSize;
	} else if (p_fifo->full) {
		return 0;
	}

	return (p_fifo->dataSize - fifo_getUsedSpace(p_fifo));
}

bool fifo_push(fifo_t *p_fifo, const uint8_t *data, const fifoSize_t *dataLen) {
	fifoSize_t i;

	if (p_fifo->full || (*dataLen > fifo_getFreeSpace(p_fifo))) {
		return false;
	}

	if (dataLen == 0) {
		return true;
	}

	for (i=0; i<*dataLen; i++) {
		/* Add one byte at a time to the buffer.  At the start of this loop,
		 * the in pointer always indicates the next empty space in the FIFO. */
		p_fifo->p_data[p_fifo->inPtr] = data[i];
		/* Increment the in pointer being sure to wrap around to 0 when we
		 * reach the end of the buffer. */
		p_fifo->inPtr = (p_fifo->inPtr + 1) % p_fifo->dataSize;
	}

	/* We know that the FIFO is not empty given that we have just been adding
	 * data to it. */
	p_fifo->empty = false;

	/* Since we have already verified that there is room available in FIFO for
	 * the number of bytes the caller wishes to add, we do not check whether
	 * the FIFO is full until we have added all bytes. */
	if (p_fifo->inPtr == p_fifo->outPtr) {
		p_fifo->full = true;
	}

	return true;
}


bool fifo_pop(fifo_t *p_fifo, uint8_t *data, fifoSize_t *dataLen) {
	fifoSize_t i;

	if (p_fifo->empty) {
		*dataLen = 0;
		return false;
	}

	if (*dataLen == 0) {
		return true;
	}

	for (i=0; i<*dataLen; i++) {
		/* Take the element from the FIFO pointed to by the out pointer.  At
		 * the beginning of this loop, it is always the oldest element in the
		 * FIFO. */
		data[i] = p_fifo->p_data[p_fifo->outPtr];

		/* Increment the out pointer being sure to wrap around to 0 once it
		 * has reached the end of the buffer. */
		p_fifo->outPtr = (p_fifo->outPtr + 1) % p_fifo->dataSize;

		/* If the out pointer has caught up to the in pointer, it means that
		 * the FIFO is now empty.  As a result, we set the empty flag, update
		 * the value of dataLen pointer so that the caller knows how many bytes
		 * were popped, and we stop trying to extract additional bytes. */
		if (p_fifo->outPtr == p_fifo->inPtr) {
			p_fifo->empty = true;
			*dataLen = i + 1;
			break;
		}
	}

	/* We just removed data, so the FIFO cannot be full. */
	p_fifo->full = false;

	return true;
}


bool fifo_peek(const fifo_t *p_fifo, uint8_t *data, fifoSize_t *dataLen) {
	fifoSize_t i;
	fifoSize_t maxDataLen;
	fifoSize_t peekPtr;

	if (p_fifo->empty) {
		*dataLen = 0;
		return false;
	}

	if (*dataLen == 0) {
		return true;
	}

	maxDataLen = fifo_getUsedSpace(p_fifo);

	/* Since we are not removing data, we do not want to modify the out
	 * pointer, so we create a copy of it called the peek pointer. */
	peekPtr = p_fifo->outPtr;

	/* We continue copying elements from the FIFO to the caller-supplied data
	 * array until either we have copied all of the data that the caller
	 * requested or we have copied all available data. */
	for (i=0; (i<*dataLen) && (i<maxDataLen); i++) {
		/* Take the element from the FIFO pointed to by the peek pointer.*/
		data[i] = p_fifo->p_data[peekPtr];

		/* Increment the peek pointer being sure to wrap around to 0 once it
		 * has reached the end of the buffer. */
		peekPtr = (peekPtr + 1) % p_fifo->dataSize;
	}

	/* Inform the caller how many bytes we actually copied in case it was
	 * less than the requested number of bytes. */
	*dataLen = i;

	return true;
}

bool fifo_discard(fifo_t *p_fifo, fifoSize_t *dataLen) {
	fifoSize_t i;

	if ((p_fifo->empty) || (*dataLen == 0)) {
		/* In case the FIFO was empty but *dataLen was not 0, we set *dataLen
		 * to 0 to inform the caller that, despite returning true, we did not
		 * actually discard anything. */
		*dataLen = 0;
		return true;
	}

	for (i=0; i<*dataLen; i++) {
		/* Increment the out pointer being sure to wrap around to 0 once it
		 * has reached the end of the buffer. */
		p_fifo->outPtr = (p_fifo->outPtr + 1) % p_fifo->dataSize;

		/* If the out pointer has caught up to the in pointer, it means that
		 * the FIFO is now empty.  As a result, we set the empty flag, update
		 * the value of dataLen pointer so that the caller knows how many bytes
		 * were popped, and we stop trying to extract additional bytes. */
		if (p_fifo->outPtr == p_fifo->inPtr) {
			p_fifo->empty = true;
			*dataLen = i + 1;
			break;
		}
	}

	/* We just removed data, so the FIFO cannot be full. */
	p_fifo->full = false;

	return true;
}

bool fifo_purge(fifo_t *p_fifo) {
	p_fifo->inPtr = p_fifo->outPtr = 0;
	p_fifo->empty = true;
	p_fifo->full = false;

	return true;
}
