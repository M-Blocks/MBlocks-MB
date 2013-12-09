/*
 * freqcntr.h
 *
 *  Created on: Dec 2, 2013
 *      Author: kwgilpin
 */

#ifndef FREQCNTR_H_
#define FREQCNTR_H_

#include <stdint.h>

void freqcntr_init(void);
void freqcntr_updateFreq(void);
uint32_t freqcntr_getFreq_Hz(void);

#endif /* FREQCNTR_H_ */
