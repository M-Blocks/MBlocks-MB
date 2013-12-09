/*
 * freqcntr.c
 *
 *  Created on: Dec 2, 2013
 *      Author: kwgilpin
 */


#include <stdint.h>
#include <stdbool.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"

#include "app_timer.h"

#include "pins.h"
#include "freqcntr.h"

static bool initialized = false;
static uint32_t freq_Hz = 0;

void freqcntr_init() {
	/* Stop and clear the timer */
	NRF_TIMER1->TASKS_STOP = 1;
	NRF_TIMER1->TASKS_CLEAR = 1;

	/* Disable the TIMER1 interrupt vector */
	NVIC_DisableIRQ(TIMER1_IRQn);

	/* Disable interrupts on all compare channels */
	NRF_TIMER1->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk
			| TIMER_INTENCLR_COMPARE1_Msk
			| TIMER_INTENCLR_COMPARE2_Msk
			| TIMER_INTENCLR_COMPARE3_Msk;

	/* Select counter mode */
	NRF_TIMER1->MODE = TIMER_MODE_MODE_Counter;

	/* 32-bit width so that we minimize risk of overflowing counter. */
	NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

	/* Configure GPIO task/event block to generate an event for every low-to-
	 * high transition of the input signal. */
	nrf_gpiote_event_config(3, BLDCTACHO_PIN_NO, NRF_GPIOTE_POLARITY_LOTOHI);

	/* Connect the rising-edge GPIOTE event to the counter's count task so that
	 * we count the number of rising edges. */

	NRF_PPI->CH[7].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[3];
	NRF_PPI->CH[7].TEP = (uint32_t)&NRF_TIMER1->TASKS_COUNT;

	/* Enable the PPI channel */
	NRF_PPI->CHEN |= (PPI_CHEN_CH7_Enabled << PPI_CHEN_CH7_Pos);

	/* Enable counting */
	NRF_TIMER1->TASKS_START = 1;

	initialized = true;
}


void freqcntr_updateFreq() {
	static uint32_t lastCallRTCTicks = 0;
	uint32_t rtcTicks;
	uint32_t elapsedTicks;

	/* Get the current time expressed as ticks of the 32.768kHz RTC */
	app_timer_cnt_get(&rtcTicks);

	/* Capture the count */
	NRF_TIMER1->TASKS_CAPTURE[0] = 1;

	/* Reset the count as soon as we have saved the old count */
	NRF_TIMER1->TASKS_CLEAR = 1;

	/* The RTC is only a 24-bit counter, so when subtracting the two
	 * tick counts to find the elapsed number of ticks, we must mask out the
	 * 8 most significant bits. */
	elapsedTicks = 0x00FFFFFF & (rtcTicks - lastCallRTCTicks);

	/* The frequency, expressed in Hertz, is the number rising edges
	 * divided by the elapsed time over which those ticks occurred.  The
	 * time is the number of ticks multiplied by the length of each tick, which
	 * is the inverse of the tick frequency. */
	freq_Hz = (NRF_TIMER1->CC[0] * 32768) / elapsedTicks;

	lastCallRTCTicks = rtcTicks;
}

uint32_t freqcntr_getFreq_Hz() {
	return freq_Hz;
}
