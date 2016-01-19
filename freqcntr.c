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
#include "nrf_assert.h"
#include "nrf_sdm.h"

#include "app_timer.h"
#include "app_error.h"

#include "global.h"
#include "pins.h"
#include "freqcntr.h"

static bool initialized = false;
static uint32_t lastUpdateTime_rtcTicks = 0;
static uint32_t freq_Hz = 0;

void freqcntr_init() {
	uint8_t softdevice_enabled;
	uint32_t err_code;

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

	NVIC_ClearPendingIRQ(TIMER1_IRQn);

	/* Select counter mode */
	NRF_TIMER1->MODE = (TIMER_MODE_MODE_Counter << TIMER_MODE_MODE_Pos);

	/* 32-bit width so that we minimize risk of overflowing counter. */
	NRF_TIMER1->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);

	/* Make the BLDCTACHO pin an input will a pull-down resistor to keep it
	 * from floating unpredictably. */
	nrf_gpio_cfg_input(BLDCTACHO_PIN_NO, NRF_GPIO_PIN_PULLDOWN);

	/* Configure GPIO task/event block to generate an event for every low-to-
	 * high transition of the input signal. */
	nrf_gpiote_event_config(FREQCNTR_GPIOTE_CHANNEL, BLDCTACHO_PIN_NO, NRF_GPIOTE_POLARITY_LOTOHI);

	/* Whether or not the soft device is enabled determines how we configure
	 * the PPI block. */
	err_code = sd_softdevice_is_enabled(&softdevice_enabled);
	APP_ERROR_CHECK(err_code);

	/* Connect the rising-edge GPIOTE event to the counter's count task so that
	 * we count the number of rising edges and then enable the PPI channel. */
	if (softdevice_enabled) {
		err_code = sd_ppi_channel_assign(FREQCNTR_PPI_CHANNEL,
				&(NRF_GPIOTE->EVENTS_IN[FREQCNTR_GPIOTE_CHANNEL]),
				&(NRF_TIMER1->TASKS_COUNT));
		APP_ERROR_CHECK(err_code);

		err_code = sd_ppi_channel_enable_set(FREQCNTR_PPI_CHENSET_MASK);
    	APP_ERROR_CHECK(err_code);
	} else {
		NRF_PPI->CH[FREQCNTR_PPI_CHANNEL].EEP = (uint32_t)&(NRF_GPIOTE->EVENTS_IN[FREQCNTR_GPIOTE_CHANNEL]);
		NRF_PPI->CH[FREQCNTR_PPI_CHANNEL].TEP = (uint32_t)&(NRF_TIMER1->TASKS_COUNT);
		NRF_PPI->CHENSET = FREQCNTR_PPI_CHENSET_MASK;
	}

	/* Enable counting */
	NRF_TIMER1->TASKS_START = 1;

	/* Save the current app_timer tick count so that the first time that the
	 * freqcntr_updateFreq() function is called it can compute a valid
	 * frequency.  Otherwise, if lastUpdateTime_rtcTicks was 0, the first
	 * frequency would be artificially low. */
	app_timer_cnt_get(&lastUpdateTime_rtcTicks);

	initialized = true;
}

void freqcntr_deinit() {
	uint8_t softdevice_enabled;
	uint32_t err_code;

	if (!initialized) {
		return;
	}

	/* Disable counting */
	NRF_TIMER1->TASKS_STOP = 1;

	/* Disable the GPIO task/event block that we were using to generate an
	 * event for every low-to-high transition of the input signal. */
	nrf_gpiote_unconfig(FREQCNTR_GPIOTE_CHANNEL);

	/* Whether or not the soft device is enabled determines how we configure
	 * the PPI block. */
	err_code = sd_softdevice_is_enabled(&softdevice_enabled);
	APP_ERROR_CHECK(err_code);

	/* Disable the PPI channel which routes rising edge events on the GPIO line
	 * to the timer's count task. */
	if (softdevice_enabled) {
		err_code = sd_ppi_channel_enable_clr(FREQCNTR_PPI_CHEN_MASK);
    	APP_ERROR_CHECK(err_code);
	} else {
		NRF_PPI->CHENCLR = FREQCNTR_PPI_CHENCLR_MASK;
	}

	freq_Hz = 0;
	initialized = false;
}


void freqcntr_updateFreq() {
	uint32_t currentTime_rtcTicks;
	uint32_t elapsedTicks, elapsedTime_us;

	if (!initialized) {
		freq_Hz = 0;
		return;
	}

	/* Get the current time expressed as ticks of the 32.768kHz RTC */
	app_timer_cnt_get(&currentTime_rtcTicks);

	/* Capture the count */
	NRF_TIMER1->TASKS_CAPTURE[0] = 1;

	/* Reset the count as soon as we have saved the old count */
	NRF_TIMER1->TASKS_CLEAR = 1;

	/* The RTC is only a 24-bit counter, so when subtracting the two
	 * tick counts to find the elapsed number of ticks, we must mask out the
	 * 8 most significant bits. */
	elapsedTicks = 0x00FFFFFF & (currentTime_rtcTicks - lastUpdateTime_rtcTicks);
	elapsedTime_us = elapsedTicks * USEC_PER_APP_TIMER_TICK;

	/* The frequency, expressed in Hertz, is the number rising edges divided by
	 * the elapsed time (in seconds) over which those ticks occurred. */
	freq_Hz = (NRF_TIMER1->CC[0] * 1000000) / elapsedTime_us;

	lastUpdateTime_rtcTicks = currentTime_rtcTicks;
}

uint32_t freqcntr_getFreq_Hz() {
	return freq_Hz;
}
