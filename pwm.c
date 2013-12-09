/*
 * pwm.c
 *
 *  Created on: Nov 18, 2013
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

#include "pins.h"
#include "pwm.h"

#define PWM0_PIN_NO	PRECHRGEN_PIN_NO
#define PWM1_PIN_NO	SMAIREF_PIN_NO
#define PWM2_PIN_NO	BLDCIREF_PIN_NO

/* This is the period, in counts, of the PWM output.  Each individual channel
 * can have an on-time period anywhere between 0 and this number, inclusive.*/
#define PERIOD	512
//***#define PERIOD	1563

static bool initialized = false;

static uint32_t onPeriods[3] = {0, 0, 0};
static uint32_t pwmPinNos[3] = {PWM0_PIN_NO, PWM1_PIN_NO, PWM2_PIN_NO};

void pwm_init() {
	/* Stop and clear the timer */
	NRF_TIMER2->TASKS_STOP = 1;
	NRF_TIMER2->TASKS_CLEAR = 1;

	/* Disable the TIMER2 interrupt vector */
	NVIC_DisableIRQ(TIMER2_IRQn);

	/* Disable interrupts on all compare channels */
	NRF_TIMER2->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk
			| TIMER_INTENCLR_COMPARE1_Msk
			| TIMER_INTENCLR_COMPARE2_Msk
			| TIMER_INTENCLR_COMPARE3_Msk;

	/* Select free-running mode */
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;

	/* HFCLK is divided by 2^(prescalar) */
	NRF_TIMER2->PRESCALER = 0;
	//***NRF_TIMER2->PRESCALER = 9;


	/* 32-bit width, because there is no harm in it given that we will use
	 * a match on CC[0] to reset the counter to 0 anyway. */
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

	/* CC[0] is used to set the period of the PWM */
	NRF_TIMER2->CC[0] = PERIOD;

	/* Un-configure GPIO task/event blocks 0, 1, and 2.  As soon as the
	 * the duty cycle of a PWM channel is set to something other than 0, the
	 * code must configure the corresponding task/event block as a task which
	 * toggles the GPIO pin in response to each incoming event.  Setting the
	 * compare registers to 0 is not sufficient to turn the PWM channels off
	 * because each match toggles the output (as opposed to setting it high
	 * or low explicitly) so the PWM outputs would not behave correctly. */
	nrf_gpiote_unconfig(0);
	nrf_gpiote_unconfig(1);
	nrf_gpiote_unconfig(2);

	/* Set the PWM pins to 0 until they are turned on. */
	nrf_gpio_pin_clear(PWM0_PIN_NO);
	nrf_gpio_pin_dir_set(PWM0_PIN_NO, NRF_GPIO_PIN_DIR_OUTPUT);

	nrf_gpio_pin_clear(PWM1_PIN_NO);
	nrf_gpio_pin_dir_set(PWM1_PIN_NO, NRF_GPIO_PIN_DIR_OUTPUT);

	nrf_gpio_pin_clear(PWM2_PIN_NO);
	nrf_gpio_pin_dir_set(PWM2_PIN_NO, NRF_GPIO_PIN_DIR_OUTPUT);

	/* Compare match on channel 0 is also used to reset the counter to 0. */
	NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
	NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER2->TASKS_CLEAR;

	/* Configure programmable peripheral interconnects to connect TIMER2
	 * compare events to GPIO tasks.  Compare events on channel 0 toggle all
	 * PWM pins.  Compare events on channels 1, 2, and 3 each toggle their
	 * respective PWM output pins.  That is, each output pin is toggled by
	 * both a channel 0 compare event and a channel 1, 2, or 3 compare event.*/
	NRF_PPI->CH[1].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
	NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

	NRF_PPI->CH[2].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[1];
	NRF_PPI->CH[2].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

	NRF_PPI->CH[3].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
	NRF_PPI->CH[3].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];

	NRF_PPI->CH[4].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[2];
	NRF_PPI->CH[4].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];

	NRF_PPI->CH[5].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
	NRF_PPI->CH[5].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[2];

	NRF_PPI->CH[6].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[3];
	NRF_PPI->CH[6].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[2];

	/* Enable PPI channels 0:6 */
	NRF_PPI->CHEN |= (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos) |
			(PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos) |
			(PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos) |
			(PPI_CHEN_CH3_Enabled << PPI_CHEN_CH3_Pos) |
			(PPI_CHEN_CH4_Enabled << PPI_CHEN_CH4_Pos) |
			(PPI_CHEN_CH5_Enabled << PPI_CHEN_CH5_Pos) |
			(PPI_CHEN_CH6_Enabled << PPI_CHEN_CH6_Pos);

	/* Start counting */
	NRF_TIMER2->TASKS_START = 1;

	initialized = true;
}

bool pwm_setOnPeriod(unsigned int channel, uint32_t onPeriod) {
	int i;

	/* Verify that the PWM system has been initialized */
	if (!initialized) {
		return false;
	}

	/* Validate the arguments */
	if (channel > 2) {
		return false;
	}

	if (onPeriod > PERIOD) {
		return false;
	}

	/* Save a copy of the new on-time period */
	onPeriods[channel] = onPeriod;

	/* Stop and then clear the timer while we update the capture registers.
	 * This is necessary because the registers are not double-buffered and the
	 * fact that we cannot read the current value of the counter, so we do not
	 * know whether the output should initially be set or cleared.  Remember,
	 * each match simply toggles the output, so we much get the initial state of
	 * the output set correctly or else the period will be inverted. */
	NRF_TIMER2->TASKS_STOP = 1;
	NRF_TIMER2->TASKS_CLEAR = 1;


	for (i=0; i<=2; i++) {
		/* We iterate over all channels because we must force all channels with
		 * an on-period great than 0 to initially drive their outputs high
		 * given that we just reset the counter to 0.  */
		if (onPeriods[i] == 0) {
			/* If the requested on-period is 0, we disable the GPIOTE channel
			 * which toggles the corresponding output and instead set the
			 * output permanently low. */
			nrf_gpiote_unconfig(i);
			nrf_gpio_pin_clear(pwmPinNos[i]);
		} else if (onPeriods[i] == PERIOD) {
			/* If the requested on-period is equal to the overall waveform
			 * period, we disable the GPIOTE channel which toggles the output
			 * and instead set the output permanently high. */
			nrf_gpiote_unconfig(i);
			nrf_gpio_pin_set(pwmPinNos[i]);
		} else {
			/* If the requested on-period is somewhere in between 0 and the
			 * overall period, we set the corresponding capture/compare
			 * register to hold the period.  We then enable the corresponding
			 * GPIOTE channel to toggle on each match event.  With the counter
			 * at 0, the PWM output will initially be high and will remain so
			 * until the counter matches CC[channel+1] at which point the
			 * output will be toggled low.  Then, when the counter matches
			 * CC[0], the counter will be reset to 0 and the output will again
			 * be toggled high. */
			NRF_TIMER2->CC[i+1] = onPeriods[i];
			nrf_gpiote_task_config(i, pwmPinNos[i], NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_HIGH);
		}
	}

	/* Have re-initialized all PWM channels, we restart the counter. Note that
	 * each time we change any channel's period, we we do cause a glitch on the
	 * other channels.  This is acceptable given that we only intend that these
	 * PWM outputs to be low-pass filtered to generate analog voltage levels.*/
	NRF_TIMER2->TASKS_START = 1;

	return true;
}

uint32_t pwm_getOnPeriod(unsigned int channel) {
	if (!initialized || (channel > 2)) {
		return 0;
	}

	return onPeriods[channel];
}

uint32_t pwm_getPeriod() {
	return PERIOD;
}
