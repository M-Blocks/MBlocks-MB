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
#include "nrf_sdm.h"
#include "nrf_assert.h"

#include "app_util.h"

#include "global.h"
#include "pins.h"
#include "pwm.h"

#define CHANNEL_COUNT	2

#define PWM0_PIN_NO		PRECHRGEN_PIN_NO
#define PWM1_PIN_NO		BLDCIREF_PIN_NO

/* This is the period, in counts, of the PWM output.  Each individual channel
 * can have an on-time period anywhere between 0 and this number, inclusive.*/
#define PERIOD	512

static bool initialized = false;

static uint32_t onPeriods[CHANNEL_COUNT] = {0, 0};
static uint32_t pwmPinNos[CHANNEL_COUNT] = {PWM0_PIN_NO, PWM1_PIN_NO};
static uint32_t pwmGPIOTEChnls[CHANNEL_COUNT] = {PWM_CH0_GPIOTE_CHANNEL, PWM_CH1_GPIOTE_CHANNEL};

void pwm_init() {
	uint32_t err_code;
	uint8_t softdevice_enabled;

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

	NVIC_ClearPendingIRQ(TIMER2_IRQn);

	/* Select free-running mode */
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;

	/* HFCLK is divided by 2^(prescalar) */
	NRF_TIMER2->PRESCALER = 0;

	/* 32-bit width, because there is no harm in it given that we will use
	 * a match on CC[0] to reset the counter to 0 anyway. */
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

	/* CC[0] is used to set the period of the PWM */
	NRF_TIMER2->CC[0] = PERIOD;

	/* Un-configure GPIO task/event blocks 0 and 1.  As soon as the
	 * the duty cycle of a PWM channel is set to something other than 0, the
	 * code must configure the corresponding task/event block as a task which
	 * toggles the GPIO pin in response to each incoming event.  Setting the
	 * compare registers to 0 is not sufficient to turn the PWM channels off
	 * because each match toggles the output (as opposed to setting it high
	 * or low explicitly) so the PWM outputs would not behave correctly. */
	nrf_gpiote_unconfig(PWM_CH0_GPIOTE_CHANNEL);
	nrf_gpiote_unconfig(PWM_CH1_GPIOTE_CHANNEL);

	/* Set the PWM pins to 0 until they are turned on. */
	nrf_gpio_pin_clear(PWM0_PIN_NO);
    GPIO_PIN_CONFIG((PWM0_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

	nrf_gpio_pin_clear(PWM1_PIN_NO);
    GPIO_PIN_CONFIG((PWM1_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

	/* Compare match on channel 0 is used to reset the counter to 0. */
	NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);

	/* Whether or not the soft device is enabled determines how we configure
	 * the PPI block. */
	err_code = sd_softdevice_is_enabled(&softdevice_enabled);
	ASSERT(err_code == NRF_SUCCESS);

	/* Configure programmable peripheral interconnects to connect TIMER2
	 * compare events to GPIO tasks.  Compare events on channel 0 toggle all
	 * PWM pins.  Compare events on channels 1 and 2 each toggle their
	 * respective PWM output pins.  That is, each output pin is toggled by
	 * both a channel 0 compare event and a channel 1 or 2 compare event.*/
	if (softdevice_enabled) {
		err_code = sd_ppi_channel_assign(PWM_CH0_RESET_PPI_CHANNEL,
				&(NRF_TIMER2->EVENTS_COMPARE[0]),
				&(NRF_GPIOTE->TASKS_OUT[PWM_CH0_GPIOTE_CHANNEL]));
		ASSERT(err_code == NRF_SUCCESS);

		err_code = sd_ppi_channel_assign(PWM_CH0_MATCH_PPI_CHANNEL,
				&(NRF_TIMER2->EVENTS_COMPARE[1]),
				&(NRF_GPIOTE->TASKS_OUT[PWM_CH0_GPIOTE_CHANNEL]));
		ASSERT(err_code == NRF_SUCCESS);

		err_code = sd_ppi_channel_assign(PWM_CH1_RESET_PPI_CHANNEL,
				&(NRF_TIMER2->EVENTS_COMPARE[0]),
				&(NRF_GPIOTE->TASKS_OUT[PWM_CH1_GPIOTE_CHANNEL]));
		ASSERT(err_code == NRF_SUCCESS);

		err_code = sd_ppi_channel_assign(PWM_CH1_MATCH_PPI_CHANNEL,
				&(NRF_TIMER2->EVENTS_COMPARE[2]),
				&(NRF_GPIOTE->TASKS_OUT[PWM_CH1_GPIOTE_CHANNEL]));
		ASSERT(err_code == NRF_SUCCESS);

		/* Enable PPI channels */
		err_code = sd_ppi_channel_enable_set(
				PWM_CH0_RESET_PPI_CHEN_MASK | PWM_CH0_MATCH_PPI_CHEN_MASK |
				PWM_CH1_RESET_PPI_CHEN_MASK | PWM_CH1_MATCH_PPI_CHEN_MASK );
		ASSERT(err_code == NRF_SUCCESS);
	} else {
		NRF_PPI->CH[PWM_CH0_RESET_PPI_CHANNEL].EEP = (uint32_t)&(NRF_TIMER2->EVENTS_COMPARE[0]);
		NRF_PPI->CH[PWM_CH0_RESET_PPI_CHANNEL].TEP = (uint32_t)&(NRF_GPIOTE->TASKS_OUT[PWM_CH0_GPIOTE_CHANNEL]);

		NRF_PPI->CH[PWM_CH0_MATCH_PPI_CHANNEL].EEP = (uint32_t)&(NRF_TIMER2->EVENTS_COMPARE[1]);
		NRF_PPI->CH[PWM_CH0_MATCH_PPI_CHANNEL].TEP = (uint32_t)&(NRF_GPIOTE->TASKS_OUT[PWM_CH0_GPIOTE_CHANNEL]);

		NRF_PPI->CH[PWM_CH1_RESET_PPI_CHANNEL].EEP = (uint32_t)&(NRF_TIMER2->EVENTS_COMPARE[0]);
		NRF_PPI->CH[PWM_CH1_RESET_PPI_CHANNEL].TEP = (uint32_t)&(NRF_GPIOTE->TASKS_OUT[PWM_CH1_GPIOTE_CHANNEL]);

		NRF_PPI->CH[PWM_CH1_MATCH_PPI_CHANNEL].EEP = (uint32_t)&(NRF_TIMER2->EVENTS_COMPARE[2]);
		NRF_PPI->CH[PWM_CH1_MATCH_PPI_CHANNEL].TEP = (uint32_t)&(NRF_GPIOTE->TASKS_OUT[PWM_CH1_GPIOTE_CHANNEL]);

		/* Enable PPI channels */
		NRF_PPI->CHENSET =
				PWM_CH0_RESET_PPI_CHENSET_MASK | PWM_CH0_MATCH_PPI_CHENSET_MASK |
				PWM_CH1_RESET_PPI_CHENSET_MASK | PWM_CH1_MATCH_PPI_CHENSET_MASK;
	}

	/* Note, we do not start the timer counting here.  There is no point given
	 * that both output channels initially have an on period of 0. */

	initialized = true;
}

void pwm_deinit() {
	uint32_t err_code;
	uint8_t softdevice_enabled;
	uint8_t i;

	if (!initialized) {
		return;
	}

	NRF_TIMER2->TASKS_STOP = 1;

	/* Whether or not the soft device is enabled determines how we configure
	 * the PPI block. */
	err_code = sd_softdevice_is_enabled(&softdevice_enabled);
	ASSERT(err_code == NRF_SUCCESS);

	/* Disable the PPI channels used for PWM */
	if (softdevice_enabled) {
		err_code = sd_ppi_channel_enable_clr(
				PWM_CH0_RESET_PPI_CHEN_MASK | PWM_CH0_MATCH_PPI_CHEN_MASK |
				PWM_CH1_RESET_PPI_CHEN_MASK | PWM_CH1_MATCH_PPI_CHEN_MASK );
		ASSERT(err_code == NRF_SUCCESS);
	} else {
		NRF_PPI->CHENCLR =
				PWM_CH0_RESET_PPI_CHENCLR_MASK | PWM_CH0_MATCH_PPI_CHENCLR_MASK |
				PWM_CH1_RESET_PPI_CHENCLR_MASK | PWM_CH1_MATCH_PPI_CHENCLR_MASK;
	}


	/* Disable the GPIOTE connections to all PWM channels and set the outputs
	 * to zero. */
	for (i=0; i<CHANNEL_COUNT; i++) {
		nrf_gpiote_unconfig(pwmGPIOTEChnls[i]);
		nrf_gpio_pin_clear(pwmPinNos[i]);
		onPeriods[i] = 0;
	}

	initialized = false;
}


bool pwm_setOnPeriod(unsigned int channel, uint32_t onPeriod) {
	int i;
	bool timerNeeded = false;

	/* Verify that the PWM system has been initialized */
	if (!initialized) {
		return false;
	}

	/* Validate the arguments */
	if (channel >= CHANNEL_COUNT) {
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

	for (i=0; i<CHANNEL_COUNT; i++) {
		/* We iterate over all channels because we must force all channels with
		 * an on-period great than 0 to initially drive their outputs high
		 * given that we just reset the counter to 0.  */
		if (onPeriods[i] == 0) {
			/* If the requested on-period is 0, we disable the GPIOTE channel
			 * which toggles the corresponding output and instead set the
			 * output permanently low. */
			nrf_gpiote_unconfig(pwmGPIOTEChnls[i]);
			nrf_gpio_pin_clear(pwmPinNos[i]);
		} else if (onPeriods[i] == PERIOD) {
			/* If the requested on-period is equal to the overall waveform
			 * period, we disable the GPIOTE channel which toggles the output
			 * and instead set the output permanently high. */
			nrf_gpiote_unconfig(pwmGPIOTEChnls[i]);
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
			nrf_gpiote_task_config(pwmGPIOTEChnls[i], pwmPinNos[i],
					NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_HIGH);
			/* If one of the channels has a period between 0 and 100%, we
			 * actually need to start the timer.  Otherwise, the timer is just
			 * burning extra current, so we'll keep it off. */
			timerNeeded = true;
		}
	}

	/* Have re-initialized all PWM channels, we restart the counter. Note that
	 * each time we change any channel's period, we do cause a glitch on the
	 * other channels.  This is acceptable given that we only intend that these
	 * PWM outputs to be low-pass filtered to generate analog voltage levels.
	 * Note that we only actually start the timer if one of the channels has
	 * a period somewhere between, but not including 0 and 100%. */
	if (timerNeeded) {
		NRF_TIMER2->TASKS_START = 1;
	}

	return true;
}

uint32_t pwm_getOnPeriod(unsigned int channel) {
	if (!initialized || (channel > 1)) {
		return 0;
	}

	return onPeriods[channel];
}

uint32_t pwm_getPeriod() {
	if (!initialized) {
		return 0;
	}

	return PERIOD;
}
