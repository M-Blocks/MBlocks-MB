/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup rtc_example_main main.c
 * @{
 * @ingroup rtc_example
 * @brief Real Time Counter Example Application main file.
 *
 * This file contains the source code for a sample application using the Real Time Counter (RTC).
 * 
 */

#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "bsp.h"

#define LFCLK_FREQUENCY           (32768UL)                                 /**< LFCLK frequency in Hertz, constant. */
#define RTC_FREQUENCY             (8UL)                                     /**< Required RTC working clock RTC_FREQUENCY Hertz. Changable. */
#define COMPARE_COUNTERTIME       (3UL)                                     /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define COUNTER_PRESCALER         ((LFCLK_FREQUENCY / RTC_FREQUENCY) - 1)   /* f = LFCLK/(prescaler + 1) */

#ifdef BSP_LED_0
    #define TICK_EVENT_OUTPUT     BSP_LED_0                                 /**< Pin number for indicating tick event. */
#endif
#ifndef TICK_EVENT_OUTPUT
    #error "Please indicate output pin"
#endif
#ifdef BSP_LED_1
    #define COMPARE_EVENT_OUTPUT   BSP_LED_1                                /**< Pin number for indicating compare event. */
#endif
#ifndef COMPARE_EVENT_OUTPUT
    #error "Please indicate output pin"
#endif


/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}


/** @brief Function for configuring the RTC with TICK to 100Hz and COMPARE0 to 10 sec.
 */
static void rtc_config(void)
{
    NVIC_EnableIRQ(RTC0_IRQn);                                  // Enable Interrupt for the RTC in the core.
    NRF_RTC0->PRESCALER = COUNTER_PRESCALER;                    // Set prescaler to a TICK of RTC_FREQUENCY.
    NRF_RTC0->CC[0]     = COMPARE_COUNTERTIME * RTC_FREQUENCY;  // Compare0 after approx COMPARE_COUNTERTIME seconds.

    // Enable TICK event and TICK interrupt:
    NRF_RTC0->EVTENSET = RTC_EVTENSET_TICK_Msk;
    NRF_RTC0->INTENSET = RTC_INTENSET_TICK_Msk;

    // Enable COMPARE0 event and COMPARE0 interrupt:
    NRF_RTC0->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
    NRF_RTC0->INTENSET = RTC_INTENSET_COMPARE0_Msk;
}


/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
void RTC0_IRQHandler()
{
    if ((NRF_RTC0->EVENTS_TICK != 0) &&
        ((NRF_RTC0->INTENSET & RTC_INTENSET_TICK_Msk) != 0))
    {
        NRF_RTC0->EVENTS_TICK = 0;
        nrf_gpio_pin_toggle(TICK_EVENT_OUTPUT);
    }

    if ((NRF_RTC0->EVENTS_COMPARE[0] != 0) &&
        ((NRF_RTC0->INTENSET & RTC_INTENSET_COMPARE0_Msk) != 0))
    {
        NRF_RTC0->EVENTS_COMPARE[0] = 0;
        nrf_gpio_pin_toggle(COMPARE_EVENT_OUTPUT);
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    LEDS_CONFIGURE(((1<<COMPARE_EVENT_OUTPUT) | (1<<TICK_EVENT_OUTPUT)));
    LEDS_OFF((1<<COMPARE_EVENT_OUTPUT) | (1<<TICK_EVENT_OUTPUT));

    lfclk_config();
    rtc_config();

    NRF_RTC0->TASKS_START = 1;

    while (true)
    {
        // Do nothing.
    }
}


/**  @} */
