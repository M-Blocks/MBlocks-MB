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
 * @defgroup nrf_dev_timer_example_main main.c
 * @{
 * @ingroup nrf_dev_timer_example
 * @brief Timer Example Application main file.
 *
 * This file contains the source code for a sample application using Timer0.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "bsp.h"


#define TIMER_DELAY_MS           (500UL)             /**< Timer Delay in milli-seconds. */

static void timer_init(void);
static void nrf_timer_delay_ms(uint_fast16_t volatile number_of_ms);

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
    timer_init();

	  // Toggle LEDs.
    while (true)
    {
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
            LEDS_INVERT(1 << leds_list[i]);
            nrf_timer_delay_ms(TIMER_DELAY_MS);
        }
    }
}


/**
 * @brief Function for timer initialization.
 */
static void timer_init()
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }
    NRF_TIMER0->MODE        = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.
    NRF_TIMER0->PRESCALER   = 9;                           // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    NRF_TIMER0->BITMODE     = TIMER_BITMODE_BITMODE_16Bit; // 16 bit mode.
}


/** @brief Function for using the peripheral hardware timers to generate an event after requested number of milliseconds.
 *
 * @param[in] timer Timer to be used for delay, values from @ref p_timer
 * @param[in] number_of_ms Number of milliseconds the timer will count.
 * @note This function will power ON the requested timer, wait until the delay, and then power OFF that timer.
 */
static void nrf_timer_delay_ms(uint_fast16_t volatile number_of_ms)
{
    NRF_TIMER0->TASKS_CLEAR = 1;                           // clear the task first to be usable for later.

    // With 32 us ticks, we need to multiply by 31.25 to get milliseconds.
    NRF_TIMER0->CC[0]       = number_of_ms * 31;
    NRF_TIMER0->CC[0]      += number_of_ms / 4;
    NRF_TIMER0->TASKS_START = 1; // Start timer.

    while (NRF_TIMER0->EVENTS_COMPARE[0] == 0)
    {
        // Do nothing.
    }

    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->TASKS_STOP        = 1; // Stop timer.
}


/** @} */
