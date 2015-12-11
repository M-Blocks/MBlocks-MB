/*
This software is subject to the license described in the license.txt file included with
this software distribution.
You may not use this file except in compliance with this license.

Copyright � Dynastream Innovations Inc. 2015
All rights reserved.
*/

#include <stdint.h>

#include "app_error.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "ant_stack_config.h"

#include "ant_scalable_encrypted_tx.h"


/**@brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{
    uint32_t err_code;

    err_code = softdevice_ant_evt_handler_set(ant_scalable_encrypted_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_handler_init(NRF_CLOCK_LFCLKSRC, NULL, 0, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    uint32_t err_code;

    LEDS_CONFIGURE(LEDS_MASK);

    softdevice_setup();

    // Setup Channel_0 as a TX Master Only.
    ant_scalable_encrypted_channel_tx_broadcast_setup();

    // Main loop.
    for (;;)
    {
        // Put CPU in sleep if possible.
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


