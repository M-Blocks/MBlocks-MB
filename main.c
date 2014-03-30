/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup from button, advertise, get a connection
 * restart advertising on disconnect and if no new connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified with 'YOUR_JOB' indicates where
 * and how you can customize.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "boards/nrf6310.h"
#include "app_error.h"
#include "app_uart.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_nrf6310_pins.h"
#include "app_scheduler.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "app_timer.h"
#include "ble_debug_assert_handler.h"

#include "global.h"
#include "pins.h"
#include "util.h"
#include "gitversion.h"
#include "db.h"
#include "adc.h"
#include "pwm.h"
#include "freqcntr.h"
#include "spi.h"
#include "twi_master.h"
#include "bldc.h"
#include "a4960.h"
#include "sma.h"
#include "mpu6050.h"
#include "power.h"
#include "led.h"
#include "commands.h"
#include "cmdline.h"

#include "bleApp.h"
#include "ble_sps.h"
#include "ble_vns.h"

//#define WAKEUP_BUTTON_PIN               NRF6310_BUTTON_0                            /**< Button used to wake up the application. */
// YOUR_JOB: Define any other buttons to be used by the applications:
// #define MY_BUTTON_PIN                   NRF6310_BUTTON_1


#define BLE_APPEARANCE_CABLE_REPLACEMENT	1800

// YOUR_JOB: Modify these according to requirements.

#define APP_GPIOTE_MAX_USERS            4                                           /**< Maximum number of users of the GPIOTE handler. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                12                                          /**< Maximum number of events in the scheduler queue. */


#if (0)
static app_timer_id_t bldc_tacho_freq_update_timer_id;
#endif



/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {
	led_setAllOff();
	led_setState(LED_RED, LED_STATE_FAST_FLASH);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief Assert macro callback function.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Service error handler.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the 
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
/*
// YOUR_JOB: Uncomment this function and make it handle error situations sent back to your 
//           application by the services it uses.
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
} */


static void on_uart_evt(app_uart_evt_t *p_app_uart_event) {
	;
}

static void uart_init(void) {
	app_uart_comm_params_t comm_params;
	uint32_t err_code;

	comm_params.rx_pin_no = UART_RX_PIN_NO;
	comm_params.tx_pin_no = UART_TX_PIN_NO;
	comm_params.rts_pin_no = (uint8_t)UART_PIN_DISCONNECTED;
	comm_params.cts_pin_no = (uint8_t)UART_PIN_DISCONNECTED;
	comm_params.flow_control = APP_UART_FLOW_CONTROL_DISABLED;
	comm_params.use_parity = false;
	comm_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200;

	APP_UART_FIFO_INIT(&comm_params, 64, 512, on_uart_evt, 3, err_code);
	//APP_UART_FIFO_INIT(&comm_params, 128, 128, on_uart_evt, 3, err_code);
	APP_ERROR_CHECK(err_code);
}


#if (0)
static void bldc_tacho_freq_update_timer_hander(void *p_context) {
	UNUSED_PARAMETER(p_context);
	freqcntr_updateFreq();
}
#endif

/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	uint32_t err_code;

    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    /* Create a timer which will be used to flash the LEDs */
    err_code = app_timer_create(&led_timer_id, APP_TIMER_MODE_REPEATED, led_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&power_timer_id, APP_TIMER_MODE_REPEATED, power_timerHandler);
    APP_ERROR_CHECK(err_code);

#if (0)
    err_code = app_timer_create(&bldc_tacho_freq_update_timer_id, APP_TIMER_MODE_REPEATED, bldc_tacho_freq_update_timer_hander);
    APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Start timers.
*/
static void timers_start(void) {
	uint32_t err_code;

    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
    
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); */

	/* Start the timer which controls the LEDs */
	err_code = app_timer_start(led_timer_id, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

	/* Start timer which manages battery charging.  It will fire every 200 ms. */
	err_code = app_timer_start(power_timer_id, APP_TIMER_TICKS(200, APP_TIMER_PRESCALER), NULL);

#if(0)
	err_code = app_timer_start(bldc_tacho_freq_update_timer_id, APP_TIMER_TICKS(10, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Event Scheduler initialization. */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/* YOUR_JOB: Uncomment this function if you need to handle button events.
static void button_event_handler(uint8_t pin_no)
{
    switch (pin_no)
    {
        case MY_BUTTON_PIN:
            // Code to handle MY_BUTTON keypresses
            break;
        
        // Handle any other buttons
            
        default:
            APP_ERROR_HANDLER(pin_no);
    }
}
*/

/**@brief Initialize GPIOTE handler module. */
static void gpiote_init(void) {
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}

/**@brief Power manager. */
static void power_manage(void) {
#if (0)
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
#endif // 0
}


static void gpio_init(void) {
    nrf_gpio_pin_clear(BAT1DISCHRG_PIN_NO);
    GPIO_PIN_CONFIG((BAT1DISCHRG_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(BAT2DISCHRG_PIN_NO);
    GPIO_PIN_CONFIG((BAT2DISCHRG_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(BAT3DISCHRG_PIN_NO);
    GPIO_PIN_CONFIG((BAT3DISCHRG_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(BAT4DISCHRG_PIN_NO);
    GPIO_PIN_CONFIG((BAT4DISCHRG_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(LIPROCTL3_PIN_NO);
    GPIO_PIN_CONFIG((LIPROCTL3_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(LIPROCTL4_PIN_NO);
    GPIO_PIN_CONFIG((LIPROCTL4_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(CHRGEN_PIN_NO);
    GPIO_PIN_CONFIG((CHRGEN_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(PRECHRGEN_PIN_NO);
    GPIO_PIN_CONFIG((PRECHRGEN_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(BLDCRESETN_PIN_NO);
    GPIO_PIN_CONFIG((BLDCRESETN_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    /* Despite plans to use PWM output to control the BLDC motor's speed, we
     * have found that adjusting the current limit (via the A4960's REF pin)
     * accomplishes the same task.  If the PWM is unused, it must be tied high
     * so that the A4960 does not brake the motor. */
    nrf_gpio_pin_set(BLDCSPEED_PIN_NO);
    GPIO_PIN_CONFIG((BLDCSPEED_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);


    /* Make the BLDCTACHO pin an input.  We will detect and count all rising edges */
    GPIO_PIN_CONFIG((BLDCTACHO_PIN_NO),
    		GPIO_PIN_CNF_DIR_Input,
    		GPIO_PIN_CNF_INPUT_Connect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);

    nrf_gpio_pin_clear(SMAEN_PIN_NO);
    GPIO_PIN_CONFIG((SMAEN_PIN_NO),
    		GPIO_PIN_CNF_DIR_Output,
    		GPIO_PIN_CNF_INPUT_Disconnect,
    		GPIO_PIN_CNF_PULL_Disabled,
    		GPIO_PIN_CNF_DRIVE_S0S1,
    		GPIO_PIN_CNF_SENSE_Disabled);
}


/**@brief Application main function.
 */
int main(void) {
	uint8_t c;
	char str[64];
	uint8_t strSize;
	uint16_t bootCurrent_mA;

	/* Start 16 MHz crystal oscillator */
	NRF_CLOCK ->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK ->TASKS_HFCLKSTART = 1;

	/* Wait for the external oscillator to start up */
	while (NRF_CLOCK ->EVENTS_HFCLKSTARTED == 0);

    // Initialize
    led_init();

    timers_init();
    gpiote_init();
    //buttons_init();

    gpio_init();
    uart_init();
    twi_master_init();
    sma_init();
    pwm_init();
    freqcntr_init();
    spi_init();
    commands_init();

    bleApp_stackInit();
    scheduler_init();
    bleApp_gapParamsInit();
    bleApp_servicesInit();
    bleApp_connParamsInit();
    bleApp_secParamsInit();
    bleApp_advertisingInit();

    // Start execution
    timers_start();

    /* Reset the daughterboard by pulling the SCL line low for at least 2ms */
    db_reset();

    app_uart_put_string("\r\n");
    app_uart_put_string("\r\n");
    app_uart_put_string("MBlocks-MB ");
    app_uart_put_string(gitVersionStr);
    app_uart_put_string("\r\n");

    strSize = sizeof(str);
    if (db_getVersion(str, &strSize)) {
    	app_uart_put_string("DB: OK (");
    	app_uart_put_string(str);
    	app_uart_put_string(")\r\n");
    } else {
    	app_uart_put_string("DB: Fail\r\n");
    }

    if (bldc_init()) {
    	app_uart_put_string("A4960: OK\r\n");
    } else {
    	app_uart_put_string("A4960: Fail\r\n");
    }

    if (mpu6050_init(MPU6050_I2C_ADDR)) {
    	app_uart_put_string("MPU-6050: OK\r\n");
    } else {
    	app_uart_put_string("MPU-6050: Fail\r\n");
    }

    bootCurrent_mA = power_getChargeCurrent_mA();
    if (bootCurrent_mA < 20) {
    	snprintf(str, sizeof(str), "Boot current: OK (%u mA)\r\n", bootCurrent_mA);
    } else {
    	snprintf(str, sizeof(str), "Boot current: Fail (%u mA)\r\n", bootCurrent_mA);
    }
    app_uart_put_string(str);

    app_uart_put_string("\r\n");

    if (bleApp_isAdvertisingEnabled()) {
    	//bleApp_advertisingStart();
    }

    // Enter main loop
    for (;;) {
        app_sched_execute();

        while (app_uart_get(&c) == NRF_SUCCESS) {
        	cmdline_newChar(c);
        }

#if (ENABLE_BLE_COMMANDS == 1)
        while (ble_sps_get_char(&m_sps, &c)) {
        	cmdline_newChar(c);
        }
#endif

        power_manage();
    }
}


/** 
 * @}
 */
