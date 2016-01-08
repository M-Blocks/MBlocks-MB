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
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "nrf51_bitfields.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_debug_assert_handler.h"

#include "app_scheduler.h"
#include "app_timer.h"
#include "app_timer_appsh.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_uart.h"

#include "global.h"
#include "pins.h"
#include "util.h"
#include "uart.h"
#include "db.h"
#include "fb.h"
#include "adc.h"
#include "pwm.h"
#include "freqcntr.h"
#include "spi.h"
#include "twi_master_config.h"
#include "twi_master.h"
#include "bldc.h"
#include "a4960.h"
#include "sma.h"
#include "mpu6050.h"
#include "imu.h"
#include "power.h"
#include "led.h"
// #include "commands.h"
// #include "cmdline.h"
// #include "message.h"

#include "bleApp.h"
#include "ble_sps.h"
#include "ble_vns.h"

//#define WAKEUP_BUTTON_PIN               NRF6310_BUTTON_0                            /**< Button used to wake up the application. */
// YOUR_JOB: Define any other buttons to be used by the applications:
// #define MY_BUTTON_PIN                   NRF6310_BUTTON_1


#define BLE_APPEARANCE_CABLE_REPLACEMENT    1800

// YOUR_JOB: Modify these according to requirements.

#define APP_GPIOTE_MAX_USERS            4                                           /**< Maximum number of users of the GPIOTE handler. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                12                                          /**< Maximum number of events in the scheduler queue. */


APP_TIMER_DEF(motionCheckTimerID); 

static void main_timersInit(void);
static void main_schedulerInit(void);
static void main_gpioteInit(void);
static void main_gpioInit(void);
static void main_configUnusedPins(void);

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


/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
void main_timersInit() {
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
}

/**@brief Event Scheduler initialization. */
void main_schedulerInit() {
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Initialize GPIOTE handler module. */
void main_gpioteInit() {
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}

void main_gpioInit() {
    main_configUnusedPins();

    nrf_gpio_pin_clear(BAT1DISCHRG_PIN_NO);
    nrf_gpio_cfg_output(BAT1DISCHRG_PIN_NO);

    nrf_gpio_pin_clear(BAT2DISCHRG_PIN_NO);
    nrf_gpio_cfg_output(BAT2DISCHRG_PIN_NO);

    nrf_gpio_pin_clear(BAT3DISCHRG_PIN_NO);
    nrf_gpio_cfg_output(BAT3DISCHRG_PIN_NO);

    nrf_gpio_pin_clear(BAT4DISCHRG_PIN_NO);
    nrf_gpio_cfg_output(BAT4DISCHRG_PIN_NO);

    /* The S-8243B datasheet states that CTL3 and CTL4 pins should not both be
     * low.  So, we default to setting them high. */
    nrf_gpio_pin_set(LIPROCTL3_PIN_NO);
    nrf_gpio_cfg_output(LIPROCTL3_PIN_NO);

    nrf_gpio_pin_set(LIPROCTL4_PIN_NO);
    nrf_gpio_cfg_output(LIPROCTL4_PIN_NO);

    nrf_gpio_pin_clear(CHRGEN_PIN_NO);
    nrf_gpio_cfg_output(CHRGEN_PIN_NO);

    /* Make the SCK, MOSI, and CS pins outputs with well defined states */
    nrf_gpio_pin_clear(SPI_SCK_PIN_NO);
    nrf_gpio_cfg_output(SPI_SCK_PIN_NO);

    nrf_gpio_pin_clear(SPI_MOSI_PIN_NO);
    nrf_gpio_cfg_output(SPI_MOSI_PIN_NO);

    nrf_gpio_pin_set(SPI_BLDCCS_PIN_NO);
    nrf_gpio_cfg_output(SPI_BLDCCS_PIN_NO);

    /* Make sure the MISO pin has a pull-down so that it does not float and
     * consume extra current. */
    nrf_gpio_cfg_input(SPI_MOSI_PIN_NO, NRF_GPIO_PIN_PULLDOWN);

    /* The BLDCRESETN pin serves as the active-low reset for the A4960 but it
     * also enables the switched battery rail when high. This switched battery
     * rail supplies the A4960 and the SMA controller. */
    nrf_gpio_pin_clear(BLDCRESETN_PIN_NO);
    nrf_gpio_cfg_output(BLDCRESETN_PIN_NO);

    /* Despite plans to use PWM output to control the BLDC motor's speed, we
     * have found that adjusting the current limit (via the A4960's REF pin)
     * accomplishes the same task.  If the PWM is unused, the BLDCSPEED pin
     * must be tied high so that the A4960 does not brake the motor.  That
     * said, the A4960 consume about 50uA additional current when the pin is
     * high, so we keep it low for now. */
    nrf_gpio_pin_clear(BLDCSPEED_PIN_NO);
    nrf_gpio_cfg_output(BLDCSPEED_PIN_NO);

    /* Make the BLDCTACHO pin an input with pull-down.  We will detect and
     * count all rising edges.*/
    nrf_gpio_cfg_input(BLDCTACHO_PIN_NO, NRF_GPIO_PIN_PULLDOWN);

    /* Make the VINSENSE, LIPROVBATOUT, and ICHARGE pins inputs with
     * their input buffers disabled so that they do not consume current when
     * the pins are at a voltage somewhere between 0 and VCC. */
    NRF_GPIO->PIN_CNF[VINSENSE_PIN_NO] =
            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
            (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
            (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[LIPROVBATOUT_PIN_NO] =
            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
            (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
            (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIO->PIN_CNF[ICHARGE_PIN_NO] =
            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
            (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
            (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

    /* Make the TXD pin an output which drives high */
    nrf_gpio_pin_set(UART_TX_PIN_NO);
    nrf_gpio_cfg_output(UART_TX_PIN_NO);

    /* Make the RXD pin an input will a pull-up */
    nrf_gpio_pin_set(UART_RX_PIN_NO);
    nrf_gpio_cfg_input(UART_RX_PIN_NO, NRF_GPIO_PIN_PULLUP);

    /* Set the PWM pins to 0 until they are turned on. */
    nrf_gpio_pin_clear(PRECHRGEN_PIN_NO);
    nrf_gpio_cfg_output(PRECHRGEN_PIN_NO);

    nrf_gpio_pin_clear(BLDCIREF_PIN_NO);
    nrf_gpio_cfg_output(BLDCIREF_PIN_NO);

    /* The SMAPWM is controlled by software-based PWM, but we still need to
     * set its initial state to 0. */
    nrf_gpio_pin_clear(SMAPWM_PIN_NO);
    nrf_gpio_cfg_output(SMAPWM_PIN_NO);

    /* Ensure that the SCL and SDA pins are inputs.  We assume that they do not
     * need to be pulled-up as there should be external pull-up resistors on
     * the I2C bus. */
    nrf_gpio_cfg_input(TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(TWI_MASTER_CONFIG_DATA_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

    /* The IMUINT pin should be configured as an input without pull-up or pull-
     * down as the accelerometer default to making its interrupt pin a push-
     * pull output.*/
    nrf_gpio_cfg_input(IMUINT_PIN_NO, NRF_GPIO_PIN_NOPULL);

    /* Configure the LED pins as outputs */
    nrf_gpio_pin_clear(LED_RED_PIN_NO);
    nrf_gpio_cfg_output(LED_RED_PIN_NO);

    nrf_gpio_pin_clear(LED_GREEN_PIN_NO);
    nrf_gpio_cfg_output(LED_GREEN_PIN_NO);

    nrf_gpio_pin_clear(LED_BLUE_PIN_NO);
    nrf_gpio_cfg_output(LED_BLUE_PIN_NO);
}

void main_configUnusedPins() {
    /* P0.26 is truly unconnected */
    nrf_gpio_pin_clear(26);
    nrf_gpio_cfg_output(26);

    /* P0.03 is truly unconnected.*/
    nrf_gpio_pin_clear(3);
    nrf_gpio_cfg_output(3);

    /* P0.02 is truly unconnected.*/
    nrf_gpio_pin_clear(2);
    nrf_gpio_cfg_output(2);
}


/**@brief Application main function.
 */
int main(void) {

    nrf_delay_ms(10);

    main_schedulerInit();

    /* The timer subsystem must be initialized before we can create timers. */
    main_timersInit();
    main_gpioteInit();
    main_gpioInit();

    led_init();

    for (;;) {
        nrf_delay_ms(200);
        led_setAllOn();
        nrf_delay_ms(200);
        led_setAllOff();
    }
}


/** 
 * @}
 */