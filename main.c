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
#include "app_gpiote.h"
#include "app_button.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_uart.h"

#include "global.h"
#include "pins.h"
#include "util.h"
#include "gitversion.h"
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
#include "imu.h"
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

static bool sleepRequested = false;
static uint32_t sleepTime_sec = 600;
static bool sleeping = false;
static uint32_t lastCharTime_rtcTicks = 0;
static app_timer_id_t motionCheckTimerID = TIMER_NULL;
static bool motionDetected = false;


static void main_timersInit(void);
static void main_timersStart(void);
static void main_schedulerInit(void);
static void main_gpioteInit(void);
static void main_gpioInit(void);
static void main_configUnusedPins(void);

static void main_powerManage(void);
static void main_motionCheckTimerHandler(void *context);

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


/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
void main_timersInit() {
    uint32_t err_code;

	// Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    if (motionCheckTimerID == TIMER_NULL) {
    	err_code = app_timer_create(&motionCheckTimerID, APP_TIMER_MODE_REPEATED, main_motionCheckTimerHandler);
    	APP_ERROR_CHECK(err_code);
    }
}

/**@brief Start timers.
*/
void main_timersStart() {
    uint32_t err_code;
	/* YOUR_JOB: Start your timers. below is an example of how to start a timer.
    
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); */

	/* We have located most timer creation and start function in subroutines
	 * related to the timer's purpose.  Typically, timers are created and
	 * started in *_init() functions and stopped in *_deinit() functions.
	 * This makes it much easier to stop all relevant timers when entering
	 * idle or off mode. */
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
	uint8_t c;
	char str[64];
	uint8_t strSize;
	uint16_t bootCurrent_mA;
	bool mpu6050Initialized, dmpInitialized;
	bool dbAwakeAfterBoot;
	bool ledsOnAfterBoot;
	uint32_t currentTime_rtcTicks;

	nrf_delay_ms(10);

	main_schedulerInit();

	/* The timer subsystem must be initialized before we can create timers. */
    main_timersInit();
    main_gpioteInit();

    bleApp_stackInit();
    main_gpioInit();
    uart_init();

    /* We need to initialize the IMU's DMP before other sub-systems because
     * it takes long enough that multiple timers can expire during its
     * initialization, and if too many timers expire, and too many events are
     * added to the scheduler's queue, the queue will overflow and the nRF will
     * reset. */
    twi_master_init();
    mpu6050Initialized = imu_init(MPU6050_I2C_ADDR);
    dmpInitialized = imu_initDMP();

    led_init();
    pwm_init();
    spi_init();
    power_init();
    commands_init();

    //main_schedulerInit();
    bleApp_gapParamsInit();
    bleApp_servicesInit();
    bleApp_connParamsInit();
    bleApp_secParamsInit();
    bleApp_advertisingInit();

    main_timersStart();

    led_setAllOn();
    ledsOnAfterBoot = true;

    /* Reset the daughterboard by pulling the SCL line low */
    db_reset();
    dbAwakeAfterBoot = true;

    app_uart_put_string("\r\n");
    app_uart_put_string("\r\n");
    app_uart_put_string("MBlocks-MB ");
    app_uart_put_string(gitVersionLongStr);
#ifdef DEBUG
	app_uart_put_string(" (Debug)");
#elif defined(RELEASE)
	app_uart_put_string(" (Release)");
#else
	app_uart_put_string(" (Unknown)");
#endif
    app_uart_put_string("\r\n");

    strSize = sizeof(str);
    if (db_getVersion(str, strSize)) {
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

	if (mpu6050Initialized) {
    	app_uart_put_string("MPU-6050: OK\r\n");
    } else {
    	app_uart_put_string("MPU-6050: Fail\r\n");
    }

    if (dmpInitialized) {
    	app_uart_put_string("DMP: OK\r\n");
    } else {
    	app_uart_put_string("DMP: Fail\r\n");
    }

    bootCurrent_mA = power_getChargeCurrent_mA();
    if (bootCurrent_mA < 20) {
    	snprintf(str, sizeof(str), "Boot current: OK (%u mA)\r\n", bootCurrent_mA);
    } else {
    	snprintf(str, sizeof(str), "Boot current: Fail (%u mA)\r\n", bootCurrent_mA);
    }
    app_uart_put_string(str);
    app_uart_put_string("\r\n");

    imu_enableDMP();

    // Enter main loop
    for (;;) {
        app_sched_execute();

        while (app_uart_get(&c) == NRF_SUCCESS) {
        	cmdline_newChar(c);
        	app_timer_cnt_get(&lastCharTime_rtcTicks);
        }

#if (ENABLE_BLE_COMMANDS == 1)
        while (ble_sps_get_char(&m_sps, &c)) {
        	cmdline_newChar(c);
        	app_timer_cnt_get(&lastCharTime_rtcTicks);
        }
#endif

        /* One second after reboot, we turn off the LEDs and start advertising */
        if (ledsOnAfterBoot && (app_timer_cnt_get(&currentTime_rtcTicks) == NRF_SUCCESS) &&
        		(currentTime_rtcTicks * USEC_PER_APP_TIMER_TICK >= 1000000)) {
        	led_setAllOff();
        	ledsOnAfterBoot = false;

            if (bleApp_isAdvertisingEnabled()) {
            	bleApp_advertisingStart();
            }
        }

        /* Three seconds after reboot, we put the daughterboard and faceboards
         * to sleep. */
        if (dbAwakeAfterBoot && (app_timer_cnt_get(&currentTime_rtcTicks) == NRF_SUCCESS) &&
        		(currentTime_rtcTicks * USEC_PER_APP_TIMER_TICK >= 3000000)) {
        	db_sleep(true);
        	dbAwakeAfterBoot = false;
        	fb_sleep(0, true);
        }

        main_powerManage();
    }
}

/**@brief Power manager. */
void main_powerManage() {
	uint32_t err_code;
	uint32_t currentTime_rtcTicks;
	uint32_t elapsedTime_rtcTicks;

	uint32_t elapsedCharTime_sec;
	bool chargerActive;

	app_timer_cnt_get(&currentTime_rtcTicks);

	elapsedTime_rtcTicks = 0x00FFFFFF & (currentTime_rtcTicks - lastCharTime_rtcTicks);
	elapsedCharTime_sec = (elapsedTime_rtcTicks * USEC_PER_APP_TIMER_TICK) / 1000000;


	if ((power_getChargeState() == POWER_CHARGESTATE_OFF) ||
			(power_getChargeState() == POWER_CHARGESTATE_STANDBY) ||
			(power_getChargeState() == POWER_CHARGESTATE_ERROR)) {
		chargerActive = false;
	} else {
		chargerActive = true;
	}


	if (sleeping && motionDetected) {
		/* If the processor was sleeping, the IMU has detected motion, we must
		 * re-initialize the peripherals that we de-initialized before entering
		 * sleep mode. */

		/* Turn on all LEDs on the mainboard and the daughterboard for 1 second
		 * as an indication that the M-Blocks has exited sleep. */
		db_setLEDs(true, true, true);
		fb_setTopLEDs(0, true, true, true);
		fb_setBottomLEDs(0, true, true, true);

		nrf_gpio_pin_set(LED_RED_PIN_NO);
		nrf_gpio_pin_set(LED_GREEN_PIN_NO);
		nrf_gpio_pin_set(LED_BLUE_PIN_NO);
		nrf_delay_ms(1000);

		/* Turn off all of the LEDs.  The led_init() function will handle this
		 * on the mainboard. */
		db_setLEDs(false, false, false);
		fb_setTopLEDs(0, false, false, false);
		fb_setBottomLEDs(0, false, false, false);

		/* Sleep the daughterboard and the faceboards */
		db_sleep(true);
		fb_sleep(0, true);


		led_init();

		uart_init();
		pwm_init();
		spi_init();
		power_init();
		bldc_init();

		if (motionCheckTimerID != TIMER_NULL) {
			err_code = app_timer_stop(motionCheckTimerID);
			APP_ERROR_CHECK(err_code);
		}

		imu_enableSleepMode();
		imu_enableDMP();

		sleeping = false;

		/* So that the device does not go back to sleep immediately, we pretend
		 * that we just received a character on one of the serial communication
		 * interfaces.  This effectively resets the character timer. */
		app_timer_cnt_get(&lastCharTime_rtcTicks);

		bleApp_setAdvertisingEnabled(true);

		/* Restart charging */
		power_setChargeState(POWER_CHARGESTATE_STANDBY);

		app_uart_put_string("Awoken from sleep\r\n");
	} else if (sleepRequested ||
			((elapsedCharTime_sec > sleepTime_sec) && (sleepTime_sec != 0) && !chargerActive)) {
		/* If we have received a sleep command, or it has been a long time
		 * since we received the last character over one of the serial
		 * interfaces, (and the batteries are not currently charging, we go to
		 * sleep. */

		/* Clear the sleep requested flag so that we do not re-enter sleep
		 * immediately after waking up.*/
		sleepRequested = false;

		if (!sleeping) {
			app_uart_put_string("Going to sleep\r\n");
			nrf_delay_ms(10);

			/* In case the charger was in standby or an error state, we force
			 * it off before going to sleep. */
			power_setChargeState(POWER_CHARGESTATE_OFF);

			/* Turn off power to the SMA controller and BLDC driver */
			power_setVBATSWState(VBATSW_SUPERUSER, false);
			/* Ensure that the daughterboard is in sleep mode */
			db_sleep(true);

			/* Put all faceboards to sleep, too */
			fb_sleep(0, true);

			/* Terminate the BLE connection, if one exists */
			if (m_sps.conn_handle != BLE_CONN_HANDLE_INVALID) {
				err_code = sd_ble_gap_disconnect(m_sps.conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
				APP_ERROR_CHECK(err_code);
			}

			/* Stop advertising */
			bleApp_setAdvertisingEnabled(false);

			/* Disable the DMP by placing the IMU into sleep mode and then
			 * enable low-power motion detection.  We'll use motion to
			 * wake-up from sleep. */
			imu_enableSleepMode();
			imu_enableMotionDetection(true);

			/* Start the timer which we'll use to check whether the IMU has
			 * sensed motion. */
		    if (motionCheckTimerID != TIMER_NULL) {
		    	err_code = app_timer_start(motionCheckTimerID, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
		    	APP_ERROR_CHECK(err_code);
		    }

		    /* Clear the motion detected flag so that we do not wake-up
		     * immediately. */
		    motionDetected = false;

		    led_deinit();
		    uart_deinit();
		    twi_master_deinit();
		    pwm_deinit();
		    spi_deinit();
		    power_deinit();
		    bldc_deinit();
		}

		sleeping = true;
	}

	err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}

void main_motionCheckTimerHandler(void *context) {
	static uint32_t lastLEDFlashTime_rtcTicks = 0;
	bool newMotionDetected;
	uint32_t currentTime_rtcTicks;
	uint32_t elapsedTime_sec;

	app_timer_cnt_get(&currentTime_rtcTicks);
	elapsedTime_sec = ((0x00FFFFFF & (currentTime_rtcTicks - lastLEDFlashTime_rtcTicks)) * USEC_PER_APP_TIMER_TICK) / 1000000;

	if (sleeping && (elapsedTime_sec >= 30)) {
		nrf_gpio_pin_set(LED_RED_PIN_NO);
		lastLEDFlashTime_rtcTicks = currentTime_rtcTicks;
		nrf_delay_ms(25);
	}

	if (imu_checkForMotion(&newMotionDetected) && newMotionDetected) {
		motionDetected = true;
	}

	nrf_gpio_pin_clear(LED_RED_PIN_NO);
}

void main_setSleepRequested(bool requested) {
	sleepRequested = requested;
}

bool main_setSleepTime(uint32_t time_sec) {
	if (time_sec > 3600) {
		return false;
	}

	sleepTime_sec = time_sec;
	return true;
}

uint32_t main_getSleepTime() {
	return sleepTime_sec;
}




/** 
 * @}
 */
