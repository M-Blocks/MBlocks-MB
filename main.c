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
#include "adc.h"
#include "pwm.h"
#include "freqcntr.h"
#include "spi.h"
#include "bldc.h"
#include "a4960.h"
#include "sma.h"
#include "power.h"
#include "led.h"
#include "commands.h"
#include "cmdline.h"

#include "fifo.h"
#include "ble_sps.h"
#include "ble_vns.h"

#define VERSION_STRING					"MBlocks-MB v1.0"

//#define WAKEUP_BUTTON_PIN               NRF6310_BUTTON_0                            /**< Button used to wake up the application. */
// YOUR_JOB: Define any other buttons to be used by the applications:
// #define MY_BUTTON_PIN                   NRF6310_BUTTON_1

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define BLE_APPEARANCE_CABLE_REPLACEMENT	1800

// YOUR_JOB: Modify these according to requirements.
#define SECOND_1_25_MS_UNITS            800                                         /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                         /**< Definition of 1 second, when 1 unit is 10 ms. */
//#define MIN_CONN_INTERVAL             (SECOND_1_25_MS_UNITS / 2)                  /**< Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units. */
#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 100)                /**< Minimum acceptable connection interval (10 ms), Connection interval uses 1.25 ms units. */
//#define MAX_CONN_INTERVAL             (SECOND_1_25_MS_UNITS)                      /**< Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 10)                 /**< Maximum acceptable connection interval (100 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                    /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            4                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

static void advertising_start(void);
static void advertising_stop(void);
static void gap_params_init(void);
static void advertising_init(void);


#if (0)
static app_timer_id_t bldc_tacho_freq_update_timer_id;
#endif


static char deviceName[BLE_GAP_DEVNAME_MAX_LEN + 1] = "BLE SPP 67:6F:C7";

ble_vns_t m_vns;

#define BLE_TX_FIFO_SIZE	384
#define BLE_RX_FIFO_SIZE	64

static fifo_t ble_sps_txFifo;
static uint8_t ble_sps_txFifoData[BLE_TX_FIFO_SIZE];
static fifo_t ble_sps_rxFifo;
static uint8_t ble_sps_rxFifoData[BLE_RX_FIFO_SIZE];

ble_sps_t m_sps;

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

	//APP_UART_FIFO_INIT(&comm_params, 128, 512, on_uart_evt, 3, err_code);
	APP_UART_FIFO_INIT(&comm_params, 128, 128, on_uart_evt, 3, err_code);
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

#if (0)
    err_code = app_timer_create(&bldc_tacho_freq_update_timer_id, APP_TIMER_MODE_REPEATED, bldc_tacho_freq_update_timer_hander);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile) 
 *          parameters of the device. It also sets the permissions and appearance.
 */
void gap_params_init(void) {
    uint32_t err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr;
    ble_gatts_attr_md_t attr_md;
    ble_uuid_t uuid;

    uint16_t service_handle;
    uint16_t char_handle;
    uint16_t desc_handle;

#if (1)
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (uint8_t *)deviceName, strlen(deviceName));
    APP_ERROR_CHECK(err_code);

#if (0)
	/* We now add a descriptor attribute for the user description of the device
	 * name. Here we configure its attribute's metadata to allow open reads but
	 * no writes. */
	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	attr_md.vloc = BLE_GATTS_VLOC_STACK;

	/* Indicate the that attribute is a user description descriptor */
	BLE_UUID_BLE_ASSIGN(uuid, BLE_UUID_DESCRIPTOR_CHAR_USER_DESC);

	/* Configure the value of the attribute. */
	memset(&attr, 0, sizeof(attr));
	attr.p_uuid = &uuid;
	attr.p_attr_md = &attr_md;
	attr.init_len = 11;
	attr.init_offs = 0;
	attr.max_len = 11;
	attr.p_value = (uint8_t *)"Device Name";

    err_code = sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &attr, &desc_handle);
    APP_ERROR_CHECK(err_code);
#endif
#endif

#if (0)
    /* Create the GAP service */
    BLE_UUID_BLE_ASSIGN(uuid, BLE_UUID_GAP);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid, &service_handle);
    APP_ERROR_CHECK(err_code);

    /* We add two characteristics to the GAP serivce: */

	/* The first characteristic is the device name, which should be read-able.
	 * Here, we use the characteristics metadata to make the device name
	 * characteristic readable.  We also add a user description field stating
	 * what the characteristic represents. */
	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.p_char_user_desc = (uint8_t *)"Device Name";
	char_md.char_user_desc_max_size = 11;
	char_md.char_user_desc_size = 11;
	char_md.p_user_desc_md = NULL;
	char_md.p_char_pf = NULL;
	char_md.p_cccd_md = NULL;
	char_md.p_sccd_md = NULL;

	/* The attribute is the part of the characteristic that holds data, in this
	 * case, the device's actual name.  Here we configure its attribute's
	 * metadata to allow open reads but no writes. */
	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	attr_md.vloc = BLE_GATTS_VLOC_STACK;

	/* Set the UUID of the device name attribute correctly */
	BLE_UUID_BLE_ASSIGN(uuid, BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME);

	/* Configure the device name attribute itself */
	memset(&attr, 0, sizeof(attr));
	attr.p_uuid = &uuid;
	attr.p_attr_md = &attr_md;
	attr.init_len = strlen(deviceName);
	attr.init_offs = 0;
	attr.max_len = strlen(deviceName);
	attr.p_value = (uint8_t *)deviceName;

	/* Finally, add the device name characteristic to the GAP service */
	err_code = sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr, &char_handle);
	APP_ERROR_CHECK(err_code);

#endif /* 0 */


    /* YOUR_JOB: Use an appearance value matching the application's use case.*/
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);
    
#if (0)
    /* Add a user description to the appearance characteristic */
    attr.p_value = (uint8_t *)"Appearance";

    err_code = sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &attr, &handle);
    APP_ERROR_CHECK(err_code);
#endif /* 0 */

#if (1)
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
#endif /* 1 */
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    ble_advdata_manuf_data_t manuf_data;
    uint8_t spp_key[3] = {0x00, 0x00, 0x00};
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    // YOUR_JOB: Use UUIDs for service(s) used in your application.
    ble_uuid_t adv_uuids[] = {{SPS_UUID_SERVICE, m_sps.service_uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    //advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt	= sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids	= adv_uuids;
    manuf_data.company_identifier 	= 0xFFFF;
    manuf_data.data.size			= 3;
    manuf_data.data.p_data			= spp_key;
    advdata.p_manuf_specific_data	= &manuf_data;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.name_type				= BLE_ADVDATA_FULL_NAME;
    scanrsp.include_appearance		= true;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize services that will be used by the application.
 */
static void services_init(void)
{
    // YOUR_JOB: Add code to initialize the services used by the application.
	uint32_t err_code;

	ble_vns_init_t vns_init;
	vns_init.version = 0x0100;
	err_code = ble_vns_init(&m_vns, &vns_init);
	APP_ERROR_CHECK(err_code);

	ble_sps_init_t sps_init;
	fifo_init(&ble_sps_txFifo, ble_sps_txFifoData, sizeof(ble_sps_txFifoData));
	fifo_init(&ble_sps_rxFifo, ble_sps_rxFifoData, sizeof(ble_sps_rxFifoData));
	sps_init.p_txFifo = &ble_sps_txFifo;
	sps_init.p_rxFifo = &ble_sps_rxFifo;
	err_code = ble_sps_init(&m_sps, &sps_init);
	APP_ERROR_CHECK(err_code);
}


/**@brief Initialize security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
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

#if(0)
	err_code = app_timer_start(bldc_tacho_freq_update_timer_id, APP_TIMER_TICKS(10, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Start advertising.
 */
void advertising_start(void) {
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    led_setState(LED_BLUE, LED_STATE_ON);
}

void advertising_stop(void) {
	uint32_t err_code;

	err_code = sd_ble_gap_adv_stop();
	APP_ERROR_CHECK(err_code);

	led_setState(LED_BLUE, LED_STATE_OFF);
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code = NRF_SUCCESS;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        	led_setState(LED_BLUE, LED_STATE_SLOW_FLASH);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
        	m_conn_handle = BLE_CONN_HANDLE_INVALID;
        	led_setState(LED_BLUE, LED_STATE_OFF);
            advertising_start();
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT) {
            	led_setState(LED_BLUE, LED_STATE_OFF);

                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                //GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
                //err_code = sd_power_system_off();
            }
            break;

        case BLE_GAP_EVT_RSSI_CHANGED:
        	break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    /* 
    YOUR_JOB: Add service ble_evt handlers calls here, like, for example:
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    */
    ble_vns_on_ble_evt(&m_vns, p_ble_evt);
    ble_sps_on_ble_evt(&m_sps, p_ble_evt);
}


/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    // YOUR_JOB: If the MTU size is changed by the application, the MTU_SIZE parameter to
    //           BLE_STACK_HANDLER_INIT() must be changed accordingly.
    /*
	BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           true);
    */
	BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_1000MS_CALIBRATION,
			BLE_L2CAP_MTU_DEF,
			ble_evt_dispatch,
			true);
}


/**@brief Event Scheduler initialization.
 */
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

/**@brief Initialize GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}



/**@brief Power manager.
 */
static void power_manage(void) {
#if (0)
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
#endif // 0
}


static void on_gpio_evt(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low) {}

static void gpio_init(void) {
    app_gpiote_user_id_t p_user_id;

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


#if 0
    /* Request event messages when either P0.10 (BT_CTS) or P0.11 (BT_SLEEP)
     * transition in either direction (high-to-low or low-to-high). */
    uint32_t err_code = app_gpiote_user_register(&p_user_id, (1<<10 | 1<<11), (1<<10 | 1<<11), on_gpio_evt);
    APP_ERROR_CHECK(err_code);

    err_code = app_gpiote_user_enable(p_user_id);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Application main function.
 */
int main(void) {
	uint8_t c;

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
    sma_init();
    pwm_init();
    bldc_init();
    freqcntr_init();
    spi_init();
    commands_init();

    ble_stack_init();
    scheduler_init();
    gap_params_init();
    services_init();
    conn_params_init();
    sec_params_init();
    advertising_init();

    app_uart_put_string("\r\n");
    app_uart_put_string("\r\n");
    app_uart_put_string(VERSION_STRING);
    app_uart_put_string("\r\n");

    // Start execution
    timers_start();

    advertising_start();

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
