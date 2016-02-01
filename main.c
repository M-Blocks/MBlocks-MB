/**@file main.c
 * @brief Main application entry point.
 *
 * @author Sebastian Claici
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_gpio.h"

#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"

#include "app_scheduler.h"
#include "app_util.h"
#include "app_error.h"
#include "app_gpiote.h"
#include "app_timer.h"

#include "global.h"
#include "fifo.h"
#include "ble_sps.h"
#include "ble_vns.h"
#include "pins.h"
#include "led.h"

#include "SEGGER_RTT.h"


#define APP_GPIOTE_MAX_USERS 4

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

#define DEVICE_NAME			"BLE SPP 67:6F:C7"
#define APP_ADV_INTERVAL		40
#define APP_ADV_TIMEOUT_IN_SECONDS	180

#define APP_TIMER_MAX_TIMERS    8
#define APP_TIMER_OP_QUEUE_SIZE 2

#define MIN_CONN_INTERVAL	MSEC_TO_UNITS(500, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL	MSEC_TO_UNITS(1000, UNIT_1_25_MS)
#define SLAVE_LATENCY		0
#define CONN_SUP_TIMEOUT	MSEC_TO_UNITS(4000, UNIT_10_MS)

#define FIRST_CONN_PARAMS_UPDATE_DELAY	APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
#define NEXT_CONN_PARAMS_UPDATE_DELAY	APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
#define MAX_CONN_PARAMS_UPDATE_COUNT	3

#define SEC_PARAM_TIMEOUT               30
#define SEC_PARAM_IO_CAPABILITIES	BLE_GAP_IO_CAPS_NONE
#define SEC_PARAM_BOND			1
#define SEC_PARAM_MITM			0
#define SEC_PARAM_OOB			0
#define SEC_PARAM_MIN_KEY_SIZE		7
#define SEC_PARAM_MAX_KEY_SIZE		16

#define BLE_TX_FIFO_SIZE	512
#define BLE_RX_FIFO_SIZE	64

#define DEAD_BEEF 0xDEADBEEF

static ble_gap_sec_params_t	m_sec_params;
static ble_gap_adv_params_t	m_adv_params;

static ble_sps_t m_sps;
static ble_vns_t m_vns;

static fifo_t ble_sps_txFifo;
static uint8_t ble_sps_txFifoData[BLE_TX_FIFO_SIZE];
static fifo_t ble_sps_rxFifo;
static uint8_t ble_sps_rxFifoData[BLE_RX_FIFO_SIZE];


/**@brief Function for the GAP initialization
 */
static void gap_params_init(void) {
    uint32_t			err_code;
    ble_gap_conn_params_t	gap_conn_params;
    ble_gap_conn_sec_mode_t	sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
					  (const uint8_t *)DEVICE_NAME,
					  strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality
 */
static void advertising_init(void) {
    uint32_t			err_code;
    ble_advdata_t		advdata;
    ble_advdata_t		scanrsp;
    ble_advdata_manuf_data_t	manufdata;

    uint8_t spp_key[3] = {0x00, 0x00, 0x00};
    uint8_t	flags  = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] =
    {
	{SPS_UUID_SERVICE, m_sps.service_uuid_type}
    };

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type		    = BLE_ADVDATA_NO_NAME;
    advdata.flags.size		    = sizeof(flags);
    advdata.flags.p_data	    = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
    manufdata.company_identifier    = 0xFFFF;
    manufdata.data.size		    = 3;
    manufdata.data.p_data	    = spp_key;
    advdata.p_manuf_specific_data   = &manufdata;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.name_type	       = BLE_ADVDATA_FULL_NAME;
    scanrsp.include_appearance = true;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);

    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.type	     = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp	     = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}


static void services_init(void) {
    uint32_t		err_code;
    ble_vns_init_t	vns_init;
    ble_sps_init_t	sps_init;

    vns_init.version = 0x0100;
    err_code = ble_vns_init(&m_vns, &vns_init);
    APP_ERROR_CHECK(err_code);

    fifo_init(&ble_sps_txFifo, ble_sps_txFifoData, sizeof(ble_sps_txFifoData));
    fifo_init(&ble_sps_rxFifo, ble_sps_rxFifoData, sizeof(ble_sps_rxFifoData));
    sps_init.p_txFifo = &ble_sps_txFifo;
    sps_init.p_rxFifo = &ble_sps_rxFifo;
    
    err_code = ble_sps_init(&m_sps, &sps_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the security parameters
 */
static void sec_params_init(void) {
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.oob	      = SEC_PARAM_OOB;
    m_sec_params.mitm	      = SEC_PARAM_MITM;
    m_sec_params.bond	      = SEC_PARAM_BOND;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Function for starting the application timers
 */
static void application_timers_start(void) {
    led_timers_start();
}


/**@brief Function for starting advertising
 */
static void advertising_start(void) {
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    led_set_state(LED_BLUE, LED_STATE_SLOW_FLASH);
}


/**@brief Function for handling a connection parameters error.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the connection parameters module.
 */
static void conn_params_init(void) {
    uint32_t               err_code;
    ble_conn_params_init_t connection_params_init;
    
    memset(&connection_params_init, 0, sizeof(connection_params_init));

    connection_params_init.p_conn_params                  = NULL;
    connection_params_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    connection_params_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    connection_params_init.disconnect_on_fail             = true;
    connection_params_init.evt_handler                    = NULL;
    connection_params_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&connection_params_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the application's BLE stack events
 *
 * @param[in]  p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t *p_ble_evt) {
    uint32_t				 err_code;
    static uint16_t			 s_conn_handle = BLE_CONN_HANDLE_INVALID;
    static ble_gap_evt_auth_status_t	 s_auth_status;
    ble_gap_enc_info_t			*p_enc_info;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
	led_set_state(LED_BLUE, LED_STATE_ON);
	s_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	break;

    case BLE_GAP_EVT_DISCONNECTED:
	led_set_state(LED_BLUE, LED_STATE_SLOW_FLASH);
	advertising_start();
	break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
	err_code = sd_ble_gap_sec_params_reply(s_conn_handle,
					       BLE_GAP_SEC_STATUS_SUCCESS,
					       &m_sec_params);
	APP_ERROR_CHECK(err_code);
	break;

    case BLE_GAP_EVT_AUTH_STATUS:
	s_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
	break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
	err_code = sd_ble_gatts_sys_attr_set(s_conn_handle, NULL, 0);
	APP_ERROR_CHECK(err_code);
	break;

    case BLE_GAP_EVT_SEC_INFO_REQUEST:
	p_enc_info = &s_auth_status.periph_keys.enc_info;
	if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div) {
	    err_code = sd_ble_gap_sec_info_reply(s_conn_handle, p_enc_info, NULL);
	    APP_ERROR_CHECK(err_code);
	} else {
	    // No keys found for this device
	    err_code = sd_ble_gap_sec_info_reply(s_conn_handle, NULL, NULL);
	    APP_ERROR_CHECK(err_code);
	}
	break;

    case BLE_GAP_EVT_TIMEOUT:
	if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
	{
	    led_set_all_off();
	}
	break;
	
    default:
	break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler
 *
 * @param[in]  p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t *p_ble_evt) {
    ble_vns_on_ble_evt(&m_vns, p_ble_evt);
    ble_sps_on_ble_evt(&m_sps, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


void ble_stack_start(void) {
    uint32_t err_code;

    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, false);
    
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


void ble_stack_stop(void) {
    uint32_t err_code;

    err_code = softdevice_handler_sd_disable();
    APP_ERROR_CHECK(err_code);
}


void ble_app_start(void) {
    SEGGER_RTT_WriteString(0, "Initializing GAP parameters.\n");
    gap_params_init();
    SEGGER_RTT_WriteString(0, "Initializing services.\n");
    services_init();
    SEGGER_RTT_WriteString(0, "Initializing CONN parameters.\n");
    conn_params_init();
    SEGGER_RTT_WriteString(0, "Initializing security parameters.\n");
    sec_params_init();
    SEGGER_RTT_WriteString(0, "Initializing advertising.\n");
    advertising_init();
    SEGGER_RTT_WriteString(0, "Starting application timers.\n");
    application_timers_start();
    SEGGER_RTT_WriteString(0, "Starting advertising.\n");
    advertising_start();
}


void ble_app_stop(void) {
    uint32_t err_code;
    
    // Stop any impending connection parameters update.
    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
}


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
    char err[20];
    sprintf(err, "Error Code: %u\n", (unsigned) error_code);
    SEGGER_RTT_WriteString(0, err);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
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
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the timer module initialization
 */
static void timers_init(void) {
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}


/**@brief Function for initializing the GPIOTE module.
 */
static void gpiote_init(void) {
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for configuring the GPIO pins.
 */
static void gpio_init(void) {
    nrf_gpio_pin_clear(LED_RED_PIN_NO);
    nrf_gpio_cfg_output(LED_RED_PIN_NO);

    nrf_gpio_pin_clear(LED_BLUE_PIN_NO);
    nrf_gpio_cfg_output(LED_BLUE_PIN_NO);

    nrf_gpio_pin_clear(LED_GREEN_PIN_NO);
    nrf_gpio_cfg_output(LED_GREEN_PIN_NO);
}

/**@brief Function for power management
 */
static void power_manage(void) {
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry
 */
int main(void) {
    SEGGER_RTT_WriteString(0, "Initializing timers.\n");
    timers_init();
    SEGGER_RTT_WriteString(0, "Initializing GPIOTE.\n");
    gpiote_init();
    SEGGER_RTT_WriteString(0, "Configuring GPIO pins.\n");
    gpio_init();
    SEGGER_RTT_WriteString(0, "Starting BLE stack.\n");
    ble_stack_start();
    SEGGER_RTT_WriteString(0, "Starting BLE app.\n");
    ble_app_start();
    SEGGER_RTT_WriteString(0, "All systems up.\n");

    for (;;) {
	app_sched_execute();
	power_manage();
    }
}
