/*
 * ble.c
 *
 *  Created on: Jan 31, 2014
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf_error.h"
#include "nrf_sdm.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_stack_handler.h"
#include "ble_debug_assert_handler.h"

#include "global.h"
#include "fifo.h"
#include "led.h"
#include "ble_vns.h"
#include "ble_sps.h"
#include "bleApp.h"

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define BLE_TX_FIFO_SIZE	384
#define BLE_RX_FIFO_SIZE	64

/* Version number service */
ble_vns_t m_vns;

/* Serial port service */
ble_sps_t m_sps;

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static fifo_t ble_sps_txFifo;
static uint8_t ble_sps_txFifoData[BLE_TX_FIFO_SIZE];
static fifo_t ble_sps_rxFifo;
static uint8_t ble_sps_rxFifoData[BLE_RX_FIFO_SIZE];

static bool advertisingEnabled = true;

static char deviceName[BLE_GAP_DEVNAME_MAX_LEN + 1] = "BLE SPP 67:6F:C7";

static void bleApp_evtDispatch(ble_evt_t * p_ble_evt);
static void bleApp_onEvt(ble_evt_t * p_ble_evt);
static void bleApp_onConnParamsEvt(ble_conn_params_evt_t * p_evt);
static void bleApp_connParamsErrorHandler(uint32_t nrf_error);

/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void bleApp_stackInit() {
    // YOUR_JOB: If the MTU size is changed by the application, the MTU_SIZE parameter to
    //           BLE_STACK_HANDLER_INIT() must be changed accordingly.
    /*
	BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           true);
    */

	BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION,
			BLE_L2CAP_MTU_DEF,
			bleApp_evtDispatch,
			true);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void bleApp_evtDispatch(ble_evt_t * p_ble_evt) {
    bleApp_onEvt(p_ble_evt);

    /* Pass event to the SDK-supplied connection parameters module */
    ble_conn_params_on_ble_evt(p_ble_evt);

    /* Pass events to custom services */
    ble_vns_on_ble_evt(&m_vns, p_ble_evt);
    ble_sps_on_ble_evt(&m_sps, p_ble_evt);
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void bleApp_onEvt(ble_evt_t * p_ble_evt) {
    uint32_t                         err_code = NRF_SUCCESS;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        	led_setState(LED_BLUE, LED_STATE_ON);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        	m_conn_handle = BLE_CONN_HANDLE_INVALID;
        	led_setState(LED_BLUE, LED_STATE_OFF);
        	if (advertisingEnabled) {
        		bleApp_advertisingStart();
        	}
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

/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
void bleApp_gapParamsInit() {
    uint32_t err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (uint8_t *)deviceName, strlen(deviceName));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.*/
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize services that will be used by the application.
 */
void bleApp_servicesInit() {
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



/**@brief Initialize security parameters. */
void bleApp_secParamsInit() {
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Initialize the Connection Parameters module. */
void bleApp_connParamsInit() {
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = bleApp_onConnParamsEvt;
    cp_init.error_handler                  = bleApp_connParamsErrorHandler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
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
void bleApp_onConnParamsEvt(ble_conn_params_evt_t * p_evt) {
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
void bleApp_connParamsErrorHandler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}


bool bleApp_isConnected() {
	if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
		return false;
	}

	return true;
}

void bleApp_setAdvertisingEnabled(bool enable) {
	advertisingEnabled = enable;

	if (advertisingEnabled && (m_conn_handle == BLE_CONN_HANDLE_INVALID)) {
		/* If advertising is enabled, we need to actually start advertising
		 * here (assuming we are not already connected) because advertising is
		 * normally only started when the previous connection is
		 * disconnected. */
		bleApp_advertisingStart();
	} else if (!advertisingEnabled && (m_conn_handle == BLE_CONN_HANDLE_INVALID)) {
		/* Likewise, if we have just disabled advertising, and there is no
		 * active connection, we stop advertising here.  If there is an active
		 * connection, we leave it intact, but once its disconnects, we will
		 * not start advertising again. */
		bleApp_advertisingStop();
	}
}

bool bleApp_isAdvertisingEnabled() {
	return advertisingEnabled;
}

/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void bleApp_advertisingInit() {
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

/**@brief Start advertising.
 */
void bleApp_advertisingStart() {
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
    /* Allow invalid state errors, trap everything else.  We do this because we
     * may already be advertising when we try to start advertising again, and
     * that's okay. */
    if (err_code == NRF_ERROR_INVALID_STATE) {
    	;
    } else {
    	APP_ERROR_CHECK(err_code);
    }

    led_setState(LED_BLUE, LED_STATE_SLOW_FLASH);
}

void bleApp_advertisingStop() {
	uint32_t err_code;

	err_code = sd_ble_gap_adv_stop();
    /* Allow invalid state errors, trap everything else.  We do this because we
     * may not be advertising at the moment when we try to stop advertising,
     * and even though that generates an error, it's okay. */
    if (err_code == NRF_ERROR_INVALID_STATE) {
    	;
    } else {
    	APP_ERROR_CHECK(err_code);
    }

	led_setState(LED_BLUE, LED_STATE_OFF);
}
