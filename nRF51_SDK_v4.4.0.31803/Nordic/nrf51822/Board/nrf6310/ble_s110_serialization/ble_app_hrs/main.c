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
 * @defgroup ble_sdk_app_hrs_main main.c
 * @{
 * @ingroup ble_sdk_app_hrs
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "ble_nrf6310_pins.h"
#include "ble_sensorsim.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "ble_bondmngr.h"
#include "ble_radio_notification.h"
#include "ble_flash.h"
#include "ble_debug_assert_handler.h"
#include "app_gpiote.h"

#include "nrf_delay.h"

#define WAKEUP_BUTTON_PIN                    NRF6310_BUTTON_0                               /**< Button used to wake up the application. */
#define BONDMNGR_DELETE_BUTTON_PIN_NO        NRF6310_BUTTON_1                               /**< Button used for deleting all bonded masters during startup. */

#define DEVICE_NAME                          "Nordic_HRM"                                   /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                    "NordicSemiconductor"                          /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                     40                                             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                            /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER                  0                                              /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 6                                              /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                              /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)     /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                    81                                             /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                    100                                            /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT              1                                              /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL             APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)     /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                       140                                            /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                       300                                            /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                 10                                             /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                 APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)      /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                      100                                            /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                      500                                            /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT                1                                              /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL     APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)     /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)               /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)              /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                        0                                              /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)                /**< Connection supervisory timeout (4 seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)     /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)     /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                              /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT                    30                                             /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                              /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                              /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                           /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                             /**< Maximum encryption key size. */

#define FLASH_PAGE_SYS_ATTR                  (BLE_FLASH_PAGE_END - 3)                       /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                      (BLE_FLASH_PAGE_END - 1)                       /**< Flash page used for bond manager bonding information. */

#define DEAD_BEEF                            0xDEADBEEF                                     /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define CONN_CHIP_RESET_PIN_NO              30                                              /**< Pin used for reseting the connectivity chip. */
#define CONN_CHIP_RESET_TIME                50                                              /**< The time to keep the reset line to the connectivity chip low (in milliseconds). */
#define CONN_CHIP_WAKEUP_TIME               500                                             /**< The time for connectivity chip to reset and become ready to receive serialized commands (in milliseconds). */

#define SCHED_MAX_EVENT_DATA_SIZE           MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                                BLE_STACK_HANDLER_SCHED_EVT_SIZE)           /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    30                                              /**< Maximum number of events in the scheduler queue. */

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;       /**< Handle of the current connection. */
static ble_gap_sec_params_t                  m_sec_params;                                  /**< Security requirements for this application. */
static ble_gap_adv_params_t                  m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static ble_bas_t                             m_bas;                                         /**< Structure used to identify the battery service. */
static ble_hrs_t                             m_hrs;                                         /**< Structure used to identify the heart rate service. */
static bool                                  m_rr_interval_enabled = true;                  /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static ble_sensorsim_cfg_t                   m_battery_sim_cfg;                             /**< Battery Level sensor simulator configuration. */
static ble_sensorsim_state_t                 m_battery_sim_state;                           /**< Battery Level sensor simulator state. */
static ble_sensorsim_cfg_t                   m_heart_rate_sim_cfg;                          /**< Heart Rate sensor simulator configuration. */
static ble_sensorsim_state_t                 m_heart_rate_sim_state;                        /**< Heart Rate sensor simulator state. */
static ble_sensorsim_cfg_t                   m_rr_interval_sim_cfg;                         /**< RR Interval sensor simulator configuration. */
static ble_sensorsim_state_t                 m_rr_interval_sim_state;                       /**< RR Interval sensor simulator state. */

static app_timer_id_t                        m_battery_timer_id;                            /**< Battery timer. */
static app_timer_id_t                        m_heart_rate_timer_id;                         /**< Heart rate measurement timer. */
static app_timer_id_t                        m_rr_interval_timer_id;                        /**< RR interval timer. */
static app_timer_id_t                        m_sensor_contact_timer_id;                     /**< Sensor contact detected timer. */

extern void                                  hci_transport_timeout_handle(void * p_context);/**< Declaration of HCI Transport timeout handler. This timeout handler must be handled in APP-LO to ensure correct behavior accrording to @ref nrf51_system_integration_serialization. */

/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    for (;;) ;
    //    NVIC_SystemReset();
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
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;
    
    battery_level = (uint8_t)ble_sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
    
    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void heart_rate_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt = 0;
    uint32_t        err_code;
    uint16_t        heart_rate;
    
    UNUSED_PARAMETER(p_context);

    heart_rate = (uint16_t)ble_sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
    
    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    //       of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void rr_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;
        
        rr_interval = (uint16_t)ble_sensorsim_measure(&m_rr_interval_sim_state,
                                                      &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;
    
    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
}


/**@brief Function for the scheduler to be used by the @ref app_timer module on timeouts.
 *
 * @details This function schedules the timeout handler to be executed in the main thread.
 *          Timeout handles are scheduled using the @ref app_scheduler, except 
 *          @ref hci_transport_timeout_handle.  
 *          hci_transport_timeout_handle will be executed immidiately according to 
 *          requirements.
 * 
 * @param[in]  timeout_handler  Function to be executed or scheduled.
 * @param[in]  p_context        General purpose pointer. Will be passed to the timeout handler 
 *                              upon execution of the timeout handler.
 */
uint32_t timer_event_schedule(app_timer_timeout_handler_t timeout_handler,
                        void *                      p_context)
{
    if (timeout_handler == hci_transport_timeout_handle)
    {
        timeout_handler(p_context);
        return NRF_SUCCESS;
    }
    else
    {
        app_timer_event_t timer_event;

        timer_event.timeout_handler = timeout_handler;
        timer_event.p_context       = p_context;
    
        return app_sched_event_put(&timer_event, sizeof(timer_event), app_timer_evt_get);
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module
    static uint32_t timer_buffer[CEIL_DIV(APP_TIMER_BUF_SIZE((APP_TIMER_MAX_TIMERS), (APP_TIMER_OP_QUEUE_SIZE) + 1), sizeof(uint32_t))];
    
    err_code = app_timer_init(APP_TIMER_PRESCALER,                                            
                              APP_TIMER_MAX_TIMERS,                                           
                              (APP_TIMER_OP_QUEUE_SIZE) + 1,                                   
                              timer_buffer,                                          
                              timer_event_schedule);       
    APP_ERROR_CHECK(err_code);                                                                 

    // Create timers
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rr_interval_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rr_interval_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = 
    {
        {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE}, 
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising)
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}


/**@brief Function for initializing the services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t        body_sensor_location;
    
    // Initialize Heart Rate Service
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;
    
    memset(&hrs_init, 0, sizeof(hrs_init));
    
    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);
    
    // Initialize Battery Service
    memset(&bas_init, 0, sizeof(bas_init));
    
    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    
    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));
    
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_sim_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;
    
    ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
    
    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;
    
    ble_sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);
    
    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;
    
    ble_sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief Function for initializing the security parameters.
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


/**@brief Function for starting the application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
    
    // Start application timers
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
    
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
    
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Function for handling the Connection Parameters Module.
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


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
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
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Since we are not in a connection and have not started advertising, store bonds
            err_code = ble_bondmngr_bonded_masters_store();
            APP_ERROR_CHECK(err_code);

            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                BLE_GAP_SEC_STATUS_SUCCESS, 
                                                &m_sec_params);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
                
                // Put connectivity chip to system-off mode
                err_code = sd_power_system_off();    
                APP_ERROR_CHECK(err_code);
                
                // Power off the application chip
                NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
            }
            break;
            
        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_bondmngr_on_ble_evt(p_ble_evt);
    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           true);
}


/**@brief Function for initializing the buttons.
 */
static void buttons_init(void)
{
    // Set Wakeup and Bonds Delete buttons as wakeup sources
    GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
    GPIO_WAKEUP_BUTTON_CONFIG(BONDMNGR_DELETE_BUTTON_PIN_NO);
}    


/**@brief Function for handling a Bond Manager error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void bond_manager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for the Bond Manager initialization.
 */
static void bond_manager_init(void)
{
    uint32_t            err_code;
    ble_bondmngr_init_t bond_init_data;
    bool                bonds_delete;

    // Clear all bonded masters if the Bonds Delete button is pushed
    bonds_delete = (nrf_gpio_pin_read(BONDMNGR_DELETE_BUTTON_PIN_NO) == 0);

    // Initialize the Bond Manager
    bond_init_data.flash_page_num_bond     = FLASH_PAGE_BOND;
    bond_init_data.flash_page_num_sys_attr = FLASH_PAGE_SYS_ATTR;
    bond_init_data.evt_handler             = NULL;
    bond_init_data.error_handler           = bond_manager_error_handler;
    bond_init_data.bonds_delete            = bonds_delete;

    err_code = ble_bondmngr_init(&bond_init_data);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Radio Notification event.
 */
static void radio_notification_init(void)
{
    uint32_t err_code;

    err_code = ble_radio_notification_init(NRF_APP_PRIORITY_HIGH,
                                           NRF_RADIO_NOTIFICATION_DISTANCE_4560US,
                                           ble_flash_on_radio_active_evt);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


static void connectivity_chip_reset(void)
{
    GPIO_PIN_CONFIG(CONN_CHIP_RESET_PIN_NO,                       
                    GPIO_PIN_CNF_DIR_Output,      
                    GPIO_PIN_CNF_INPUT_Disconnect,
                    GPIO_PIN_CNF_PULL_Pullup,   
                    GPIO_PIN_CNF_DRIVE_S0S1,      
                    GPIO_PIN_CNF_SENSE_Disabled);
    
    // Signal a reset to the connectivity chip by setting the 
    // reset pin on the connectivity chip low.
    nrf_gpio_pin_clear(CONN_CHIP_RESET_PIN_NO);
    nrf_delay_ms(CONN_CHIP_RESET_TIME);
    
    // Set the reset level to high again.
    nrf_gpio_pin_set(CONN_CHIP_RESET_PIN_NO);
    
    // Wait for connectivity chip to be ready.
    nrf_delay_ms(CONN_CHIP_WAKEUP_TIME);
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    APP_GPIOTE_INIT(1); 
    connectivity_chip_reset();
    
    leds_init();
    buttons_init();    
    bond_manager_init();
    timers_init();
    ble_stack_init();
    scheduler_init();
    ble_error_log_init();
    
    nrf_gpio_pin_set(NRF6310_LED_5);
    gap_params_init();
    nrf_gpio_pin_clear(NRF6310_LED_5);
    advertising_init();
    services_init();
    sensor_sim_init();
    conn_params_init();
    sec_params_init();
    radio_notification_init();
   
    // Start execution
    application_timers_start();
    advertising_start();
    
    // Enter main loop
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

/** 
 * @}
 */
