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

#include "ble_vns.h"
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_uart.h"

#include "fifo.h"

static void on_connect(ble_vns_t * p_vns, ble_evt_t * p_ble_evt);
static void on_disconnect(ble_vns_t * p_vns, ble_evt_t * p_ble_evt);

static uint32_t version_char_add(ble_vns_t * p_vns, const ble_vns_init_t * p_vns_init);

/**@brief Connect event handler.
 *
 * @param[in]   p_lbs       Serial Port Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void on_connect(ble_vns_t * p_vns, ble_evt_t * p_ble_evt) {
    p_vns->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Disconnect event handler.
 *
 * @param[in]   p_lbs       Serial Port Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void on_disconnect(ble_vns_t * p_vns, ble_evt_t * p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);

    p_vns->conn_handle = BLE_CONN_HANDLE_INVALID;
}


void ble_vns_on_ble_evt(ble_vns_t * p_vns, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_vns, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_vns, p_ble_evt);
            break;

        default:
            break;
    }
}




/**@brief Add version characteristic.
 *
 * @param[in]   p_vns        Version Number Service structure.
 * @param[in]   p_vns_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t version_char_add(ble_vns_t * p_vns, const ble_vns_init_t * p_vns_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_version_char_uuid;
    ble_gatts_attr_md_t attr_md;

	/* The characteristic should be read-able only. This block of code
	 * configures the characteristic's descriptors. */
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    /* The attribute is the part of the characteristic that holds data. Here
     * we configure its metadata to allow open reads.  We disallow writes. */
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc		= BLE_GATTS_VLOC_STACK;
    attr_md.vlen		= 1;

#if (0)
    /* Add a vendor specific 128-bit UUID to the BLE stack's table of UUIDs.
     * In particular, this UUID will be used to identify the acknowledged data
     * characteristic. An index identifying this new UUID will be stored in
     * p_vns->config_char_uuid_type. */
    ble_uuid128_t base_uuid = VNS_UUID_VERSION_CHAR_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_vns->version_char_uuid_type);
    if (err_code != NRF_SUCCESS) {
    	return err_code;
    }
#endif

    /* Create a more easily used 24-bit UUID from the new vendor-specific
     * acknowledged data characteristic UUID base and the two TimeLow bytes
     * that are specific to the acknowledged data characteristic. */
#if (0)
    ble_version_char_uuid.type = p_vns->version_char_uuid_type;
#else
    ble_version_char_uuid.type = p_vns->service_uuid_type;
#endif
    ble_version_char_uuid.uuid = VNS_UUID_VERSION_CHAR;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_version_char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = 2;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = 2;
    attr_char_value.p_value = (uint8_t *)(&p_vns_init->version);

    return sd_ble_gatts_characteristic_add(p_vns->service_handle, &char_md,
    		 &attr_char_value, &p_vns->version_char_handles);
}


uint32_t ble_vns_init(ble_vns_t * p_vns, const ble_vns_init_t * p_vns_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_service_uuid;


    /* Initialize service structure */
    p_vns->conn_handle = BLE_CONN_HANDLE_INVALID;

    /* Add a vendor specific 128-bit UUID to the BLE stack's table of UUIDs.
     * In particular, this UUID will be used to identify the Serial Port
     * Service.  An index identifying this new UUID will be stored in
     * p_vns->service_uuid_type. */
    ble_uuid128_t base_uuid = VNS_UUID_SERVICE_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_vns->service_uuid_type);
    if (err_code != NRF_SUCCESS) {
    	return err_code;
    }
    
    /* Create a more easily used 24-bit UUID from the new vendor-specific
     * service UUID base and the two TimeLow bytes. */
    ble_service_uuid.type = p_vns->service_uuid_type;
    ble_service_uuid.uuid = VNS_UUID_SERVICE;

    /* Add the version number service to the BLE stack using the 24-bit
     * abbreviated UUID which we just created. */
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_service_uuid, &p_vns->service_handle);
    if (err_code != NRF_SUCCESS){
    	return err_code;
    }

    // Add version characteristic
    err_code = version_char_add(p_vns, p_vns_init);
    if (err_code != NRF_SUCCESS) {
    	return err_code;
    }

    
    return NRF_SUCCESS;
}

