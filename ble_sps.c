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

#include "SEGGER_RTT.h"

#include "ble_sps.h"
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_uart.h"

#include "util.h"
#include "fifo.h"

static bool initialized = false;

static void on_connect(ble_sps_t * p_sps, ble_evt_t * p_ble_evt);
static void on_disconnect(ble_sps_t * p_sps, ble_evt_t * p_ble_evt);
static void on_write(ble_sps_t * p_sps, ble_evt_t * p_ble_evt);
static void on_rw_authorize_request(ble_sps_t *p_sps, ble_evt_t *p_ble_evt);
static void on_hvc(ble_sps_t *p_sps, ble_evt_t *p_ble_evt);
static void on_timeout(ble_sps_t *p_sps, ble_evt_t *p_ble_evt);

static void xfer_writeData_to_rxFifo(ble_sps_t *p_sps);
static void indicate_txFifo(ble_sps_t *p_sps);

static uint32_t config_char_add(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init);
static uint32_t ackdata_char_add(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init);

/**@brief Connect event handler.
 *
 * @param[in]   p_lbs       Serial Port Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void on_connect(ble_sps_t * p_sps, ble_evt_t * p_ble_evt) {

#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
    SEGGER_RTT_WriteString(0, "\r\nON_CONNECT\r\n");
#endif

    p_sps->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Disconnect event handler.
 *
 * @param[in]   p_lbs       Serial Port Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void on_disconnect(ble_sps_t * p_sps, ble_evt_t * p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);

#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
    SEGGER_RTT_WriteString(0, "\r\nON_DISCONNECT\r\n");
#endif

    p_sps->conn_handle = BLE_CONN_HANDLE_INVALID;

    p_sps->indicateDataLen = 0;
    p_sps->awaitingIndicationConfirm = 0;
    p_sps->writeDataLen = 0;
    p_sps->writeAuthorizationPending = 0;

    fifo_purge(p_sps->p_rxFifo);
    fifo_purge(p_sps->p_txFifo);
}

/**@brief Write event handler.
 *
 * @param[in]   p_sps       Serial Port Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void on_write(ble_sps_t * p_sps, ble_evt_t * p_ble_evt)
{

#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
    SEGGER_RTT_WriteString(0, "\r\nON_WRITE\r\n");
#endif

    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_sps->ackdata_char_handles.cccd_handle) &&
		   (p_evt_write->len == 2)) {
       /* If indications had previously not been enabled and now have been, we
        * should consider attempting to send whatever has been queued up in
        * the transmit queue at this point. */
    	;
   }
}

void on_rw_authorize_request(ble_sps_t *p_sps, ble_evt_t *p_ble_evt) {
	ble_gatts_evt_rw_authorize_request_t *auth_req;

#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
    SEGGER_RTT_WriteString(0, "\r\nON_RW_AUTHORIZE_REQUEST\r\n");
#endif

	auth_req = &p_ble_evt->evt.gatts_evt.params.authorize_request;

	/* Verify that this request is a write request to the acknowledged data
	 * attribute. If it is, save the data and set the flag which indicates that
	 * we need to authorize the write (by sending a message back to the client)
	 * as soon as there is space available in the receive FIFO. */
	if ((auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) &&
			(auth_req->request.write.handle == p_sps->ackdata_char_handles.value_handle)) {
		memcpy(p_sps->writeData, &auth_req->request.write.data, auth_req->request.write.len);
		p_sps->writeDataLen = (uint8_t)auth_req->request.write.len;
		p_sps->writeAuthorizationPending = 1;
	} else {
		;
	}

	/* Call function which copies the contents of p_sps->writeData to the UART
	 * receive FIFO if there is room available.  The call will also authorize
	 * the write that the client just attempted. */
	xfer_writeData_to_rxFifo(p_sps);
}

void on_hvc(ble_sps_t *p_sps, ble_evt_t *p_ble_evt) {
	fifoSize_t numBytesIndicated;

#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
    SEGGER_RTT_WriteString(0, "\r\nON_HVC\r\n");
#endif

	if (p_ble_evt->evt.gatts_evt.params.hvc.handle ==
			p_sps->ackdata_char_handles.value_handle) {

		/* The client just confirmed that it properly received the indication.
		 * As such, we can now remove the indicated data from the UART's
		 * transmit FIFO. */
		numBytesIndicated = p_sps->indicateDataLen;
		fifo_discard(p_sps->p_txFifo, &numBytesIndicated);
		p_sps->indicateDataLen = 0;
		p_sps->awaitingIndicationConfirm = 0;

		/* Attempt to indicate any additional data in the transmit FIFO. */
		indicate_txFifo(p_sps);
	}
}

void on_timeout(ble_sps_t *p_sps, ble_evt_t *p_ble_evt) {

#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
    SEGGER_RTT_WriteString(0, "\r\nON_TIMEOUT\r\n");
#endif

	/* If the client did not confirmation our attempt to indicate the data we
	 * sent to it before the built-in timeout, we now allow more data to be
	 * sent. */
	p_sps->awaitingIndicationConfirm = 0;

	/* Re-attempt to indicate the data in the transmit FIFO. */
	indicate_txFifo(p_sps);
}

void ble_sps_on_ble_evt(ble_sps_t * p_sps, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sps, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sps, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sps, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	on_rw_authorize_request(p_sps, p_ble_evt);
        	break;

        case BLE_GATTS_EVT_HVC:
        	on_hvc(p_sps, p_ble_evt);
        	break;

        case BLE_GATTS_EVT_TIMEOUT:
        	on_timeout(p_sps, p_ble_evt);
        	break;

        default:
            break;
    }
}

void xfer_writeData_to_rxFifo(ble_sps_t *p_sps) {
	fifoSize_t length;
	ble_gatts_rw_authorize_reply_params_t authorize_reply_params;

	/* The writeAuthorizationPending flag is used to indicate whether there is
	 * data that needs to be transferred from the write buffer to the receive
	 * FIFO. To clarify: the write buffer is written by the BLE client, so
	 * therefore, its contents should be transferred to the receive FIFO for
	 * processing. */
	if (!p_sps->writeAuthorizationPending) {
		return;
	}

	/* If we are able to move the received data that was written by the BLE
	 * client to the receive FIFO, we can acknowledge the write so that the
	 * client knows its attempt to send us data was successful. */
	length = (fifoSize_t)p_sps->writeDataLen;
	if (fifo_push(p_sps->p_rxFifo, p_sps->writeData, &length)) {
		/* The write buffer is now empty. */
		p_sps->writeDataLen = 0;

		/* Inform the BLE stack that it should authorize the client's write. */
		authorize_reply_params.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
		authorize_reply_params.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
		sd_ble_gatts_rw_authorize_reply(p_sps->conn_handle, &authorize_reply_params);

#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
		SEGGER_RTT_WriteString(0, "\r\nREQUEST GRANTED\r\n");
#endif

		/* Allow future writes. */
		p_sps->writeAuthorizationPending = 0;
	} else {
#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
		SEGGER_RTT_WriteString(0, "\r\nNO ROOM IN RX FIFO FOR WRITE DATA\r\n");
#endif
	}
}

void indicate_txFifo(ble_sps_t *p_sps) {
	ble_gatts_hvx_params_t params;
	uint8_t data[20];
	fifoSize_t dataLen;
	uint16_t dataLen_uint16;
	uint32_t err_code;

	/* We should consider checking whether indication is enabled. */

	/* Until the client has confirmed the last block of indicated data, we do
	 * not attempt to transmit additional data. */
	if (p_sps->awaitingIndicationConfirm) {
		return;
	}

	/* Attempt to peek at up to 20 bytes from the transmit FIFO.  Twenty bytes
	 * is the maximum payload of a BLE packet. */
	dataLen = 20;
	if (!fifo_peek(p_sps->p_txFifo, data, &dataLen)) {
		return;
	}

	/* Prepare to indicate this data to the client */
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_INDICATION;
	params.handle = p_sps->ackdata_char_handles.value_handle;
	params.p_data = data;
	/* We don't know the size of dataLen, which is of type fifoSize_t, so we
	 * must cast it to a uint16_t because that is what the
	 * ble_gatts_hvx_params_t structure expects. */
	dataLen_uint16 = (uint16_t)dataLen;
	params.p_len = &dataLen_uint16;

	/* Set the flag indicating that we're awaiting confirmation before we
	 * attempt to notify the client so that in case the client responds very
	 * quickly, we do not receiving the confirmation before we can even set
	 * the flag indicating that we were waiting for the confirmation. */
	p_sps->awaitingIndicationConfirm = 1;
	/* We need to remember how many bytes we have attempted to indicate to the
	 * client so that when the client confirms the indication we can discard
	 * those bytes from the transmit FIFO. */
	p_sps->indicateDataLen = (uint8_t)dataLen;

	err_code = sd_ble_gatts_hvx(p_sps->conn_handle, &params);
	if (err_code != NRF_SUCCESS) {
		/* If there was a problem with indication, stop expecting a
		 * confirmation in response from the client. */
		p_sps->awaitingIndicationConfirm = 0;
		p_sps->indicateDataLen = 0;
	}
}


/**@brief Add remote configuration characteristic.
 *
 * @param[in]   p_sps        Serial Port Service structure.
 * @param[in]   p_sps_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t config_char_add(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_config_char_uuid;
    ble_gatts_attr_md_t attr_md;
    uint32_t err_code;

    /* The client characteristic configuration descriptor is used to control
     * whether the server (the nRF51822) can indicate or notify to client.  We
     * allow the client to both read and write the CCCD. */
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	/* The CCCD should be located on the stack */
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	/* The characteristic should be write-able and capable of indicating. This
	 * block of code configures the characteristic's descriptors. */
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.indicate = 1;
    char_md.char_props.write = 1;
    char_md.char_props.read = 0;
    char_md.p_char_user_desc = (uint8_t *)"Configuration";
    char_md.char_user_desc_max_size = 13;
    char_md.char_user_desc_size = 13;
    char_md.p_user_desc_md = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    /* The attribute is the part of the characteristic that holds data. Here
     * we configure its metadata to allow open writes.  We disallow reads
     * since we instead transfer all data via indication. */
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc		= BLE_GATTS_VLOC_STACK;
    attr_md.vlen		= 1;


    /* Add a vendor specific 128-bit UUID to the BLE stack's table of UUIDs.
     * In particular, this UUID will be used to identify the acknowledged data
     * characteristic. An index identifying this new UUID will be stored in
     * p_sps->config_char_uuid_type. */
    ble_uuid128_t base_uuid = SPS_UUID_CONFIG_CHAR_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_sps->config_char_uuid_type);
    APP_ERROR_CHECK(err_code);

    /* Create a more easily used 24-bit UUID from the new vendor-specific
     * acknowledged data characteristic UUID base and the two TimeLow bytes
     * that are specific to the acknowledged data characteristic. */
    ble_config_char_uuid.type = p_sps->config_char_uuid_type;
    ble_config_char_uuid.uuid = SPS_UUID_CONFIG_CHAR;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_config_char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = 20;
    attr_char_value.p_value = NULL;

    return sd_ble_gatts_characteristic_add(p_sps->service_handle, &char_md,
    		 &attr_char_value, &p_sps->config_char_handles);
}

/**@brief Add acknowledged data characteristic.
 *
 * @param[in]   p_sps        Serial Port Service structure.
 * @param[in]   p_sps_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ackdata_char_add(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_ackdata_char_uuid;
    ble_gatts_attr_md_t attr_md;
    uint32_t err_code;

    /* The client characteristic configuration descriptor is used to control
     * whether the server (the nRF51822) can indicate or notify to client.  We
     * allow the client to both read and write the CCCD. */
	memset(&cccd_md, 0, sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	/* The CCCD should be located on the stack */
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	/* The characteristic should be write-able and capable of indicating. This
	 * block of code configures the characteristic's descriptors. */
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.indicate = 1;
    char_md.char_props.write = 1;
    char_md.char_props.read = 0;
    char_md.p_char_user_desc = (uint8_t *)"Acknowledged Data Stream";
    char_md.char_user_desc_max_size = 24;
    char_md.char_user_desc_size = 24;
    char_md.p_user_desc_md = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    /* The attribute is the part of the characteristic that holds data. Here
     * we configure its metadata to allow open writes, but we do require
     * write authorization so that the client an only write if there is space
     * available in the receive buffer.  We disallow reads since we instead
     * transfer all data via indication. */
    memset(&attr_md, 0, sizeof(attr_md));
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc		= BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth		= 0;
    attr_md.wr_auth		= 1;
    attr_md.vlen		= 1;


    /* Add a vendor specific 128-bit UUID to the BLE stack's table of UUIDs.
     * In particular, this UUID will be used to identify the acknowledged data
     * characteristic. An index identifying this new UUID will be stored in
     * p_sps->ackdata_char_uuid_type. */
    ble_uuid128_t base_uuid = SPS_UUID_ACKDATA_CHAR_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_sps->ackdata_char_uuid_type);
    APP_ERROR_CHECK(err_code);

    /* Create a more easily used 24-bit UUID from the new vendor-specific
     * acknowledged data characteristic UUID base and the two TimeLow bytes
     * that are specific to the acknowledged data characteristic. */
    ble_ackdata_char_uuid.type = p_sps->ackdata_char_uuid_type;
    ble_ackdata_char_uuid.uuid = SPS_UUID_ACKDATA_CHAR;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_ackdata_char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = 20;
    attr_char_value.p_value = NULL;

    return sd_ble_gatts_characteristic_add(p_sps->service_handle, &char_md,
    		 &attr_char_value, &p_sps->ackdata_char_handles);
}


uint32_t ble_sps_init(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_service_uuid;

    /* Initialize service structure */
    p_sps->conn_handle = BLE_CONN_HANDLE_INVALID;

    p_sps->p_txFifo = p_sps_init->p_txFifo;
    p_sps->p_rxFifo = p_sps_init->p_rxFifo;

    p_sps->awaitingIndicationConfirm = false;
    p_sps->indicateDataLen = 0;

    p_sps->writeAuthorizationPending = false;
    p_sps->writeDataLen = 0;

    /* Add a vendor specific 128-bit UUID to the BLE stack's table of UUIDs.
     * In particular, this UUID will be used to identify the Serial Port
     * Service.  An index identifying this new UUID will be stored in
     * p_sps->service_uuid_type. */
    ble_uuid128_t base_uuid = SPS_UUID_SERVICE_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_sps->service_uuid_type);
    if (err_code != NRF_SUCCESS) {
    	return err_code;
    }
    
    /* Create a more easily used 24-bit UUID from the new vendor-specific
     * service UUID base and the two TimeLow bytes. */
    ble_service_uuid.type = p_sps->service_uuid_type;
    ble_service_uuid.uuid = SPS_UUID_SERVICE;

#if ((BLE_DEBUG == 1) && (ENABLE_BLE_COMMANDS != 1))
    SEGGER_RTT_WriteString(0, "\r\nBLE_SPS_INIT\r\n");
#endif

    /* Add the Serial Port Service to the BLE stack using the 24-bit
     * abbreviated UUID which we just created. */
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_service_uuid, &p_sps->service_handle);
    if (err_code != NRF_SUCCESS){
    	return err_code;
    }

    // Add configuration characteristic
    err_code = config_char_add(p_sps, p_sps_init);
    if (err_code != NRF_SUCCESS) {
    	return err_code;
    }

    // Add acknowledged data characteristics
    err_code = ackdata_char_add(p_sps, p_sps_init);
    if (err_code != NRF_SUCCESS) {
    	return err_code;
    }
    
    initialized = true;

    return NRF_SUCCESS;
}

bool ble_sps_getInitialized() {
	return initialized;
}


bool ble_sps_put_char(ble_sps_t *p_sps, uint8_t c) {
	fifoSize_t dataLen = 1;

	if (p_sps->conn_handle == BLE_CONN_HANDLE_INVALID) {
		/* We do not populate the FIFO if the connection is not established
		 * as that would just be a waste of time, we do return true to
		 * indicate that the FIFO was not full. */
		return true;
	}

	if (fifo_push(p_sps->p_txFifo, &c, &dataLen) && (dataLen == 1)) {
		indicate_txFifo(p_sps);
		return true;
	}

	return false;
}

bool ble_sps_put_string(ble_sps_t *p_sps, const uint8_t *str) {
	fifoSize_t length;

	if (p_sps->conn_handle == BLE_CONN_HANDLE_INVALID) {
		/* We do not populate the FIFO if the connection is not established
		 * as that would just be a waste of time, we do return true to
		 * indicate that the FIFO was not full. */
		return true;
	}

	length = strlen((const char *)str);

	if (fifo_push(p_sps->p_txFifo, str, &length)) {
		indicate_txFifo(p_sps);
		return true;
	}

	return false;
}

bool ble_sps_get_char(ble_sps_t *p_sps, uint8_t *c) {
	fifoSize_t numChar = 1;

	if (fifo_pop(p_sps->p_rxFifo, c, &numChar) && (numChar == 1)) {
		/* Having just extracted a character from the receive FIFO, there
		 * is more space available in the receive FIFO, so we we see whether
		 * we can transfer the contents of the write buffer (i.e. the data
		 * written by the BLE client) to the receive buffer. */
		xfer_writeData_to_rxFifo(p_sps);
		return true;
	}

	return false;
}
