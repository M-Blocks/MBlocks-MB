/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
#include "ble_conn.h"
#include "conn_mw_ble.h"
#include "ble_serialization.h"

uint32_t conn_mw_ble_tx_buffer_count_get(uint8_t const * const p_rx_buf,
                                         uint32_t              rx_buf_len,
                                         uint8_t * const       p_tx_buf,
                                         uint32_t * const      p_tx_buf_len)
{
    SER_ASSERT_NOT_NULL(p_rx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf_len);

    uint8_t   count;
    uint8_t * p_count = &count;

    uint32_t err_code = NRF_SUCCESS;
    uint32_t sd_err_code;

    err_code = ble_tx_buffer_count_get_req_dec(p_rx_buf, rx_buf_len, &p_count);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    sd_err_code = sd_ble_tx_buffer_count_get(p_count);

    err_code = ble_tx_buffer_count_get_rsp_enc(sd_err_code, p_tx_buf, p_tx_buf_len, p_count);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t conn_mw_ble_uuid_vs_add(uint8_t const * const p_rx_buf,
                                 uint32_t              rx_buf_len,
                                 uint8_t * const       p_tx_buf,
                                 uint32_t * const      p_tx_buf_len)
{
    SER_ASSERT_NOT_NULL(p_rx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf_len);

    ble_uuid128_t   uuid;
    ble_uuid128_t * p_uuid = &uuid;
    uint8_t         uuid_type;
    uint8_t *       p_uuid_type = &uuid_type;

    uint32_t err_code = NRF_SUCCESS;
    uint32_t sd_err_code;

    err_code = ble_uuid_vs_add_req_dec(p_rx_buf, rx_buf_len, &p_uuid, &p_uuid_type);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    sd_err_code = sd_ble_uuid_vs_add(p_uuid, p_uuid_type);

    err_code = ble_uuid_vs_add_rsp_enc(sd_err_code, p_tx_buf, p_tx_buf_len, p_uuid_type);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);
    return err_code;
}

uint32_t conn_mw_ble_uuid_decode(uint8_t const * const p_rx_buf,
                                 uint32_t              rx_buf_len,
                                 uint8_t * const       p_tx_buf,
                                 uint32_t * const      p_tx_buf_len)
{
    SER_ASSERT_NOT_NULL(p_rx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf_len);

    uint8_t      raw_uuid[16];
    uint8_t      uuid_len   = sizeof (raw_uuid);
    uint8_t *    p_raw_uuid = raw_uuid;
    ble_uuid_t   uuid;
    ble_uuid_t * p_uuid   = &uuid;
    uint32_t     err_code = NRF_SUCCESS;
    uint32_t     sd_err_code;

    err_code = ble_uuid_decode_req_dec(p_rx_buf, rx_buf_len, &uuid_len, &p_raw_uuid, &p_uuid);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    sd_err_code = sd_ble_uuid_decode(uuid_len, p_raw_uuid, p_uuid);

    err_code = ble_uuid_decode_rsp_enc(sd_err_code, p_tx_buf, p_tx_buf_len, p_uuid);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t conn_mw_ble_uuid_encode(uint8_t const * const p_rx_buf,
                                 uint32_t              rx_buf_len,
                                 uint8_t * const       p_tx_buf,
                                 uint32_t * const      p_tx_buf_len)
{
    SER_ASSERT_NOT_NULL(p_rx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf_len);

    uint8_t      raw_uuid[16];
    uint8_t      uuid_len   = sizeof (raw_uuid);
    uint8_t *    p_uuid_len = &uuid_len;
    uint8_t *    p_raw_uuid = raw_uuid;
    ble_uuid_t   uuid;
    ble_uuid_t * p_uuid   = &uuid;
    uint32_t     err_code = NRF_SUCCESS;
    uint32_t     sd_err_code;

    err_code = ble_uuid_encode_req_dec(p_rx_buf, rx_buf_len, &p_uuid, &p_uuid_len, &p_raw_uuid);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    sd_err_code = sd_ble_uuid_encode(p_uuid, p_uuid_len, p_raw_uuid);

    err_code = ble_uuid_encode_rsp_enc(sd_err_code, p_tx_buf, p_tx_buf_len, uuid_len, p_raw_uuid);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t conn_mw_ble_version_get(uint8_t const * const p_rx_buf,
                                 uint32_t              rx_buf_len,
                                 uint8_t * const       p_tx_buf,
                                 uint32_t * const      p_tx_buf_len)
{
    SER_ASSERT_NOT_NULL(p_rx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf_len);

    ble_version_t   version;
    ble_version_t * p_version = &version;

    uint32_t err_code = NRF_SUCCESS;
    uint32_t sd_err_code;

    err_code = ble_version_get_req_dec(p_rx_buf, rx_buf_len, &p_version);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    sd_err_code = sd_ble_version_get(p_version);

    err_code = ble_version_get_rsp_enc(sd_err_code, p_tx_buf, p_tx_buf_len, p_version);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t conn_mw_ble_opt_get(uint8_t const * const p_rx_buf,
                             uint32_t              rx_buf_len,
                             uint8_t * const       p_tx_buf,
                             uint32_t * const      p_tx_buf_len)
{
    SER_ASSERT_NOT_NULL(p_rx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf_len);

    uint32_t   opt_id;
    ble_opt_t  opt;
    ble_opt_t *p_opt = &opt;

    uint32_t err_code = NRF_SUCCESS;
    uint32_t sd_err_code;

    err_code = ble_opt_get_req_dec(p_rx_buf, rx_buf_len, &opt_id, &p_opt);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    sd_err_code = sd_ble_opt_get(opt_id, p_opt);

    err_code = ble_opt_get_rsp_enc(sd_err_code, p_tx_buf, p_tx_buf_len, opt_id, p_opt);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t conn_mw_ble_opt_set(uint8_t const * const p_rx_buf,
                             uint32_t              rx_buf_len,
                             uint8_t * const       p_tx_buf,
                             uint32_t * const      p_tx_buf_len)
{
    SER_ASSERT_NOT_NULL(p_rx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf);
    SER_ASSERT_NOT_NULL(p_tx_buf_len);

    uint32_t   opt_id;
    ble_opt_t  opt;
    ble_opt_t *p_opt = &opt;

    uint32_t err_code = NRF_SUCCESS;
    uint32_t sd_err_code;

    err_code = ble_opt_set_req_dec(p_rx_buf, rx_buf_len, &opt_id, &p_opt);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    sd_err_code = sd_ble_opt_set(opt_id, p_opt);

    err_code = ble_opt_set_rsp_enc(sd_err_code, p_tx_buf, p_tx_buf_len);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}