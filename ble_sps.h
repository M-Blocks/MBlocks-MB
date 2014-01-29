
#ifndef BLE_SPS_H__
#define BLE_SPS_H__

#include <stdint.h>
#include <stdbool.h>

#include "ble.h"
#include "ble_srv_common.h"

#include "fifo.h"


//#define SPS_UUID_SERVICE_BASE		{0x1D, 0x56, 0x88, 0xDE, 0x86, 0x6D, 0x3A, 0xA4, 0xEC, 0x46, 0xA1, 0xBD, 0x00, 0x00, 0xEC, 0xF6}
#define SPS_UUID_SERVICE_BASE		{0xF6, 0xEC, 0x37, 0xDB, 0xBD, 0xA1, 0x46, 0xEC, 0xA4, 0x3A, 0x6D, 0x86, 0x00, 0x00, 0x56, 0x1D}
#define SPS_UUID_SERVICE			0x88DE

#define SPS_UUID_CONFIG_CHAR_BASE	{0xFA, 0xC2, 0x87, 0xA8, 0xF4, 0xC6, 0xF0, 0xB9, 0xA8, 0x4C, 0x27, 0xD5, 0x00, 0x00, 0xB0, 0x7F}
#define SPS_UUID_CONFIG_CHAR		0x3169

#define SPS_UUID_ACKDATA_CHAR_BASE	{0xB3, 0x31, 0x07, 0x54, 0x42, 0xAF, 0xF7, 0x9A, 0x98, 0x49, 0x18, 0x25, 0x00, 0x00, 0x20, 0xAF}
#define SPS_UUID_ACKDATA_CHAR		0xFBAC


// Forward declaration of the ble_sps_t type.
typedef struct ble_sps_s ble_sps_t;


/**@brief Serial Port Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct {
	fifo_t *p_txFifo;
	fifo_t *p_rxFifo;
} ble_sps_init_t;

/**@brief Serial Port Service structure. This contains various status information for the service. */
typedef struct ble_sps_s
{
    uint16_t					service_handle;
    uint8_t						service_uuid_type;
    ble_gatts_char_handles_t	config_char_handles;
    uint8_t						config_char_uuid_type;
    ble_gatts_char_handles_t	ackdata_char_handles;
    uint8_t						ackdata_char_uuid_type;
    uint16_t					conn_handle;
    fifo_t						*p_txFifo;
    fifo_t						*p_rxFifo;
    uint8_t						indicateDataLen;
    uint8_t						awaitingIndicationConfirm;
    uint8_t						writeData[20];
    uint8_t						writeDataLen;
    uint8_t						writeAuthorizationPending;
} ble_sps_t;


uint32_t ble_sps_init(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init);

void ble_sps_on_ble_evt(ble_sps_t * p_lbs, ble_evt_t * p_ble_evt);

bool ble_sps_put_char(ble_sps_t *p_sps, uint8_t c);
bool ble_sps_put_string(ble_sps_t *p_sps, uint8_t *str);
bool ble_sps_get_char(ble_sps_t *p_sps, uint8_t *c);

#endif // BLE_SPS_H__
