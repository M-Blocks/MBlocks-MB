
#ifndef BLE_VNS_H__
#define BLE_VNS_H__

#include <stdint.h>
#include <stdbool.h>

#include "ble.h"
#include "ble_srv_common.h"

#include "fifo.h"


#define VNS_UUID_SERVICE_BASE		{0x45, 0xCA, 0xEE, 0x34, 0xF0, 0x0F, 0xA4, 0x80, 0x4C, 0xDA, 0x11, 0x12, 0x00, 0x00, 0x64, 0x7B}
#define VNS_UUID_SERVICE			0x1500

#define VNS_UUID_VERSION_CHAR_BASE	{0x45, 0xCA, 0xEE, 0x34, 0xF0, 0x0F, 0xA4, 0x80, 0x4C, 0xDA, 0x11, 0x12, 0x00, 0x00, 0x64, 0x7B}
#define VNS_UUID_VERSION_CHAR		0x1501


// Forward declaration of the ble_vns_t type.
typedef struct ble_vns_s ble_vns_t;


/**@brief Version Number Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct {
	uint16_t version;
} ble_vns_init_t;

/**@brief Serial Port Service structure. This contains various status information for the service. */
typedef struct ble_vns_s
{
    uint16_t					service_handle;
    uint8_t						service_uuid_type;
    ble_gatts_char_handles_t	version_char_handles;
    uint8_t						version_char_uuid_type;
    uint16_t					conn_handle;
} ble_vns_t;


uint32_t ble_vns_init(ble_vns_t * p_vns, const ble_vns_init_t * p_vns_init);

void ble_vns_on_ble_evt(ble_vns_t * p_vns, ble_evt_t * p_ble_evt);


#endif // BLE_VNS_H__
