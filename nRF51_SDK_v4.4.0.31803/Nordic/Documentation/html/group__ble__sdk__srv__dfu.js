var group__ble__sdk__srv__dfu =
[
    [ "ble_dfu_pkt_write_t", "structble__dfu__pkt__write__t.html", [
      [ "len", "structble__dfu__pkt__write__t.html#a5723e60ffd628510c699eddbce90be23", null ],
      [ "p_data", "structble__dfu__pkt__write__t.html#a8304c4af5da6e830b86d7199dc9a22e6", null ]
    ] ],
    [ "ble_pkt_rcpt_notif_req_t", "structble__pkt__rcpt__notif__req__t.html", [
      [ "num_of_pkts", "structble__pkt__rcpt__notif__req__t.html#ad1a812483ca4035acbf9eb77fa22b356", null ]
    ] ],
    [ "ble_dfu_evt_t", "structble__dfu__evt__t.html", [
      [ "ble_dfu_evt_type", "structble__dfu__evt__t.html#ae656b64a7be550e59177a066245edd05", null ],
      [ "ble_dfu_pkt_write", "structble__dfu__evt__t.html#a2b50e4aeb29e518563d05cac62a11c87", null ],
      [ "pkt_rcpt_notif_req", "structble__dfu__evt__t.html#acb7ac5249e4f1fb385b6cbd7d2e19d13", null ],
      [ "evt", "structble__dfu__evt__t.html#ac973b0a2d90600e34748110b47871a7a", null ]
    ] ],
    [ "ble_dfu_s", "structble__dfu__s.html", [
      [ "conn_handle", "structble__dfu__s.html#a0d5ffe38d68e48d81e61fc6a4999ae68", null ],
      [ "service_handle", "structble__dfu__s.html#a363e00d9262febd8d752ed2f933be1e4", null ],
      [ "uuid_type", "structble__dfu__s.html#a72397dca62a7f7a53fdaf3c61035ba41", null ],
      [ "dfu_pkt_handles", "structble__dfu__s.html#aa022441926142455605e5830628a5240", null ],
      [ "dfu_ctrl_pt_handles", "structble__dfu__s.html#a092f5f6593ac8b6f17bbbaf72ceaee67", null ],
      [ "dfu_status_rep_handles", "structble__dfu__s.html#a6cca6355a9b761b93212dad8e92df6f3", null ],
      [ "evt_handler", "structble__dfu__s.html#a11147a19adf77be7ac28efef09fd6d69", null ],
      [ "error_handler", "structble__dfu__s.html#a893a7b5845beb42d9e5b650fd012f7f1", null ]
    ] ],
    [ "ble_dfu_init_t", "structble__dfu__init__t.html", [
      [ "evt_handler", "structble__dfu__init__t.html#a11147a19adf77be7ac28efef09fd6d69", null ],
      [ "error_handler", "structble__dfu__init__t.html#a893a7b5845beb42d9e5b650fd012f7f1", null ]
    ] ],
    [ "BLE_DFU_SERVICE_UUID", "group__ble__sdk__srv__dfu.html#ga1e7a6fccfdd1670cff7559b4f9e86dab", null ],
    [ "BLE_DFU_PKT_CHAR_UUID", "group__ble__sdk__srv__dfu.html#ga81add6c06bce99096b28e6dce865656b", null ],
    [ "BLE_DFU_CTRL_PT_UUID", "group__ble__sdk__srv__dfu.html#ga11ed6309b542001d0de47adf71bfc51d", null ],
    [ "BLE_DFU_STATUS_REP_UUID", "group__ble__sdk__srv__dfu.html#ga88a3fd6be571cda31e8510de9f999be8", null ],
    [ "ble_dfu_t", "group__ble__sdk__srv__dfu.html#ga03ad2477271b1db6cdcbf6285b5ccf3f", null ],
    [ "ble_dfu_evt_handler_t", "group__ble__sdk__srv__dfu.html#ga4c1ddf1bdd414c49f408f4ffd965fa00", null ],
    [ "ble_dfu_evt_type_t", "group__ble__sdk__srv__dfu.html#gae3254bda56bfd26221277e8a1696a4b5", [
      [ "BLE_DFU_START", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5a1cd5b94b536bc25cfb45cdb61b9c709f", null ],
      [ "BLE_DFU_RECEIVE_INIT_DATA", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5a5be4b8df677deabd7113748634358df3", null ],
      [ "BLE_DFU_RECEIVE_APP_DATA", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5ac907d625d9e7c33dca02f1743331d24c", null ],
      [ "BLE_DFU_VALIDATE", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5a3df5efa7854ec1d5732e0e8adc832b46", null ],
      [ "BLE_DFU_ACTIVATE_N_RESET", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5afdf5ae3efb8a4af74123482bd550f67e", null ],
      [ "BLE_DFU_SYS_RESET", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5aea137924204da831e6ca6bc18dcf877f", null ],
      [ "BLE_DFU_PKT_RCPT_NOTIF_ENABLED", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5a3e439ee06700ce37e86c9d2300e84421", null ],
      [ "BLE_DFU_PKT_RCPT_NOTIF_DISABLED", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5a91bdead0f698d2fb99368a1a7915b6dd", null ],
      [ "BLE_DFU_PACKET_WRITE", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5aedcdcc2fa4f3f0209fdbf3805fff2865", null ],
      [ "BLE_DFU_BYTES_RECEIVED_SEND", "group__ble__sdk__srv__dfu.html#ggae3254bda56bfd26221277e8a1696a4b5afbcdbbd6e91c10027a383ab2dd47ec80", null ]
    ] ],
    [ "ble_dfu_procedure_t", "group__ble__sdk__srv__dfu.html#ga8c02cda322aae2f242e2ade7c8284cfc", [
      [ "BLE_DFU_START_PROCEDURE", "group__ble__sdk__srv__dfu.html#gga8c02cda322aae2f242e2ade7c8284cfcab142998d13fb320ca462d7b1a3a817c9", null ],
      [ "BLE_DFU_INIT_PROCEDURE", "group__ble__sdk__srv__dfu.html#gga8c02cda322aae2f242e2ade7c8284cfca4d599927936783c1113c316f478c7c8c", null ],
      [ "BLE_DFU_RECEIVE_APP_PROCEDURE", "group__ble__sdk__srv__dfu.html#gga8c02cda322aae2f242e2ade7c8284cfca37c2f6b73e2fbc65ba4c2a8fae85dd2e", null ],
      [ "BLE_DFU_VALIDATE_PROCEDURE", "group__ble__sdk__srv__dfu.html#gga8c02cda322aae2f242e2ade7c8284cfcaa8545d7cc421ed4637c6a95a8178437d", null ],
      [ "BLE_DFU_PKT_RCPT_REQ_PROCEDURE", "group__ble__sdk__srv__dfu.html#gga8c02cda322aae2f242e2ade7c8284cfca0378c3ced232f1d71ebb7bd15fab0958", null ]
    ] ],
    [ "ble_dfu_resp_val_t", "group__ble__sdk__srv__dfu.html#ga4373c9bf123c0fd7858aac7a75923a81", [
      [ "BLE_DFU_RESP_VAL_SUCCESS", "group__ble__sdk__srv__dfu.html#gga4373c9bf123c0fd7858aac7a75923a81a41c5cb143d8b4a872692b0c99078d14d", null ],
      [ "BLE_DFU_RESP_VAL_INVALID_STATE", "group__ble__sdk__srv__dfu.html#gga4373c9bf123c0fd7858aac7a75923a81a32d406725831e1e6ee854fe1b3db8f80", null ],
      [ "BLE_DFU_RESP_VAL_NOT_SUPPORTED", "group__ble__sdk__srv__dfu.html#gga4373c9bf123c0fd7858aac7a75923a81ab543d26f426831ba0a75ef5276d0c370", null ],
      [ "BLE_DFU_RESP_VAL_DATA_SIZE", "group__ble__sdk__srv__dfu.html#gga4373c9bf123c0fd7858aac7a75923a81a9c2aeed38374c47c55f912b2239838ff", null ],
      [ "BLE_DFU_RESP_VAL_CRC_ERROR", "group__ble__sdk__srv__dfu.html#gga4373c9bf123c0fd7858aac7a75923a81aadbf20d30047e2884daa2d0acaf280a7", null ],
      [ "BLE_DFU_RESP_VAL_OPER_FAILED", "group__ble__sdk__srv__dfu.html#gga4373c9bf123c0fd7858aac7a75923a81a773dc1a4192071f3a68388456bbd0e63", null ]
    ] ],
    [ "ble_dfu_on_ble_evt", "group__ble__sdk__srv__dfu.html#ga41b1425f99ffb9fff07ea41166cea6a7", null ],
    [ "ble_dfu_init", "group__ble__sdk__srv__dfu.html#gae07a32975c8366cda747a0734a3f43ea", null ],
    [ "ble_dfu_response_send", "group__ble__sdk__srv__dfu.html#gad7ddce358287772b50ae654138ff3ac5", null ],
    [ "ble_dfu_bytes_rcvd_report", "group__ble__sdk__srv__dfu.html#gaafc505a19500b0f629ff5210ff3e754d", null ],
    [ "ble_dfu_pkts_rcpt_notify", "group__ble__sdk__srv__dfu.html#gadf6fde821bd72c16b61442d0627f4e50", null ]
];