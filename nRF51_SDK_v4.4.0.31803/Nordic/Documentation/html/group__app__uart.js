var group__app__uart =
[
    [ "app_uart_comm_params_t", "structapp__uart__comm__params__t.html", [
      [ "rx_pin_no", "structapp__uart__comm__params__t.html#ace9f2a3a73fe0b9c614806848718bdbd", null ],
      [ "tx_pin_no", "structapp__uart__comm__params__t.html#a28e4b5b95355b20bd94d65885502a84c", null ],
      [ "rts_pin_no", "structapp__uart__comm__params__t.html#aa477b770b4ec569c2019563f48217a0a", null ],
      [ "cts_pin_no", "structapp__uart__comm__params__t.html#a6c9e999fedd22169cce803da4e9946c7", null ],
      [ "flow_control", "structapp__uart__comm__params__t.html#a1ef065fdca013d2eb2fcf37356ca03e2", null ],
      [ "use_parity", "structapp__uart__comm__params__t.html#a617510ef12764b30fcda2b8c8ebe94d1", null ],
      [ "baud_rate", "structapp__uart__comm__params__t.html#a148f33bbcda8087a77d8ba30f7e3c502", null ]
    ] ],
    [ "app_uart_buffers_t", "structapp__uart__buffers__t.html", [
      [ "rx_buf", "structapp__uart__buffers__t.html#abb15c7b538096e612c6d3b0000936966", null ],
      [ "rx_buf_size", "structapp__uart__buffers__t.html#ae01327dd23f7bb0d4bba2076cddc61ad", null ],
      [ "tx_buf", "structapp__uart__buffers__t.html#a8a75dbee0db96d1e078035555512308c", null ],
      [ "tx_buf_size", "structapp__uart__buffers__t.html#a2cfaba328bf2058ff9cbe02ae3932208", null ]
    ] ],
    [ "app_uart_evt_t", "structapp__uart__evt__t.html", [
      [ "evt_type", "structapp__uart__evt__t.html#ae63d2cc586c56b666478feb056445bd1", null ],
      [ "error_communication", "structapp__uart__evt__t.html#a72e7b5647aa406845a8e8d19b708397b", null ],
      [ "error_code", "structapp__uart__evt__t.html#abf2d7e54e9bb8675882ab068a0885e37", null ],
      [ "value", "structapp__uart__evt__t.html#a638e4503e0ae6ce655b7ad2e17e8f0ad", null ],
      [ "data", "structapp__uart__evt__t.html#acfad7d3136bbcaed7dd7a3a36077f35e", null ]
    ] ],
    [ "UART_PIN_DISCONNECTED", "group__app__uart.html#gaa790ff39126a00c4308f30f1139f4efa", null ],
    [ "APP_UART_FIFO_INIT", "group__app__uart.html#ga25b7b23e541732b945b5992b5519a10f", null ],
    [ "APP_UART_INIT", "group__app__uart.html#gab37ec15202892ba42f454daacaa38b7f", null ],
    [ "app_uart_event_handler_t", "group__app__uart.html#ga61ec6f8c1f6fb0ad95c4935b80deaf7e", null ],
    [ "app_uart_flow_control_t", "group__app__uart.html#gad0b0f33b12902ce08681e06f304f0cba", [
      [ "APP_UART_FLOW_CONTROL_DISABLED", "group__app__uart.html#ggad0b0f33b12902ce08681e06f304f0cbaae7fd58fef6c10140a659be29f0b81b8b", null ],
      [ "APP_UART_FLOW_CONTROL_ENABLED", "group__app__uart.html#ggad0b0f33b12902ce08681e06f304f0cbaa76123f43769a3f237d5a670844c6d235", null ],
      [ "APP_UART_FLOW_CONTROL_LOW_POWER", "group__app__uart.html#ggad0b0f33b12902ce08681e06f304f0cbaa26a16d7573f75722e220954fa4fbf27f", null ]
    ] ],
    [ "app_uart_connection_state_t", "group__app__uart.html#ga683d8281c00679b243c06fd9b7815557", [
      [ "APP_UART_DISCONNECTED", "group__app__uart.html#gga683d8281c00679b243c06fd9b7815557acb220f64509a4882b0b79f1a4620398c", null ],
      [ "APP_UART_CONNECTED", "group__app__uart.html#gga683d8281c00679b243c06fd9b7815557a026c1999c42fd293cc7ead39f6de1cf2", null ]
    ] ],
    [ "app_uart_evt_type_t", "group__app__uart.html#ga9346b21b144fd9499e24853bbf781e17", [
      [ "APP_UART_DATA_READY", "group__app__uart.html#gga9346b21b144fd9499e24853bbf781e17ad0c6e0d9cef6b81de23e2f2583013ef1", null ],
      [ "APP_UART_FIFO_ERROR", "group__app__uart.html#gga9346b21b144fd9499e24853bbf781e17ac4e8a2753ab64a36a3bd0e723a3f26ea", null ],
      [ "APP_UART_COMMUNICATION_ERROR", "group__app__uart.html#gga9346b21b144fd9499e24853bbf781e17a6ee822a8a07d1be09ac336495a5238d8", null ],
      [ "APP_UART_TX_EMPTY", "group__app__uart.html#gga9346b21b144fd9499e24853bbf781e17a9fa032c57eadcef66e102eb78a04c2d1", null ],
      [ "APP_UART_DATA", "group__app__uart.html#gga9346b21b144fd9499e24853bbf781e17a5b4c2503e1e658ff28dcaad17eff6ac4", null ]
    ] ],
    [ "app_uart_init", "group__app__uart.html#gae650b57bf30da0f26ae409782de9fcbd", null ],
    [ "app_uart_get", "group__app__uart.html#gacddb5b7b711ef104f9eb181a13bc4503", null ],
    [ "app_uart_put", "group__app__uart.html#ga2e4c8407274a151e72ed5a226529dc36", null ],
    [ "app_uart_get_connection_state", "group__app__uart.html#ga4360a21365ae56f22344f84c20933d64", null ],
    [ "app_uart_flush", "group__app__uart.html#ga2b867b8dfc7209b87e2e8f0da0741105", null ],
    [ "app_uart_close", "group__app__uart.html#ga8b5b23229363aefa52be33360dfb5fde", null ]
];