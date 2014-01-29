/*
This software is subject to the license described in the license.txt file included with this software distribution. 
You may not use this file except in compliance with this license. 
Copyright © Dynastream Innovations Inc. 2012
All rights reserved.
*/

/** @file
 * @brief ANT HRM RX profile device simulator configuration, setup and main processing loop.
 * This file is based on implementation originally made by Dynastream Innovations Inc. - June 2012
 *
 * @defgroup ant_hrm_rx_example ANT HRM RX example
 * @{
 * @ingroup nrf_ant_hrm
 *
 * @brief Example of ANT HRM RX profile.
 * 
 * This example connects to the nearest Heart Rate Monitor (HRM) in range and prints the UART data 
 * when it is received from the device.
 *
 * @par HRM specific configuration options 
 *
 * The following compile time configuration options are available to suit various HRM receiver implementations:
 * - HRMRX_NETWORK_KEY       ANT PLUS network key.
 * - ANTPLUS_NETWORK_NUMBER  Channel network key.
 * - HRMRX_ANT_CHANNEL       Channel number.
 * - HRMRX_DEVICE_NUMBER     Device number.
 * - HRMRX_DEVICE_TYPE       Device type.
 * - HRMRX_TRANS_TYPE        Transmission type.
 * - ANTPLUS_RF_FREQ         Channel radio frequency - offset from 2400 MHz.
 * - HRMRX_MSG_PERIOD        Channel period - in 32 kHz counts. 
 *
 * @par Development phase configuration options
 *
 * Depending on the selected compile time configuration, the HRM receiver will trace the profile 
 * specific state information to the UART.
 *
 * The following options for the compile time configuration are available in the development phase of the HRM receiver implementation:
 * - TRACE_HRM_PAGE_NMBR - output page number of the current page to UART.
 * - TRACE_HRM_PAGE_1 - output page 1 data to UART.
 * - TRACE_HRM_PAGE_2 - output page 2 data to UART.
 * - TRACE_HRM_PAGE_3 - output page 3 data to UART.
 * - TRACE_HRM_PAGE_4 - output page 4 data to UART.
 * - TRACE_UART - UART output enabled.
 * 
 * @note The ANT+ Network Key is available for ANT+ Adopters. Please refer to http://thisisant.com to become an ANT+ Adopter and access the key. 
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_error.h"
#include "nrf.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "hrm_rx.h"
#include "app_uart.h"
#include "boards.h"
#include "nordic_common.h"

#define ANT_EVENT_MSG_BUFFER_MIN_SIZE 32u  /**< Minimum size of an ANT event message buffer. */

#define UART_TX_BUF_SIZE              256u /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE              1u   /**< UART RX buffer size. */


/**@brief Function for stack Interrupt handling. 
 */
void PROTOCOL_EVENT_IRQHandler(void)
{

}


/**@brief Function for handling softdevice asserts and does not return.
 * 
 * Traces out the user supplied parameters and busy loops. 
 *
 * @param[in] pc          Value of the program counter.
 * @param[in] line_num    Line number where the assert occurred.
 * @param[in] p_file_name Pointer to the file name.
 */
void softdevice_assert_callback(uint32_t pc, uint16_t line_num, const uint8_t * p_file_name)
{
#if defined(TRACE_UART)
    printf("ASSERT-softdevice_assert_callback\n");
    printf("PC: %#x\n", pc);
    printf("File name: %s\n", (const char*)p_file_name);
    printf("Line number: %u\n", line_num);
#endif // TRACE_UART   

    for (;;)
    {
        // No implementation needed.
    }
}


/**@brief Function for handling an error. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the error occurred.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
#if defined(TRACE_UART)
    printf("ASSERT-app_error_handler\n");
    printf("Error code: %u\n", error_code);
    printf("File name: %s\n", (const char*)p_file_name);
    printf("Line number: %u\n", line_num);
#endif // TRACE_UART   

    for (;;)
    {
        // No implementation needed.
    }
}


#if defined(TRACE_UART)
/**@brief Function for handling an UART error.
 *
 * @param[in] p_event     Event supplied to the handler.
 */
void uart_error_handle(app_uart_evt_t * p_event)
{
    if ((p_event->evt_type == APP_UART_FIFO_ERROR) || 
        (p_event->evt_type == APP_UART_COMMUNICATION_ERROR))
    {
        // Copy parameters to static variables because parameters are not accessible in the 
        // debugger.
        static volatile app_uart_evt_t uart_event;

        uart_event.evt_type = p_event->evt_type;
        uart_event.data     = p_event->data;
        UNUSED_VARIABLE(uart_event);  
    
        for (;;)
        {
            // No implementation needed.
        }
    }
}
#endif // defined(TRACE_UART)         


/**@brief Function for application main entry, does not return.
 */
int main(void)
{   
    uint32_t err_code;
    
    // Setup UART. 
#if defined(TRACE_UART)
    const app_uart_comm_params_t comm_params =  
    {
        RX_PIN_NUMBER, 
        TX_PIN_NUMBER, 
        RTS_PIN_NUMBER, 
        CTS_PIN_NUMBER, 
        APP_UART_FLOW_CONTROL_DISABLED, 
        false, 
        UART_BAUDRATE_BAUDRATE_Baud38400
    }; 
        
    APP_UART_FIFO_INIT(&comm_params, 
                       UART_RX_BUF_SIZE, 
                       UART_TX_BUF_SIZE, 
                       uart_error_handle, 
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
#endif
  
    err_code = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_50_PPM, softdevice_assert_callback);
    APP_ERROR_CHECK(err_code);
  
    // Set application IRQ to lowest priority.
    err_code = sd_nvic_SetPriority(PROTOCOL_EVENT_IRQn, NRF_APP_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);  
    
    // Enable application IRQ (triggered from protocol). 
    err_code = sd_nvic_EnableIRQ(PROTOCOL_EVENT_IRQn);
    APP_ERROR_CHECK(err_code);    
    
    // Open HRM RX channel.
    hrm_rx_open();

    uint8_t event;
    uint8_t ant_channel;  
    uint8_t event_message_buffer[ANT_EVENT_MSG_BUFFER_MIN_SIZE]; 
    
    printf("Enter hrm rx main processing loop...\n");   
    
    // Main loop.   
    for (;;)
    {   
        err_code = sd_app_event_wait(); 
        APP_ERROR_CHECK(err_code);    

        // Extract and process all pending ANT events.      
        do
        {
            err_code = sd_ant_event_get(&ant_channel, &event, event_message_buffer);
            if (err_code == NRF_SUCCESS)
            {
                if (event == EVENT_RX)
                {
                    // We are only interested of RX events.
                    hrm_rx_channel_event_handle(event_message_buffer);            
                }
            }            
        } 
        while (err_code == NRF_SUCCESS);
    }
} 

/**
 *@}
 **/
