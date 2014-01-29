/*
This software is subject to the license described in the license.txt file included with this software distribution. 
You may not use this file except in compliance with this license. 
Copyright © Dynastream Innovations Inc. 2012
All rights reserved.
*/

/**@file
 * @brief The ANT-FS client sample application.
 *
 * This file is based on implementation originally made by Dynastream Innovations Inc. - August 2012
 *
 * @defgroup ant_fs_client_main ANT-FS client device simulator
 * @{
 * @ingroup nrf_ant_fs_client
 *
 * @brief The ANT-FS client device simulator.
 *
 * The ANT-FS client device simulator example illustrates the basic operations of an ANT-FS client.
 * 
 * This example supports all three methods of authentication defined in the ANT-FS specification: pairing, passkey and pass-thru. 
 * It also supports the simulation of downloading, uploading and erasing files. Other functionality that is not defined in the ANT-FS 
 * specification, such as file system implementation and data decoding/encoding are outside of scope of this implementation.
 *
 * @note This example does not implement the client undiscoverable mode as defined by the ANT-FS specification. 
 *
 * The general architecture of the ANT-FS client device simulator is illustrated in the picture below.
 * 
 *
 * @image html ant_fs_embedded_client_device_simulator_architecture.png "Architecture overview"
 *
 * 
 * \par ANT-FS specific configuration options
 *
 * The following compile time configuration options are available to suite various ANT-FS implementations:
 * - ANTFS_NETWORK_KEY           The ANT-FS Network Key.
 * - ANTFS_DEVICE_TYPE           The Device Type (Channel ID).
 * - ANTFS_TRANS_TYPE            Transmission Type (Channel ID). 
 * - ANTFS_LINK_FREQ             RF channel frequency of the client device while in the Link layer. The default value is 2450MHz. 
 * - ANTFS_BEACON_PERIOD_STATUS  Message period of the client device while in the Link layer. The default value is 4 Hz.
 * - ANTFS_LINK_COMMAND_TIMEOUT  Time (in seconds) the client device will wait without receiving any commands from the host before switching to the Link layer.
 * - ANTFS_PAIRING_TIMEOUT       Time (in seconds) the client will wait for user response to a pairing request during the authentication stage.
 * - ANTFS_CLIENT_SERIAL_NUMBER  Serial number of the client device. The device number in the Channel ID is derived from the lower two bytes of the serial number. 
 *                               The four bytes of the serial number are sent to the host during the Authenticate layer as the device ID.
 * - ANTFS_CLIENT_NAME           Friendly name of the client device. The length of this string should not exceed ANTFS_REMOTE_FRIENDLY_NAME_MAX.
 * - ANTFS_CLIENT_DEV_TYPE       Device Type (ANT-FS Client Identifier).   
 * - ANTFS_CLIENT_MANUF_ID       Manufacturing ID (ANT-FS Client Identifier).
 * - ANTFS_CLIENT_PASSKEY        Passkey used in the pairing and passkey authentication methods. Its length should be equal to ANTFS_PASSKEY_SIZE.
 *
 * This example implements the pairing, pass key and pass-thru authentication methods, which can be enabled using compile switches. 
 * All three authentication methods are enabled by default. Upload functionality can also be enabled through a compile switch and it is by default turned on. 
 * The compile switches can be set on antfs.c and are listed below.
 * - ANTFS_AUTH_TYPE_PAIRING     Pairing authentication supported.    
 * - ANTFS_AUTH_TYPE_PASSKEY     Passkey authentication supported.    
 * - ANTFS_AUTH_TYPE_PASSTHROUGH Pass-through authentication supported.    
 * - ANTFS_INCLUDE_UPLOAD        Upload operation is supported.
 *
 * To establish communication, the host and client devices shall have corresponding configurations. The host device may wish to wildcard any of the fields in the 
 * Channel ID and Client Identifier by setting them to zero, but the network key and RF channel frequency shall  be the same. The channel period of the client 
 * need not match the channel period of the host.
 *
 * A simple user interface exists consisting of 2 keypad buttons, which are active during pairing authentication for user to accept or reject the authentication,
 * and 2 LEDs which are active during reception of EVENT_TX and pairing authentication mode. LED0 been toggled for every EVENT_TX received from the and stack
 * and LED1 turned on when in pairing authentication mode. The LEDs can be configured in the leddriver.c file.
 *
 * \par General operation
 * 
 * The ANT-FS client device simulator executes automatically through each ANT-FS layer, as per the commands received from the ANT-FS host. 
 * User interaction is only required when using pairing authentication, to confirm or reject a pairing request.
 * 
 * Once a ANT-FS host detects a client with ANT channel and ANT-FS client parameters matching its search criteria, it will send a Link command to the client, 
 * specifying the Link RF channel frequency and channel period.
 * The client will automatically switch to the new frequency and channel period, and indicate in its beacon that it has moved to the Authentication stage.
 *
 * \par Authenticate
 * 
 * Once both the ANT-FS client and host are in the authentication layer the host can send a request to client device for pairing. The ANT-FS client example supports three methods
 * of authentication: pairing, passkey and pass-thru. When using the passkey and pass-thru authentication methods, the client will automatically accept or reject the 
 * authentication as outlined by the ANT-FS specification. Intervention from the user is only required when using the pairing authentication method. If the client 
 * receives a pairing request from the host, it will turn on a LED to let the user know that a host device wishes to pair with it. The user can accept or reject the 
 * pairing request by pressing the one of the appropriate buttons. If no response from the user is received before the pairing timeout expires, the request will be rejected.
 * 
 * \par Download
 *
 * A sample directory structure is implemented in this example, however, no actual files or file systems are present in the example.
 * The client will send the directory to the host if it receives a request to download the directory (file index0). When the host requests a download for any other files, 
 * the client will check if the file exists in its directory, and if there is permission to download that file. If the file can be downloaded, 
 * the client will simulate a file by sending sequential data with size matching the requested file size; otherwise, it will reject the download.
 *
 * \par Upload
 *
 * When the host requests an upload, the client will check if the file index exists in its directory, if there is permission to write on that file, 
 * and if there is enough space to write the requested data. Once the client sends a response accepting the upload request, the host can start uploading data. 
 * The device simulator does not include actual files, so data is not written to memory; however, the client keeps track of the CRC of the received data to verify 
 * the integrity of the upload.
 * 
 * \par Erase
 *
 * If the client receives a request to erase a file, it will check in its directory to see if the file exists and if there is permission to erase that file. 
 * As there are no actual files to delete in this example, if the file can be erased, 
 * the client will simply send a response to the host indicating the file was erased; otherwise, it will reject the request.
 *
 * \par Development phase configuration options
 *
 * The ANT-FS client device simulator will trace out ANT-FS protocol state information to UART when AN-FS protocol events occur. 
 * The UART peripheral can be configured in simple_uart.c file.
 *
 * The following compile time configuration options are available to assist in the development phase of the ANT-FS client implementation:
 * - TRACE_MEM_WRITE_OFF When defined tracing the file upload content buffer to UART is disabled.
 * - LEDDRIVER_ACTIVE    When defined LED usage is enabled (disabled by default).
 * - TRACE_UART          When defined UART trace outputting is enabled (disabled by default). 
 */

#include <stdint.h>
#include <stdio.h>
#include "ant_parameters.h"
#include "antfs.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "ant_interface.h"
#include "mem.h"
#include "simple_uart.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_gpiote.h"

#if defined(TRACE_UART)
    #include "app_uart.h"
    #define UART_TX_BUF_SIZE 256                                                                                                       /**< UART TX buffer size. */
    #define UART_RX_BUF_SIZE 1                                                                                                         /**< UART RX buffer size. */
#endif 

#define ANT_EVENT_MSG_BUFFER_MIN_SIZE 32u                                                                                              /**< Minimum size of ANT event message buffer. */

#define ANTFS_CLIENT_SERIAL_NUMBER    0xABCDEF12u                                                                                      /**< Serial number of client device. */
#define ANTFS_CLIENT_DEV_TYPE         416u                                                                                             /**< Beacon device type. */
#define ANTFS_CLIENT_MANUF_ID         2u                                                                                               /**< Beacon manufacturer ID. */
#define ANTFS_CLIENT_NAME             { "Ref Design" }                                                                                 /**< Client's friendly name. */
#define ANTFS_CLIENT_PASSKEY          {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10} /**< Client passkey. */

#define APP_TIMER_MAX_TIMERS          2u                                                                                               /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE       4u                                                                                               /**< Size of timer operation queues. */ 

#define APP_GPIOTE_MAX_USERS          1u                                                                                               /**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY        APP_TIMER_TICKS(50u, APP_TIMER_PRESCALER)                                                        /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

// Pairing state tracking. 
typedef enum 
{
    PAIRING_OFF = 0, /**< Pairing state not active. */ 
    PAIRING_ACCEPT,  /**< Pairing accept. */ 
    PAIRING_DENY     /**< Pairing deny. */ 
} pairing_state_t;

static const uint8_t m_friendly_name[] = ANTFS_CLIENT_NAME;    /**< Client's friendly name. */
static const uint8_t m_pass_key[]      = ANTFS_CLIENT_PASSKEY; /**< Authentication string (passkey). */

static antfs_event_return_t     m_antfs_event;                 /**< ANTFS event queue element. */
static antfs_dir_struct_t       m_temp_dir_structure;          /**< Current directory file structure. */
static antfs_request_info_t     m_response_info;               /**< Parameters for response to a download and upload request. */
static uint16_t                 m_file_index;                  /**< Index of the current file downloaded/uploaded. */
static uint32_t                 m_file_offset;                 /**< Current offset. */
static uint16_t                 m_current_crc;                 /**< Current CRC. */
static bool                     m_upload_success;              /**< Upload response. */
static volatile pairing_state_t m_pairing_state;               /**< Pairing state. */ 


/**@brief Function for handling softdevice asserts, does not return.
 * 
 * Traces out the user supplied parameters and busy loops. 
 *
 * @param[in] pc          Value of the program counter.
 * @param[in] line_num    Line number where the assert occurred.
 * @param[in] p_file_name Pointer to the file name.
 */
void softdevice_assert_callback(uint32_t pc, uint16_t line_num, const uint8_t * p_file_name)
{
    printf("ASSERT-softdevice_assert_callback\n");
    printf("PC: %#x\n", pc);
    printf("File name: %s\n", (const char*)p_file_name);
    printf("Line number: %u\n", line_num);

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
    printf("ASSERT-app_error_handler\n");
    printf("Error code: %u\n", error_code);
    printf("File name: %s\n", (const char*)p_file_name);
    printf("Line number: %u\n", line_num);

    for (;;)
    {
        // No implementation needed.
    }
}


/**@brief Function for handling protocol stack IRQ.
 *
 * Interrupt is generated by the ANT stack upon sending event to the application. 
 */
void PROTOCOL_EVENT_IRQHandler(void)
{

}


/**@brief Function for processing user feedback for ANTFS pairing authentication request.
 */
static __INLINE void pairing_user_feedback_handle(void)
{
    if (!antfs_pairing_resp_transmit((m_pairing_state == PAIRING_ACCEPT)))
    {
#if defined(ANTFS_AUTH_TYPE_PAIRING)
        // @note: If pairing is supported by the implementation the only reason this code gets 
        // executed would be if the protocol is in incorrect state, which would imply an error 
        // either in the host or the client implementation. 
        APP_ERROR_HANDLER(0);  
#endif // ANTFS_AUTH_TYPE_PAIRING
    }       
}


/**@brief Function for processing ANTFS pairing request event.
 */
static __INLINE void event_pairing_request_handle(void)
{
    const char * p_name     = antfs_hostname_get();     
    const uint32_t err_code = app_button_enable();   
    APP_ERROR_CHECK(err_code);    

    if (p_name != NULL)
    {
        printf("host name: %s\n", p_name);     
    }
}


/**@brief Function for processing ANTFS download request event.
 *
 * @param[in] p_event The event extracted from the queue to be processed.
 */
static void event_download_request_handle(const antfs_event_return_t * p_event)
{
    uint8_t response = RESPONSE_MESSAGE_OK;

    // Grab request info.
    m_file_index = p_event->file_index;

    // Read file information from directory
    if (mem_file_info_get(m_file_index, &m_temp_dir_structure))  
    {
        // Check permissions.
        if (!(m_temp_dir_structure.general_flags & ANTFS_DIR_READ_MASK))    
        {
            response = RESPONSE_MESSAGE_NOT_AVAILABLE;
            printf("Download request denied: file n/a for reading\n");
        }
        
        // Set response parameters.
        m_response_info.file_index.data           = m_file_index;   
        // File size (per directory).
        m_response_info.file_size.data            = m_temp_dir_structure.file_size_in_bytes;       
        // File is being read, so maximum size is the file size.
        m_response_info.max_file_size             = m_temp_dir_structure.file_size_in_bytes;          
        // Send the entire file in a single block if possible.
        m_response_info.max_burst_block_size.data = m_temp_dir_structure.file_size_in_bytes;  
    }
    // Index not found.
    else    
    {
        response                                  = RESPONSE_MESSAGE_NOT_EXIST;  
        m_response_info.file_index.data           = 0;      
        m_response_info.file_size.data            = 0;
        m_response_info.max_file_size             = 0;
        m_response_info.max_burst_block_size.data = 0;
        printf("Download request denied: file does not exist\n");
    }

    antfs_download_req_resp_prepare(response, &m_response_info); 
}


/**@brief Function for processing ANTFS download data event.
 * 
 * @param[in] p_event The event extracted from the queue to be processed.
 */
static void event_download_data_handle(const antfs_event_return_t * p_event)
{
    // This example does not interact with a file system, and it does not account for latency for 
    // reading or writing a file from EEPROM/flash. Prefetching the file might be useful to feed the 
    // data to download in order to maintain the burst timing.           
    if (m_file_index == p_event->file_index)     
    {
        // Only send data for a file index matching the download request.

        // Burst data block size * 8 bytes per burst packet.
        uint8_t buffer[ANTFS_BURST_BLOCK_SIZE * 8]; 
        // Offset specified by client.        
        const uint32_t offset     = p_event->offset;    
        // Size of requested block of data.        
        const uint32_t data_bytes = p_event->bytes; 

        // Read block of data from memory.
        mem_file_read(m_file_index, offset, buffer, data_bytes);    
                
        // @note: Suppress return value as no use case for handling it exists.
        UNUSED_VARIABLE(antfs_input_data_download(m_file_index, offset, data_bytes, buffer));
    }
}


/**@brief Function for processing ANTFS upload request data event.
 * 
 * @param[in] p_event The event extracted from the queue to be processed.
 */
static void event_upload_request_handle(const antfs_event_return_t * p_event)
{
    uint8_t response = RESPONSE_MESSAGE_OK;   

    if ((p_event->offset == MAX_ULONG))  
    {
        // Requesting to resume an upload.
        
        if (m_file_index != p_event->file_index)
        {
            // We do not have a save point for this file.
            m_file_offset = 0;
            m_current_crc = 0;
        }
    }
    else    
    {
        // This is a new upload.
        
        // Use requested offset and reset CRC.
        m_file_offset = p_event->offset;    
        m_current_crc = 0;                  
    }

    m_file_index = p_event->file_index;   

    // Read file information from directory.            
    if (mem_file_info_get(m_file_index, &m_temp_dir_structure))  
    {             
        // Check permissions.
        if (!(m_temp_dir_structure.general_flags & ANTFS_DIR_WRITE_MASK))    
        {
            response = RESPONSE_MESSAGE_NOT_AVAILABLE;
            printf("Upload request denied: file n/a for writing\n");
        }

        // Set response parameters.
        m_response_info.file_index.data           = m_file_index;   
        // Current valid file size is the last offset written to the file.
        m_response_info.file_size.data            = m_file_offset;   
        // Space available for writing is the file size, as specified on directory.
        m_response_info.max_file_size             = m_temp_dir_structure.file_size_in_bytes;          
        // Get the entire file in a single burst if possible.
        m_response_info.max_burst_block_size.data = m_temp_dir_structure.file_size_in_bytes;  
        // Last valid CRC.
        m_response_info.file_crc                  = m_current_crc;      
    }
    else    
    {
        // Index not found.
        
        response                                  = RESPONSE_MESSAGE_NOT_EXIST;  
        m_response_info.file_index.data           = m_file_index;
        m_response_info.file_size.data            = 0;   
        m_response_info.max_file_size             = 0;
        m_response_info.max_burst_block_size.data = 0;
        m_response_info.file_crc                  = 0;
        printf("Upload request denied: file does not exist\n");
    }

    m_upload_success = true;
    
    // @note: Suppress return value as no use case for handling it exists.
    UNUSED_VARIABLE(antfs_upload_req_resp_transmit(response, &m_response_info));
}


/**@brief Function for processing ANTFS upload data event.
 * 
 * @param[in] p_event The event extracted from the queue to be processed.
 */
static void event_upload_data_handle(const antfs_event_return_t * p_event)
{
    // This example does not interact with a file system, and it does not account for latency for 
    // reading or writing a file from EEPROM/flash. Buffering and other strategies might be useful 
    // to handle a received upload, while maintaining the burst timing.             
    if (m_upload_success && (m_file_index == p_event->file_index)) 
    {
        // Offset requested for upload. 
        const uint32_t offset     = p_event->offset;    
        // Size of current block of data.        
        const uint32_t data_bytes = p_event->bytes;     

        // Write data to file.
        if (!mem_file_write(m_file_index, offset, p_event->data, data_bytes))    
        {
            // Failed to write the data to system; do not attempt to write any more data after this, 
            // and set upload response as FAIL. 
            m_upload_success = false;     
            printf("Failed to write file to system\n");
            printf("Current offset %u, ", offset);
        }
        else
        {
            // Data was written successfully:
            // - update offset
            // - update current CRC.
            m_file_offset = offset + data_bytes;    
            m_current_crc = p_event->crc;           
        }
    }
}


/**@brief Function for processing ANTFS upload complete event.
 */
static __INLINE void event_upload_complete_handle(void)
{
    printf("ANTFS_EVENT_UPLOAD_COMPLETE\n");
    
    // @note: Suppress return value as no use case for handling it exists.
    UNUSED_VARIABLE(antfs_upload_data_resp_transmit(m_upload_success));
    if (m_upload_success)
    {
        m_file_offset = 0;
    }
}


/**@brief Function for processing ANTFS erase request event.
 * 
 * @param[in] p_event The event extracted from the queue to be processed.
 */
static void event_erase_request_handle(const antfs_event_return_t * p_event)
{
    uint8_t response = RESPONSE_MESSAGE_OK;   
    m_file_index     = p_event->file_index;    

    if (m_file_index != 0)  
    {
        // Read file information from directory.
        if (mem_file_info_get(m_file_index, &m_temp_dir_structure))  
        {
            // Check permissions.
            if (!(m_temp_dir_structure.general_flags & ANTFS_DIR_ERASE_MASK))    
            {
                response = RESPONSE_MESSAGE_FAIL;
                printf("Erase request denied: file n/a for erasing\n");
            }
            else
            {
                // Erase file.
                if (!mem_file_erase(m_file_index))
                {
                    response = RESPONSE_MESSAGE_FAIL;
                }
            }
        }
        else    
        {
            // Index not found.
            
            response = RESPONSE_MESSAGE_FAIL;    
            printf("Erase request denied: file does not exist\n");
        }
    }
    else    
    {
        // Should not delete the directory. 
        
        response = RESPONSE_MESSAGE_FAIL;
        printf("Erase request denied: can not delete directory\n");
    }
    
    antfs_erase_req_resp_transmit(response);
}

/**@brief Function for processing a single ANTFS event.
 *
 * @param[in] p_event The event extracted from the queue to be processed.
 */
static void antfs_event_process(const antfs_event_return_t * p_event)
{
    switch (p_event->event)
    {
        case ANTFS_EVENT_OPEN_COMPLETE:
            printf("ANTFS_EVENT_OPEN_COMPLETE\n");
            break;
            
        case ANTFS_EVENT_CLOSE_COMPLETE:
            printf("ANTFS_EVENT_CLOSE_COMPLETE\n");
            break;
            
        case ANTFS_EVENT_LINK:
            printf("ANTFS_EVENT_LINK\n");
            break;
            
        case ANTFS_EVENT_AUTH:
            printf("ANTFS_EVENT_AUTH\n");
            break;
            
        case ANTFS_EVENT_TRANS:
            printf("ANTFS_EVENT_TRANS\n");
            break;
            
        case ANTFS_EVENT_PAIRING_REQUEST:
            printf("ANTFS_EVENT_PAIRING_REQUEST\n");       
            event_pairing_request_handle();
            break;
            
        case ANTFS_EVENT_PAIRING_TIMEOUT:
            printf("ANTFS_EVENT_PAIRING_TIMEOUT\n");
            break;
            
        case ANTFS_EVENT_DOWNLOAD_REQUEST:
            printf("ANTFS_EVENT_DOWNLOAD_REQUEST\n");
            event_download_request_handle(p_event);
            break;        
            
        case ANTFS_EVENT_DOWNLOAD_START:
            printf("ANTFS_EVENT_DOWNLOAD_START\n");
            break;
            
        case ANTFS_EVENT_DOWNLOAD_REQUEST_DATA:
            event_download_data_handle(p_event);
            break;
            
        case ANTFS_EVENT_DOWNLOAD_COMPLETE:
            printf("ANTFS_EVENT_DOWNLOAD_COMPLETE\n");
            break;
            
        case ANTFS_EVENT_DOWNLOAD_FAIL:
            printf("ANTFS_EVENT_DOWNLOAD_FAIL\n");
            break;
            
        case ANTFS_EVENT_UPLOAD_REQUEST:
            printf("ANTFS_EVENT_UPLOAD_REQUEST\n");
            event_upload_request_handle(p_event);
            break;
            
        case ANTFS_EVENT_UPLOAD_START:
            printf("ANTFS_EVENT_UPLOAD_START\n");
            break;
            
        case ANTFS_EVENT_UPLOAD_DATA:
            event_upload_data_handle(p_event);
            break;
            
        case ANTFS_EVENT_UPLOAD_FAIL:
            printf("ANTFS_EVENT_UPLOAD_FAIL\n");
            // @note: Suppress return value as no use case for handling it exists.
            UNUSED_VARIABLE(antfs_upload_data_resp_transmit(false));
            break;
            
        case ANTFS_EVENT_UPLOAD_COMPLETE:
            printf("ANTFS_EVENT_UPLOAD_COMPLETE\n");
            event_upload_complete_handle();
            break;
            
        case ANTFS_EVENT_ERASE_REQUEST:
            printf("ANTFS_EVENT_ERASE_REQUEST\n"); 
            event_erase_request_handle(p_event);
            break;
            
        default:
            break;
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
#endif


/**@brief Function for handling button events.
 *
 * Provides the user input regarding pairing request to the ANT-FS module.
 *
 * @param[in] pin_no The pin number of the button pressed.
 */
void button_event_handle(uint8_t pin_no)
{    
    const uint32_t err_code = app_button_disable();   
    APP_ERROR_CHECK(err_code);
    
    switch (pin_no)
    {
        case BUTTON0:
            m_pairing_state = PAIRING_ACCEPT;
            break;
        case BUTTON1:
            m_pairing_state = PAIRING_DENY;
            break;            
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }    
}


/**@brief Function for configuring and setting up the softdevice. 
 */
static __INLINE void softdevice_setup(void)
{
    printf("softdevice_setup\n");
    
    uint32_t err_code = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_50_PPM, 
                                             softdevice_assert_callback);
    APP_ERROR_CHECK(err_code);

    // Configure application-specific interrupts. Set application IRQ to lowest priority and enable 
    // application IRQ (triggered from ANT protocol stack).
    err_code = sd_nvic_SetPriority(PROTOCOL_EVENT_IRQn, NRF_APP_PRIORITY_LOW); 
    APP_ERROR_CHECK(err_code);    
    err_code = sd_nvic_EnableIRQ(PROTOCOL_EVENT_IRQn);      
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry, does not return.
 */
int main(void)
{
    uint32_t err_code;
    
#ifdef TRACE_UART 
    // Configure and make UART ready for usage.
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

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
      
    // Initialize GPIOTE module.  
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
      
    // Initialize and enable button handler module. 
    static app_button_cfg_t buttons[] =
    {
        {BUTTON0, false, NRF_GPIO_PIN_PULLUP, button_event_handle},
        {BUTTON1, false, NRF_GPIO_PIN_PULLUP, button_event_handle},        
    };    
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);    
        
    softdevice_setup();
        
    const antfs_params_t params =
    {
        ANTFS_CLIENT_SERIAL_NUMBER, 
        ANTFS_CLIENT_DEV_TYPE, 
        ANTFS_CLIENT_MANUF_ID, 
        ANTFS_LINK_FREQ,
        ANTFS_DEFAULT_BEACON | DATA_AVAILABLE_FLAG_MASK, 
        m_pass_key, 
        m_friendly_name
    };
    
    antfs_init(&params);
    antfs_channel_setup();

    m_pairing_state = PAIRING_OFF; 
    
    uint8_t event;
    uint8_t ant_channel;
    uint8_t event_message_buffer[ANT_EVENT_MSG_BUFFER_MIN_SIZE];  
    bool    allow_sleep;    
    for (;;)
    {
        allow_sleep = true;
        
        // Process ANT-FS event queue.
        if (antfs_event_extract(&m_antfs_event))
        {
            antfs_event_process(&m_antfs_event);
            allow_sleep = false;
        }

        // Process ANT event queue.
        if (sd_ant_event_get(&ant_channel, &event, event_message_buffer) == NRF_SUCCESS)
        {
            antfs_message_process(event_message_buffer);
            allow_sleep = false;
        }

        // Process user feedback for pairing authentication request.
        if (m_pairing_state != PAIRING_OFF)
        {
            pairing_user_feedback_handle();
            
            // Reset to default state as been processed.
            m_pairing_state = PAIRING_OFF;  
            allow_sleep     = false;
        }
        
        // Sleep if allowed.
        if (allow_sleep)
        {
            err_code = sd_app_event_wait();
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**
 *@}
 **/
