#include <stdlib.h>
#include <string.h>

#include "app_timer.h"

#include "fb.h"
#include "util.h"
#include "global.h"
#include "cmdline.h"

#include "message.h"

app_timer_id_t messageTimerID = TIMER_NULL;

static bool initialized = false;

static void message_timeoutHandler(void *p_context);

void message_init() {
    uint32_t err_code;

    if (messageTimerID == TIMER_NULL) {
	err_code = app_timer_create(&messageTimerID, APP_TIMER_MODE_REPEATED, message_timeoutHandler);
	APP_ERROR_CHECK(err_code);
    }

    err_code = app_timer_start(messageTimerID, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

    initialized = true;
}

void message_deinit() {
    uint32_t err_code;

    if (!initialized) {
	return;
    }

    if (messageTimerID != TIMER_NULL) {
	err_code = app_timer_stop(messageTimerID);
	APP_ERROR_CHECK(err_code);
    }

    initialized = false;
}

void message_timeoutHandler(void *p_context) {
    static char buffer[6][128];
    static short bufferLen[6];

    // read as many bytes as we can; split at '\n'
    // keep buffer; once '|' is seen, parse message
    uint8_t count;
    uint8_t rxData[100];

    for (int faceNum = 0; faceNum <= 5; faceNum++) {		
	if (fb_getRxBufferConsumedCount(faceNum + 1, &count)) {
	    if (count == 0) {
		continue;
	    }
	    if (count > 100) {
		count = 100;
	    }

	    fb_receiveFromRxBuffer(faceNum + 1, count, rxData);
	    for (int i = 0; i < count; i++) {
		short len = bufferLen[faceNum];
		if (rxData[i] > 0x7F) {
		    continue;
		}
		if ((char) rxData[i] == '|') {
		    // message has been received, send it for processing
		    buffer[faceNum][len] = '\0';
		    process_message(buffer[faceNum]);

		    bufferLen[faceNum] = 0;
		} else {
		    buffer[faceNum][len] = (char) rxData[i];
		    bufferLen[faceNum] = (len + 1) % 128;
		}
	    }
	}
    }
}

/**@brief Process message and execute command
 *
 * Messages are formatted:
 *		<type>;<sender MAC>+<sender count>;<command>
 */
void process_message(char *msg) {
    static int msg_cnt = 0;
    char macaddr[13];
    char recaddr[13];
    
    MACaddress(macaddr);
    macaddr[12] = '\0';
    
    app_uart_put_string(msg);
    app_uart_put_string("\r\n");
    char *token = strtok(msg, ";");
    if (strcmp(token, "sendcmd") == 0) {
	token = strtok(NULL, ";");
	strncpy(recaddr, token, sizeof(recaddr));
	if (strcmp(recaddr, macaddr) == 0) {
	    // extract ID
	    token = strtok(NULL, ";");
	    int msg_id = atoi(token);
	    if (msg_id > msg_cnt) {
		// extract command
		msg_cnt = msg_id;
		token = strtok(NULL, ";");
		cmdLine_execCmd(token);
	    }
	}
    } else if (strcmp(token, "bcstcmd") == 0) {
	// extract ID
	token = strtok(NULL, ";");
	int msg_id = atoi(token);
	if (msg_id > msg_cnt) {
	    // extract command
	    msg_cnt = msg_id;
	    token = strtok(NULL, ";");
	    cmdLine_execCmd(token);
	}
    }
}
