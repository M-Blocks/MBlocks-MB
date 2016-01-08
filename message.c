#include <stdlib.h>
#include <string.h>

#include "app_timer.h"

#include "ble_gap.h"

#include "cmdline.h"
#include "fb.h"
#include "util.h"
#include "global.h"

#include "message.h"

#define MAC_ADDRESS_SIZE 17

app_timer_id_t messageTimerID = TIMER_NULL;

static uint32_t msgTime[10];
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
	static char buffer[2][100];
	static int bufferlen[2];

	uint8_t count;
	uint8_t rxData[100];
	char msg[100];

	for (int faceNum = 1; faceNum <= 1; faceNum++) {			// TODO
		if (fb_getRxBufferConsumedCount(faceNum, &count)) {
			if (count == 0) {
				continue;
			}

			fb_receiveFromRxBuffer(faceNum, count, rxData);
			for (int i = 0; i < count; i++) {
				if (rxData[i] == 0xB7) {		// skip header characters
					continue;
				} else if ((char) rxData[i] == '}') {
					strncpy(msg, buffer[faceNum], bufferlen[faceNum]);
					msg[bufferlen[faceNum]] = '\0';
					process_message(msg);
					
					bufferlen[faceNum] = 0;
				} else {
					buffer[faceNum][bufferlen[faceNum]] = (char) rxData[i];
					bufferlen[faceNum] += 1;
				}
			}
		}
	}
}

void push_message(int faceNum, char *txData) {
	/* Push message on face faceNum. If faceNum = 0, send on all faces. */
	if (faceNum > 6)
		return;

	char str[100];
	int numBytes = strlen(txData);

	if (faceNum != 0) {
		if (fb_sendToTxBuffer(faceNum, numBytes, (uint8_t *)txData)) {
			snprintf(str, sizeof(str), "Wrote %u bytes to IR transmit buffer on faceboard %u\r\n", numBytes, faceNum);
		} else {
			snprintf(str, sizeof(str), "Failed to write to IR transmit buffer on faceboard %u\r\n", faceNum);
		}
		app_uart_put_string(str);
	} else {
		for (int faceNum = 1; faceNum <= 6; faceNum++) {
			if (fb_sendToTxBuffer(faceNum, numBytes, (uint8_t *)txData)) {
				snprintf(str, sizeof(str), "Wrote %u bytes to IR transmit buffer on faceboard %u\r\n", numBytes, faceNum);
			} else {
				snprintf(str, sizeof(str), "Failed to write to IR transmit buffer on faceboard %u\r\n", faceNum);
			}
			app_uart_put_string(str);
		}
	}
}

void process_message(char *msg) {
	uint32_t err_code;
	ble_gap_addr_t mac_addr;
	err_code = sd_ble_gap_address_get(&mac_addr);
	APP_ERROR_CHECK(err_code);

	char macAddress[30];
	snprintf(macAddress, sizeof(macAddress), "%02x:%02x:%02x:%02x:%02x:%02x", 
		mac_addr.addr[5], mac_addr.addr[4], mac_addr.addr[3],
		mac_addr.addr[2], mac_addr.addr[1], mac_addr.addr[0]);

	char str[200];
	snprintf(str, sizeof(str), "Received at %s: %s\r\n", macAddress, msg);
	app_uart_put_string(str);

	char bcastCpy[100];
	char duplicateStr[30];
	strcpy(bcastCpy, msg);

	char *token = strtok(msg, ";");
	if (strcmp(token, "sendcmd") == 0) {
		token = strtok(NULL, ";");		// get message ID
		strcpy(duplicateStr, token);
		if(duplicate(duplicateStr)) {
			return;
		}			 
		
		token = strtok(NULL, ";");
		if (strncmp(token, macAddress, MAC_ADDRESS_SIZE) == 0) {
			token = strtok(NULL, ";");
			
			cmdLine_execCmd(token);
		}
	} else if (strcmp(token, "bdcastcmd") == 0) {
		token = strtok(NULL, ";");		// get message ID
		strcpy(duplicateStr, token);
		if(duplicate(duplicateStr)) {
			return;
		}			 
		
		token = strtok(NULL, ";");

		cmdLine_execCmd(token);
		push_message(0, bcastCpy);
	}
}

bool duplicate(char *msgid) {
	char *token = strtok(msgid, "+");
	int h = atoi(token);
	
	token = strtok(NULL, "+");
	int cTime = atoi(token);

	if (msgTime[h] >= cTime)
		return true;

	msgTime[h] = cTime;
	return false;
}