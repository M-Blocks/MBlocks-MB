#include <stdlib.h>
#include <string.h>

#include "app_timer.h"

#include "fb.h"
#include "util.h"
#include "global.h"

#include "message.h"

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
	static char buffer[50];
	static int bufferLen;

	// read as many bytes as we can; split at '\n'
	// keep buffer; once '\n' is seen, parse message
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
				if ((char) rxData[i] == '\n') {
					// message has been received, send it for processing
					strncpy(msg, buffer, bufferLen);
					msg[bufferLen] = '\0';
					process_message(msg);

					// reset variables
					bufferLen = 0;
				} else {
					buffer[bufferLen] = (char) rxData[i];
					bufferLen += 1;
				}
			}
		}
	}
}

void push_message(char *txData) {
	/* Push message on all faces. */
	char str[64];
	int numBytes = strlen(txData);
	for (int faceNum = 1; faceNum <= 6; faceNum++) {
		if (fb_sendToTxBuffer(faceNum, numBytes, (uint8_t *)txData)) {
			snprintf(str, sizeof(str), "Wrote %u bytes to IR transmit buffer on faceboard %u\r\n", numBytes, faceNum);
		} else {
			snprintf(str, sizeof(str), "Failed to write to IR transmit buffer on faceboard %u\r\n", faceNum);
		}
		app_uart_put_string(str);
	}
}

void prepare_message_send(const char *type, int msgCnt, char *destID, char *msg) {
	char intStr[6];
	snprintf(intStr, 6, "%d", msgCnt);

	char macAddress[] = "C6:EA:B8:01:3D:EE";
	// Prepare message ID: 17 bytes for MAC + 1 byte + 6 bytes for int
	char msgID[25];
	strcpy(msgID, macAddress);
	strcat(msgID, "+");
	strcat(msgID, intStr);

	char txData[100];
	strcpy(txData, type); 	strcat(txData, ";");
	strcat(txData, msgID); 	strcat(txData, ";");
	strcat(txData, destID); strcat(txData, ";");
	strcat(txData, msg);	strcat(txData, "\n");

	process_message(txData);
	//push_message(txData);
}

void prepare_message_bdcast(const char *type, int msgCnt, char *msg) {
	char intStr[6];
	snprintf(intStr, 6, "%d", msgCnt);

	char macAddress[] = "C6:EA:B8:01:3D:EE";  // TODO
	// Prepare message ID: 17 bytes for MAC + 1 byte + 6 bytes for int
	char msgID[25];
	strcpy(msgID, macAddress);
	strcat(msgID, "+");
	strcat(msgID, intStr);

	char txData[100];
	strcpy(txData, type); 	strcat(txData, ";");
	strcat(txData, msgID); 	strcat(txData, ";");
	strcat(txData, msg);	strcat(txData, "\n");

	app_uart_put_string(txData);
	app_uart_put_string("\r\n");
	//push_message(txData);
}

void process_message(char *msg) {
	char bcastCpy[100];
	strcpy(bcastCpy, msg);

	char *token = strtok(msg, ";");
	if (strcmp(token, "SENDCMD") == 0) {
		token = strtok(NULL, ";");		// get message ID
		// if(duplicate(token)) {
		// 	return;
		// }			 
		
		token = strtok(NULL, ";");
		char macAddress[] = "c6:ea:b8:01:3d:ee";  // TODO
		if (strcmp(token, macAddress) == 0) {
			token = strtok(NULL, ";");
			//cmdLine_execCmd(token);
		}
	}
	if (strcmp(token, "BDCASTCMD") == 0) {
		token = strtok(NULL, ";");		// get message ID
		// if(duplicate(token)) {
		// 	return;
		// }			 
		
		token = strtok(NULL, ";");
		// cmdLine_execCmd(token);
	}
}

// TO DO: find neighbors
void ack_message(char *msg) {

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