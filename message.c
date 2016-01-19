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
	static char buffer[100];
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
				if ((char) rxData[i] == '|') {
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

/**@brief Process message and execute command
 *
 * Messages are formatted:
 *		<type>;<sender MAC>+<sender count>;<command>
 */
void process_message(char *msg) {
	char *token = strtok(msg, ";");
	if (strcmp(token, "SENDCMD") == 0) {
		token = strtok(NULL, ";");
		// extract command
		token = strtok(NULL, ";");
		cmdLine_execCmd(token);
	}
}