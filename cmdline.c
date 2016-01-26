/*
 * cmdline.c
 *
 *  Created on: Nov 20, 2013
 *      Author: kwgilpin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "cmdline.h"

static const cmdFcnPair_t *cmdTable;
static char cmdStr[MAX_CMDSTR_LEN] = "";

const char emptyCmdStr[] = "";

void cmdline_loadCmds(const cmdFcnPair_t cmds[]) {
    cmdTable = cmds;
}

void cmdline_newChar(char c) {
    size_t cmdStrLen;

    c = (char)tolower((int)c);

    cmdStrLen = strlen(cmdStr);

    if ((c == '\n') || (c == '\r')) {
	// If we received a CR or LF, attempt to execute whatever command
	// is in the buffer.
	cmdLine_execCmd(cmdStr);
	// Effectively empty the command buffer after executing the command
	cmdStr[0] = '\0';
    } else if ((c < 32) || (c > 126)) {
	// If the character is outside of the printable range, return without
	// doing anything with it.
	return;
    } else if (cmdStrLen + 1 < MAX_CMDSTR_LEN) {
	// If there is still space in the buffer, add the new character to
	// the end of it being sure to always maintain null termination.
	cmdStr[cmdStrLen+1] = '\0';
	cmdStr[cmdStrLen] = c;
    }
}

bool cmdLine_execCmd(const char *cmd) {
    uint8_t i;
    const char *args;

    for (i=0; ; i++) {
	// Scan through all lines of the command table, comparing the received
	// command with the current line of the table.


	if (strlen(cmdTable[i].cmd) == 0) {
	    // If we reach the last line in the command table without a match,
	    // break out of the loop and return to the caller.
	    return false;
	}

	// If the command string does not match the string at the current index
	// of the table, continue to the next index.
	if (strncmp(cmd, cmdTable[i].cmd, strlen(cmdTable[i].cmd)) != 0) {
	    continue;
	}

	if ((cmd[strlen(cmdTable[i].cmd)] != '\0') && (cmd[strlen(cmdTable[i].cmd)] != ' ')) {
	    // If there is not a space after the command name, and if the
	    // command is not just the name, we declare the command invalid.
	    // This means that "temperature" cannot be substituted for "temp".
	    continue;
	}

	// If the function associated with the command is invalid, continue
	// scanning the list for the same command with a valid function pointer
	if (cmdTable[i].fcn == NULL) {
	    continue;
	}

	// Command arguments start just after the command itself
	args = &cmd[strlen(cmdTable[i].cmd)];

	// Execute the command and return true
	cmdTable[i].fcn(args);
	return true;
    }

    // If we did not find a match, return false.
    return false;
}
