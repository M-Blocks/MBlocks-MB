/*
 * cmdline.h
 *
 *  Created on: Nov 20, 2013
 *      Author: kwgilpin
 */

#ifndef CMDLINE_H_
#define CMDLINE_H_

typedef struct {
    const char *cmd;
    void (*fcn)(const char *);
} cmdFcnPair_t;

void cmdline_loadCmds(const cmdFcnPair_t cmds[]);
void cmdline_newChar(char c);
bool cmdLine_execCmd(const char *cmd);

#endif /* CMDLINE_H_ */
