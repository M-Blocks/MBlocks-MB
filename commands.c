/*
 * commands.c
 *
 *  Created on: Nov 20, 2013
 *      Author: kwgilpin
 */


#include <stdio.h>
#include <string.h>
#include <math.h>

#include "ble_hci.h"
#include "ble_gap.h"

#include "app_error.h"
#include "app_uart.h"

#include "util.h"
#include "gitversion.h"
#include "pwm.h"
#include "adc.h"
#include "power.h"
#include "db.h"
#include "bldc.h"
#include "sma.h"
#include "freqcntr.h"
#include "cmdline.h"
#include "mechbrake.h"
#include "bleApp.h"
#include "ble_sps.h"
#include "led.h"
#include "commands.h"

extern ble_sps_t m_sps;

static void cmdVersion(const char *args);
static void cmdPWMSet(const char *args);
/* Power management commands */
static void cmdVIn(const char *args);
static void cmdVBat(const char *args);
static void cmdICharge(const char *args);
static void cmdBatShort(const char *args);
static void cmdCharge(const char *args);
static void cmdVBATSW(const char *args);
/* Motor control commands */
static void cmdBLDCAccel(const char *args);
static void cmdBLDCSpeed(const char *args);
static void cmdBLDCStop(const char *args);
static void cmdBLDCRPM(const char *args);
static void cmdBLDCDirRev(const char *args);
static void cmdBLDCSetKP(const char *args);
static void cmdBLDCSetKI(const char *args);
/* Brake commands */
static void cmdSimpleBrake(const char *args);
static void cmdBrake(const char *args);
static void cmdBrakeDirRev(const char *args);
/* SMA commands */
static void cmdSMA(const char *args);
/* LED commands */
static void cmdLED(const char *args);
/* Daughterboard commands */
static void cmdDBReset(const char *args);
static void cmdDBTemp(const char *args);
/* BLE Serial Port Service Testing Commands */
static void cmdBLETx(const char *args);
static void cmdBLERx(const char *args);
static void cmdBLEDiscon(const char *args);
static void cmdBLEAdv(const char *args);

static void cmdJump(const char *args);
static void cmdJumpR(const char *args);
static void cmdSeq(const char *args);
static void cmdProg(const char *args);
static void cmdDelay(const char *args);
static void cmdWobble(const char *args);


// These string are what the command line processes looking for the user to
// type on the serial terminal.
static const char cmdVersionStr[]= "ver";
static const char cmdPWMSetStr[] = "pwmset";
/* Power management commands */
static const char cmdVInStr[] = "vin";
static const char cmdVBatStr[] = "vbat";
static const char cmdIChargeStr[] = "icharge";
static const char cmdBatShortStr[] = "batshort";
static const char cmdChargeStr[] = "charge";
static const char cmdVBATSWStr[] = "vbatsw";
/* Motor control commands */
static const char cmdBLDCAccelStr[] = "bldcaccel";
static const char cmdBLDCSpeedStr[] = "bldcspeed";
static const char cmdBLDCStopStr[] = "bldcstop";
static const char cmdBLDCRPMStr[] = "bldcrpm";
static const char cmdBLDCDirRevStr[] = "bldcdirrev";
static const char cmdBLDCSetKPStr[] = "bldckp";
static const char cmdBLDCSetKIStr[] = "bldcki";
/* SMA commands */
static const char cmdSMAStr[] = "sma";
/* Mechanical brake commands */
static const char cmdSimpleBrakeStr[] = "brake";
static const char cmdBrakeStr[] = "brakeseq";
static const char cmdBrakeDirRevStr[] = "brakedirrev";
/* LED commands */
static const char cmdLEDStr[] = "led";
/* Daughterboard commands */
static const char cmdDBResetStr[] = "dbreset";
static const char cmdDBTempStr[] = "dbtemp";
/* BLE Serial Port Service Testing Commands */
static const char cmdBLETxStr[] = "bletx";
static const char cmdBLERxStr[] = "blerx";
static const char cmdBLEDisconStr[] = "blediscon";
static const char cmdBLEAdvStr[] = "bleadv";
static const char cmdEmptyStr[] = "";

static const char cmdJumpStr[] = "jump";
static const char cmdJumpRStr[] = "jumpr";
static const char cmdSeqStr[] = "seq";
static const char cmdProgStr[] = "prog";
static const char cmdDelayStr[] = "delay";
static const char cmdWobbleStr[] = "wobble";

// This table correlates the command strings above to the actual functions that
// are called when the user types the command into the terminal and presses
// enter.
static cmdFcnPair_t cmdTable[] = {
	{cmdVersionStr, cmdVersion},
	{cmdPWMSetStr, cmdPWMSet},
	/* Power management commands */
	{cmdVInStr, cmdVIn},
	{cmdVBatStr, cmdVBat},
	{cmdIChargeStr, cmdICharge},
	{cmdBatShortStr, cmdBatShort},
	{cmdChargeStr, cmdCharge},
	{cmdVBATSWStr, cmdVBATSW},
	/* Motor control commands */
	{cmdBLDCAccelStr, cmdBLDCAccel},
	{cmdBLDCSpeedStr, cmdBLDCSpeed},
	{cmdBLDCStopStr, cmdBLDCStop},
	{cmdBLDCRPMStr, cmdBLDCRPM},
	{cmdBLDCDirRevStr, cmdBLDCDirRev},
	{cmdBLDCSetKPStr, cmdBLDCSetKP},
	{cmdBLDCSetKIStr, cmdBLDCSetKI},
	/* SMA commands */
	{cmdSMAStr, cmdSMA},
	/* Mechanical brake commands */
	{cmdSimpleBrakeStr, cmdSimpleBrake},
	{cmdBrakeStr, cmdBrake},
	{cmdBrakeDirRevStr, cmdBrakeDirRev},
	/* LED commands */
	{cmdLEDStr, cmdLED},
	/* Daughterboard commands */
	{cmdDBResetStr, cmdDBReset},
	{cmdDBTempStr, cmdDBTemp},
	/* BLE Serial Port Service Testing Commands */
	{cmdBLETxStr, cmdBLETx},
	{cmdBLERxStr, cmdBLERx},
	{cmdBLEDisconStr, cmdBLEDiscon},
	{cmdBLEAdvStr, cmdBLEAdv},
	{cmdJumpStr, cmdJump},
	{cmdJumpRStr, cmdJumpR},
	{cmdSeqStr, cmdSeq},
	{cmdProgStr, cmdProg},
	{cmdDelayStr, cmdDelay},
	{cmdWobbleStr, cmdWobble},
	// Always end the command table with an emptry string and null pointer
	{cmdEmptyStr, NULL}
};

void commands_init() {
	cmdline_loadCmds(cmdTable);
}

void cmdVersion(const char *args) {
	char db_gitVersionStr[64];

	if (!db_getVersion(db_gitVersionStr, sizeof(db_gitVersionStr))) {
		strcpy(db_gitVersionStr, "<unavailable>");
	}

	app_uart_put_string("\r\n");
	app_uart_put_string("MB Firmware: ");
	app_uart_put_string(gitVersionStr);
	app_uart_put_string("\r\n");
	app_uart_put_string("DB Firmware: ");
	app_uart_put_string(db_gitVersionStr);
	app_uart_put_string("\r\n");
	app_uart_put_string("\r\n");
}

void cmdPWMSet(const char *args) {
	unsigned int channel, onPeriod;

	if (sscanf(args, "%u %u", &channel, &onPeriod) != 2) {
		return;
	}

	if (pwm_setOnPeriod(channel, onPeriod)) {
		app_uart_put_string("PWM set successfully\r\n");
	}
}

/*****************************/
/* Power management commands */
/*****************************/

void cmdVIn(const char *args) {
	power_printVIn();
}

void cmdVBat(const char *args) {
	power_printBatteryVoltages();
}

void cmdICharge(const char *args) {
	unsigned int iLimit_mA;
	char str[50];

	if (power_getChargeState() != POWER_CHARGESTATE_MANUAL) {
		app_uart_put_string("Command only allowed when charge state set to Manual\r\n");
		return;
	}

	if (sscanf(args, "%u", &iLimit_mA) == 1) {
		if (power_setChargerCurrentLimit_mA(iLimit_mA)) {
			snprintf(str, sizeof(str), "Charge current limit set to %umA\r\n", iLimit_mA);
			app_uart_put_string(str);
		}
	} else {
		power_printChargeCurrent();
	}
}

void cmdBatShort(const char *args) {
	unsigned int shortBat[4];
	char str[2];
	int i;
	uint8_t discharge;

	if (power_getChargeState() != POWER_CHARGESTATE_MANUAL) {
		app_uart_put_string("Command only allowed when charge state set to Manual\r\n");
		return;
	}

	if (sscanf(args, "%u %u %u %u",
			&shortBat[0], &shortBat[1], &shortBat[2], &shortBat[3]) == 4) {

		/* Check that each of the 4 parameters is either a '0' or a '1'.
		 * Anything else and we return without modifying the discharge
		 * switches. */
		for (i=0; i<=3; i++) {
			if ((shortBat[i] != 0) && (shortBat[i] != 1)) {
				return;
			}
		}

		discharge = (shortBat[3] << 3) | (shortBat[2] << 2) | (shortBat[1] << 1) | (shortBat[0] << 0);
		power_setDischargeSwitches(discharge);
		power_printDischargeSwitchState();
	} else if ((sscanf(args, "%1s", str) == 1) && (str[0] == '?')) {
		power_printDischargeSwitchState();
	}
}

void cmdCharge(const char *args) {
	char str[50];
	int nArgs;

	nArgs = sscanf(args, "%50s", str);

	if (nArgs == -1) {
		/* With no arguments, put the charger into automatic mode. */
		power_setChargeState(POWER_CHARGESTATE_STANDBY);
		app_uart_put_string("Charge state set to Automatic\r\n");
	} else if (nArgs == 1) {
		if (strncmp(str, "off", 3) == 0) {
			power_setChargeState(POWER_CHARGESTATE_OFF);
			app_uart_put_string("Charge state set to Off\r\n");
		} else if (strncmp(str, "manual", 6) == 0) {
			power_setChargeState(POWER_CHARGESTATE_MANUAL);
			app_uart_put_string("Charge state set to Manual\r\n");
		} else if (strncmp(str, "auto", 4) == 0) {
			power_setChargeState(POWER_CHARGESTATE_STANDBY);
			app_uart_put_string("Charge state set to Automatic\r\n");
		} else if (strncmp(str, "force", 5) == 0) {
			power_setChargeState(POWER_CHARGESTATE_PRECHARGE);
			app_uart_put_string("Charge state set to PRECHARGE\r\n");
		} else if (strncmp(str, "discharge", 9) == 0) {
			power_setChargeState(POWER_CHARGESTATE_DISCHARGE);
			app_uart_put_string("Charge state set to Discharge\r\n");
		} else if (strncmp(str, "info", 4) == 0) {
			power_printDebugInfo();
		} else if (strncmp(str, "debug", 5) == 0) {
			if (power_getDebug()) {
				power_setDebug(false);
				app_uart_put_string("Charge debug output disabled\r\n");
			} else {
				power_setDebug(true);
				app_uart_put_string("Charge debug output enabled\r\n");
			}
		}
	}
}

void cmdVBATSW(const char *args) {
	unsigned int enable;

	if (sscanf(args, "%u", &enable) == 1) {
		if (enable == 1) {
			power_setVBATSWState(true);
			app_uart_put_string("VBATSW turned on\r\n");
		} else if (enable == 0) {
			power_setVBATSWState(false);
			app_uart_put_string("VBATSW turned off\r\n");
		}
	} else {
		if (power_getVBATSWState()) {
			app_uart_put_string("VBATSW is on\r\n");
		} else {
			app_uart_put_string("VBATSW is off\r\n");
		}
	}
}

/**************************/
/* Motor control commands */
/**************************/

void cmdBLDCAccel(const char *args) {
	char str[50];
	unsigned int accel_mA, time_ms;

	/* The user must specify 'f' or 'r' an acceleration current, in mA, and a time, in ms */
	if (sscanf(args, "%1s %u %u", str, &accel_mA, &time_ms) != 3) {
		return;
	}

	if (time_ms == 0) {
		return;
	}

	if (str[0] == 'f') {
		bldc_setAccel(accel_mA, time_ms, false);
		app_uart_put_string("Accelerating BLDC motor forward\r\n");
	} else if (str[0] == 'r') {
		bldc_setAccel(accel_mA, time_ms, true);
		app_uart_put_string("Accelerating BLDC motor in reverse\r\n");
	}
}

void cmdBLDCSpeed(const char *args) {
	char str[50];
	unsigned int speed_rpm;

	/* The user must specify 'f', 'r', 'c', or 'b' and a speed, in RPM */
	if (sscanf(args, "%1s %u", str, &speed_rpm) != 2) {
		return;
	}

	/* If the speed parameter is greater than 0, the first argument must either
	 * be 'f' for forward, 'r' for reverse.  If the speed is 0, the first
	 * argument must be 'c' for coast or 'b' for brake. */
	if ((speed_rpm > 0) && (str[0] == 'f')) {
		bldc_setSpeed(speed_rpm, false, false);
		app_uart_put_string("Starting BLDC motor spinning forward\r\n");
	} else if ((speed_rpm > 0) && (str[0] == 'r')) {
		bldc_setSpeed(speed_rpm, true, false);
		app_uart_put_string("Starting BLDC motor spinning in reverse\r\n");
	} else if ((speed_rpm == 0) && (str[0] == 'c')) {
		bldc_setSpeed(0, false, false);
		app_uart_put_string("Stopping BLDC motor without electric brake\r\n");
	} else if ((speed_rpm == 0) && (str[0] == 'b')) {
		bldc_setSpeed(0, false, true);
		app_uart_put_string("Stopping BLDC motor with electric brake\r\n");
	}
}

void cmdBLDCStop(const char *args) {
	char str[50];

	if (sscanf(args, "%1s", str) == 1) {
		if (str[0] == 'b') {
			cmdBLDCSpeed("b 0");
		} else if (str[0] == 'c') {
			cmdBLDCSpeed("c 0");
		}
	} else {
		cmdBLDCSpeed("c 0");
	}
}

void cmdBLDCRPM(const char *args) {
	char str[50];
	uint32_t bldcTacho_Hz;
	uint32_t rpm;

	bldcTacho_Hz = freqcntr_getFreq_Hz();

	rpm = (bldcTacho_Hz * 60) / 42;

	snprintf(str, sizeof(str), "BLDC speed: %lu RPM\r\n", rpm);
	app_uart_put_string(str);
}

void cmdBLDCDirRev(const char *args) {
	unsigned int reverse;

	if (sscanf(args, "%u", &reverse) == 1) {
		if (reverse == 1) {
			bldc_setReverseDirections(true);
			app_uart_put_string("BLDC directions reversed\r\n");
		} else if (reverse == 0) {
			bldc_setReverseDirections(false);
			app_uart_put_string("BLDC directions set to normal\r\n");
		}
	} else {
		if (bldc_getReverseDirections()) {
			app_uart_put_string("BLDC directions are reversed\r\n");
		} else {
			app_uart_put_string("BLDC directions are normal\r\n");
		}
	}
}

void cmdBLDCSetKP(const char *args) {
	int nArgs;
	char str[50];
	int32_t numerator, denominator;

	nArgs = sscanf(args, "%ld %ld", &numerator, &denominator);

	if ((nArgs == -1) || (nArgs == 0)) {
		bldc_getKP(&numerator, &denominator);
		snprintf(str, sizeof(str), "KP is %ld/%ld\r\n", numerator, denominator);
	} else if (nArgs == 2) {
		bldc_setKP(numerator, denominator);
		snprintf(str, sizeof(str), "KP set to %ld/%ld\r\n", numerator, denominator);
	} else {
		return;
	}

	app_uart_put_string(str);
}

void cmdBLDCSetKI(const char *args) {
	int nArgs;
	char str[50];
	int32_t numerator, denominator;

	nArgs = sscanf(args, "%ld %ld", &numerator, &denominator);

	if ((nArgs == -1) || (nArgs == 0)) {
		bldc_getKI(&numerator, &denominator);
		snprintf(str, sizeof(str), "KI is %ld/%ld\r\n", numerator, denominator);
	} else if (nArgs == 2) {
		bldc_setKI(numerator, denominator);
		snprintf(str, sizeof(str), "KI set to %ld/%ld\r\n", numerator, denominator);
	} else {
		return;
	}

	app_uart_put_string(str);
}

/****************/
/* SMA Commands */
/****************/

void cmdSMA(const char *args) {
	int nArgs;
	char str[50];
	unsigned int uintTemp;

	smaState_t smaState;

	nArgs = sscanf(args, "%20s %u", str, &uintTemp);

	if (nArgs == -1) {
		/* With no arguments, we return the current SMA state */
		smaState = sma_getState();

		if (smaState == SMA_STATE_EXTENDED) {
			snprintf(str, sizeof(str), "SMA State: EXTENDED\r\n");
		} else if (smaState == SMA_STATE_RETRACTING) {
			snprintf(str, sizeof(str), "SMA State: RETRACTING\r\n");
		} else if (smaState == SMA_STATE_HOLDING) {
			snprintf(str, sizeof(str), "SMA State: HOLDING\r\n");
		} else if (smaState == SMA_STATE_EXTENDING) {
			snprintf(str, sizeof(str), "SMA State: EXTENDING\r\n");
		} else {
			snprintf(str, sizeof(str), "SMA State: UNKNOWN\r\n");
		}

		app_uart_put_string(str);
		return;
	}


	if (strncmp(str, "retractcurrent", 14) == 0) {
		if (nArgs == 2) {
			if (sma_setRetractCurrent_mA(uintTemp)) {
				snprintf(str, sizeof(str), "SMA retract current set to %umA\r\n", sma_getRetractCurrent_mA());
				app_uart_put_string(str);
			}
		} else {
			snprintf(str, sizeof(str), "SMA retract current is %umA\r\n", sma_getRetractCurrent_mA());
			app_uart_put_string(str);
		}
	} else if (strncmp(str, "retracttime", 11) == 0) {
		if (nArgs == 2) {
			if (sma_setRetractTime_ms(uintTemp)) {
				snprintf(str, sizeof(str), "SMA retract time set to %ums\r\n", sma_getRetractTime_ms());
				app_uart_put_string(str);
			}
		} else {
			snprintf(str, sizeof(str), "SMA retract time is %ums\r\n", sma_getRetractTime_ms());
			app_uart_put_string(str);
		}
	} else if (strncmp(str, "holdcurrent", 11) == 0) {
		if (nArgs == 2) {
			if (sma_setHoldCurrent_mA(uintTemp)) {
				snprintf(str, sizeof(str), "SMA hold current set to %umA\r\n", sma_getHoldCurrent_mA());
				app_uart_put_string(str);
			}
		} else {
			snprintf(str, sizeof(str), "SMA hold current is %umA\r\n", sma_getHoldCurrent_mA());
			app_uart_put_string(str);
		}
	} else if ((nArgs == 2) && (strncmp(str, "retract", 7) == 0)) {
		if (sma_retract(uintTemp, NULL)) {
			snprintf(str, sizeof(str), "SMA retracting with %ums hold time...\r\n", uintTemp);
			app_uart_put_string(str);
		}
	} else if ((nArgs == 1) && (strncmp(str, "extend", 6) == 0)) {
		if (sma_extend(NULL)) {
			app_uart_put_string("SMA extending...\r\n");
		}
	}
}

/*****************************/
/* Mechanical brake commands */
/*****************************/
void cmdSimpleBrake(const char *args) {
	char str[5];
	unsigned int current_mA, time_ms;
	coilCurrentStep_t step;

	if (sscanf(args, "%5s %u %u", str, &current_mA, &time_ms) != 3) {
		return;
	}

	if (strncmp(str, "f", 1) == 0) {
		;
	} else if (strncmp(str, "r", 1) == 0) {
		current_mA = -current_mA;
	} else {
		return;
	}

	step.current_mA = (int16_t)current_mA;
	step.time_ms = (uint16_t)time_ms;

	/* Stop the motor (without electric brake) before actuating the mechanical
	 * brake. */
	bldc_setSpeed(0, false, false);

	if (mechbrake_actuate(1, &step)) {
		app_uart_put_string("Mechanical brake actuated\r\n");
	}

}


void cmdBrake(const char *args) {
	char str[MAX_CMDSTR_LEN];
	char *substr;
	unsigned int stepCount, i;
	coilCurrentStep_t steps[8];
	int current_mA;
	unsigned int time_ms;

	strncpy(str, args, sizeof(str)-1);
	str[sizeof(str)-1] = '\0';

	/* Look for the colon that divides the step count from the first step. */
	if ((substr = strtok(str, ":")) == NULL) {
		return;
	}

	/* Interpret the first token (i.e. text before the colon) to the step
	 * count. */
	if (sscanf(substr, "%u", &stepCount) != 1) {
		return;
	}

	/* We limit ourselves to 8 steps so as to not over-run the steps array. */
	if (stepCount > 8) {
		stepCount = 8;
	}

	/* Each of the promised steps is separated by a semicolon.  Here we
	 * iterate over them, extracting the current and time for each. */
	for (i=0; i<stepCount; i++) {
		/* If we did not find the next token, break out of the for loop before
		 * it completes. */
		if ((substr = strtok(NULL, ";")) == NULL) {
			break;
		}

		/* If we cannot interpret the step as a current and time, something is
		 * wrong with the formatting and we break out of the for loop. */
		if (sscanf(substr, "%d %u", &current_mA, &time_ms) != 2) {
			break;
		}

		steps[i].current_mA = (int16_t)current_mA;
		steps[i].time_ms = (uint16_t)time_ms;
	}

	/* Before naturally terminating the for loop, i will be incremented one
	 * final time so that it will be equal to stepCount if we found all
	 * steps promised by the step count. If i does not match the promised
	 * step count, something is wrong and we return without executing the
	 * brake command. */
	if (i != stepCount) {
		return;
	}

	/* Stop the motor (without electric brake) before actuating the mechanical
	 * brake. */
	bldc_setSpeed(0, false, false);

	if (mechbrake_actuate(stepCount, steps)) {
		app_uart_put_string("Mechanical brake actuated\r\n");
	}
}

void cmdBrakeDirRev(const char *args) {
	unsigned int reverse;

	if (sscanf(args, "%u", &reverse) == 1) {
		if (reverse == 1) {
			mechbrake_setReverseDirections(true);
			app_uart_put_string("Mechanical brake directions reversed\r\n");
		} else if (reverse == 0) {
			mechbrake_setReverseDirections(false);
			app_uart_put_string("Mechanical brake directions set to normal\r\n");
		}
	} else {
		if (mechbrake_getReverseDirections()) {
			app_uart_put_string("Mechanical brake directions are reversed\r\n");
		} else {
			app_uart_put_string("Mechanical brake directions are normal\r\n");
		}
	}
}

/****************/
/* LED Commands */
/****************/
void cmdLED(const char *args) {
	char str[50];
	unsigned int ledNum, ledState;

	if (sscanf(args, "%u %u", &ledNum, &ledState) != 2) {
		return;
	}

	if (led_setState(ledNum, ledState)) {
		snprintf(str, sizeof(str), "Led %u state set to %u\r\n", ledNum, ledState);
		app_uart_put_string(str);
	}
}

/**************************/
/* Daughterboard Commands */
/**************************/
void cmdDBReset(const char *args) {
	db_reset();
	app_uart_put_string("Daughterboard reset\r\n");
}

void cmdDBTemp(const char *args) {
	int16_t temperature_tenthDegC;
	int16_t ip, fp;
	char str[64];

	if (db_getTemp(&temperature_tenthDegC)) {
		ip = trunc((float)temperature_tenthDegC / 10.0);
		fp = (float)temperature_tenthDegC - (float)ip * 10.0;
		if (fp < 0) {
			fp = -fp;
		}

		snprintf(str, sizeof(str), "Daughterboard temperature: %d.%d degC\r\n", ip, fp);
		app_uart_put_string(str);
	} else {
		app_uart_put_string("Daughterboard temperature: <error>\r\n");
	}
}

/****************/
/* BLE Commands */
/****************/

void cmdBLETx(const char *args) {
	char txStr[150];
	char respStr[100];
	uint32_t nChars;

	if (sscanf(args, "%150s", txStr) != 1) {
		return;
	}

	nChars = strlen(txStr);

	if (nChars == 0) {
		return;
	}

	if (ble_sps_put_string(&m_sps, (uint8_t *)txStr)) {
		snprintf(respStr, sizeof(respStr), "Placed %lu-character string in BLE SPS transmit buffer\r\n", nChars);
	} else {
		snprintf(respStr, sizeof(respStr), "Failed to place %lu-character string in BLE SPS transmit buffer\r\n", nChars);
	}
	app_uart_put_string(respStr);
}

void cmdBLERx(const char *args) {
	int nArgs;
	uint32_t nChars;
	uint8_t c;
	char str[150];

	nArgs = sscanf(args, "%lu", &nChars);

	if (nArgs == -1) {
		/* With no arguments, print out entire BLE SPS receive buffer. */
		app_uart_put_string("BLE SPS Receive Buffer (begins on next line):\r\n");
		while (ble_sps_get_char(&m_sps, &c)) {
			app_uart_put(c);
		}
	} else if (nArgs == 1) {
		snprintf(str, sizeof(str), "First %lu characters of the BLE SPS Receive Buffer:\r\n", nChars);
		app_uart_put_string(str);
		while ((nChars-- > 0) && (ble_sps_get_char(&m_sps, &c))) {
			app_uart_put(c);
		}
	}

	app_uart_put_string("\r\n");
}

void cmdBLEDiscon(const char *args) {
	uint32_t err_code;

	if (m_sps.conn_handle == BLE_CONN_HANDLE_INVALID) {
		app_uart_put_string("BLE not connected\r\n");
		return;
	}

	app_uart_put_string("BLE disconnecting\r\n");
	err_code = sd_ble_gap_disconnect(m_sps.conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
	APP_ERROR_CHECK(err_code);
}

void cmdBLEAdv(const char *args) {
	unsigned int binaryFlag;

	if (sscanf(args, "%u", &binaryFlag) == 1) {
		if (binaryFlag == 1) {
			bleApp_setAdvertisingEnabled(true);
			app_uart_put_string("Advertising enabled\r\n");
		} else {
			bleApp_setAdvertisingEnabled(false);
			app_uart_put_string("Advertising disabled\r\n");
		}

	}
}

void cmdJump(const char *args){
	int nArgs;
	unsigned int speed;
	//unsigned int time;

	nArgs = sscanf(args, "%u", &speed);

	char temp[50];
	snprintf(temp, sizeof(temp), "Speed: %u \r\n", speed);
	app_uart_put_string(temp);
	//snprintf(temp, sizeof(temp), "Time: %u \r\n", time);
	//app_uart_put_string(temp);

	if (nArgs == -1) {
		app_uart_put_string("no args");
		//with no arguments, set a default speed and delay
		cmdLine_execCmd("bldcspeed f 5000");
		delay_ms(4000);
		cmdLine_execCmd("brake cw 4000 300");
		delay_ms(1000);
		cmdLine_execCmd("bldcstop");
	}
	else{
		app_uart_put_string("yes args");
		char str[50];
		snprintf(str, sizeof(str), "bldcspeed f %u", speed);
		cmdLine_execCmd(str);
		delay_ms(4000);
		cmdLine_execCmd("brake cw 4000 300");
		delay_ms(1000);
		cmdLine_execCmd("bldcstop");
	}
}

void cmdJumpR(const char *args){
	int nArgs;
	unsigned int speed;
	//unsigned int time;

	nArgs = sscanf(args, "%u", &speed);

	char temp[50];
	snprintf(temp, sizeof(temp), "Speed: %u \r\n", speed);
	app_uart_put_string(temp);
	//snprintf(temp, sizeof(temp), "Time: %u \r\n", time);
	//app_uart_put_string(temp);

	if (nArgs == -1) {
		app_uart_put_string("no args");
		//with no arguments, set a default speed and delay
		cmdLine_execCmd("bldcspeed r 5000");
		delay_ms(4000);
		cmdLine_execCmd("brake ccw 4000 300");
		delay_ms(1000);
		cmdLine_execCmd("bldcstop");
	}
	else{
		app_uart_put_string("yes args");
		char str[50];
		snprintf(str, sizeof(str), "bldcspeed r %u", speed);
		cmdLine_execCmd(str);
		delay_ms(4000);
		cmdLine_execCmd("brake ccw 4000 300");
		delay_ms(1000);
		cmdLine_execCmd("bldcstop");
	}
}

void cmdSeq(const char *args){
	int nArgs;
	unsigned int repeat;
	unsigned int speed;
	//unsigned int time;
	char str[50] = "";

	nArgs = sscanf(args, "%u %u", &speed, &repeat);

	if (nArgs == -1) {
		//with no arguments, set a default speed and delay
		repeat = 2;
	}
	else{
		snprintf(str, sizeof(str), "%u", speed);
	}

	for (int i = 0; i < repeat; ++i) {
		cmdJump(str);
		delay_ms(300);
	}
}

void cmdProg(const char *args){
	//allows the user to send a sequence of commands, including delays, to be executed
	//commands must be separated by semicolons

	char str[MAX_CMDSTR_LEN];

	strncpy(str, args, sizeof(str)-1);
	str[sizeof(str)-1] = '\0';

	app_uart_put_string("Started executing program.\r\n");

	char *token;
	token = strtok(str, ";");

	while(token != NULL){
		if(token[0] == ' ') token++;
		if(cmdLine_execCmd(token))
			app_uart_put_string("Executed command.\r\n");
		else
			app_uart_put_string("Failed to execute command.\r\n");
		token = strtok (NULL, ";");
	}

	app_uart_put_string("Requested program completed.\r\n");
}

void cmdDelay(const char *args){
	int nArgs;
	unsigned int time;

	nArgs = sscanf(args, "%u", &time);

	if(nArgs == -1){
		app_uart_put_string("Default delay, 500 ms\r\n");
		delay_ms(500);
	}
	else{
		char temp[50];
		snprintf(temp, sizeof(temp), "Delay: %u \r\n", time);
		app_uart_put_string(temp);
		delay_ms(time);
	}
}

void cmdWobble(const char *args){
	int nArgs;
	unsigned int repeat = 2;
	unsigned int speed;
	//unsigned int time;
	char str[50] = "";

	nArgs = sscanf(args, "%u %u", &speed, &repeat);

	if (nArgs != -1) {
		snprintf(str, sizeof(str), "%u", speed);
	}

	for (int i = 0; i < repeat; ++i) {
		cmdJump(str);
		delay_ms(300);
		cmdJumpR(str);
		delay_ms(300);
	}
}
