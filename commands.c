/*
 * commands.c
 *
 *  Created on: Nov 20, 2013
 *      Author: kwgilpin
 */


#include <stdio.h>
#include <string.h>

#include "ble_hci.h"
#include "ble_gap.h"

#include "app_error.h"
#include "app_uart.h"

#include "util.h"
#include "pwm.h"
#include "adc.h"
#include "power.h"
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
/* Motor control commands */
static void cmdBLDCSpeed(const char *args);
static void cmdBLDCRun(const char *args);
static void cmdBLDCStop(const char *args);
static void cmdBLDCOff(const char *args);
static void cmdBLDCCurrent(const char *args);
static void cmdBLDCRPM(const char *args);
static void cmdBLDCSetKP(const char *args);
static void cmdBLDCSetKI(const char *args);
/* Brake commands */
static void cmdBrake(const char *args);
/* SMA commands */
static void cmdSMA(const char *args);
/* LED commands */
static void cmdLED(const char *args);
/* BLE Serial Port Service Testing Commands */
static void cmdBLETx(const char *args);
static void cmdBLERx(const char *args);
static void cmdBLEDiscon(const char *args);
static void cmdBLEAdv(const char *args);

static void cmdJump(const char *args);
static void cmdJumpR(const char *args);
static void cmdSeq(const char *args);
static void cmdProg(char *args);
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
/* Motor control commands */
static const char cmdBLDCSpeedStr[] = "bldcspeed";
static const char cmdBLDCRunStr[] = "bldcrun";
static const char cmdBLDCStopStr[] = "bldcstop";
static const char cmdBLDCOffStr[] = "bldcoff";
static const char cmdBLDCCurrentStr[] = "bldci";
static const char cmdBLDCRPMStr[] = "bldcrpm";
static const char cmdBLDCSetKPStr[] = "bldckp";
static const char cmdBLDCSetKIStr[] = "bldcki";
/* SMA commands */
static const char cmdSMAStr[] = "sma";
/* Mechanical brake commands */
static const char cmdBrakeStr[] = "brake";
/* LED commands */
static const char cmdLEDStr[] = "led";
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
	/* Motor control commands */
	{cmdBLDCSpeedStr, cmdBLDCSpeed},
	{cmdBLDCRunStr, cmdBLDCRun},
	{cmdBLDCStopStr, cmdBLDCStop},
	{cmdBLDCOffStr, cmdBLDCOff},
	{cmdBLDCCurrentStr, cmdBLDCCurrent},
	{cmdBLDCRPMStr, cmdBLDCRPM},
	{cmdBLDCSetKPStr, cmdBLDCSetKP},
	{cmdBLDCSetKIStr, cmdBLDCSetKI},
	/* SMA commands */
	{cmdSMAStr, cmdSMA},
	/* Mechanical brake commands */
	{cmdBrakeStr, cmdBrake},
	/* LED commands */
	{cmdLEDStr, cmdLED},
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
	app_uart_put_string("\r\n");
	app_uart_put_string("MBlocks v2.0\r\n");
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

/**************************/
/* Motor control commands */
/**************************/

void cmdBLDCSpeed(const char *args) {
	char str[50];
	unsigned int speed_rpm;
	bool reverse;

	/* The user must specify 'f' or 'r' and a speed, in RPM */
	if (sscanf(args, "%1s %u", str, &speed_rpm) != 2) {
		return;
	}

	/* The first argument must either be 'f' for forward or 'r' for reverse */
	if (str[0] == 'f') {
		reverse = false;
	} else if (str[0] == 'r') {
		reverse = true;
	} else {
		return;
	}

	bldc_setSpeed(speed_rpm, reverse);

	if (!reverse) {
		snprintf(str, sizeof(str), "Starting BLDC motor spinning forward\r\n");
	} else {
		snprintf(str, sizeof(str), "Starting BLDC motor spinning in reverse\r\n");
	}
	app_uart_put_string(str);
}

void cmdBLDCRun(const char *args) {
	char str[50];
	unsigned int iLimit_mA;
	int nArgs;
	bool reverse;

	/* With no arguments, we simply supply power to the A4960 BLDC controller*/
	if ((nArgs = sscanf(args, "%1s %u", str, &iLimit_mA)) == -1) {
		bldc_on();
		snprintf(str, sizeof(str), "BLDC controller turned on\r\n");
		app_uart_put_string(str);
		return;
	}

	/* The first argument must either be 'f' for forward or 'r' for reverse */
	if (str[0] == 'f') {
		reverse = false;
	} else if (str[0] == 'r') {
		reverse = true;
	} else {
		return;
	}

	/* If the A4960 BLDC controller is not already supplied with power, do so
	 * now. */
	if (!bldc_isOn() && bldc_on()) {
		snprintf(str, sizeof(str), "BLDC controller turned on\r\n");
		app_uart_put_string(str);
	} else if (bldc_isOn()) {
		;
	} else {
		snprintf(str, sizeof(str), "Failed to turn BLDC controller on\r\n");
		app_uart_put_string(str);
		return;
	}

	/* The second argument specifies the maximum current limit (as controlled
	 * by the voltage at the REF input to the A4960).  If this argument is
	 * present, set the corresponding voltage level now. */
	if (nArgs == 2) {
		bldc_setMaxCurrent_mA(iLimit_mA);

		snprintf(str, sizeof(str), "BLDC current limit set to %umA\r\n", iLimit_mA);
		app_uart_put_string(str);
	}

	if (!reverse) {
		snprintf(str, sizeof(str), "Starting BLDC motor spinning forward\r\n");
	} else {
		snprintf(str, sizeof(str), "Starting BLDC motor spinning in reverse\r\n");
	}
	app_uart_put_string(str);

	bldc_run(reverse);
}

void cmdBLDCStop(const char *args) {
	char str[50];

	if (sscanf(args, "%1s", str) == 1) {
		if (str[0] == 'b') {
			snprintf(str, sizeof(str), "Stopping BLDC motor with electric brake\r\n");
			app_uart_put_string(str);

			bldc_stop(true);
		} else {
			return;
		}
	} else {
		snprintf(str, sizeof(str), "Stopping BLDC motor without electric brake\r\n");
		app_uart_put_string(str);

		bldc_stop(false);
	}
}

void cmdBLDCOff(const char *args) {
	char str[50];

	bldc_off();

	snprintf(str, sizeof(str), "BLDC controller turned off\r\n");
	app_uart_put_string(str);
}

void cmdBLDCCurrent(const char *args) {
	char str[50];
	unsigned int iLimit_mA;

	if (sscanf(args, "%u", &iLimit_mA) != 1) {
		return;
	}

	bldc_setMaxCurrent_mA(iLimit_mA);

	snprintf(str, sizeof(str), "BLDC current limit set to %umA\r\n", iLimit_mA);
	app_uart_put_string(str);
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
void cmdBrake(const char *args) {
	char str[5];
	unsigned int current_mA, time_ms;
	mechbrake_dir_t dir;

	if (sscanf(args, "%5s %u %u", str, &current_mA, &time_ms) != 3) {
		return;
	}

	if (strncmp(str, "cw", 2) == 0) {
		dir = MECHBRAKE_DIR_CW;
	} else if (strncmp(str, "ccw", 3) == 0) {
		dir = MECHBRAKE_DIR_CCW;
	} else {
		return;
	}

	app_uart_put_string("Brake!");
	mechbrake_actuate(dir, (uint16_t)current_mA, (uint16_t)time_ms);
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

	nArgs = sscanf(args, "%u %u", &speed);

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

	nArgs = sscanf(args, "%u %u", &speed);

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

void cmdProg(char *args){
	//allows the user to send a sequence of commands, including delays, to be executed
	//commands must be separated by semicolons

	app_uart_put_string("Started executing program.\r\n");

	const char delimiters[] = ";";
	char *token;

	token = strtok(args, delimiters);

	while(token != NULL){
		if(token[0] == ' ') token++;
		if(cmdLine_execCmd(token))
			app_uart_put_string("Executed command.\r\n");
		else
			app_uart_put_string("Failed to execute command.\r\n");
		token = strtok (NULL, delimiters);
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
