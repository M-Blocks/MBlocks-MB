/*
 * commands.c
 *
 *  Created on: Nov 20, 2013
 *      Author: kwgilpin
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "nrf_soc.h"
#include "nrf_delay.h"

#include "ble_hci.h"
#include "ble_gap.h"

#include "app_error.h"
#include "app_uart.h"

#include "util.h"
#include "global.h"
#include "gitversion.h"
#include "main.h"
#include "twi_master.h"
#include "pwm.h"
#include "adc.h"
#include "power.h"
#include "db.h"
#include "fb.h"
#include "bldc.h"
#include "sma.h"
#include "freqcntr.h"
#include "cmdline.h"
#include "mechbrake.h"
#include "motionEvent.h"
#include "mpu6050.h"
#include "imu.h"
#include "bleApp.h"
#include "ble_sps.h"
#include "led.h"
#include "parasite.h"

#include "commands.h"

extern ble_sps_t m_sps;

static void cmdVersion(const char *args);
/* Power management commands */
static void cmdVBat(const char *args);
static void cmdCharge(const char *args);
/* Motor control commands */
static void cmdBLDCAccel(const char *args);
static void cmdBLDCSpeed(const char *args);
static void cmdBLDCStop(const char *args);
/* Brake commands */
static void cmdSimpleBrake(const char *args);
static void cmdBrake(const char *args);
/* SMA commands */
static void cmdSMA(const char *args);
/* LED commands */
static void cmdLED(const char *args);
/* Faceboard commands */
static void cmdFBRGBLED(const char *args);
static void cmdFBLight(const char *args);
static void cmdFBIRManualLEDs(const char *args);
static void cmdFBTx(const char *args);
static void cmdFBMsgQueue(const char *args);
static void cmdFBMsgTx(const char *args);
static void cmdFBTxCount(const char *args);
static void cmdFBTxLEDs(const char *args);
static void cmdFBRx(const char *args);
static void cmdFBRxCount(const char *args);
static void cmdFBRxFlush(const char *args);
static void cmdFBRxAmbient(const char *args);
static void cmdFBRxAmbientCount(const char *args);
static void cmdFBRxEnable(const char *args);
static void cmdFBSleep(const char *args);
/* IMU commands */
static void cmdIMUSelect(const char *args);
static void cmdIMUInit(const char *args);
static void cmdIMUGravity(const char *args);
static void cmdIMUGravityInt(const char *args);
static void cmdIMUGravityFloat(const char *args);
/* BLE Serial Port Service Testing Commands */
static void cmdBLEDiscon(const char *args);
static void cmdBLEAdv(const char *args);
static void cmdBLEMACAddr(const char *args);
/* Motion commands */
static void cmdChangePlane(const char *args);
static void cmdInertialActuation(const char *args);
/* Parasite board commands */
static void cmdParasiteOn(const char *args);
static void cmdParasiteOff(const char *args);
static void cmdParasiteReset(const char *args);

// These string are what the command line processes looking for the user to
// type on the serial terminal.
static const char cmdVersionStr[] = "ver";
/* Power management commands */
static const char cmdVBatStr[] = "vbat";
static const char cmdChargeStr[] = "charge";
/* Motor control commands */
static const char cmdBLDCAccelStr[] = "bldcaccel";
static const char cmdBLDCSpeedStr[] = "bldcspeed";
static const char cmdBLDCStopStr[] = "bldcstop";
static const char cmdBLDCRPMStr[] = "bldcrpm";
/* SMA commands */
static const char cmdSMAStr[] = "sma";
/* Mechanical brake commands */
static const char cmdSimpleBrakeStr[] = "brake";
/* LED commands */
static const char cmdLEDStr[] = "led";
/* Faceboard commands */
static const char cmdFBRGBLEDStr[] = "fbrgbled";
static const char cmdFBLightStr[] = "fblight";
static const char cmdFBIRManualLEDsStr[] = "fbirled";
static const char cmdFBTxStr[] = "fbtx";
static const char cmdFBMsgQueueStr[] = "fbtxqueue";
static const char cmdFBMsgTxStr[] = "fbtxmsg";
static const char cmdFBTxCountStr[] = "fbtxcnt";
static const char cmdFBTxLEDsStr[] = "fbtxled";
static const char cmdFBRxStr[] = "fbrx";
static const char cmdFBRxCountStr[] = "fbrxcnt";
static const char cmdFBRxFlushStr[] = "fbrxflush";
static const char cmdFBRxAmbientStr[] = "fbrxamb";
static const char cmdFBRxAmbientCountStr[] = "fbrxambcnt";
static const char cmdFBRxEnableStr[] = "fbrxen";
static const char cmdFBSleepStr[] = "fbsleep";
/* IMU commands */
static const char cmdIMUSelectStr[] = "imuselect";
static const char cmdIMUInitStr[] = "imuinit";
static const char cmdIMUGravityStr[] = "imugravity";
static const char cmdIMUGravityIntStr[] = "imugravityi";
static const char cmdIMUGravityFloatStr[] = "imugravityf";
/* BLE Serial Port Service Testing Commands */
static const char cmdBLEDisconStr[] = "blediscon";
static const char cmdBLEAdvStr[] = "bleadv";
static const char cmdBLEMACAddrStr[] = "blemac";
static const char cmdEmptyStr[] = "";
/* Motion commands */
static const char cmdChangePlaneStr[] = "cp";
static const char cmdInertialActuationStr[] = "ia";
/* Parasite board commands */
static const char cmdParasiteOnStr[] = "espon";
static const char cmdParasiteOffStr[] = "espoff";
static const char cmdParasiteResetStr[] = "esprst";

// This table correlates the command strings above to the actual functions that
// are called when the user types the command into the terminal and presses
// enter.
static cmdFcnPair_t cmdTable[] = {
    {cmdVersionStr, cmdVersion},
    /* Power management commands */
    {cmdVBatStr, cmdVBat},
    {cmdChargeStr, cmdCharge},
    /* Motor control commands */
    {cmdBLDCAccelStr, cmdBLDCAccel},
    {cmdBLDCSpeedStr, cmdBLDCSpeed},
    {cmdBLDCStopStr, cmdBLDCStop},
    /* SMA commands */
    {cmdSMAStr, cmdSMA },
    /* Mechanical brake commands */
    {cmdSimpleBrakeStr, cmdSimpleBrake},
    /* LED commands */
    {cmdLEDStr, cmdLED },
    /* Faceboard commands */
    {cmdFBRGBLEDStr, cmdFBRGBLED},
    {cmdFBLightStr, cmdFBLight},
    {cmdFBIRManualLEDsStr, cmdFBIRManualLEDs},
    {cmdFBTxStr, cmdFBTx},
    {cmdFBMsgQueueStr, cmdFBMsgQueue},
    {cmdFBMsgTxStr, cmdFBMsgTx},
    {cmdFBTxCountStr, cmdFBTxCount},
    {cmdFBTxLEDsStr, cmdFBTxLEDs},
    {cmdFBRxStr, cmdFBRx},
    {cmdFBRxCountStr, cmdFBRxCount},
    {cmdFBRxFlushStr, cmdFBRxFlush},
    {cmdFBRxEnableStr, cmdFBRxEnable},
    {cmdFBRxAmbientCountStr, cmdFBRxAmbientCount},
    {cmdFBRxAmbientStr, cmdFBRxAmbient},
    {cmdFBSleepStr, cmdFBSleep},
    /* IMU commands */
    {cmdIMUSelectStr, cmdIMUSelect},
    {cmdIMUInitStr, cmdIMUInit},
    {cmdIMUGravityStr, cmdIMUGravity},
    {cmdIMUGravityIntStr, cmdIMUGravityInt},
    {cmdIMUGravityFloatStr, cmdIMUGravityFloat},
    /* BLE Serial Port Service Testing Commands */
    {cmdBLEDisconStr, cmdBLEDiscon},
    {cmdBLEAdvStr, cmdBLEAdv},
    {cmdBLEMACAddrStr, cmdBLEMACAddr},
    /* Motion commands */
    {cmdChangePlaneStr, cmdChangePlane },
    {cmdInertialActuationStr, cmdInertialActuation },
    /* Parasite board commands */
    {cmdParasiteOnStr, cmdParasiteOn},
    {cmdParasiteOffStr, cmdParasiteOff},
    {cmdParasiteResetStr, cmdParasiteReset},
// Always end the command table with an emptry string and null pointer
    { cmdEmptyStr, NULL } };

/* Callbacks for printing status info after command completion */
static void cmdMotionPrimitiveHandler(void *p_event_data, uint16_t event_size);
static void cmdMotionEventHandler(void *p_event_data, uint16_t event_size);

void commands_init() {
    cmdline_loadCmds(cmdTable);
}

void cmdVersion(const char *args) {
    char str[64];
    uint8_t faceNum;

    app_uart_put_string("\r\n");
    app_uart_put_string("MB Firmware: ");
    app_uart_put_string(gitVersionLongStr);
#ifdef DEBUG
    app_uart_put_string(" (Debug)");
#elif defined(RELEASE)
    app_uart_put_string(" (Release)");
#else
    app_uart_put_string(" (Unknown)");
#endif
    app_uart_put_string("\r\n");

    app_uart_put_string("DB Firmware: ");
    if (!db_getVersion(str, sizeof(str))) {
	strcpy(str, "<unavailable>");
    }
    app_uart_put_string(str);
    app_uart_put_string("\r\n");
    
    for (faceNum = 1; faceNum <= 6; faceNum++) {
	snprintf(str, sizeof(str), "Face %d Firmware: ", faceNum);
	app_uart_put_string(str);
	if (!fb_getVersion(faceNum, str, 64)) {
	    strcpy(str, "<unavailable>");
	}
	app_uart_put_string(str);
	app_uart_put_string("\r\n");
    }

    app_uart_put_string("\r\n");
    app_uart_put_string("\r\n");
}

/*****************************/
/* Power management commands */
/*****************************/
void cmdVBat(const char *args) {
    power_printBatteryVoltages();
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
	    app_uart_put_string("Charge state set to Precharge\r\n");
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
void cmdBLDCAccel(const char *args) {
    char str[4];
    unsigned int accel_mA, time_ms;

    /* The user must specify 'f' or 'r' an acceleration current, in mA, and a time, in ms */
    if (sscanf(args, "%1s %u %u", str, &accel_mA, &time_ms) != 3) {
	return;
    }

    if (time_ms == 0) {
	return;
    }

    if (str[0] == 'f') {
	if (bldc_setAccel(accel_mA, time_ms, false, cmdMotionPrimitiveHandler)) {
	    app_uart_put_string("Accelerating BLDC motor forward...\r\n");
	} else {
	    app_uart_put_string("Problem accelerating BLDC motor\r\n");
	}
    } else if (str[0] == 'r') {
	if (bldc_setAccel(accel_mA, time_ms, true, cmdMotionPrimitiveHandler)) {
	    app_uart_put_string("Accelerating BLDC motor in reverse...\r\n");
	} else {
	    app_uart_put_string("Problem accelerating BLDC motor\r\n");
	}
    }
}

void cmdBLDCSpeed(const char *args) {
    char str[4];
    unsigned int speed_rpm;

    /* The user must specify 'f', 'r', 'c', or 'b' and a speed, in RPM */
    if (sscanf(args, "%1s %u", str, &speed_rpm) != 2) {
	return;
    }

    /* If the speed parameter is greater than 0, the first argument must either
     * be 'f' for forward, 'r' for reverse.  If the speed is 0, the first
     * argument must be 'c' for coast or 'b' for brake. */
    if ((speed_rpm > 0) && (str[0] == 'f')) {
	if (bldc_setSpeed(speed_rpm, false, 0, cmdMotionPrimitiveHandler)) {
	    app_uart_put_string("Starting BLDC motor spinning forward...\r\n");
	} else {
	    app_uart_put_string("Problem starting BLDC motor\r\n");
	}
    } else if ((speed_rpm > 0) && (str[0] == 'r')) {
	if (bldc_setSpeed(speed_rpm, true, 0, cmdMotionPrimitiveHandler)) {
	    app_uart_put_string("Starting BLDC motor spinning in reverse...\r\n");
	} else {
	    app_uart_put_string("Problem starting BLDC motor\r\n");
	}
    } else if ((speed_rpm == 0) && (str[0] == 'c')) {
	if (bldc_setSpeed(0, false, 0, cmdMotionPrimitiveHandler)) {
	    app_uart_put_string(
		"Stopping BLDC motor without electric brake...\r\n");
	} else {
	    app_uart_put_string("Problem stopping BLDC motor\r\n");
	}
    } else if ((speed_rpm == 0) && (str[0] == 'b')) {
	if (bldc_setSpeed(0, false, BLDC_EBRAKE_COMPLETE_STOP_TIME_MS,
			  cmdMotionPrimitiveHandler)) {
	    app_uart_put_string("Stopping BLDC motor with electric brake...\r\n");
	} else {
	    app_uart_put_string("Problem stopping BLDC motor\r\n");
	}
    }
}

void cmdBLDCStop(const char *args) {
    char str[4];

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

/****************/
/* SMA Commands */
/****************/
void cmdSMA(const char *args) {
    int nArgs;
    char str[32];
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
		snprintf(str, sizeof(str),
			 "SMA retract current set to %umA\r\n",
			 sma_getRetractCurrent_mA());
		app_uart_put_string(str);
	    }
	} else {
	    snprintf(str, sizeof(str), "SMA retract current is %umA\r\n",
		     sma_getRetractCurrent_mA());
	    app_uart_put_string(str);
	}
    } else if (strncmp(str, "retracttime", 11) == 0) {
	if (nArgs == 2) {
	    if (sma_setRetractTime_ms(uintTemp)) {
		snprintf(str, sizeof(str), "SMA retract time set to %ums\r\n",
			 sma_getRetractTime_ms());
		app_uart_put_string(str);
	    }
	} else {
	    snprintf(str, sizeof(str), "SMA retract time is %ums\r\n",
		     sma_getRetractTime_ms());
	    app_uart_put_string(str);
	}
    } else if (strncmp(str, "holdcurrent", 11) == 0) {
	if (nArgs == 2) {
	    if (sma_setHoldCurrent_mA(uintTemp)) {
		snprintf(str, sizeof(str), "SMA hold current set to %umA\r\n",
			 sma_getHoldCurrent_mA());
		app_uart_put_string(str);
	    }
	} else {
	    snprintf(str, sizeof(str), "SMA hold current is %umA\r\n",
		     sma_getHoldCurrent_mA());
	    app_uart_put_string(str);
	}
    } else if ((nArgs == 2) && (strncmp(str, "retract", 7) == 0)) {
	if (sma_retract(uintTemp, cmdMotionPrimitiveHandler)) {
	    snprintf(str, sizeof(str),
		     "SMA retracting with %ums hold time...\r\n", uintTemp);
	    app_uart_put_string(str);
	}
    } else if ((nArgs == 1) && (strncmp(str, "extend", 6) == 0)) {
	if (sma_extend(cmdMotionPrimitiveHandler)) {
	    app_uart_put_string("SMA extending...\r\n");
	}
    }
}

/*****************************/
/* Mechanical brake commands */
/*****************************/
void cmdSimpleBrake(const char *args) {
    char str[5];
    unsigned int params[2];
    uint16_t current_mA, time_ms;
    coilCurrentStep_t step;

    if (sscanf(args, "%5s %u %u", str, &params[0], &params[1]) < 2) {
	return;
    }

    if (strncmp(str, "f", 1) == 0) {
	current_mA = params[0];
	time_ms = params[1];
    } else if (strncmp(str, "r", 1) == 0) {
	current_mA = -params[0];
	time_ms = params[1];
    } else if (strncmp(str, "e", 1) == 0) {
	time_ms = params[0];
	if (bldc_setSpeed(0, false, time_ms, cmdMotionPrimitiveHandler)) {
	    app_uart_put_string("Activating electronic brake...\r\n");
	}
	return;
    } else {
	return;
    }

    step.current_mA = (int16_t) current_mA;
    step.time_ms = (uint16_t) time_ms;

    /* Stop the motor (without electric brake) before actuating the mechanical
     * brake. */
    bldc_setSpeed(0, false, 0, NULL );

    if (mechbrake_actuate(1, &step, cmdMotionPrimitiveHandler)) {
	app_uart_put_string("Activating mechanical brake...\r\n");
    } else {
	db_sleep(true);
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
	snprintf(str, sizeof(str), "Led %u state set to %u\r\n", ledNum,
		 ledState);
	app_uart_put_string(str);
    }
}

/**********************/
/* Faceboard Commands */
/**********************/

void cmdFBRGBLED(const char *args) {
    int nArgs;
    char topBottomStr[3];
    char rgbStr[4];
    unsigned int faces[6];
    bool red, green, blue;
    bool top, bottom;
    uint8_t faceCount;
    uint8_t i;

    /* We expect the command to take a least 3 arguments:
     *  1) a string composed of the characters 'r', 'g', or 'b' (in any order) with a length of 1-3 characters
     *  2) a string composed of the characters 't' and 'b' (with either first) with a length of 1-2 characters
     *  3) a numeric list of faces (e.g. "2 3 6")
     */
    if ((nArgs = sscanf(args, "%3s %2s %u %u %u %u %u %u", rgbStr, topBottomStr, &faces[0], &faces[1], &faces[2], &faces[3], &faces[4], &faces[5])) < 3) {
	return;
    }

    if ((strchr(rgbStr, 'r') != NULL) || (strchr(rgbStr, 'R') != NULL)) {
	red = true;
    } else {
	red = false;
    }

    if ((strchr(rgbStr, 'g') != NULL) || (strchr(rgbStr, 'G') != NULL)) {
	green = true;
    } else {
	green = false;
    }

    if ((strchr(rgbStr, 'b') != NULL) || (strchr(rgbStr, 'B') != NULL)) {
	blue = true;
    } else {
	blue = false;
    }

    if ((strchr(topBottomStr, 't') != NULL) || (strchr(topBottomStr, 'T') != NULL)) {
	top = true;
    } else {
	top = false;
    }

    if ((strchr(topBottomStr, 'b') != NULL) || (strchr(topBottomStr, 'B') != NULL)) {
	bottom = true;
    } else {
	bottom = false;
    }

    /* Return if neither the top of bottom LEDs are specified */
    if (!bottom && !top) {
	return;
    }

    /* The number of faces in the list is the total number of arguments minus
     * the 'rgb' and 'tb' arugments. */
    faceCount = nArgs - 2;

    /* Return without doing anything if we find an invalid face listed */
    for (i = 0; i < faceCount; i++) {
	if ((faces[i] < 1) || (faces[i] > 6)) {
	    return;
	}
    }

    if (top) {
	for (i = 0; i < faceCount; i++) {
	    fb_setTopLEDs(faces[i], red, green, blue);
	}
    }

    if (bottom) {
	for (i = 0; i < faceCount; i++) {
	    fb_setBottomLEDs(faces[i], red, green, blue);
	}
    }

    app_uart_put_string("Faceboard LEDs set\r\n");

    return;
}

void cmdFBLight(const char *args) {
    unsigned int faceNum;
    int16_t ambientLight;
    char str[100];

    if (sscanf(args, "%u", &faceNum) != 1) {
	return;
    }

    if ((faceNum < 1) || (faceNum > 6)) {
	return;
    }

    bool enabled;
    if (fb_getRxEnable(faceNum, &enabled)) {
	if (!enabled) {
	    fb_setRxEnable(faceNum, true);
	    delay_ms(50);
	}
    }

    ambientLight = fb_getAmbientLight(faceNum);
    if (ambientLight >= 0) {
	snprintf(str, sizeof(str), "Faceboard %u ambient light: %d\r\n", faceNum, ambientLight);
    } else {
	snprintf(str, sizeof(str), "Faceboard %u ambient light: <error>\r\n", faceNum);
    }

    app_uart_put_string(str);
    fb_setRxEnable(faceNum, false);
}

void cmdFBIRManualLEDs(const char *args) {
    int nArgs;
    unsigned int faceNum;
    unsigned int leds[4];
    bool ledsOn[4];
    int i;
    bool first;
    char ledsStr[16];
    char str[100];

    if ((nArgs = sscanf(args, "%u %u %u %u %u", &faceNum, &leds[0], &leds[1], &leds[2], &leds[3])) < 1) {
	return;
    }

    if (faceNum > 6) {
	return;
    }

    ledsOn[0] = ledsOn[1] = ledsOn[2] = ledsOn[3] = false;
    for (i = 0; i < nArgs - 1; i++) {
	if ((1 <= leds[i]) && (leds[i] <= 4 )) {
	    ledsOn[leds[i]-1] = true;
	} else {
	    /* Return if we encounter an invalid LED number */
	    return;
	}
    }

    ledsStr[0] = '\0';
    first = true;
    if (ledsOn[0]) {
	strcat(ledsStr, "1");
	first = false;
    }

    if (ledsOn[1]) {
	if (!first) {
	    if (!ledsOn[2] && !ledsOn[3]) {
		strcat(ledsStr, " and ");
	    } else {
		strcat(ledsStr, ", ");
	    }
	}
	strcat(ledsStr, "2");
	first = false;
    }

    if (ledsOn[2]) {
	if (!first) {
	    if (!ledsOn[3]) {
		strcat(ledsStr, " and ");
	    } else {
		strcat(ledsStr, ", ");
	    }
	}
	strcat(ledsStr, "3");
	first = false;
    }

    if (ledsOn[3]) {
	if (!first) {
	    strcat(ledsStr, " and ");
	}
	strcat(ledsStr, "4");
	first = false;
    }

    if (fb_setIRManualLEDs(faceNum, ledsOn[0], ledsOn[1], ledsOn[2], ledsOn[3])) {
	if (!ledsOn[0] && !ledsOn[1] && !ledsOn[2] && !ledsOn[3]) {
	    if (faceNum == 0) {
		snprintf(str, sizeof(str), "Turned off all IR LEDs on all faceboards\r\n");
	    } else {
		snprintf(str, sizeof(str), "Turned off all IR LEDs on faceboard %u\r\n", faceNum);
	    }
	} else {
	    if (faceNum == 0) {
		snprintf(str, sizeof(str), "Turned on IR LED(s) %s (only) on all faceboards\r\n", ledsStr);
	    } else {
		snprintf(str, sizeof(str), "Turned on IR LED(s) %s (only) on faceboard %u\r\n", ledsStr, faceNum);
	    }
	}
    } else {
	snprintf(str, sizeof(str), "Failed to turn IR LED(s) on/off\r\n");
    }


    app_uart_put_string(str);
}
 
void cmdFBTx(const char *args) {
    unsigned int faceNum;
    unsigned int numBytes;
    char txData[256];
    char str[100];

    if (sscanf(args, "%u %[^\t\n]", &faceNum, txData) != 2) {
	return;
    }

    if ((faceNum < 1) || (faceNum > 6)) {
	return;
    }

    numBytes = strlen(txData);

    if (fb_sendToTxBuffer(faceNum, numBytes, (uint8_t *)txData)) {
	snprintf(str, sizeof(str), "Wrote %u bytes to IR transmit buffer on faceboard %u\r\n", numBytes, faceNum);
    } else {
	snprintf(str, sizeof(str), "Failed to write to IR transmit buffer on faceboard %u\r\n", faceNum);
    }

    app_uart_put_string(str);
}

void cmdFBMsgQueue(const char *args) {
    unsigned int faceNum;
    unsigned int numBytes;
    char txData[128];
    char str[100];

    if (sscanf(args, "%u %[^\t\n]", &faceNum, txData) != 2) {
	return;
    }

    if ((faceNum < 1) || (faceNum > 6)) {
	return;
    }
    numBytes = strlen(txData);

    if (fb_queueToTxBuffer(faceNum, numBytes, (uint8_t *)txData)) {
	snprintf(str, sizeof(str), "Queued %u bytes on faceboard %u\r\n", numBytes, faceNum);
    } else {
	snprintf(str, sizeof(str), "Failed to queue to IR transmit buffer on faceboard %u\r\n", faceNum);
    }
    app_uart_put_string(str);
}

void cmdFBMsgTx(const char *args) {
    unsigned int faceNum;
    unsigned int flashPostTx;
    char str[100];

    if (sscanf(args, "%u %u", &faceNum, &flashPostTx) != 2) {
	return;
    }
    
    if (fb_sendMsgToTxBuffer(faceNum, true)) {
	snprintf(str, sizeof(str), "Sent queued message on faceboard %u\r\n", faceNum);
    } else {
	snprintf(str, sizeof(str), "Failed to transmit queued message on faceboard %u\r\n", faceNum);
    }
    app_uart_put_string(str);
}

void cmdFBTxCount(const char *args) {
    unsigned int faceNum;
    uint8_t count;
    char str[100];


    if (sscanf(args, "%u", &faceNum) != 1) {
	return;
    }

    if ((faceNum < 1) || (faceNum > 6)) {
	return;
    }

    if (fb_getTxBufferAvailableCount(faceNum, &count)) {
	snprintf(str, sizeof(str), "Faceboard %u IR transmit buffer bytes available: %u\r\n", faceNum, count);
    } else {
	snprintf(str, sizeof(str), "Failed to get IR transmit buffer bytes available count from faceboard %u\r\n", faceNum);
    }

    app_uart_put_string(str);
}

void cmdFBTxLEDs(const char *args) {
    int nArgs;
    unsigned int faceNum;
    unsigned int leds[4];
    bool ledsOn[4];
    int i;
    bool first;
    char ledsStr[16];
    char str[100];


    if ((nArgs = sscanf(args, "%u %u %u %u %u", &faceNum, &leds[0], &leds[1], &leds[2], &leds[3])) < 1) {
	return;
    }

    if (faceNum > 6) {
	return;
    }

    ledsOn[0] = ledsOn[1] = ledsOn[2] = ledsOn[3] = false;
    for (i = 0; i < nArgs - 1; i++) {
	if ((1 <= leds[i]) && (leds[i] <= 4 )) {
	    ledsOn[leds[i]-1] = true;
	} else {
	    /* Return if we encounter an invalid LED number */
	    return;
	}
    }

    ledsStr[0] = '\0';
    first = true;
    if (ledsOn[0]) {
	strcat(ledsStr, "1");
	first = false;
    }

    if (ledsOn[1]) {
	if (!first) {
	    if (!ledsOn[2] && !ledsOn[3]) {
		strcat(ledsStr, " and ");
	    } else {
		strcat(ledsStr, ", ");
	    }
	}
	strcat(ledsStr, "2");
	first = false;
    }

    if (ledsOn[2]) {
	if (!first) {
	    if (!ledsOn[3]) {
		strcat(ledsStr, " and ");
	    } else {
		strcat(ledsStr, ", ");
	    }
	}
	strcat(ledsStr, "3");
	first = false;
    }

    if (ledsOn[3]) {
	if (!first) {
	    strcat(ledsStr, " and ");
	}
	strcat(ledsStr, "4");
	first = false;
    }

    if (fb_setIRTxLEDs(faceNum, ledsOn[0], ledsOn[1], ledsOn[2], ledsOn[3])) {
	if (!ledsOn[0] && !ledsOn[1] && !ledsOn[2] && !ledsOn[3]) {
	    if (faceNum == 0) {
		snprintf(str, sizeof(str), "Disabled all IR transmit LEDs on all faceboards\r\n");
	    } else {
		snprintf(str, sizeof(str), "Disabled all IR transmit LEDs on faceboard %u\r\n", faceNum);
	    }
	} else {
	    if (faceNum == 0) {
		snprintf(str, sizeof(str), "Enabled IR transmit LED(s) %s (only) on all faceboards\r\n", ledsStr);
	    } else {
		snprintf(str, sizeof(str), "Enabled IR transmit LED(s) %s (only) on faceboard %u\r\n", ledsStr, faceNum);
	    }
	}
    } else {
	snprintf(str, sizeof(str), "Failed to enable/disable IR transmit LED(s)\r\n");
    }


    app_uart_put_string(str);
}

void cmdFBRx(const char *args) {
    unsigned int faceNum;
    unsigned int numBytes;
    uint8_t rxData[128];
    char str[100];


    if (sscanf(args, "%u %u", &faceNum, &numBytes) != 2) {
	return;
    }

    if ((faceNum < 1) || (faceNum > 6)) {
	return;
    }

    if (numBytes == 0) {
	return;
    }

    if (numBytes > sizeof(rxData) - 1) {
	numBytes = sizeof(rxData) - 1;
    }

    rxData[0] = '\0';
    if (fb_receiveFromRxBuffer(faceNum, numBytes, rxData)) {
	rxData[numBytes] = '\0';
	snprintf(str, sizeof(str), "Read %u bytes from IR receive buffer on faceboard %u:\r\n", numBytes, faceNum);
	app_uart_put_string(str);
	app_uart_put_string((char *)rxData);
	app_uart_put_string("\r\n");
    } else {
	snprintf(str, sizeof(str), "Failed to read IR receive buffer on faceboard %u\r\n", faceNum);
	app_uart_put_string(str);
    }
}

void cmdFBRxCount(const char *args) {
    unsigned int faceNum;
    uint8_t count;
    char str[100];

    if (sscanf(args, "%u", &faceNum) != 1) {
	return;
    }

    if ((faceNum < 1) || (faceNum > 6)) {
	return;
    }

    if (fb_getRxBufferConsumedCount(faceNum, &count)) {
	snprintf(str, sizeof(str), "Faceboard %u IR receive buffer bytes consumed: %u\r\n", faceNum, count);
    } else {
	snprintf(str, sizeof(str), "Failed to get IR receive buffer bytes consumed count from faceboard %u\r\n", faceNum);
    }

    app_uart_put_string(str);
}

void cmdFBRxFlush(const char *args) {
    unsigned int faceNum = 0;
    char str[100];

    /* Without any argument, faceNum will retain its default value of 0 so that
     * the command will apply to all Faceboards. */
    sscanf(args, "%u", &faceNum);

    if (faceNum > 6) {
	return;
    }

    if (fb_flushRxBuffer(faceNum)) {
	if (faceNum == 0) {
	    snprintf(str, sizeof(str), "Flushed receive IR receive buffers on all faceboards\r\n");
	} else {
	    snprintf(str, sizeof(str), "Flushed receive IR receive buffer on faceboard %u\r\n", faceNum);
	}
    } else {
	snprintf(str, sizeof(str), "Failed to flush IR receive buffer(s) on faceboard(s)\r\n");
    }

    app_uart_put_string(str);
}

void cmdFBRxAmbient(const char *args) {
    unsigned int faceNum = 0;
    unsigned int numBytes;
    char str[100];

    sscanf(args, "%u %u", &faceNum, &numBytes);
    if (faceNum > 6 || faceNum <= 0) {
	return;
    }

    uint8_t rxData[200];
    if (fb_getRxAmbientBuffer(faceNum, numBytes, rxData)) {
	rxData[numBytes] = '\0';
	snprintf(str, sizeof(str), "Read %u bytes from ambient light buffer on faceboard %u:\r\n", numBytes, faceNum);
	app_uart_put_string(str);
	app_uart_put_string((char *)rxData);
	app_uart_put_string("\r\n");
    } else {
	snprintf(str, sizeof(str), "Failed to read ambient light buffer on faceboard %u\r\n", faceNum);
	app_uart_put_string(str);
    }
}

void cmdFBRxAmbientCount(const char *args) {
    unsigned int faceNum = 0;
    char str[100];

    sscanf(args, "%u", &faceNum);
    if (faceNum > 6 || faceNum <= 0) {
	return;
    }

    uint8_t numBytes;
    if (fb_getRxAmbientBufferConsumedCount(faceNum, &numBytes)) {
	snprintf(str, sizeof(str), "Faceboard %u ambient light buffer bytes consumed: %u\r\n", faceNum, numBytes);
    } else {
	snprintf(str, sizeof(str), "Failed to get ambient light buffer consumed count from faceboard %u\r\n", faceNum);
    }
    app_uart_put_string(str);
}

void cmdFBRxEnable(const char *args) {
    int nArgs;
    unsigned int faceNum;
    unsigned int enable;
    bool enabled;
    char str[100];

    if ((nArgs = sscanf(args, "%u %u", &faceNum, &enable)) < 1) {
	return;
    }

    if (faceNum > 6) {
	return;
    }

    if ((nArgs == 2) && (enable == 1)) {
	if (fb_setRxEnable(faceNum, true)) {
	    if (faceNum == 0) {
		snprintf(str, sizeof(str), "IR receiver and ambient light sensor enabled on all faceboards\r\n");
	    } else {
		snprintf(str, sizeof(str), "IR receiver and ambient light sensor enabled on faceboard %u\r\n", faceNum);
	    }
	} else {
	    snprintf(str, sizeof(str), "Failed to enable IR receiver and ambient light sensor on faceboard(s)\r\n");
	}
    } else if ((nArgs == 2) && (enable == 0)) {
	if (fb_setRxEnable(faceNum, false)) {
	    if (faceNum == 0) {
		snprintf(str, sizeof(str), "IR receiver and ambient light sensor disabled on all faceboards\r\n");
	    } else {
		snprintf(str, sizeof(str), "IR receiver and ambient light sensor disabled on faceboard %u\r\n", faceNum);
	    }
	} else {
	    snprintf(str, sizeof(str), "Failed to disable IR receiver and ambient light sensor on faceboard(s)\r\n");
	}
    } else if ((nArgs == 1) && (faceNum != 0) && fb_getRxEnable(faceNum, &enabled)) {
	if (enabled) {
	    snprintf(str, sizeof(str), "IR receiver and ambient light sensor are enabled on faceboard %u\r\n", faceNum);
	} else {
	    snprintf(str, sizeof(str), "IR receiver and ambient light sensor are disabled on faceboard %u\r\n", faceNum);
	}
    } else {

    }

    app_uart_put_string(str);
}

void cmdFBSleep(const char *args) {
    unsigned int faceNum = 0;
    char str[100];

    sscanf(args, "%u", &faceNum);

    if (faceNum > 6) {
	return;
    }

    fb_sleep(faceNum, true);

    if (faceNum == 0) {
	app_uart_put_string("All faceboards put to sleep\r\n");
    } else {
	snprintf(str, sizeof(str), "Faceboard %u put to sleep\r\n", faceNum);
	app_uart_put_string(str);
    }
}

/****************/
/* IMU commands */
/****************/

void cmdIMUSelect(const char *args) {
    int nArgs;
    char imuStr[2];
    char str[100];

    nArgs = sscanf(args, "%1s", imuStr);

    if (nArgs == 1) {
	if ((imuStr[0] == 'c') || (imuStr[0] == 'C')) {
	    mpu6050_setAddress(MPU6050_I2C_ADDR_CENTRAL);
	} else if ((imuStr[0] == 'f') || (imuStr[0] == 'F')) {
	    mpu6050_setAddress(MPU6050_I2C_ADDR_FACE);
	}
    }

    snprintf(str, sizeof(str), "Active IMU: %s\r\n", mpu6050_getName());
    app_uart_put_string(str);
}

void cmdIMUInit(const char *args) {
    bool mpu6050Initialized;
    bool dmpInitialized;
    char str[100];

    snprintf(str, sizeof(str), "Re-initializing %s IMU...\r\n", mpu6050_getName());
    app_uart_put_string(str);

    mpu6050Initialized = imu_init();
    dmpInitialized = imu_initDMP();

    if (mpu6050Initialized) {
	app_uart_put_string("  MPU-6050: OK\r\n");
    } else {
    	app_uart_put_string("  MPU-6050: Fail\r\n");
    }

    if (dmpInitialized) {
    	app_uart_put_string("  DMP: OK\r\n");
    } else {
    	app_uart_put_string("  DMP: Fail\r\n");
    }
}

void cmdIMUGravityInt(const char *args) {
  char str[200];
  vector16_t v16;

  if (!imu_getGravity16(&v16)) {
    snprintf(str, sizeof(str), "FAILED\r\n");
    app_uart_put_string(str);
    return;
  }

  snprintf(str, sizeof(str), "%d %d %d\r\n", v16.x, v16.y, v16.z);
  app_uart_put_string(str);
}

void cmdIMUGravityFloat(const char *args) {
  char str[200];
  vectorFloat_t gravity;

  if (!imu_getGravityFloat(&gravity)) {
    snprintf(str, sizeof(str), "FAILED\r\n");
    app_uart_put_string(str);
    return;
  }

  snprintf(str, sizeof(str, "%f %f %f\r\n"), gravity.x, gravity.y, gravity.z);
  app_uart_put_string(str);
}

void cmdIMUGravity(const char *args) {
    char str[200];
    vector16_t v16;

    if (!imu_getGravity16(&v16)) {
	snprintf(str, sizeof(str), "Failed to read gravity vector from %s IMU\r\n", mpu6050_getName());
	app_uart_put_string(str);
	return;
    }

    snprintf(str, sizeof(str), "Gravity vector (int) of %s IMU: [%d %d %d]\r\n", mpu6050_getName(), v16.x, v16.y, v16.z);
    app_uart_put_string(str);

#if (1)
    uint8_t i;
    vectorFloat_t gravity;
    float angleDiff;

    if (!imu_getGravityFloat(&gravity)) {
	return;
    }

    snprintf(str, sizeof(str), "Gravity vector (float) of %s IMU: [%f %f %f]\r\n", mpu6050_getName(), gravity.x, gravity.y, gravity.z);
    app_uart_put_string(str);

    for (i=0; i<3; i++) {
	angleDiff = fabs(imu_getVectorFloatAngle(&gravity, &frameAlignmentVectorsFloat[i]));

	snprintf(str, sizeof(str), "Angle between gravity vector (%s IMU) and [%f %f %f]: %f degrees\r\n",
		 mpu6050_getName(),
		 frameAlignmentVectorsFloat[i].x, frameAlignmentVectorsFloat[i].y, frameAlignmentVectorsFloat[i].z,
		 angleDiff);
	app_uart_put_string(str);
    }
#endif
}

/****************/
/* BLE Commands */
/****************/

void cmdBLEDiscon(const char *args) {
    uint32_t err_code;

    if (m_sps.conn_handle == BLE_CONN_HANDLE_INVALID) {
	app_uart_put_string("BLE not connected\r\n");
	return;
    }

    app_uart_put_string("BLE disconnecting\r\n");
    err_code = sd_ble_gap_disconnect(m_sps.conn_handle,
				     BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
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

void cmdBLEMACAddr(const char *args) {
    char str[100];
    MACaddress(str);
    app_uart_put_string(str);
    app_uart_put_string("\r\n");
}

/*******************/
/* Motion commands */
/*******************/
void cmdChangePlane(const char *args) {
    int nArgs;
    char modeStr[2];
    char dirStr[2];
    int params[4];
    uint16_t accelCurrent_mA, accelTime_ms;
    uint16_t speed_rpm, ebrakeTime_ms;
    uint16_t coastTime_ms;
    bool reverse;

    if ((nArgs = sscanf(args, "%1s %1s %d %d %d %d", modeStr, dirStr, &params[0], &params[1], &params[2], &params[3])) < 4) {
	return;
    }

    if (dirStr[0] == 'f') {
	reverse = false;
    } else if (dirStr[0] == 'r') {
	reverse = true;
    } else {
	return;
    }

    if ((modeStr[0] == 'a') && (params[0] > 0) && (params[1] > 0)) {
	accelCurrent_mA = params[0];
	accelTime_ms = params[1];
	if (motionEvent_startAccelPlaneChange(accelCurrent_mA, accelTime_ms, reverse,
					      cmdMotionEventHandler)) {
	    app_uart_put_string("Starting acceleration based plane change...\r\n");
	}
    } else if ((modeStr[0] == 'b') && (params[0] > 0) && (params[1] > 0)) {
	speed_rpm = params[0];
	ebrakeTime_ms = params[1];

	if ((nArgs >= 5) && (params[2] > 0)) {
	    accelCurrent_mA = params[2];
	} else {
	    accelCurrent_mA = 0;
	}

	if ((nArgs >= 6) && (params[3] > 0)) {
	    accelTime_ms = params[3];
	} else {
	    accelTime_ms = 0;
	}

	if (motionEvent_startEBrakePlaneChange(speed_rpm, ebrakeTime_ms, accelCurrent_mA, accelTime_ms, reverse,
					       cmdMotionEventHandler)) {
	    app_uart_put_string("Starting e-brake based plane change...\r\n");
	}
    } else if ((modeStr[0] == 'm') && (nArgs == 6) && (params[0] > 0) && (params[1] > 0) && (params[2] > 0) && (params[3] > 0)) {
	accelCurrent_mA = params[0];
	accelTime_ms = params[1];
	coastTime_ms = params[2];
	ebrakeTime_ms = params[3];
	if (motionEvent_startAccelBrakePlaneChange(accelCurrent_mA, accelTime_ms, coastTime_ms, ebrakeTime_ms,
						   reverse, cmdMotionEventHandler)) {
	    app_uart_put_string("Starting mixed accel/e-brake based plane change...\r\n");
	}
    }
}

void cmdInertialActuation(const char *args) {
    int nArg;
    char dirStr[3];
    char eBrakeAccelStr[3];
    char accelReverseStr[3];
    unsigned int bldcSpeed_rpm, brakeCurrent_mA, brakeTime_ms;
    unsigned int eBrakeAccelStartDelay_ms = 0;
    bool reverse;
    bool eBrake, accel;
    bool accelReverse = false;

    if ((nArg = sscanf(args, "%1s %u %u %u %1s %u %1s", dirStr, &bldcSpeed_rpm,
		       &brakeCurrent_mA, &brakeTime_ms,
		       eBrakeAccelStr, &eBrakeAccelStartDelay_ms, accelReverseStr)) < 4) {
	return;
    }

    if (dirStr[0] == 'f') {
	reverse = false;
    } else if (dirStr[0] == 'r') {
	reverse = true;
    } else {
	return;
    }

    if (nArg >= 6) {
	if (eBrakeAccelStr[0] == 'e') {
	    eBrake = true;
	    accel = false;
	} else if (eBrakeAccelStr[0] == 'a') {
	    eBrake = false;
	    accel = true;

	    if ((nArg == 7) && (accelReverseStr[0] == 'r')) {
		accelReverse = true;
	    } else {
		accelReverse = false;
	    }
	} else {
	    eBrake = false;
	    accel = false;
	}
    } else {
	eBrake = false;
	accel = false;
    }

    if (motionEvent_startInertialActuation(bldcSpeed_rpm, brakeCurrent_mA,
					   brakeTime_ms, reverse, eBrake, accel, eBrakeAccelStartDelay_ms, accelReverse, cmdMotionEventHandler)) {
	app_uart_put_string("Starting inertial actuation...\r\n");
    }
}

/****************************/
/*  Parasite board commands */
/****************************/

void cmdParasiteOn(const char *args) {
  app_uart_put_string("Turning on ESP board.\r\n");
  parasite_turnon();
  delay_ms(70);
}

void cmdParasiteOff(const char *args) {
  app_uart_put_string("Turning off ESP board.\r\n");
  parasite_turnoff();
  delay_ms(70);
}

void cmdParasiteReset(const char *args) {
  app_uart_put_string("Resetting ESP board.\r\n");
  parasite_reset();
  delay_ms(500);
}

/*************/
/* Callbacks */
/*************/

void cmdMotionPrimitiveHandler(void *p_event_data, uint16_t event_size) {
    motionPrimitive_t motionPrimitive;

    motionPrimitive = *(motionPrimitive_t *) p_event_data;

    switch (motionPrimitive) {
    case MOTION_PRIMITIVE_MECHBRAKE_SUCCESS:
	app_uart_put_string("Successfully actuated mechanical brake\r\n");
	db_sleep(true);
	break;
    case MOTION_PRIMITIVE_MECHBRAKE_FAILURE:
	app_uart_put_string("Failed to actuate mechanical brake\r\n");
	db_sleep(true);
	break;
    case MOTION_PRIMITIVE_MECHBRAKE_TIMEOUT:
	app_uart_put_string("Timeout while actuating mechanical brake\r\n");
	db_sleep(true);
	break;
    case MOTION_PRIMITIVE_SMA_RETRACTED:
	app_uart_put_string("SMA retracted\r\n");
	break;
    case MOTION_PRIMITIVE_SMA_EXTENDING:
	app_uart_put_string("SMA extending\r\n");
	break;
    case MOTION_PRIMITIVE_SMA_EXTENDED:
	app_uart_put_string("SMA extended\r\n");
	break;
    case MOTION_PRIMITIVE_BLDC_STABLE:
	app_uart_put_string("BLDC motor speed stabilized\r\n");
	break;
    case MOTION_PRIMITIVE_BLDC_TIMEOUT:
	app_uart_put_string("BLDC motor timeout\r\n");
	break;
    case MOTION_PRIMITIVE_BLDC_ACCEL_COMPLETE:
	app_uart_put_string("BLDC motor acceleration complete\r\n");
	break;
    case MOTION_PRIMITIVE_BLDC_COASTING:
	app_uart_put_string("BLDC motor coasting to stop\r\n");
	break;
    case MOTION_PRIMITIVE_BLDC_STOPPED:
	app_uart_put_string("BLDC motor stopped\r\n");
	break;
    default:
	app_uart_put_string("Motion primitive unrecognized\r\n");
    }
}

void cmdMotionEventHandler(void *p_event_data, uint16_t event_size) {
    motionEvent_t motionEvent;

    motionEvent = *(motionEvent_t *) p_event_data;

    switch (motionEvent) {
    case MOTION_EVENT_PLANE_CHANGE_SUCCESS:
	app_uart_put_string("Successfully changed planes\r\n");
	break;
    case MOTION_EVENT_PLANE_CHANGE_FAILURE:
	app_uart_put_string("Failed to change planes\r\n");
	break;
    case MOTION_EVENT_INERTIAL_ACTUATION_COMPLETE:
	app_uart_put_string("Inertial actuation complete\r\n");
	break;
    case MOTION_EVENT_INERTIAL_ACTUATION_FAILURE:
	app_uart_put_string("Inertial actuation failure\r\n");
	break;
    default:
	app_uart_put_string("Motion event not recognized\r\n");
    }
}
