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
#include "commands.h"
#include "message.h"
#include "lightTracker.h"

extern ble_sps_t m_sps;

static void cmdVersion(const char *args);
static void cmdBootloader(const char *args);
static void cmdPWMSet(const char *args);
/* Power management commands */
static void cmdVIn(const char *args);
static void cmdVBat(const char *args);
static void cmdICharge(const char *args);
static void cmdBatShort(const char *args);
static void cmdCharge(const char *args);
static void cmdVBATSW(const char *args);
static void cmdSleep(const char *args);
static void cmdSleepTime(const char *args);
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
static void cmdDBSleep(const char *args);
static void cmdDBTemp(const char *args);
/* Faceboard commands */
static void cmdFBRGBLED(const char *args);
static void cmdFBLight(const char *args);
static void cmdFBIRManualLEDs(const char *args);
static void cmdFBTx(const char *args);
static void cmdFBMsgTx(const char *args);
static void cmdFBTxCount(const char *args);
static void cmdFBTxLEDs(const char *args);
static void cmdFBRx(const char *args);
static void cmdFBRxCount(const char *args);
static void cmdFBRxFlush(const char *args);
static void cmdFBRxEnable(const char *args);
static void cmdFBSleep(const char *args);
/* Message commands */
static void cmdMsgSendCmd(const char *args);
static void cmdMsgBdcastCmd(const char *args);
/* IMU commands */
static void cmdIMUSelect(const char *args);
static void cmdIMUInit(const char *args);
static void cmdIMUWrite(const char *args);
static void cmdIMURead(const char *args);
static void cmdIMUMotion(const char *args);
static void cmdIMUQuaternion(const char *args);
static void cmdIMUGravity(const char *args);
static void cmdIMUGyros(const char *args);
/* BLE Serial Port Service Testing Commands */
static void cmdBLETx(const char *args);
static void cmdBLERx(const char *args);
static void cmdBLEDiscon(const char *args);
static void cmdBLEAdv(const char *args);
static void cmdBLEMACAddr(const char *args);
/* Motion commands */
static void cmdChangePlane(const char *args);
static void cmdInertialActuation(const char *args);
/* */
static void cmdLightTracker(const char *args);
static void cmdTestCurr(const char *args);

// These string are what the command line processes looking for the user to
// type on the serial terminal.
static const char cmdVersionStr[] = "ver";
static const char cmdBootloaderStr[] = "bootloader";
static const char cmdPWMSetStr[] = "pwmset";
/* Power management commands */
static const char cmdVInStr[] = "vin";
static const char cmdVBatStr[] = "vbat";
static const char cmdIChargeStr[] = "icharge";
static const char cmdBatShortStr[] = "batshort";
static const char cmdChargeStr[] = "charge";
static const char cmdVBATSWStr[] = "vbatsw";
static const char cmdSleepStr[] = "sleep";
static const char cmdSleepTimeStr[] = "sleeptime";
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
static const char cmdDBSleepStr[] = "dbsleep";
static const char cmdDBTempStr[] = "dbtemp";
/* Faceboard commands */
static const char cmdFBRGBLEDStr[] = "fbrgbled";
static const char cmdFBLightStr[] = "fblight";
static const char cmdFBIRManualLEDsStr[] = "fbirled";
static const char cmdFBTxStr[] = "fbtx";
static const char cmdFBMsgTxStr[] = "fbtxmsg";
static const char cmdFBTxCountStr[] = "fbtxcount";
static const char cmdFBTxLEDsStr[] = "fbtxled";
static const char cmdFBRxStr[] = "fbrx";
static const char cmdFBRxCountStr[] = "fbrxcount";
static const char cmdFBRxFlushStr[] = "fbrxflush";
static const char cmdFBRxEnableStr[] = "fbrxen";
static const char cmdFBSleepStr[] = "fbsleep";
/* IMU commands */
static const char cmdIMUSelectStr[] = "imuselect";
static const char cmdIMUInitStr[] = "imuinit";
static const char cmdIMUWriteStr[] = "imuwrite";
static const char cmdIMUReadStr[] = "imuread";
static const char cmdIMUMotionStr[] = "imumotion";
static const char cmdIMUQuaternionStr[] = "imuquat";
static const char cmdIMUGravityStr[] = "imugravity";
static const char cmdIMUGyrosStr[] = "imugyros";
/* BLE Serial Port Service Testing Commands */
static const char cmdBLETxStr[] = "bletx";
static const char cmdBLERxStr[] = "blerx";
static const char cmdBLEDisconStr[] = "blediscon";
static const char cmdBLEAdvStr[] = "bleadv";
static const char cmdBLEMACAddrStr[] = "blemac";
static const char cmdEmptyStr[] = "";
/* Motion commands */
static const char cmdChangePlaneStr[] = "cp";
static const char cmdInertialActuationStr[] = "ia";
/* */
static const char cmdLightTrackerStr[] = "ltrack";
static const char cmdTestCurrStr[] = "tstfun";

// This table correlates the command strings above to the actual functions that
// are called when the user types the command into the terminal and presses
// enter.
static cmdFcnPair_t cmdTable[] = {
		{cmdVersionStr, cmdVersion},
		{cmdBootloaderStr, cmdBootloader},
		{cmdPWMSetStr, cmdPWMSet},
		/* Power management commands */
		{cmdVInStr, cmdVIn},
		{cmdVBatStr, cmdVBat},
		{cmdIChargeStr, cmdICharge},
		{cmdBatShortStr, cmdBatShort},
		{cmdChargeStr, cmdCharge},
		{cmdVBATSWStr, cmdVBATSW},
		{cmdSleepStr, cmdSleep},
		{cmdSleepTimeStr, cmdSleepTime},
		/* Motor control commands */
		{cmdBLDCAccelStr, cmdBLDCAccel},
		{cmdBLDCSpeedStr, cmdBLDCSpeed},
		{cmdBLDCStopStr, cmdBLDCStop},
		{cmdBLDCRPMStr, cmdBLDCRPM},
		{cmdBLDCDirRevStr, cmdBLDCDirRev},
		{cmdBLDCSetKPStr, cmdBLDCSetKP},
		{cmdBLDCSetKIStr, cmdBLDCSetKI},
		/* SMA commands */
		{cmdSMAStr, cmdSMA },
		/* Mechanical brake commands */
		{cmdSimpleBrakeStr, cmdSimpleBrake},
		{cmdBrakeStr, cmdBrake },
		{cmdBrakeDirRevStr, cmdBrakeDirRev},
		/* LED commands */
		{cmdLEDStr, cmdLED },
		/* Daughterboard commands */
		{cmdDBResetStr, cmdDBReset},
		{cmdDBSleepStr, cmdDBSleep},
		{cmdDBTempStr, cmdDBTemp},
		/* Faceboard commands */
		{cmdFBRGBLEDStr, cmdFBRGBLED},
		{cmdFBLightStr, cmdFBLight},
		{cmdFBIRManualLEDsStr, cmdFBIRManualLEDs},
		{cmdFBTxStr, cmdFBTx},
		{cmdFBMsgTxStr, cmdFBMsgTx},
		{cmdFBTxCountStr, cmdFBTxCount},
		{cmdFBTxLEDsStr, cmdFBTxLEDs},
		{cmdFBRxStr, cmdFBRx},
		{cmdFBRxCountStr, cmdFBRxCount},
		{cmdFBRxFlushStr, cmdFBRxFlush},
		{cmdFBRxEnableStr, cmdFBRxEnable},
		{cmdFBSleepStr, cmdFBSleep},
		/* IMU commands */
		{cmdIMUSelectStr, cmdIMUSelect},
		{cmdIMUInitStr, cmdIMUInit},
		{cmdIMUWriteStr, cmdIMUWrite},
		{cmdIMUReadStr, cmdIMURead},
		{cmdIMUMotionStr, cmdIMUMotion},
		{cmdIMUQuaternionStr, cmdIMUQuaternion},
		{cmdIMUGravityStr, cmdIMUGravity},
		{cmdIMUGyrosStr, cmdIMUGyros},
		/* BLE Serial Port Service Testing Commands */
		{cmdBLETxStr, cmdBLETx},
		{cmdBLERxStr, cmdBLERx},
		{cmdBLEDisconStr, cmdBLEDiscon},
		{cmdBLEAdvStr, cmdBLEAdv},
		{cmdBLEMACAddrStr, cmdBLEMACAddr},
		/* Motion commands */
		{cmdChangePlaneStr, cmdChangePlane },
		{cmdInertialActuationStr, cmdInertialActuation },
		/* */
		{cmdLightTrackerStr, cmdLightTracker},
		{cmdTestCurrStr, cmdTestCurr},
// Always end the command table with an emptry string and null pointer
		{ cmdEmptyStr, NULL } };

/* Callbacks for printing status info after command completion */
static void cmdLightTrackerHandler(void *p_event_data, uint16_t event_size);
static void cmdMotionPrimitiveHandler(void *p_event_data, uint16_t event_size);
static void cmdMotionEventHandler(void *p_event_data, uint16_t event_size);

void commands_init() {
	cmdline_loadCmds(cmdTable);
}

void cmdVersion(const char *args) {
	char db_gitVersionStr[64];
	char fb_gitVersionStr[6][64];
	uint8_t faceNum;

	if (!db_getVersion(db_gitVersionStr, sizeof(db_gitVersionStr))) {
		strcpy(db_gitVersionStr, "<unavailable>");
	}

	for (faceNum = 1; faceNum <= 6; faceNum++) {
		if (!fb_getVersion(faceNum, fb_gitVersionStr[faceNum-1], 64)) {
			strcpy(fb_gitVersionStr[faceNum-1], "<unavailable>");
		}
	}

	app_uart_put_string("\r\n");

	app_uart_put_string("MB Firmware: ");
#ifdef DEBUG
	app_uart_put_string(" (Debug)");
#elif defined(RELEASE)
	app_uart_put_string(" (Release)");
#else
	app_uart_put_string(" (Unknown)");
#endif
	app_uart_put_string("\r\n");

	app_uart_put_string("DB Firmware: ");
	app_uart_put_string(db_gitVersionStr);
	app_uart_put_string("\r\n");

	app_uart_put_string("Face 1 Firmware: ");
	app_uart_put_string(fb_gitVersionStr[0]);
	app_uart_put_string("\r\n");

	app_uart_put_string("Face 2 Firmware: ");
	app_uart_put_string(fb_gitVersionStr[1]);
	app_uart_put_string("\r\n");

	app_uart_put_string("Face 3 Firmware: ");
	app_uart_put_string(fb_gitVersionStr[2]);
	app_uart_put_string("\r\n");

	app_uart_put_string("Face 4 Firmware: ");
	app_uart_put_string(fb_gitVersionStr[3]);
	app_uart_put_string("\r\n");

	app_uart_put_string("Face 5 Firmware: ");
	app_uart_put_string(fb_gitVersionStr[4]);
	app_uart_put_string("\r\n");

	app_uart_put_string("Face 6 Firmware: ");
	app_uart_put_string(fb_gitVersionStr[5]);
	app_uart_put_string("\r\n");

	app_uart_put_string("\r\n");
	app_uart_put_string("\r\n");
}

void cmdBootloader(const char *args) {
	app_uart_put_string("Starting bootloader...\r\n");
	nrf_delay_ms(250);

	/* Set the MSb in the general purpose retention register in order to
	 * instruct the bootloader to attempt to download a new image instead
	 * of rebooting back into the application code.  */
	sd_power_gpregret_set(0x80);
	sd_nvic_SystemReset();
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
		app_uart_put_string(
				"Command only allowed when charge state set to Manual\r\n");
		return;
	}

	if (sscanf(args, "%u", &iLimit_mA) == 1) {
		if (power_setChargerCurrentLimit_mA(iLimit_mA)) {
			snprintf(str, sizeof(str), "Charge current limit set to %umA\r\n",
					iLimit_mA);
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
		app_uart_put_string(
				"Command only allowed when charge state set to Manual\r\n");
		return;
	}

	if (sscanf(args, "%u %u %u %u", &shortBat[0], &shortBat[1], &shortBat[2],
			&shortBat[3]) == 4) {

		/* Check that each of the 4 parameters is either a '0' or a '1'.
		 * Anything else and we return without modifying the discharge
		 * switches. */
		for (i = 0; i <= 3; i++) {
			if ((shortBat[i] != 0) && (shortBat[i] != 1)) {
				return;
			}
		}

		discharge = (shortBat[3] << 3) | (shortBat[2] << 2) | (shortBat[1] << 1)
				| (shortBat[0] << 0);
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

void cmdVBATSW(const char *args) {
	unsigned int enable;

	if (sscanf(args, "%u", &enable) == 1) {
		if (enable == 1) {
			power_setVBATSWState(VBATSW_SUPERUSER, true);
			app_uart_put_string("VBATSW turned on\r\n");
		} else if (enable == 0) {
			power_setVBATSWState(VBATSW_SUPERUSER, false);
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

void cmdSleep(const char *args) {
	/* Stop charging because the code in main_powerManage prevents the cube
	 * from entering sleep mode if it is actively charging or discharging. */
	power_setChargeState(POWER_CHARGESTATE_OFF);
	main_setSleepRequested(true);
}

void cmdSleepTime(const char *args) {
	char str[64];
	int nargs;
	unsigned int minutes;
	uint32_t seconds;

	if ((nargs = sscanf(args, "%u", &minutes)) == -1) {
		seconds = main_getSleepTime();
		if (seconds == 0) {
			app_uart_put_string("Sleep timer is disabled\r\n");
		} else {
			snprintf(str, sizeof(str), "Sleep timer is set to %u minutes\r\n",
					(unsigned int) (seconds / 60));
			app_uart_put_string(str);
		}
	} else if (nargs == 1) {
		if (main_setSleepTime(minutes * 60)) {
			if (minutes != 0) {
				snprintf(str, sizeof(str), "Sleep timer set to %u minutes\r\n",
						minutes);
				app_uart_put_string(str);
			} else {
				app_uart_put_string("Sleep timer disabled\r\n");
			}
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
		snprintf(str, sizeof(str), "KP set to %ld/%ld\r\n", numerator,
				denominator);
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
		snprintf(str, sizeof(str), "KI set to %ld/%ld\r\n", numerator,
				denominator);
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

void cmdBrake(const char *args) {
	char str[MAX_CMDSTR_LEN];
	char *substr;
	unsigned int stepCount, i;
	coilCurrentStep_t steps[8];
	int current_mA;
	unsigned int time_ms;

	strncpy(str, args, sizeof(str) - 1);
	str[sizeof(str) - 1] = '\0';

	/* Look for the colon that divides the step count from the first step. */
	if ((substr = strtok(str, ":")) == NULL ) {
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
	for (i = 0; i < stepCount; i++) {
		/* If we did not find the next token, break out of the for loop before
		 * it completes. */
		if ((substr = strtok(NULL, ";")) == NULL ) {
			break;
		}

		/* If we cannot interpret the step as a current and time, something is
		 * wrong with the formatting and we break out of the for loop. */
		if (sscanf(substr, "%d %u", &current_mA, &time_ms) != 2) {
			break;
		}

		steps[i].current_mA = (int16_t) current_mA;
		steps[i].time_ms = (uint16_t) time_ms;
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
	bldc_setSpeed(0, false, 0, NULL );

	if (mechbrake_actuate(stepCount, steps, cmdMotionPrimitiveHandler)) {
		app_uart_put_string("Activating mechanical brake...\r\n");
	} else {
		db_sleep(true);
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
			app_uart_put_string(
					"Mechanical brake directions set to normal\r\n");
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
		snprintf(str, sizeof(str), "Led %u state set to %u\r\n", ledNum,
				ledState);
		app_uart_put_string(str);
	}
}

/**************************/
/* Daughterboard Commands */
/**************************/
void cmdDBReset(const char *args) {
	db_reset();
	app_uart_put_string("Daughterboard and faceboards reset\r\n");
}

void cmdDBSleep(const char *args) {
	unsigned int enableSleep;

	if (sscanf(args, "%u", &enableSleep) != 1) {
		return;
	} else if (enableSleep == 1) {
		if (db_sleep(true)) {
			app_uart_put_string("Daughterboard put to sleep\r\n");
		} else {
			app_uart_put_string("Failed to put daughterboard to sleep\r\n");
		}
	} else if (enableSleep == 0) {
		if (db_sleep(false)) {
			app_uart_put_string("Daughterboard removed from sleep\r\n");
		} else {
			app_uart_put_string(
					"Failed to remove daughterboard from sleep\r\n");
		}
	}
}

void cmdDBTemp(const char *args) {
	int16_t temperature_tenthDegC;
	int16_t ip, fp;
	char str[64];

	if (db_getTemp(&temperature_tenthDegC)) {
		ip = trunc((float) temperature_tenthDegC / 10.0);
		fp = (float) temperature_tenthDegC - (float) ip * 10.0;
		if (fp < 0) {
			fp = -fp;
		}

		snprintf(str, sizeof(str), "Daughterboard temperature: %d.%d degC\r\n",
				ip, fp);
		app_uart_put_string(str);
	} else {
		app_uart_put_string("Daughterboard temperature: <error>\r\n");
	}

	db_sleep(true);
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

	if (sscanf(args, "%u %[^\n]", &faceNum, txData) != 2) {
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

void cmdFBMsgTx(const char *args) {
	unsigned int faceNum;
	unsigned int flashPostTx;
	unsigned int numBytes;
	char txData[256];
	char str[100];

	if (sscanf(args, "%u %u %[^\n]", &faceNum, &flashPostTx, txData) != 2) {
		return;
	}

	if ((faceNum < 1) || (faceNum > 6)) {
		return;
	}
	app_uart_put_string(str);

	numBytes = strlen(txData);

	if (fb_queueToTxBuffer(faceNum, numBytes, (uint8_t *)txData)) {
		snprintf(str, sizeof(str), "Queued %u bytes on faceboard %u\r\n", numBytes, faceNum);
	} else {
		snprintf(str, sizeof(str), "Failed to queue to IR transmit buffer on faceboard %u\r\n", faceNum);
	}
	app_uart_put_string(str);

	if (fb_sendMsgToTxBuffer(faceNum, flashPostTx)) {
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
	uint8_t rxData[256];
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

void cmdIMUWrite(const char *args) {
	unsigned int addr, data;
	char str[64];

	if ((sscanf(args, "%x%s%x", &addr, str, &data) != 3) || (strlen(str) != 1)
			|| (str[0] != ':') || (addr > 0xFF) || (data > 0xFF)) {
		return;
	}

	twi_master_init();

	if (mpu6050_writeReg((uint8_t) addr, (uint8_t) data)) {
		snprintf(str, sizeof(str), "Wrote 0x%02X to %s IMU register at 0x%02X\r\n",
				(uint8_t) data, mpu6050_getName(), (uint8_t) addr);
		app_uart_put_string(str);
	} else {
		snprintf(str, sizeof(str), "Write to %s IMU register failed\r\n", mpu6050_getName());
		app_uart_put_string(str);
	}

	twi_master_deinit();
}

void cmdIMURead(const char *args) {
	unsigned int addr;
	uint8_t data;
	char str[100];

	if ((sscanf(args, "%x", &addr) != 1) || (addr > 0xFF)) {
		return;
	}

	twi_master_init();

	if (mpu6050_readReg((uint8_t)addr, &data)) {
		snprintf(str, sizeof(str),
				"Read 0x%02X from %s IMU register at 0x%02X\r\n",
				data, mpu6050_getName(), (uint8_t) addr);
		app_uart_put_string(str);
	} else {
		snprintf(str, sizeof(str), "Read from %s IMU register failed\r\n", mpu6050_getName());
		app_uart_put_string(str);
	}

	twi_master_deinit();
}

void cmdIMUMotion(const char *args) {
	bool motionDetected;
	char str[100];

	if (imu_checkForMotion(&motionDetected) && motionDetected) {
		snprintf(str, sizeof(str), "Motion detected by %s IMU\r\n", mpu6050_getName());
		app_uart_put_string(str);
	} else {
		snprintf(str, sizeof(str), "Motion not detected by %s IMU\r\n", mpu6050_getName());
		app_uart_put_string(str);
	}
}

void cmdIMUQuaternion(const char *args) {
	char str[100];
	quaternion16_t q16;

	if (!imu_getUnitQuaternion16(&q16)) {
		snprintf(str, sizeof(str), "Failed to read quaternion data from %s IMU\r\n", mpu6050_getName());
		app_uart_put_string(str);
		return;
	}

	snprintf(str, sizeof(str), "Rotation of %s IMU: [%d %d %d %d]\r\n", mpu6050_getName(), q16.w, q16.x, q16.y, q16.z);
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

void cmdIMUGyros(const char *args) {
	char str[64];
	vector16_t v16;

	if (!imu_getGyros16(&v16)) {
		snprintf(str, sizeof(str), "Failed to read gyroscope data from %s IMU\r\n", mpu6050_getName());
		app_uart_put_string(str);
		return;
	}

	snprintf(str, sizeof(str), "Gyros (%s IMU): [%d %d %d]\r\n", mpu6050_getName(), v16.x, v16.y, v16.z);
	app_uart_put_string(str);
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

	if (ble_sps_put_string(&m_sps, (uint8_t *) txStr)) {
		snprintf(respStr, sizeof(respStr),
				"Placed %lu-character string in BLE SPS transmit buffer\r\n",
				nChars);
	} else {
		snprintf(respStr, sizeof(respStr),
				"Failed to place %lu-character string in BLE SPS transmit buffer\r\n",
				nChars);
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
		app_uart_put_string(
				"BLE SPS Receive Buffer (begins on next line):\r\n");
		while (ble_sps_get_char(&m_sps, &c)) {
			app_uart_put(c);
		}
	} else if (nArgs == 1) {
		snprintf(str, sizeof(str),
				"First %lu characters of the BLE SPS Receive Buffer:\r\n",
				nChars);
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

	// fb_setIRTxLEDs(0, false, false, false, false);
	fb_setRxEnable(0, 0);
	for (int faceNum = 1; faceNum <= 6; faceNum++) {
		fb_setBottomLEDs(faceNum, false, false, false);
		fb_setTopLEDs(faceNum, false, false, false);
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
	uint32_t err_code;
	char str[100];

	if (m_sps.conn_handle == BLE_CONN_HANDLE_INVALID) {
		app_uart_put_string("BLE not connected\r\n");
		return;
	}

	ble_gap_addr_t mac_addr;
	err_code = sd_ble_gap_address_get(&mac_addr);
	APP_ERROR_CHECK(err_code);

	snprintf(str, sizeof(str), "%02x:%02x:%02x:%02x:%02x:%02x\r\n", 
		mac_addr.addr[5], mac_addr.addr[4], mac_addr.addr[3],
		mac_addr.addr[2], mac_addr.addr[1], mac_addr.addr[0]);
	app_uart_put_string(str);
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

/*********************/
/*  Complex Commands */
/*********************/
/* */
void cmdLightTracker(const char *args) {
	unsigned int threshold, bldcSpeed_rpm, brakeCurrent_mA, brakeTime_ms;
	sscanf(args, "%u %u %u %u", &bldcSpeed_rpm, &brakeCurrent_mA, &brakeTime_ms, &threshold);

	if (lightTracker_startLightTracker(bldcSpeed_rpm, brakeCurrent_mA, brakeTime_ms,
			threshold, cmdLightTrackerHandler)) {
		app_uart_put_string("Starting light tracker...\r\n");
	}
}

void cmdTestCurr(const char *str) {
	return;
}

/*************/
/* Callbacks */
/*************/
void cmdLightTrackerHandler(void *p_event_data, uint16_t event_size) {
	trackerEvent_t trackerEvent;
	trackerEvent = *(trackerEvent_t *) p_event_data;

	switch (trackerEvent) {
	case LIGHT_TRACKER_EVENT_COMPLETE:
		app_uart_put_string("Successfully tracked light source.\r\n");
		break;
	}
}


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