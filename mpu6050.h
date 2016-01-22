/*
 * mpu6050.h
 *
 *  Created on: Mar 27, 2014
 *      Author: kwgilpin
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#include "nrf_drv_twi.h"

#define MPU6050_AUX_VDDIO_REG_ADDR				0X00
#define MPU6050_YG_OFFS_TC_REG_ADDR				0X01
#define MPU6050_ZG_OFFS_TC_REG_ADDR				0X02
#define MPU6050_X_FINE_GAIN_REG_ADDR			0X03
#define MPU6050_Y_FINE_GAIN_REG_ADDR			0X04
#define MPU6050_Z_FINE_GAIN_REG_ADDR			0X05
#define MPU6050_XA_OFFS_H_REG_ADDR				0X06
#define MPU6050_XA_OFFS_L_TC_REG_ADDR			0X07
#define MPU6050_YA_OFFS_H_REG_ADDR				0X08
#define MPU6050_YA_OFFS_L_LT_REG_ADDR			0X09
#define MPU6050_ZA_OFFS_H_REG_ADDR				0X0A
#define MPU6050_ZA_OFFS_L_LT_REG_ADDR			0X0B
#define MPU6050_XG_OFFS_USRH_REG_ADDR			0X13
#define MPU6050_XG_OFFS_USRL_REG_ADDR			0X14
#define MPU6050_YG_OFFS_USRH_REG_ADDR			0X15
#define MPU6050_YG_OFFS_USRL_REG_ADDR			0X16
#define MPU6050_ZG_OFFS_USRH_REG_ADDR			0X17
#define MPU6050_ZG_OFFS_USRL_REG_ADDR			0X18
#define MPU6050_SMPLRT_DIV_REG_ADDR				0X19
#define MPU6050_CONFIG_REG_ADDR					0X1A
#define MPU6050_GYRO_CONFIG_REG_ADDR			0X1B
#define MPU6050_ACCEL_CONFIG_REG_ADDR			0X1C
#define MPU6050_FF_THR_REG_ADDR					0X1D
#define MPU6050_FF_DUR_REG_ADDR					0X1E
#define MPU6050_MOT_THR_REG_ADDR				0X1F
#define MPU6050_MOT_DUR_REG_ADDR				0X20
#define MPU6050_ZRMOT_THR_REG_ADDR				0X21
#define MPU6050_ZRMOT_DUR_REG_ADDR				0X22
#define MPU6050_FIFO_EN_REG_ADDR				0X23
#define MPU6050_I2C_MST_CTRL_REG_ADDR			0X24
#define MPU6050_I2C_SLV0_ADDR_REG_ADDR			0X25
#define MPU6050_I2C_SLV0_REG_REG_ADDR			0X26
#define MPU6050_I2C_SLV0_CTRL_REG_ADDR			0x27
#define MPU6050_I2C_SLV1_ADDR_REG_ADDR			0X28
#define MPU6050_I2C_SLV1_REG_REG_ADDR			0X29
#define MPU6050_I2C_SLV1_CTRL_REG_ADDR			0x2A
#define MPU6050_I2C_SLV2_ADDR_REG_ADDR			0X2B
#define MPU6050_I2C_SLV2_REG_REG_ADDR			0X2C
#define MPU6050_I2C_SLV2_CTRL_REG_ADDR			0x2D
#define MPU6050_I2C_SLV3_ADDR_REG_ADDR			0X2E
#define MPU6050_I2C_SLV3_REG_REG_ADDR			0X2F
#define MPU6050_I2C_SLV3_CTRL_REG_ADDR			0x30
#define MPU6050_I2C_SLV4_ADDR_REG_ADDR			0X31
#define MPU6050_I2C_SLV4_REG_REG_ADDR			0X32
#define MPU6050_I2C_SLV4_DO_REG_ADDR			0x33
#define MPU6050_I2C_SLV4_CTRL_REG_ADDR			0x24
#define MPU6050_I2C_SLV4_DI_REG_ADDR			0x35
#define MPU6050_I2C_MST_STATUS_REG_ADDR			0X36
#define MPU6050_INT_PIN_CFG_REG_ADDR			0X37
#define MPU6050_INT_ENABLE_REG_ADDR				0X38
#define MPU6050_DMP_INT_STATUS_REG_ADDR			0X39
#define MPU6050_INT_STATUS_REG_ADDR				0X3A
#define MPU6050_ACCEL_XOUT_H_REG_ADDR			0X3B
#define MPU6050_ACCEL_XOUT_L_REG_ADDR			0X3C
#define MPU6050_ACCEL_YOUT_H_REG_ADDR			0X3D
#define MPU6050_ACCEL_YOUT_L_REG_ADDR			0X3E
#define MPU6050_ACCEL_ZOUT_H_REG_ADDR			0X3F
#define MPU6050_ACCEL_ZOUT_L_REG_ADDR			0X40
#define MPU6050_TEMP_OUT_H_REG_ADDR				0X41
#define MPU6050_TEMP_OUT_L_REG_ADDR				0X42
#define MPU6050_GYRO_XOUT_H_REG_ADDR			0X43
#define MPU6050_GYRO_XOUT_L_REG_ADDR			0X44
#define MPU6050_GYRO_YOUT_H_REG_ADDR			0X45
#define MPU6050_GYRO_YOUT_L_REG_ADDR			0X46
#define MPU6050_GYRO_ZOUT_H_REG_ADDR			0X47
#define MPU6050_GYRO_ZOUT_L_REG_ADDR			0X48
#define MPU6050_EXT_SENS_DATA_00_REG_ADDR		0x49
#define MPU6050_EXT_SENS_DATA_01_REG_ADDR		0x4A
#define MPU6050_EXT_SENS_DATA_02_REG_ADDR		0x4B
#define MPU6050_EXT_SENS_DATA_03_REG_ADDR		0x4C
#define MPU6050_EXT_SENS_DATA_04_REG_ADDR		0x4D
#define MPU6050_EXT_SENS_DATA_05_REG_ADDR		0x4E
#define MPU6050_EXT_SENS_DATA_06_REG_ADDR		0x4F
#define MPU6050_EXT_SENS_DATA_07_REG_ADDR		0x50
#define MPU6050_EXT_SENS_DATA_08_REG_ADDR		0x51
#define MPU6050_EXT_SENS_DATA_09_REG_ADDR		0x52
#define MPU6050_EXT_SENS_DATA_10_REG_ADDR		0x53
#define MPU6050_EXT_SENS_DATA_11_REG_ADDR		0x54
#define MPU6050_EXT_SENS_DATA_12_REG_ADDR		0x55
#define MPU6050_EXT_SENS_DATA_13_REG_ADDR		0x56
#define MPU6050_EXT_SENS_DATA_14_REG_ADDR		0x57
#define MPU6050_EXT_SENS_DATA_15_REG_ADDR		0x58
#define MPU6050_EXT_SENS_DATA_16_REG_ADDR		0x59
#define MPU6050_EXT_SENS_DATA_17_REG_ADDR		0x5A
#define MPU6050_EXT_SENS_DATA_18_REG_ADDR		0x5B
#define MPU6050_EXT_SENS_DATA_19_REG_ADDR		0x5C
#define MPU6050_EXT_SENS_DATA_20_REG_ADDR		0x5D
#define MPU6050_EXT_SENS_DATA_21_REG_ADDR		0x5E
#define MPU6050_EXT_SENS_DATA_22_REG_ADDR		0x5F
#define MPU6050_EXT_SENS_DATA_23_REG_ADDR		0x60
#define MPU6050_MOT_DETECT_STATUS_REG_ADDR		0X61
#define MPU6050_I2C_SLV0_DO_REG_ADDR			0X63
#define MPU6050_I2C_SLV1_DO_REG_ADDR			0X64
#define MPU6050_I2C_SLV2_DO_REG_ADDR			0X65
#define MPU6050_I2C_SLV3_DO_REG_ADDR			0X66
#define MPU6050_I2C_MST_DELAY_CTRL_REG_ADDR		0X67
#define MPU6050_SIGNAL_PATH_RESET_REG_ADDR		0X68
#define MPU6050_MOT_DETECT_CTRL_REG_ADDR		0X69
#define MPU6050_USER_CTRL_REG_ADDR				0X6A
#define MPU6050_PWR_MGMT_1_REG_ADDR				0X6B
#define MPU6050_PWR_MGMT_2_REG_ADDR				0X6C
#define MPU6050_BANK_SEL_REG_ADDR				0X6D
#define MPU6050_MEM_START_ADDR_REG_ADDR			0X6E
#define MPU6050_MEM_R_W_REG_ADDR				0X6F
#define MPU6050_DMP_CFG_1_REG_ADDR				0X70
#define MPU6050_DMP_CFG_2_REG_ADDR				0X71
#define MPU6050_FIFO_COUNTH_REG_ADDR			0X72
#define MPU6050_FIFO_COUNTL_REG_ADDR			0X73
#define MPU6050_FIFO_R_W_REG_ADDR				0X74
#define MPU6050_WHO_AM_I_REG_ADDR				0X75

/* AUX_VDDIO register (0x00) bit definitions */
#define MPU6050_OTP_BANK_VLD_POSN				0
#define MPU6050_OTP_BANK_VLD_MASK				(0x01 << MPU6050_OTP_BANK_VLD_POSN)
#define MPU6050_XG_OFFS_TC_POSN					1
#define MPU6050_XG_OFFS_TC_MASK					(0x3F << MPU6050_XG_OFFS_TC_POSN)
#define MPU6050_AUX_VDDIO_POSN					7
#define MPU6050_AXU_VDDIO_MASK					(0x01 << MPU6050_AUX_VDDIO_POSN)

/* YG_OFFS_TC register (0x01) bit definitions */
#define MPU6050_YG_OFFS_TC_POSN					1
#define MPU6050_YG_OFFS_TC_MASK					(0x3F << MPU6050_YG_OFFS_TC_POSN)

/* ZG_OFFS_TC register (0x02) bit definitions */
#define MPU6050_ZG_OFFS_TC_POSN					1
#define MPU6050_ZG_OFFS_TC_MASK					(0x3F << MPU6050_ZG_OFFS_TC_POSN)

/* CONFIG register (0x1A) bit definitions */
#define MPU6050_DLPF_CFG_POSN					0
#define MPU6050_DLPF_CFG_MASK					(0x07 << MPU6050_DLPF_CFG_POSN)
#define MPU6050_EXT_SYNC_SET_POSN				3
#define MPU6050_EXT_SYNC_SET_MASK				(0x07 << MPU6050_EXT_SYNC_SET_POSN)

/* DLPF_CFG field (bits 2:0 of the CONFIG register) definitions */
#define MPU6050_DLPF_CFG_260HZ_256HZ			0
#define MPU6050_DLPF_CFG_184HZ_188HZ			1
#define MPU6050_DLPF_CFG_94HZ_98HZ				2
#define MPU6050_DLPF_CFG_44HZ_42HZ				3
#define MPU6050_DLPF_CFG_21HZ_20HZ				4
#define MPU6050_DLPF_CFG_10HZ_10HZ				5
#define MPU6050_DLPF_CFG_5HZ_5ZHZ				6

/* EXT_SYNC_SET field (bits 5:3 of the CONFIG register) definitions */
#define MPU6050_EXT_SYNC_SET_DISABLED			0
#define MPU6050_EXT_SYNC_SET_TEMP_OUT_L			1
#define MPU6050_EXT_SYNC_SET_GYRO_XOUT_L		2
#define MPU6050_EXT_SYNC_SET_GYRO_YOUT_L		3
#define MPU6050_EXT_SYNC_SET_GYRO_ZOUT_L		4
#define MPU6050_EXT_SYNC_SET_ACCEL_XOUT_L		5
#define MPU6050_EXT_SYNC_SET_ACCEL_YOUT_L		6
#define MPU6050_EXT_SYNC_SET_ACCEL_ZOUT_L		7

/* GYRO_CONFIG register (0x1B) bit definitions */
#define MPU6050_FS_SEL_POSN						3
#define MPU6050_FS_SEL_MASK						(0x03 << MPU6050_FS_SEL_POSN)
#define MPU6050_ZG_ST_POSN						5
#define MPU6050_ZG_ST_MASK						(0x01 << MPU6050_ZG_ST_POSN)
#define MPU6050_YG_ST_POSN						6
#define MPU6050_YG_ST_MASK						(0x01 << MPU6050_YG_ST_POSN)
#define MPU6050_XG_ST_POSN						7
#define MPU6050_XG_ST_MASK						(0x01 << MPU6050_XG_ST_POSN)

/* FS_SEL field (bits 1:0 of the GYRO_CONFIG register) definitions */
#define MPU6050_FS_SEL_250DEGPERSEC				0
#define MPU6050_FS_SEL_500DEGPERSEC				1
#define MPU6050_FS_SEL_1000DEGPERSEC			2
#define MPU6050_FS_SEL_2000DEGPERSEC			3

/* ACCEL_CONFIG register (0x1C) bit definitions */
#define MPU6050_ACCEL_HPF_POSN					0
#define MPU6050_ACCEL_HPF_MASK					(0x07 << MPU6050_ACCEL_HPF_POSN)
#define MPU6050_AFS_SEL_POSN					3
#define MPU6050_AFS_SEL_MASK					(0x03 << MPU6050_AFS_SEL_POSN)
#define MPU6050_ZA_ST_POSN						5
#define MPU6050_ZA_ST_MASK						(0x01 << MPU6050_ZA_ST_POSN)
#define MPU6050_YA_ST_POSN						6
#define MPU6050_YA_ST_MASK						(0x01 << MPU6050_YA_ST_POSN)
#define MPU6050_XA_ST_POSN						7
#define MPU6050_XA_ST_MASK						(0x01 << MPU6050_XA_ST_POSN)

/* AFS_SEL field (bits 4:3 of the ACCEL_CONFIG register) definitions */
#define MPU6050_AFS_SEL_2G						0
#define MPU6050_AFS_SEL_4G						1
#define MPU6050_AFS_SEL_8G						2
#define MPU6050_AFS_SEL_16G						3

/* ACCEL_HPF field (bits 2:0 of the ACCEL_CONFIG register) definitions */
#define MPU6050_ACCEL_HPF_RESET					0
#define MPU6050_ACCEL_HPF_5HZ					1
#define MPU6050_ACCEL_HPF_2_5HZ					2
#define MPU6050_ACCEL_HPF_1_25HZ				3
#define MPU6050_ACCEL_HPF_0_63HZ				4
#define MPU6050_ACCEL_HPF_HOLD					7

/* INT_PIN_CFG register (0x37) bit definitions */
#define MPU6050_CLKOUT_EN_POSN					0
#define MPU6050_CLKOUT_EN_MASK					(0x01 << MPU6050_CLKOUT_EN_POSN)
#define MPU6050_I2C_BYPASS_EN_POSN				1
#define MPU6050_I2C_BYPASS_EN_MASK				(0x01 << MPU6050_I2C_BYPASS_EN_POSN)
#define MPU6050_FSYNC_INT_EN_POSN				2
#define MPU6050_FSYNC_INT_EN_MASK				(0x01 << MPU6050_FSYNC_INT_EN_POSN)
#define MPU6050_FSYNC_INT_LEVEL_POSN			3
#define MPU6050_FSYNC_INT_LEVEL_MASK			(0x01 << MPU6050_FSYNC_INT_LEVEL_POSN)
#define MPU6050_INT_RD_CLEAR_POSN				4
#define MPU6050_INT_RD_CLEAR_MASK				(0x01 << MPU6050_INT_RD_CLEAR_POSN)
#define MPU6050_LATCH_INT_EN_POSN				5
#define MPU6050_LATCH_INT_EN_MASK				(0x01 << MPU6050_LATCH_INT_EN_POSN)
#define MPU6050_INT_OPEN_POSN					6
#define MPU6050_INT_OPEN_MASK					(0x01 << MPU6050_INT_OPEN_POSN)
#define MPU6050_INT_LEVEL_POSN					7
#define MPU6050_INT_LEVEL_MASK					(0x01 << MPU6050_INT_LEVEL_POSN)

/* INT_ENABLE register (0x38) bit definitions */
#define MPU6050_RAW_RDY_EN_POSN					0
#define MPU6050_RAW_RDY_EN_MASK					(0x01 << MPU6050_RAW_RDY_EN_POSN)
#define MPU6050_DMP_INT_EN_POSN					1
#define MPU6050_DMP_INT_EN_MASK					(0x01 << MPU6050_DMP_INT_EN_POSN)
#define MPU6050_PLL_RDY_EN_POSN					2
#define MPU6050_PLL_RDY_EN_MASK					(0x01 << MPU6050_PLL_RDY_EN_POSN)
#define MPU6050_I2C_MST_INT_EN_POSN				3
#define MPU6050_I2C_MST_INT_EN_MASK				(0x01 << MPU6050_I2C_MST_INT_EN_POSN)
#define MPU6050_FIFO_OFLOW_EN_POSN				4
#define MPU6050_FIFO_OFLOW_EN_MASK				(0x01 << MPU6050_FIFO_OFLOW_EN_POSN)
#define MPU6050_ZMOT_EN_POSN					5
#define MPU6050_ZMOT_EN_MASK					(0x01 << MPU6050_ZMOT_EN_POSN)
#define MPU6050_MOT_EN_POSN						6
#define MPU6050_MOT_EN_MASK						(0x01 << MPU6050_MOT_EN_POSN)
#define MPU6050_FF_EN_POSN						7
#define MPU6050_FF_EN_MASK						(0x01 << MPU6050_FF_EN_POSN)

/* DMP_INT_STATUS register (0x39) bit definitions */
#define MPU6050_DMP_INT_0_POSN					0
#define MPU6050_DMP_INT_0_MASK					(0x01 << MPU6050_DMP_INT_0_POSN)
#define MPU6050_DMP_INT_1_POSN					1
#define MPU6050_DMP_INT_1_MASK					(0x01 << MPU6050_DMP_INT_1_POSN)
#define MPU6050_DMP_INT_2_POSN					2
#define MPU6050_DMP_INT_2_MASK					(0x01 << MPU6050_DMP_INT_2_POSN)
#define MPU6050_DMP_INT_3_POSN					3
#define MPU6050_DMP_INT_3_MASK					(0x01 << MPU6050_DMP_INT_3_POSN)
#define MPU6050_DMP_INT_4_POSN					4
#define MPU6050_DMP_INT_4_MASK					(0x01 << MPU6050_DMP_INT_4_POSN)
#define MPU6050_DMP_INT_5_POSN					5
#define MPU6050_DMP_INT_5_MASK					(0x01 << MPU6050_DMP_INT_5_POSN)

/* INT_STATUS register (0x3A) bit definitions */
#define MPU6050_RAW_RDY_INT_POSN				0
#define MPU6050_RAW_RDY_INT_MASK				(0x01 << MPU6050_RAW_RDY_INT_POSN)
#define MPU6050_DMP_INT_POSN					1
#define MPU6050_DMP_INT_MASK					(0x01 << MPU6050_DMP_INT_POSN)
#define MPU6050_PLL_RDY_INT_POSN				2
#define MPU6050_PLL_RDY_INT_MASK				(0x01 << MPU6050_PLL_RDY_INT_POSN)
#define MPU6050_I2C_MST_INT_POSN				3
#define MPU6050_I2C_MST_INT_MASK				(0x01 << MPU6050_I2C_MST_INT_POSN)
#define MPU6050_FIFO_OFLOW_INT_POSN				4
#define MPU6050_FIFO_OFLOW_INT_MASK				(0x01 << MPU6050_FIFO_OFLOW_INT_POSN)
#define MPU6050_ZMOT_INT_POSN					5
#define MPU6050_ZMOT_INT_MASK					(0x01 << MPU6050_ZMOT_INT_POSN)
#define MPU6050_MOT_INT_POSN					6
#define MPU6050_MOT_INT_MASK					(0x01 << MPU6050_MOT_INT_POSN)
#define MPU6050_FF_INT_POSN						7
#define MPU6050_FF_INT_MASK						(0x01 << MPU6050_FF_INT_POSN)

/* SIGNAL_PATH_RESET register (0x68) bit definitions */
#define MPU6050_TEMP_RESET_POSN					0
#define MPU6050_TEMP_RESET_MASK					(0x01 << MPU6050_TEMP_RESET_POSN)
#define MPU6050_ACCEL_RESET_POSN				1
#define MPU6050_ACCEL_RESET_MASK				(0x01 << MPU6050_ACCEL_RESET_POSN)
#define MPU6050_GYRO_RESET_POSN					2
#define MPU6050_GYRO_RESET_MASK					(0x01 << MPU6050_GYRO_RESET_POSN)

/* MOT_DETECT_CTRL register (0x69) bit definitions */
#define MPU6050_MOT_COUNT_POSN					0
#define MPU6050_MOT_COUNT_MASK					(0x03 << MPU6050_RAW_RDY_INT_POSN)
#define MPU6050_FF_COUNT_POSN					2
#define MPU6050_FF_COUNT_MASK					(0x03 << MPU6050_FF_COUNT_POSN)
#define MPU6050_ACCEL_ON_DELAY_POSN				4
#define MPU6050_ACCEL_ON_DELAY_MASK				(0x03 << MPU6050_ACCEL_ON_DELAY_POSN)

/* USER_CTRL register (0x6A) bit definitions */
#define MPU6050_SIG_COND_RESET_POSN				0
#define MPU6050_SIG_COND_RESET_MASK				(0x01 << MPU6050_SIG_COND_RESET_POSN)
#define MPU6050_I2C_MST_RESET_POSN				1
#define MPU6050_I2C_MST_RESET_MASK				(0x01 << MPU6050_I2C_MST_RESET_POSN)
#define MPU6050_FIFO_RESET_POSN					2
#define MPU6050_FIFO_RESET_MASK					(0x01 << MPU6050_FIFO_RESET_POSN)
#define MPU6050_DMP_RESET_POSN					3
#define MPU6050_DMP_RESET_MASK					(0x01 << MPU6050_DMP_RESET_POSN)
#define MPU6050_I2C_IF_DIS_POSN					4
#define MPU6050_I2C_IF_DIS_MASK					(0x01 << MPU6050_I2C_IF_DIS_POSN)
#define MPU6050_I2C_MST_EN_POSN					5
#define MPU6050_I2C_MST_EN_MASK					(0x01 << MPU6050_I2C_MST_EN_POSN)
#define MPU6050_FIFO_EN_POSN					6
#define MPU6050_FIFO_EN_MASK					(0x01 << MPU6050_FIFO_EN_POSN)
#define MPU6050_DMP_EN_POSN						7
#define MPU6050_DMP_EN_MASK						(0x01 << MPU6050_DMP_EN_POSN)

/* PWM_MGMT_1 register (0x6B) bit definitions */
#define MPU6050_CLK_SEL_POSN					0
#define MPU6050_CLK_SEL_MASK					(0x07 << MPU6050_CLK_SEL_POSN)
#define MPU6050_TEMP_DIS_POSN					3
#define MPU6050_TEMP_DIS_MASK					(0x01 << MPU6050_TEMP_DIS_POSN)
#define MPU6050_CYCLE_POSN						5
#define MPU6050_CYCLE_MASK						(0x01 << MPU6050_CYCLE_POSN)
#define MPU6050_SLEEP_POSN						6
#define MPU6050_SLEEP_MASK						(0x01 << MPU6050_SLEEP_POSN)
#define MPU6050_DEVICE_RESET_POSN				7
#define MPU6050_DEVICE_RESET_MASK				(0x01 << MPU6050_DEVICE_RESET_POSN)

/* CLK_SEL field (bits 2:0 of the PWM_MGMT_1 register) definitions */
#define MPU6050_CLK_SEL_INTERNAL_8MHZ			0
#define MPU6050_CLK_SEL_PLL_XGYRO				1
#define MPU6050_CLK_SEL_PLL_YGYRO				2
#define MPU6050_CLK_SEL_PLL_ZGYRO				3
#define MPU6050_CLK_SEL_PLL_EXTERNAL_32KHZ		4
#define MPU6050_CLK_SEL_PLL_EXTERNAL_19MHZ		5
#define MPU6050_CLK_SEL_STOPPED					7

/* PWM_MGMT_2 register (0x6C) bit definitions */
#define MPU6050_STBY_ZG_POSN					0
#define MPU6050_STBY_ZG_MASK					(0x01 << MPU6050_STBY_ZG_POSN)
#define MPU6050_STBY_YG_POSN					1
#define MPU6050_STBY_YG_MASK					(0x01 << MPU6050_STBY_YG_POSN)
#define MPU6050_STBY_XG_POSN					2
#define MPU6050_STBY_XG_MASK					(0x01 << MPU6050_STBY_XG_POSN)
#define MPU6050_STBY_ZA_POSN					3
#define MPU6050_STBY_ZA_MASK					(0x01 << MPU6050_STBY_ZA_POSN)
#define MPU6050_STBY_YA_POSN					4
#define MPU6050_STBY_YA_MASK					(0x01 << MPU6050_STBY_YA_POSN)
#define MPU6050_STBY_XA_POSN					5
#define MPU6050_STBY_XA_MASK					(0x01 << MPU6050_STBY_XA_POSN)
#define MPU6050_LP_WAKE_CTRL_POSN				6
#define MPU6050_LP_WAKE_CTRL_MASK				(0x03 << MPU6050_LP_WAKE_CTRL_POSN)

/* LP_WAKE_CTRL field (bits 7:6 of the PWR_MGMT_2 register) definitions */
#define MPU6050_LP_WAKE_CTRL_1_25HZ				0
#define MPU6050_LP_WAKE_CTRL_2_5HZ				1
#define MPU6050_LP_WAKE_CTRL_5HZ				2
#define MPU6050_LP_WAKE_CTRL_10HZ				3

/* BANK_SEL register (0x6D) bit definitions */
#define MPU6050_MEM_SEL_POSN					0
#define MPU6050_MEM_SEL_MASK					(0x1F << MPU6050_MEM_SEL_POSN)
#define MPU6050_CFG_USER_BANK_POSN				5
#define MPU6050_CFG_USER_BANK_MASK				(0x01 << MPU6050_CFG_USER_BANK_POSN)
#define MPU6050_PRFTCH_EN_POSN					6
#define MPU6050_PRFTCH_EN_MASK					(0x01 << MPU6050_PRFTCH_EN_POSN)

/* WAIT_AM_I register (0x75) bit definitions */
#define MPU6050_WHO_AM_I_POSN					1
#define MPU6050_WHO_AM_I_MASK					(0x3F << MPU6050_WHO_AM_I_POSN)

/* DMP Memory constants */
#define MPU6050_DMP_MEMORY_BANKS				8
#define MPU6050_DMP_MEMORY_BANK_SIZE			256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE			16

bool mpu6050_setAddress(uint8_t address);
uint8_t mpu6050_getAddress(void);
const char *mpu6050_getName(void);

bool mpu6050_writeReg(uint8_t addr, uint8_t data, nrf_drv_twi_t *m_twi_master);
bool mpu6050_readReg(uint8_t addr, uint8_t *data, nrf_drv_twi_t *m_twi_master);
bool mpu6050_writeBytes(uint8_t *addrData, uint8_t nBytes, nrf_drv_twi_t *m_twi_master);
bool mpu6050_readBytes(uint8_t addr, uint8_t *data, uint8_t nBytes, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setBits(uint8_t addr, uint8_t bitsToSet, nrf_drv_twi_t *m_twi_master);
bool mpu6050_clearBits(uint8_t addr, uint8_t bitsToClear, nrf_drv_twi_t *m_twi_master);

bool mpu6050_getWhoAmI(uint8_t *whoAmI, nrf_drv_twi_t *m_twi_master);

bool mpu6050_reset(nrf_drv_twi_t *m_twi_master);
bool mpu6050_resetSignalPaths(nrf_drv_twi_t *m_twi_master);
bool mpu6050_resetDMP(nrf_drv_twi_t *m_twi_master);
bool mpu6050_resetFIFO(nrf_drv_twi_t *m_twi_master);
bool mpu6050_resetI2CMaster(nrf_drv_twi_t *m_twi_master);

bool mpu6050_getFIFOEnabled(bool *fifoEnabled, nrf_drv_twi_t *m_twi_master);
bool mpu6050_setFIFOEnabled(bool fifoEnabled, nrf_drv_twi_t *m_twi_master);

bool mpu6050_getDMPEnabled(bool *dmpEnabled, nrf_drv_twi_t *m_twi_master);
bool mpu6050_setDMPEnabled(bool dmpEnabled, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setClockSource(uint8_t clockSource, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setExternalFrameSync(uint8_t externalSync, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setDLPFMode(uint8_t dlpfMode, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setFullScaleGyroRange(uint8_t fsGyroRange, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setAccelHPFMode(uint8_t accelHPFMode, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setFullScaleAccelRange(uint8_t fsAccelRange, nrf_drv_twi_t *m_twi_master);

bool mpu6050_getOTPBankValid(bool *otpBankValid, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setOTPBankValid(bool otpBankValid, nrf_drv_twi_t *m_twi_master);

bool mpu6050_getXGyroOffsetTC(int8_t *offset, nrf_drv_twi_t *m_twi_master);
bool mpu6050_setXGyroOffsetTC(int8_t offset, nrf_drv_twi_t *m_twi_master);

bool mpu6050_getYGyroOffsetTC(int8_t *offset, nrf_drv_twi_t *m_twi_master);
bool mpu6050_setYGyroOffsetTC(int8_t offset, nrf_drv_twi_t *m_twi_master);

bool mpu6050_getZGyroOffsetTC(int8_t *offset, nrf_drv_twi_t *m_twi_master);
bool mpu6050_setZGyroOffsetTC(int8_t offset, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setSleepEnabled(bool sleepEnabled, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setCycleEnabled(bool cycleEnabled, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setWakeupFrequency(uint8_t wakeupFreq, nrf_drv_twi_t *m_twi_master);

bool mpu6050_getFIFOCount(uint16_t *fifoCount, nrf_drv_twi_t *m_twi_master);
bool mpu6050_getFIFOBytes(uint8_t *fifoBuffer, uint16_t fifoCount, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank, nrf_drv_twi_t *m_twi_master);

bool mpu6050_setMemoryStartAddress(uint8_t address, nrf_drv_twi_t *m_twi_master);

bool mpu6050_readMemoryByte(uint8_t *data, nrf_drv_twi_t *m_twi_master);

bool mpu6050_writeMemoryByte(uint8_t data, nrf_drv_twi_t *m_twi_master);
bool mpu6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, nrf_drv_twi_t *m_twi_master);

bool mpu6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, nrf_drv_twi_t *m_twi_master);

#endif /* MPU6050_H_ */
