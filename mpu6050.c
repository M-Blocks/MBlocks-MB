/*
 * mpu6050.c
 *
 *  Created on: Mar 27, 2014
 *      Author: kwgilpin
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "twi_master.h"
#include "mpu6050.h"

static uint8_t mpu6050Address;

void mpu6050_setAddress(uint8_t address) {
	mpu6050Address = address;
}

bool mpu6050_writeReg(uint8_t addr, uint8_t data) {
	uint8_t packet[2];

	/* The caller must initialize the TWI interface */
	if (!twi_master_get_init()) {
		return false;
	}

	packet[0] = addr;
	packet[1] = data;
	return twi_master_transfer((mpu6050Address<<1), packet, sizeof(packet), TWI_ISSUE_STOP);
}

bool mpu6050_readReg(uint8_t addr, uint8_t *data) {
	bool success = true;

	/* The caller must initialize the TWI interface */
	if (!twi_master_get_init()) {
		return false;
	}

	success &= twi_master_transfer((mpu6050Address<<1), &addr, 1, TWI_DONT_ISSUE_STOP);
	success &= twi_master_transfer((mpu6050Address<<1) | TWI_READ_BIT, data, 1, TWI_ISSUE_STOP);
	return success;
}

bool mpu6050_writeBytes(uint8_t *addrData, uint8_t nBytes) {
	/* The caller must initialize the TWI interface */
	if (!twi_master_get_init()) {
		return false;
	}

	return twi_master_transfer((mpu6050Address<<1), addrData, nBytes, TWI_ISSUE_STOP);
}

bool mpu6050_readBytes(uint8_t addr, uint8_t *data, uint8_t nBytes) {
	bool success = true;

	/* The caller must initialize the TWI interface */
	if (!twi_master_get_init()) {
		return false;
	}

	success &= twi_master_transfer((mpu6050Address<<1), &addr, 1, TWI_DONT_ISSUE_STOP);
	success &= twi_master_transfer((mpu6050Address<<1) | TWI_READ_BIT, data, nBytes, TWI_ISSUE_STOP);
	return success;
}

bool mpu6050_setBits(uint8_t addr, uint8_t bitsToSet) {
	uint8_t reg;
	bool success = true;

	success |= mpu6050_readReg(addr, &reg);
	success |= mpu6050_writeReg(addr, reg | bitsToSet);

	return success;
}

bool mpu6050_clearBits(uint8_t addr, uint8_t bitsToClear) {
	uint8_t reg;
	bool success = true;

	success |= mpu6050_readReg(addr, &reg);
	success |= mpu6050_writeReg(addr, reg & ~bitsToClear);

	return success;
}

bool mpu6050_getWhoAmI(uint8_t *whoAmI) {
	uint8_t reg;
	bool success;

	success = mpu6050_readReg(MPU6050_WHO_AM_I_REG_ADDR, &reg);
	*whoAmI = (reg & MPU6050_WHO_AM_I_MASK) >> MPU6050_WHO_AM_I_POSN;

	return success;
}

bool mpu6050_reset() {
	return mpu6050_setBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_DEVICE_RESET_POSN));
}

bool mpu6050_resetSignalPaths() {
	return mpu6050_writeReg(MPU6050_SIGNAL_PATH_RESET_REG_ADDR,
			(1<<MPU6050_GYRO_RESET_POSN) | (1<<MPU6050_ACCEL_RESET_POSN) | (1<<MPU6050_TEMP_RESET_POSN));
}

bool mpu6050_resetDMP() {
	return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_RESET_POSN));
}

bool mpu6050_resetFIFO() {
	return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_RESET_POSN));
}

bool mpu6050_resetI2CMaster() {
	return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_I2C_MST_RESET_POSN));
}

bool mpu6050_getFIFOEnabled(bool *fifoEnabled) {
	uint8_t reg;

	if (!mpu6050_readReg(MPU6050_USER_CTRL_REG_ADDR, &reg)) {
		return false;
	}

	if (reg & (1<<MPU6050_FIFO_EN_POSN)) {
		*fifoEnabled = true;
	} else {
		*fifoEnabled = false;
	}

	return true;
}

bool mpu6050_setFIFOEnabled(bool fifoEnabled) {
	if (fifoEnabled) {
		return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_EN_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_EN_POSN));
	}
}

bool mpu6050_getDMPEnabled(bool *dmpEnabled) {
	uint8_t reg;

	if (!mpu6050_readReg(MPU6050_USER_CTRL_REG_ADDR, &reg)) {
		return false;
	}

	if (reg & (1<<MPU6050_DMP_EN_POSN)) {
		*dmpEnabled = true;
	} else {
		*dmpEnabled = false;
	}

	return true;
}

bool mpu6050_setDMPEnabled(bool dmpEnabled) {
	if (dmpEnabled) {
		return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_EN_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_EN_POSN));
	}
}

bool mpu6050_setClockSource(uint8_t clockSource) {
	uint8_t reg;
	bool success = true;

	clockSource <<= MPU6050_CLK_SEL_POSN;
	clockSource &= MPU6050_CLK_SEL_MASK;

	success &= mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &reg);
	reg &= ~(MPU6050_CLK_SEL_MASK);
	reg |= clockSource;
	success &= mpu6050_writeReg(MPU6050_PWR_MGMT_1_REG_ADDR, reg);

	return success;
}

bool mpu6050_setExternalFrameSync(uint8_t externalSync) {
	uint8_t reg;
	bool success = true;

	externalSync <<= MPU6050_EXT_SYNC_SET_POSN;
	externalSync &= MPU6050_EXT_SYNC_SET_MASK;

	success &= mpu6050_readReg(MPU6050_CONFIG_REG_ADDR, &reg);
	reg &= ~(MPU6050_EXT_SYNC_SET_MASK);
	reg |= externalSync;
	success &= mpu6050_writeReg(MPU6050_CONFIG_REG_ADDR, reg);

	return success;
}

bool mpu6050_setDLPFMode(uint8_t dlpfMode) {
	uint8_t reg;
	bool success = true;

	dlpfMode <<= MPU6050_DLPF_CFG_POSN;
	dlpfMode &= MPU6050_DLPF_CFG_MASK;

	success &= mpu6050_readReg(MPU6050_CONFIG_REG_ADDR, &reg);
	reg &= ~(MPU6050_DLPF_CFG_MASK);
	reg |= dlpfMode;
	success &= mpu6050_writeReg(MPU6050_CONFIG_REG_ADDR, reg);

	return success;
}

bool mpu6050_setFullScaleGyroRange(uint8_t fsGyroRange) {
	uint8_t reg;
	bool success = true;

	fsGyroRange <<= MPU6050_FS_SEL_POSN;
	fsGyroRange &= MPU6050_FS_SEL_MASK;

	success &= mpu6050_readReg(MPU6050_GYRO_CONFIG_REG_ADDR, &reg);
	reg &= ~(MPU6050_FS_SEL_MASK);
	reg |= fsGyroRange;
	success |= mpu6050_writeReg(MPU6050_GYRO_CONFIG_REG_ADDR, reg);

	return success;
}

bool mpu6050_setAccelHPFMode(uint8_t accelHPFMode) {
	uint8_t reg;
	bool success = true;

	accelHPFMode <<= MPU6050_ACCEL_HPF_POSN;
	accelHPFMode &= MPU6050_ACCEL_HPF_MASK;

	success &= mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &reg);
	reg &= ~(MPU6050_ACCEL_HPF_MASK);
	reg |= accelHPFMode;
	success &= mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, reg);

	return success;
}


bool mpu6050_setFullScaleAccelRange(uint8_t fsAccelRange) {
	uint8_t reg;
	bool success = true;

	fsAccelRange <<= MPU6050_AFS_SEL_POSN;
	fsAccelRange &= MPU6050_AFS_SEL_MASK;

	success &= mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &reg);
	reg &= ~(MPU6050_AFS_SEL_MASK);
	reg |= fsAccelRange;
	success |= mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, reg);

	return success;
}

bool mpu6050_getOTPBankValid(bool *otpBankValid) {
	uint8_t reg;
	bool success;

	success = mpu6050_readReg(MPU6050_AUX_VDDIO_REG_ADDR, &reg);
	if (reg & (1<<MPU6050_OTP_BANK_VLD_POSN)) {
		*otpBankValid = true;
	} else {
		*otpBankValid = false;
	}

	return success;
}

bool mpu6050_setOTPBankValid(bool otpBankValid) {
	if (otpBankValid) {
		return mpu6050_setBits(MPU6050_AUX_VDDIO_REG_ADDR, (1<<MPU6050_OTP_BANK_VLD_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_AUX_VDDIO_REG_ADDR, (1<<MPU6050_OTP_BANK_VLD_POSN));
	}
}

bool mpu6050_getXGyroOffsetTC(int8_t *offset) {
	uint8_t data;
	bool success;

	success = mpu6050_readReg(MPU6050_AUX_VDDIO_REG_ADDR, &data);
	data &= MPU6050_XG_OFFS_TC_MASK;
	*offset = data >> MPU6050_XG_OFFS_TC_POSN;

	return success;
}

bool mpu6050_setXGyroOffsetTC(int8_t offset) {
	uint8_t reg;
	bool success = true;

	offset <<= MPU6050_XG_OFFS_TC_POSN;
	offset &= MPU6050_XG_OFFS_TC_MASK;

	success &= mpu6050_readReg(MPU6050_AUX_VDDIO_REG_ADDR, &reg);
	reg &= ~(MPU6050_XG_OFFS_TC_MASK);
	reg |= offset;
	success &= mpu6050_writeReg(MPU6050_AUX_VDDIO_REG_ADDR, reg);

	return success;
}

bool mpu6050_getYGyroOffsetTC(int8_t *offset) {
	uint8_t data;
	bool success;

	success = mpu6050_readReg(MPU6050_YG_OFFS_TC_REG_ADDR, &data);
	data &= MPU6050_YG_OFFS_TC_MASK;
	*offset = data >> MPU6050_YG_OFFS_TC_POSN;

	return success;
}

bool mpu6050_setYGyroOffsetTC(int8_t offset) {
	uint8_t data;

	data = offset << MPU6050_YG_OFFS_TC_POSN;
	data &= MPU6050_YG_OFFS_TC_MASK;

	return mpu6050_writeReg(MPU6050_YG_OFFS_TC_REG_ADDR, data);
}

bool mpu6050_getZGyroOffsetTC(int8_t *offset) {
	uint8_t data;
	bool success;

	success = mpu6050_readReg(MPU6050_ZG_OFFS_TC_REG_ADDR, &data);
	data &= MPU6050_ZG_OFFS_TC_MASK;
	*offset = data >> MPU6050_ZG_OFFS_TC_POSN;

	return success;
}

bool mpu6050_setZGyroOffsetTC(int8_t offset) {
	uint8_t data;

	data = offset << MPU6050_ZG_OFFS_TC_POSN;
	data &= MPU6050_ZG_OFFS_TC_MASK;

	return mpu6050_writeReg(MPU6050_ZG_OFFS_TC_REG_ADDR, data);
}

bool mpu6050_setSleepEnabled(bool sleepEnabled) {
	if (sleepEnabled) {
		return mpu6050_setBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_SLEEP_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_SLEEP_POSN));
	}
}

bool mpu6050_setCycleEnabled(bool cycleEnabled) {
	if (cycleEnabled) {
		return mpu6050_setBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_CYCLE_POSN));
	} else {
		return mpu6050_clearBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_CYCLE_POSN));
	}
}

bool mpu6050_setWakeupFrequency(uint8_t wakeupFreq) {
	uint8_t reg;
	bool success = true;

	wakeupFreq <<= MPU6050_LP_WAKE_CTRL_POSN;
	wakeupFreq &= MPU6050_LP_WAKE_CTRL_MASK;

	success &= mpu6050_readReg(MPU6050_PWR_MGMT_2_REG_ADDR, &reg);
	reg &= ~(MPU6050_LP_WAKE_CTRL_MASK);
	reg |= wakeupFreq;
	success &= mpu6050_writeReg(MPU6050_PWR_MGMT_2_REG_ADDR, reg);

	return success;
}

bool mpu6050_getFIFOCount(uint16_t *fifoCount) {
	bool success;
	uint8_t data[2];

	success = mpu6050_readBytes(MPU6050_FIFO_COUNTH_REG_ADDR, data, 2);
	*fifoCount = (((uint16_t)data[0]) << 8 | (uint16_t)data[1]);

	return success;
}

bool mpu6050_getFIFOBytes(uint8_t *fifoBuffer, uint16_t fifoCount) {
	if (fifoCount == 0) {
		return true;
	}

	return mpu6050_readBytes(MPU6050_FIFO_R_W_REG_ADDR, fifoBuffer, fifoCount);
}

bool mpu6050_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;

    if (userBank) {
    	bank |= 0x20;
    }

    if (prefetchEnabled) {
    	bank |= 0x40;
    }

    return mpu6050_writeReg(MPU6050_BANK_SEL_REG_ADDR, bank);
}

bool mpu6050_setMemoryStartAddress(uint8_t address) {
	return mpu6050_writeReg(MPU6050_MEM_START_ADDR_REG_ADDR, address);
}

bool mpu6050_readMemoryByte(uint8_t *data) {
	return mpu6050_readReg(MPU6050_MEM_R_W_REG_ADDR, data);
}

bool mpu6050_writeMemoryByte(uint8_t data) {
	return mpu6050_writeReg(MPU6050_MEM_R_W_REG_ADDR, data);
}

bool mpu6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
	bool success = true;
	uint8_t chunkSize;
	uint16_t i;

	mpu6050_setMemoryBank(bank, false, false);
    mpu6050_setMemoryStartAddress(address);

    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) {
        	chunkSize = dataSize - i;
        }

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) {
        	chunkSize = 256 - address;
        }

        // read the chunk of data as specified
        success &= mpu6050_readBytes(MPU6050_MEM_R_W_REG_ADDR, data + i, chunkSize);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) {
            	bank++;
            }
            success &= mpu6050_setMemoryBank(bank, false, false);
            success &= mpu6050_setMemoryStartAddress(address);
        }
    }

    return success;
}

bool mpu6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
	bool success = true;
    uint8_t chunkSize;
    uint16_t i;
    uint8_t progBuffer[1+MPU6050_DMP_MEMORY_CHUNK_SIZE];
    uint8_t verifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];

    success &= mpu6050_setMemoryBank(bank, false, false);
    success &= mpu6050_setMemoryStartAddress(address);

    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) {
        	chunkSize = dataSize - i;
        }

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) {
        	chunkSize = 256 - address;
        }

        progBuffer[0] = MPU6050_MEM_R_W_REG_ADDR;
        memcpy(&progBuffer[1], data + i, chunkSize);
        mpu6050_writeBytes(progBuffer, chunkSize + 1);

        if (verify) {
        	success &= mpu6050_setMemoryBank(bank, false, false);
        	success &= mpu6050_setMemoryStartAddress(address);
        	success &= mpu6050_readBytes(MPU6050_MEM_R_W_REG_ADDR, verifyBuffer, chunkSize);
            if (memcmp(&progBuffer[1], verifyBuffer, chunkSize) != 0) {
                success = false;
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) {
            	bank++;
            }
            success &= mpu6050_setMemoryBank(bank, false, false);
            success &= mpu6050_setMemoryStartAddress(address);
        }
    }
    return success;
}
