/*
 * mpu6050.c
 *
 *  Created on: Mar 27, 2014
 *      Author: kwgilpin, sclaici
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_drv_twi.h"
#include "global.h"
#include "mpu6050.h"

static uint8_t mpu6050Address = MPU6050_I2C_ADDR_CENTRAL;

bool mpu6050_setAddress(uint8_t address) {
    if ((address != MPU6050_I2C_ADDR_CENTRAL) && (address != MPU6050_I2C_ADDR_FACE)) {
	return false;
    }

    mpu6050Address = address;
    return true;
}

uint8_t mpu6050_getAddress() {
    return mpu6050Address;
}

const char *mpu6050_getName() {
    if (mpu6050Address == MPU6050_I2C_ADDR_CENTRAL) {
	return "central";
    } else if (mpu6050Address == MPU6050_I2C_ADDR_FACE) {
	return "face 1";
    } else {
	return "unknown";
    }
}

bool mpu6050_writeReg(uint8_t addr, uint8_t data, nrf_drv_twi_t *m_twi_master) {
    uint8_t packet[2];
    ret_code_t err_code;
    
    packet[0] = addr;
    packet[1] = data;
    err_code = nrf_drv_twi_tx(m_twi_master, mpu6050Address, packet, sizeof(packet), true);

    return (err_code == NRF_SUCCESS);
}

bool mpu6050_readReg(uint8_t addr, uint8_t *data, nrf_drv_twi_t *m_twi_master) {
    bool success = true;
    ret_code_t err_code;
    
    err_code = nrf_drv_twi_tx(m_twi_master, mpu6050Address, &addr, 1, false);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(m_twi_master, mpu6050Address, data, 1, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    return success;
}

bool mpu6050_writeBytes(uint8_t *addrData, uint8_t nBytes, nrf_drv_twi_t *m_twi_master) {
    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(m_twi_master, mpu6050Address, addrData, nBytes, true);

    return (err_code == NRF_SUCCESS);
}

bool mpu6050_readBytes(uint8_t addr, uint8_t *data, uint8_t nBytes, nrf_drv_twi_t *m_twi_master) {
    bool success = true;
    ret_code_t err_code;
    
    err_code = nrf_drv_twi_tx(m_twi_master, mpu6050Address, &addr, 1, false);
    if (err_code != NRF_SUCCESS)
	success = false;
    err_code = nrf_drv_twi_rx(m_twi_master, mpu6050Address, data, nBytes, true);
    if (err_code != NRF_SUCCESS)
	success = false;

    return success;
}

bool mpu6050_setBits(uint8_t addr, uint8_t bitsToSet, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    success |= mpu6050_readReg(addr, &reg, m_twi_master);
    success |= mpu6050_writeReg(addr, reg | bitsToSet, m_twi_master);

    return success;
}

bool mpu6050_clearBits(uint8_t addr, uint8_t bitsToClear, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    success |= mpu6050_readReg(addr, &reg, m_twi_master);
    success |= mpu6050_writeReg(addr, reg & ~bitsToClear, m_twi_master);

    return success;
}

bool mpu6050_getWhoAmI(uint8_t *whoAmI, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success;

    success = mpu6050_readReg(MPU6050_WHO_AM_I_REG_ADDR, &reg, m_twi_master);
    *whoAmI = (reg & MPU6050_WHO_AM_I_MASK) >> MPU6050_WHO_AM_I_POSN;

    return success;
}

bool mpu6050_reset(nrf_drv_twi_t *m_twi_master) {
    return mpu6050_setBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_DEVICE_RESET_POSN), m_twi_master);
}

bool mpu6050_resetSignalPaths(nrf_drv_twi_t *m_twi_master) {
    return mpu6050_writeReg(MPU6050_SIGNAL_PATH_RESET_REG_ADDR,
			    (1<<MPU6050_GYRO_RESET_POSN) | (1<<MPU6050_ACCEL_RESET_POSN) | (1<<MPU6050_TEMP_RESET_POSN),
			    m_twi_master);
}

bool mpu6050_resetDMP(nrf_drv_twi_t *m_twi_master) {
    return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_RESET_POSN), m_twi_master);
}

bool mpu6050_resetFIFO(nrf_drv_twi_t *m_twi_master) {
    return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_RESET_POSN), m_twi_master);
}

bool mpu6050_resetI2CMaster(nrf_drv_twi_t *m_twi_master) {
    return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_I2C_MST_RESET_POSN), m_twi_master);
}

bool mpu6050_getFIFOEnabled(bool *fifoEnabled, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;

    if (!mpu6050_readReg(MPU6050_USER_CTRL_REG_ADDR, &reg, m_twi_master)) {
	return false;
    }

    if (reg & (1<<MPU6050_FIFO_EN_POSN)) {
	*fifoEnabled = true;
    } else {
	*fifoEnabled = false;
    }

    return true;
}

bool mpu6050_setFIFOEnabled(bool fifoEnabled, nrf_drv_twi_t *m_twi_master) {
    if (fifoEnabled) {
	return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_EN_POSN), m_twi_master);
    } else {
	return mpu6050_clearBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_FIFO_EN_POSN), m_twi_master);
    }
}

bool mpu6050_getDMPEnabled(bool *dmpEnabled, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;

    if (!mpu6050_readReg(MPU6050_USER_CTRL_REG_ADDR, &reg, m_twi_master)) {
	return false;
    }

    if (reg & (1<<MPU6050_DMP_EN_POSN)) {
	*dmpEnabled = true;
    } else {
	*dmpEnabled = false;
    }

    return true;
}

bool mpu6050_setDMPEnabled(bool dmpEnabled, nrf_drv_twi_t *m_twi_master) {
    if (dmpEnabled) {
	return mpu6050_setBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_EN_POSN), m_twi_master);
    } else {
	return mpu6050_clearBits(MPU6050_USER_CTRL_REG_ADDR, (1<<MPU6050_DMP_EN_POSN), m_twi_master);
    }
}

bool mpu6050_setClockSource(uint8_t clockSource, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    clockSource <<= MPU6050_CLK_SEL_POSN;
    clockSource &= MPU6050_CLK_SEL_MASK;

    success &= mpu6050_readReg(MPU6050_PWR_MGMT_1_REG_ADDR, &reg, m_twi_master);
    reg &= ~(MPU6050_CLK_SEL_MASK);
    reg |= clockSource;
    success &= mpu6050_writeReg(MPU6050_PWR_MGMT_1_REG_ADDR, reg, m_twi_master);

    return success;
}

bool mpu6050_setExternalFrameSync(uint8_t externalSync, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    externalSync <<= MPU6050_EXT_SYNC_SET_POSN;
    externalSync &= MPU6050_EXT_SYNC_SET_MASK;

    success &= mpu6050_readReg(MPU6050_CONFIG_REG_ADDR, &reg, m_twi_master);
    reg &= ~(MPU6050_EXT_SYNC_SET_MASK);
    reg |= externalSync;
    success &= mpu6050_writeReg(MPU6050_CONFIG_REG_ADDR, reg, m_twi_master);

    return success;
}

bool mpu6050_setDLPFMode(uint8_t dlpfMode, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    dlpfMode <<= MPU6050_DLPF_CFG_POSN;
    dlpfMode &= MPU6050_DLPF_CFG_MASK;

    success &= mpu6050_readReg(MPU6050_CONFIG_REG_ADDR, &reg, m_twi_master);
    reg &= ~(MPU6050_DLPF_CFG_MASK);
    reg |= dlpfMode;
    success &= mpu6050_writeReg(MPU6050_CONFIG_REG_ADDR, reg, m_twi_master);

    return success;
}

bool mpu6050_setFullScaleGyroRange(uint8_t fsGyroRange, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    fsGyroRange <<= MPU6050_FS_SEL_POSN;
    fsGyroRange &= MPU6050_FS_SEL_MASK;

    success &= mpu6050_readReg(MPU6050_GYRO_CONFIG_REG_ADDR, &reg, m_twi_master);
    reg &= ~(MPU6050_FS_SEL_MASK);
    reg |= fsGyroRange;
    success |= mpu6050_writeReg(MPU6050_GYRO_CONFIG_REG_ADDR, reg, m_twi_master);

    return success;
}

bool mpu6050_setAccelHPFMode(uint8_t accelHPFMode, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    accelHPFMode <<= MPU6050_ACCEL_HPF_POSN;
    accelHPFMode &= MPU6050_ACCEL_HPF_MASK;

    success &= mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &reg, m_twi_master);
    reg &= ~(MPU6050_ACCEL_HPF_MASK);
    reg |= accelHPFMode;
    success &= mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, reg, m_twi_master);

    return success;
}


bool mpu6050_setFullScaleAccelRange(uint8_t fsAccelRange, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    fsAccelRange <<= MPU6050_AFS_SEL_POSN;
    fsAccelRange &= MPU6050_AFS_SEL_MASK;

    success &= mpu6050_readReg(MPU6050_ACCEL_CONFIG_REG_ADDR, &reg, m_twi_master);
    reg &= ~(MPU6050_AFS_SEL_MASK);
    reg |= fsAccelRange;
    success |= mpu6050_writeReg(MPU6050_ACCEL_CONFIG_REG_ADDR, reg, m_twi_master);

    return success;
}

bool mpu6050_getOTPBankValid(bool *otpBankValid, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success;

    success = mpu6050_readReg(MPU6050_AUX_VDDIO_REG_ADDR, &reg, m_twi_master);
    if (reg & (1<<MPU6050_OTP_BANK_VLD_POSN)) {
	*otpBankValid = true;
    } else {
	*otpBankValid = false;
    }

    return success;
}

bool mpu6050_setOTPBankValid(bool otpBankValid, nrf_drv_twi_t *m_twi_master) {
    if (otpBankValid) {
	return mpu6050_setBits(MPU6050_AUX_VDDIO_REG_ADDR, (1<<MPU6050_OTP_BANK_VLD_POSN), m_twi_master);
    } else {
	return mpu6050_clearBits(MPU6050_AUX_VDDIO_REG_ADDR, (1<<MPU6050_OTP_BANK_VLD_POSN), m_twi_master);
    }
}

bool mpu6050_getXGyroOffsetTC(int8_t *offset, nrf_drv_twi_t *m_twi_master) {
    uint8_t data;
    bool success;

    success = mpu6050_readReg(MPU6050_AUX_VDDIO_REG_ADDR, &data, m_twi_master);
    data &= MPU6050_XG_OFFS_TC_MASK;
    *offset = data >> MPU6050_XG_OFFS_TC_POSN;

    return success;
}

bool mpu6050_setXGyroOffsetTC(int8_t offset, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    offset <<= MPU6050_XG_OFFS_TC_POSN;
    offset &= MPU6050_XG_OFFS_TC_MASK;

    success &= mpu6050_readReg(MPU6050_AUX_VDDIO_REG_ADDR, &reg, m_twi_master);
    reg &= ~(MPU6050_XG_OFFS_TC_MASK);
    reg |= offset;
    success &= mpu6050_writeReg(MPU6050_AUX_VDDIO_REG_ADDR, reg, m_twi_master);

    return success;
}

bool mpu6050_getYGyroOffsetTC(int8_t *offset, nrf_drv_twi_t *m_twi_master) {
    uint8_t data;
    bool success;

    success = mpu6050_readReg(MPU6050_YG_OFFS_TC_REG_ADDR, &data, m_twi_master);
    data &= MPU6050_YG_OFFS_TC_MASK;
    *offset = data >> MPU6050_YG_OFFS_TC_POSN;

    return success;
}

bool mpu6050_setYGyroOffsetTC(int8_t offset, nrf_drv_twi_t *m_twi_master) {
    uint8_t data;

    data = offset << MPU6050_YG_OFFS_TC_POSN;
    data &= MPU6050_YG_OFFS_TC_MASK;

    return mpu6050_writeReg(MPU6050_YG_OFFS_TC_REG_ADDR, data, m_twi_master);
}

bool mpu6050_getZGyroOffsetTC(int8_t *offset, nrf_drv_twi_t *m_twi_master) {
    uint8_t data;
    bool success;

    success = mpu6050_readReg(MPU6050_ZG_OFFS_TC_REG_ADDR, &data, m_twi_master);
    data &= MPU6050_ZG_OFFS_TC_MASK;
    *offset = data >> MPU6050_ZG_OFFS_TC_POSN;

    return success;
}

bool mpu6050_setZGyroOffsetTC(int8_t offset, nrf_drv_twi_t *m_twi_master) {
    uint8_t data;

    data = offset << MPU6050_ZG_OFFS_TC_POSN;
    data &= MPU6050_ZG_OFFS_TC_MASK;

    return mpu6050_writeReg(MPU6050_ZG_OFFS_TC_REG_ADDR, data, m_twi_master);
}

bool mpu6050_setSleepEnabled(bool sleepEnabled, nrf_drv_twi_t *m_twi_master) {
    if (sleepEnabled) {
	return mpu6050_setBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_SLEEP_POSN), m_twi_master);
    } else {
	return mpu6050_clearBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_SLEEP_POSN), m_twi_master);
    }
}

bool mpu6050_setCycleEnabled(bool cycleEnabled, nrf_drv_twi_t *m_twi_master) {
    if (cycleEnabled) {
	return mpu6050_setBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_CYCLE_POSN), m_twi_master);
    } else {
	return mpu6050_clearBits(MPU6050_PWR_MGMT_1_REG_ADDR, (1<<MPU6050_CYCLE_POSN), m_twi_master);
    }
}

bool mpu6050_setWakeupFrequency(uint8_t wakeupFreq, nrf_drv_twi_t *m_twi_master) {
    uint8_t reg;
    bool success = true;

    wakeupFreq <<= MPU6050_LP_WAKE_CTRL_POSN;
    wakeupFreq &= MPU6050_LP_WAKE_CTRL_MASK;

    success &= mpu6050_readReg(MPU6050_PWR_MGMT_2_REG_ADDR, &reg, m_twi_master);
    reg &= ~(MPU6050_LP_WAKE_CTRL_MASK);
    reg |= wakeupFreq;
    success &= mpu6050_writeReg(MPU6050_PWR_MGMT_2_REG_ADDR, reg, m_twi_master);

    return success;
}

bool mpu6050_getFIFOCount(uint16_t *fifoCount, nrf_drv_twi_t *m_twi_master) {
    bool success;
    uint8_t data[2];

    success = mpu6050_readBytes(MPU6050_FIFO_COUNTH_REG_ADDR, data, 2, m_twi_master);
    *fifoCount = (((uint16_t)data[0]) << 8 | (uint16_t)data[1]);

    return success;
}

bool mpu6050_getFIFOBytes(uint8_t *fifoBuffer, uint16_t fifoCount, nrf_drv_twi_t *m_twi_master) {
    if (fifoCount == 0) {
	return true;
    }

    return mpu6050_readBytes(MPU6050_FIFO_R_W_REG_ADDR, fifoBuffer, fifoCount, m_twi_master);
}

bool mpu6050_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank, nrf_drv_twi_t *m_twi_master) {
    bank &= 0x1F;

    if (userBank) {
    	bank |= 0x20;
    }

    if (prefetchEnabled) {
    	bank |= 0x40;
    }

    return mpu6050_writeReg(MPU6050_BANK_SEL_REG_ADDR, bank, m_twi_master);
}

bool mpu6050_setMemoryStartAddress(uint8_t address, nrf_drv_twi_t *m_twi_master) {
    return mpu6050_writeReg(MPU6050_MEM_START_ADDR_REG_ADDR, address, m_twi_master);
}

bool mpu6050_readMemoryByte(uint8_t *data, nrf_drv_twi_t *m_twi_master) {
    return mpu6050_readReg(MPU6050_MEM_R_W_REG_ADDR, data, m_twi_master);
}

bool mpu6050_writeMemoryByte(uint8_t data, nrf_drv_twi_t *m_twi_master) {
    return mpu6050_writeReg(MPU6050_MEM_R_W_REG_ADDR, data, m_twi_master);
}

bool mpu6050_readMemoryBlock(uint8_t *data, uint16_t dataSize,
			     uint8_t bank, uint8_t address, nrf_drv_twi_t *m_twi_master) {
    bool success = true;
    uint8_t chunkSize;
    uint16_t i;

    mpu6050_setMemoryBank(bank, false, false, m_twi_master);
    mpu6050_setMemoryStartAddress(address, m_twi_master);

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
        success &= mpu6050_readBytes(MPU6050_MEM_R_W_REG_ADDR, data + i, chunkSize, m_twi_master);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) {
            	bank++;
            }
            success &= mpu6050_setMemoryBank(bank, false, false, m_twi_master);
            success &= mpu6050_setMemoryStartAddress(address, m_twi_master);
        }
    }

    return success;
}

bool mpu6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize,
			      uint8_t bank, uint8_t address, bool verify,
			      nrf_drv_twi_t *m_twi_master) {
    bool success = true;
    uint8_t chunkSize;
    uint16_t i;
    uint8_t progBuffer[1+MPU6050_DMP_MEMORY_CHUNK_SIZE];
    uint8_t verifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];

    success &= mpu6050_setMemoryBank(bank, false, false, m_twi_master);
    success &= mpu6050_setMemoryStartAddress(address, m_twi_master);

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
        mpu6050_writeBytes(progBuffer, chunkSize + 1, m_twi_master);

        if (verify) {
	    success &= mpu6050_setMemoryBank(bank, false, false, m_twi_master);
	    success &= mpu6050_setMemoryStartAddress(address, m_twi_master);
	    success &= mpu6050_readBytes(MPU6050_MEM_R_W_REG_ADDR, verifyBuffer, chunkSize, m_twi_master);
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
            success &= mpu6050_setMemoryBank(bank, false, false, m_twi_master);
            success &= mpu6050_setMemoryStartAddress(address, m_twi_master);
        }
    }
    return success;
}
