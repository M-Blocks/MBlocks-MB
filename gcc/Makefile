TARGET_CHIP := NRF51422_QFAA_E0
BOARD := BOARD_MBLOCKS_MB_2P1

C_SOURCE_FILES += main.c gitversion.c global.c uart.c db.c adc.c power.c util.c pwm.c freqcntr.c spi.c bldc.c a4960.c spi_master.c sma.c led.c mechbrake.c cmdline.c commands.c motionEvent.c
C_SOURCE_FILES += twi_hw_master.c
C_SOURCE_FILES += mpu6050.c imu.c
C_SOURCE_FILES += bleApp.c ble_sps.c ble_vns.c fifo.c 
C_SOURCE_FILES += nrf_delay.c

C_SOURCE_FILES += app_scheduler.c app_button.c app_gpiote.c app_timer.c app_uart_fifo.c app_fifo.c 
C_SOURCE_FILES += ble_advdata.c ble_conn_params.c
C_SOURCE_FILES += softdevice_handler.c

## The Following files are only needed when the ble_debug_assert_handler() function is called from the app_error_handler() in main ##
C_SOURCE_FILES += ble_debug_assert_handler.c ble_error_log.c ble_flash.c 

CFLAGS += -DBLE_DEBUG=0
CFLAGS += -DENABLE_BLE_COMMANDS=1
CFLAGS += -DHARDWARE_VERSION=2
CFLAGS += -DBLE_STACK_SUPPORT_REQD

ASMFLAGS += -D__HEAP_SIZE=512 

LIBFLAGS += -lm

SDK_PATH = ../nrf51_sdk_v6_1_0_b2ec2e6/nrf51422/
SOFTDEVICE = ../s310_nrf51422_1.0.0/s310_nrf51422_1.0.0_softdevice.hex

OUTPUT_FILENAME := MBlocks-MB

DEVICE_VARIANT := xxaa

USE_S310 = 1
USE_SOFTDEVICE := S310

include $(SDK_PATH)Source/templates/gcc/Makefile.common
