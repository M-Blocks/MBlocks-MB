BOARD := BOARD_NRF6310

C_SOURCE_FILES += main.c gitversion.c global.c uart.c db.c adc.c power.c util.c pwm.c freqcntr.c spi.c bldc.c a4960.c spi_master.c sma.c led.c mechbrake.c cmdline.c commands.c motionEvent.c
C_SOURCE_FILES += twi_hw_master.c
C_SOURCE_FILES += mpu6050.c imu.c
C_SOURCE_FILES += bleApp.c ble_sps.c fifo.c ble_vns.c
C_SOURCE_FILES += nrf_delay.c app_scheduler.c app_button.c app_gpiote.c app_timer.c app_uart_fifo.c app_fifo.c ble_advdata.c ble_stack_handler.c ble_conn_params.c 
## The Following files are only needed when the ble_debug_assert_handler() function is called from the app_error_handler() in main ##
C_SOURCE_FILES += ble_debug_assert_handler.c ble_error_log.c ble_flash.c 

CFLAGS += -DBLE_DEBUG=0
CFLAGS += -DENABLE_BLE_COMMANDS=1

SDK_PATH = ../nRF51_SDK_v4.4.0.31803/Nordic/nrf51822/
OUTPUT_FILENAME := MBlocks-MB

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_S110 = 1

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

include $(SDK_PATH)Source/templates/gcc/Makefile.common
