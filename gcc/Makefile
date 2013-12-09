BOARD := BOARD_NRF6310

C_SOURCE_FILES += main.c adc.c power.c util.c pwm.c freqcntr.c spi.c bldc.c a4960.c spi_master.c sma.c cmdline.c commands.c retarget.c nrf_delay.c app_scheduler.c app_button.c app_gpiote.c app_timer.c app_uart.c app_fifo.c ble_advdata.c ble_stack_handler.c ble_conn_params.c

#SDK_PATH = C:/Nordic_Semiconductor/nRF51_SDK_v4.2.0.25053/Nordic/nrf51822/
SDK_PATH = ../../nRF51_SDK_v4.2.0.25053/Nordic/nrf51822/
OUTPUT_FILENAME := MBlocks-HWTest

DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

USE_S110 = 1

USE_SOFTDEVICE := S110
#USE_SOFTDEVICE := S210

include $(SDK_PATH)Source/templates/gcc/Makefile.common
