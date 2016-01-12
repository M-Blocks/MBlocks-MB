## Hardware definitions
DEVICE := NRF51
DEVICESERIES := nrf51
DEVICE_VARIANT := xxac

CPU := cortex-m0
TARGET_CHIP := NRF51422_QFAC_R

USE_S310 = 1
SOFTDEVICE_NAME := S310

## Softdevice hex file location
SOFTDEVICE_HEXFILE = ../s310_nrf51422_3.0.0/s310_nrf51422_3.0.0_softdevice.hex

## Bootloader hex file location
BOOTLOADER_HEXFILE = ../bootloader/bootloader_gcc_s310_bootloader_xxaa.hex

## C Source Files
C_SOURCE_FILES += main.c global.c led.c pwm.c spi.c uart.c util.c power.c adc.c db.c fb.c bldc.c a4960.c freqcntr.c mpu6050.c imu.c motionEvent.c sma.c mechbrake.c commands.c cmdline.c
C_SOURCE_FILES += bleApp.c ble_sps.c ble_vns.c fifo.c
## The Following files are only needed when the ble_debug_assert_handler() function is called from the app_error_handler() in main ##
C_SOURCE_FILES += ble_debug_assert_handler.c

## C Flags

#flags common to all targets
CFLAGS += -DRELEASE -Os
CFLAGS += -DBLE_DEBUG=0
CFLAGS += -DNRF51
CFLAGS += -DENABLE_BLE_COMMANDS=1
CFLAGS += -DHARDWARE_VERSION=2
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DS310
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# Assembler flags
ASMFLAGS += -DRELEASE -Os
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF51
ASMFLAGS += -D__HEAP_SIZE=128
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DS310

## Libraries
LIBFLAGS += -lm

## Linker Flags
LDFLAGS += -L"$(GNU_INSTALL_ROOT)/arm-none-eabi/lib/armv6-m"
LDFLAGS += -L"$(GNU_INSTALL_ROOT)/lib/gcc/arm-none-eabi/$(GNU_VERSION)/armv6-m"
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
#LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nano.specs -lc -u _printf_float
LDFLAGS += -mcpu=$(CPU) -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT) 

LINKER_SCRIPT = gcc_$(DEVICESERIES)_s310_$(DEVICE_VARIANT).ld

## Output Filename Base
OUTPUT_FILENAME_BASE := MBlocks-MB

## Output Filename Suffix
OUTPUT_FILENAME_SUFFIX := Release

## Output Filename Complete
OUTPUT_FILENAME := $(OUTPUT_FILENAME_BASE)_$(OUTPUT_FILENAME_SUFFIX)_$(SOFTDEVICE_NAME)_$(DEVICE_VARIANT)

SDK_PATH = ../nRF51_SDK_10.0.0_dc26b5e/
TEMPLATE_PATH = $(SDK_PATH)components/toolchain/gcc/

#source common to all targets
C_SOURCE_FILES += \
$(abspath $(SDK_PATH)components/libraries/button/app_button.c) \
$(abspath $(SDK_PATH)components/libraries/scheduler/app_scheduler.c) \
$(abspath $(SDK_PATH)components/libraries/timer/app_timer.c) \
$(abspath $(SDK_PATH)components/libraries/timer/app_timer_appsh.c) \
$(abspath $(SDK_PATH)components/libraries/util/nrf_assert.c) \
$(abspath $(SDK_PATH)components/libraries/util/app_error.c) \
$(abspath $(SDK_PATH)components/libraries/uart/app_uart_fifo.c) \
$(abspath $(SDK_PATH)components/libraries/sensorsim/sensorsim.c) \
$(abspath $(SDK_PATH)components/libraries/spi/spi_master.c) \
$(abspath $(SDK_PATH)components/libraries/spi/spi_slave.c) \
$(abspath $(SDK_PATH)components/libraries/fifo/app_fifo.c) \
$(abspath $(SDK_PATH)components/libraries/gpiote/app_gpiote.c) \
$(abspath $(SDK_PATH)components/drivers_nrf/delay/nrf_delay.c) \
$(abspath $(SDK_PATH)components/drivers_nrf/twi_master/deprecated/twi_hw_master.c) \
$(abspath $(SDK_PATH)components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath $(SDK_PATH)components/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath $(SDK_PATH)components/drivers_nrf/common/nrf_drv_common.c) \
$(abspath $(SDK_PATH)components/ant/ant_stack_config/ant_stack_config.c) \
$(abspath $(SDK_PATH)components/ble/common/ble_advdata.c) \
$(abspath $(SDK_PATH)components/ble/common/ble_conn_params.c) \
$(abspath $(SDK_PATH)components/ble/ble_error_log/ble_error_log.c) \
$(abspath $(SDK_PATH)components/ble/ble_flash/ble_flash.c) \
$(abspath $(SDK_PATH)components/toolchain/system_nrf51.c) \
$(abspath $(SDK_PATH)components/softdevice/common/softdevice_handler/softdevice_handler.c) \
$(abspath $(SDK_PATH)components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c) \
$(abspath ../includes/RTT/SEGGER_RTT.c) \
$(abspath ../includes/RTT/SEGGER_RTT_printf.c)

C_PATHS += ../ $(SDK_PATH) $(TEMPLATE_PATH) \
			   $(wildcard $(SDK_PATH)components/*/) \
			   $(wildcard $(SDK_PATH)components/ant/*/) \
			   $(wildcard $(SDK_PATH)components/softdevice/common/*/) \
			   $(wildcard $(SDK_PATH)components/drivers_nrf/*/) \
			   $(wildcard $(SDK_PATH)components/ble/*/) \
			   $(wildcard $(SDK_PATH)components/libraries/*/) \
			   $(wildcard $(SDK_PATH)components/drivers_nrf/twi_master/*/) \
			   $(wildcard ../includes/RTT/) \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath $(SDK_PATH)components/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
INCLUDEPATHS += -I$(abspath ../)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/softdevice/s310/headers)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/config)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/delay)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/gpiote)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/hal)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/twi_master)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/twi_master/deprecated)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/common)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/uart)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/pstorage)

INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/util)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/scheduler)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/sensorsim)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/uart)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/button)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/timer)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/scheduler)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/gpiote)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/spi)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/fifo)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/trace)

INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/device_manager)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/common)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/ble_error_log)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/ble_services/ble_dis)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/device)

INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ant/ant_stack_config)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ant/ant_stack_config/config)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/toolchain/gcc)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/toolchain)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/ble_services/ble_bas)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/ble_debug_assert_handler)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/softdevice/common/softdevice_handler)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/ble_services/ble_hrs)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/ble_flash)

INCLUDEPATHS += -I$(abspath ../includes/RTT)


## Output directories
OBJECT_DIRECTORY := _build
LISTING_DIRECTORY := _build
OUTPUT_BINARY_DIRECTORY := _build

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

####################################################################
# Rules                                                            #
####################################################################

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS += $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_PATHS += $(SDK_PATH) $(TEMPLATE_PATH) $(SDK_PATH)components/*/

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS += $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASSEMBLER_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASSEMBLER_OBJECTS)

# Include automatically previously generated dependencies
-include $(addprefix $(OBJECT_DIRECTORY)/, $(COBJS:.o=.d))

### Targets

## Default build target
.PHONY: all
all: clean
all: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex 
all: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithSoftdevice.hex 
all: $(OUTPUT_BINARY_DIRECTORY)/MBlocks-MB_s310_xxaa-IncSD.hex
all: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_CompleteImage.hex

.PHONY: clean
clean:
	$(RM) $(OUTPUT_BINARY_DIRECTORY)

echostuff:
	@echo C_OBJECTS: [$(C_OBJECTS)]
	@echo C_SOURCE_FILES: [$(C_SOURCE_FILES)]
	@echo C_PATHS: [$(C_PATHS)]
	@echo $(CC)

## Create build directories
$(BUILD_DIRECTORIES):
	$(MK) $@

## Create objects from C source files
$(OBJECT_DIRECTORY)/%.o: %.c
# Build header dependencies
	$(CC) $(CFLAGS) $(INCLUDEPATHS) -M $< -MF "$(@:.o=.d)" -MT $@
# Do the actual compilation
	$(CC) $(CFLAGS) $(INCLUDEPATHS) -c -o $@ $<

## Assemble .s files
$(OBJECT_DIRECTORY)/%.o: %.s
	$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

## Link C and assembler objects to an .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(C_OBJECTS) $(ASSEMBLER_OBJECTS) $(LIBRARIES)
	$(CC) $(LDFLAGS) $(C_OBJECTS) $(ASSEMBLER_OBJECTS) $(LIBFLAGS) $(LIBRARIES) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $@
	$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

## Merge the basic application image with the softdevice
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithSoftdevice.hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex
	 mergehex --merge $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex $(SOFTDEVICE_HEXFILE) --output $@

$(OUTPUT_BINARY_DIRECTORY)/MBlocks-MB_s310_xxaa-IncSD.hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex
	 mergehex --merge $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex $(SOFTDEVICE_HEXFILE) --output $@

## Merge the basic application image with the bootloader and the softdevice, and configure the bootloader configuration settings to mark the application image valid 
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_CompleteImage.hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex
	mergehex --merge $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex $(BOOTLOADER_HEXFILE) --output $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithBootloaderCode.hex
	mergehex --merge $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithBootloaderCode.hex $(SOFTDEVICE_HEXFILE) --output $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithBootloaderCodeAndSoftdevice.hex
	bash -c "../bin/appendBootloaderSettings.sh $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithBootloaderCodeAndSoftdevice.hex $@"
	#$(RM) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithBootloaderCode.hex
	#$(RM) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithBootloaderCodeAndSoftdevice.hex
	

include $(TEMPLATE_PATH)Makefile.windows

## Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE	  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

MK 				:= mkdir
RM 				:= rm -rf

