## Hardware definitions
DEVICE := NRF51
DEVICESERIES := nrf51
DEVICE_VARIANT := xxaa

CPU := cortex-m0
TARGET_CHIP := NRF51422_QFAA_E0
BOARD := BOARD_MBLOCKS_MB_2P1

USE_S310 = 1
SOFTDEVICE_NAME := S310

## Softdevice hex file location
SOFTDEVICE_HEXFILE = ../s310_nrf51422_1.0.0/s310_nrf51422_1.0.0_softdevice.hex

## Bootloader hex file location
BOOTLOADER_HEXFILE = ../bootloader/bootloader_gcc_s310_bootloader_xxaa.hex

## C Source Files
C_SOURCE_FILES += main.c gitversion.c global.c uart.c db.c fb.c adc.c power.c util.c pwm.c freqcntr.c spi.c bldc.c a4960.c spi_master.c sma.c led.c mechbrake.c cmdline.c commands.c motionEvent.c
C_SOURCE_FILES += twi_hw_master.c
C_SOURCE_FILES += mpu6050.c imu.c
C_SOURCE_FILES += bleApp.c ble_sps.c ble_vns.c fifo.c 
C_SOURCE_FILES += nrf_delay.c

C_SOURCE_FILES += app_scheduler.c app_button.c app_gpiote.c app_timer.c app_uart_fifo.c app_fifo.c 
C_SOURCE_FILES += ble_advdata.c ble_conn_params.c
C_SOURCE_FILES += softdevice_handler.c

C_SOURCE_FILES += system_$(DEVICESERIES).c

## The Following files are only needed when the ble_debug_assert_handler() function is called from the app_error_handler() in main ##
C_SOURCE_FILES += ble_debug_assert_handler.c ble_error_log.c ble_flash.c 

## C Flags

CFLAGS += -DRELEASE -Os
CFLAGS += -DBLE_DEBUG=0
CFLAGS += -DENABLE_BLE_COMMANDS=1
CFLAGS += -DHARDWARE_VERSION=2
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -mcpu=$(CPU) -mthumb -mabi=aapcs -D$(DEVICE) -D$(BOARD) -D$(TARGET_CHIP) --std=gnu99
CFLAGS += -Wall
CFLAGS += -mfloat-abi=soft

## Assembly source files
ASSEMBLER_SOURCE_FILES += gcc_startup_$(DEVICESERIES).s

## Assembly Flags
ASMFLAGS += -DRELEASE -Os
ASMFLAGS += -D__HEAP_SIZE=512 
ASMFLAGS += -x assembler-with-cpp

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

## Paths
SDK_PATH = ../nrf51_sdk_v6_1_0_b2ec2e6/nrf51422/

SDK_INCLUDE_PATH = $(SDK_PATH)/Include/
SDK_SOURCE_PATH = $(SDK_PATH)/Source/
TEMPLATE_PATH += $(SDK_SOURCE_PATH)/templates/gcc/

INCLUDEPATHS += -I"../"
INCLUDEPATHS += -I"$(SDK_PATH)Include"
INCLUDEPATHS += -I"$(SDK_PATH)Include/gcc"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ext_sensors"
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble"
INCLUDEPATHS += -I"$(SDK_PATH)Include/ble/ble_services"
INCLUDEPATHS += -I"$(SDK_PATH)Include/s310"

## Output directories
OBJECT_DIRECTORY := _build
LISTING_DIRECTORY := _build
OUTPUT_BINARY_DIRECTORY := _build

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )



####################################################################
# Rules                                                            #
####################################################################

C_SOURCE_FILENAMES = $(notdir $(C_SOURCE_FILES) )
ASSEMBLER_SOURCE_FILENAMES = $(notdir $(ASSEMBLER_SOURCE_FILES) )

# Make a list of source paths
C_SOURCE_PATHS += ../ $(SDK_SOURCE_PATH) $(TEMPLATE_PATH) $(wildcard $(SDK_SOURCE_PATH)*/)  $(wildcard $(SDK_SOURCE_PATH)ext_sensors/*/) $(wildcard $(SDK_SOURCE_PATH)ble/*/)
ASSEMBLER_SOURCE_PATHS = ../ $(SDK_SOURCE_PATH) $(TEMPLATE_PATH) $(wildcard $(SDK_SOURCE_PATH)*/)

C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILENAMES:.c=.o) )
ASSEMBLER_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASSEMBLER_SOURCE_FILENAMES:.s=.o) )

# Set source lookup paths
vpath %.c $(C_SOURCE_PATHS)
vpath %.s $(ASSEMBLER_SOURCE_PATHS)

# Include automatically previously generated dependencies
-include $(addprefix $(OBJECT_DIRECTORY)/, $(COBJS:.o=.d))

### Targets

## Default build target
.PHONY: all
all: clean
all: updateGitVersion
all: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex 
all: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithSoftdevice.hex 
all: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_CompleteImage.hex
all:
	bash -c "../bin/append_gitVersionStr.sh $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_BootloaderInput.hex ../gitversion.c"
	bash -c "../bin/append_gitVersionStr.sh $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_AppWithSoftdevice.hex ../gitversion.c"
	bash -c "../bin/append_gitVersionStr.sh $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME)_CompleteImage.hex ../gitversion.c"


.PHONY: clean
clean:
	$(RM) $(OUTPUT_BINARY_DIRECTORY)

echostuff:
	@echo C_OBJECTS: [$(C_OBJECTS)]
	@echo C_SOURCE_FILES: [$(C_SOURCE_FILES)]
	@echo C_SOURCE_PATHS: [$(C_SOURCE_PATHS)]

.PHONY: updateGitVersion
updateGitVersion:
	bash -c "../bin/update_gitVersionStr.sh ../gitversion_template.h ../gitversion.h"
	bash -c "../bin/update_gitVersionStr.sh ../gitversion_template.c ../gitversion.c"

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

