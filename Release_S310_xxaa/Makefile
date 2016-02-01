## Hardware definitions
DEVICE := NRF51
DEVICESERIES := nrf51
DEVICE_VARIANT := xxac

CPU := cortex-m0
TARGET_CHIP := NRF51422_QFAC_R
BOARD := BOARD_MBLOCKS_MB_2P1

USE_S310 = 1
SOFTDEVICE_NAME := S310

## Softdevice hex file location
SOFTDEVICE_HEXFILE = ../s310_nrf51422_2.0.1/s310_nrf51422_2.0.1_softdevice.hex

## C Source Files
C_SOURCE_FILES += main.c ble_sps.c ble_vns.c fifo.c led.c

## C Flags

#flags common to all targets
CFLAGS += -DRELEASE -Os
CFLAGS += -DBLE_DEBUG=0
CFLAGS += -DNRF51
CFLAGS += -DENABLE_BLE_COMMANDS=1
CFLAGS += -DHARDWARE_VERSION=2
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DS310
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DBOARD_PCA10028
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# Assembler flags
ASMFLAGS += -DRELEASE -Os
ASMFLAGS += -DNRF51
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DS310
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DBOARD_PCA10028

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

SDK_PATH = ../nRF51_SDK_7.2.0_cf547b5/
TEMPLATE_PATH = $(SDK_PATH)components/toolchain/gcc/

#source common to all targets
C_SOURCE_FILES += $(abspath $(SDK_PATH)components/libraries/button/app_button.c) \
	          $(abspath $(SDK_PATH)components/libraries/scheduler/app_scheduler.c) \
		  $(abspath $(SDK_PATH)components/libraries/timer/app_timer.c) \
		  $(abspath $(SDK_PATH)components/libraries/util/nrf_assert.c) \
		  $(abspath $(SDK_PATH)components/libraries/util/app_error.c) \
		  $(abspath $(SDK_PATH)components/libraries/uart/app_uart_fifo.c) \
		  $(abspath $(SDK_PATH)components/libraries/fifo/app_fifo.c) \
		  $(abspath $(SDK_PATH)components/libraries/gpiote/app_gpiote.c) \
		  $(abspath $(SDK_PATH)components/drivers_nrf/delay/nrf_delay.c) \
		  $(abspath $(SDK_PATH)components/ble/common/ble_advdata.c) \
		  $(abspath $(SDK_PATH)components/ble/common/ble_conn_params.c) \
		  $(abspath $(SDK_PATH)components/ble/common/ble_srv_common.c) \
		  $(abspath $(SDK_PATH)components/toolchain/system_nrf51.c) \
		  $(abspath $(SDK_PATH)components/softdevice/common/softdevice_handler/softdevice_handler.c) \
		  $(abspath ../includes/RTT/SEGGER_RTT.c) \
		  $(abspath ../includes/RTT/SEGGER_RTT_printf.c)

C_PATHS += ../ $(SDK_PATH) $(TEMPLATE_PATH) \
			   $(wildcard $(SDK_PATH)components/*/) \
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
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/hal)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/drivers_nrf/uart)

INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/util)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/scheduler)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/button)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/timer)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/gpiote)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/fifo)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/libraries/)

INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/device_manager)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/common)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/ble/ble_error_log)

INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/toolchain/gcc)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/toolchain)
INCLUDEPATHS += -I$(abspath $(SDK_PATH)components/softdevice/common/softdevice_handler)

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

.PHONY: clean
clean:
	$(RM) $(OUTPUT_BINARY_DIRECTORY)

echostuff:
	@echo C_OBJECTS: [$(C_OBJECTS)]
	@echo C_SOURCE_FILES: [$(C_SOURCE_FILES)]
	@echo C_SOURCE_PATHS: [$(C_SOURCE_PATHS)]

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

