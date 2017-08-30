############
# I2C example make file
#
# Adapted from @superjax's Generic Makefile Template:
# https://github.com/superjax/airbourne/blob/f4/f4/examples/spi/Makefile
#
# @author len0rd
# @since 2017-08-29

TARGET = i2c

BOARD ?= REVO

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/ttyUSB0

#################################
# GNU ARM Embedded Toolchain
#################################
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
CP=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

#################################
# Working directories
#################################
ROOT			= ../..
SRC_DIR		 	= $(ROOT)/src
CMSIS_DIR	 	= $(ROOT)/lib/CMSIS
STDPERIPH_DIR	= $(ROOT)/lib/STM32F4xx_StdPeriph_Driver
USBCORE_DIR		= $(ROOT)/lib/STM32_USB_Device_Library/Core
USBOTG_DIR		= $(ROOT)/lib/STM32_USB_OTG_Driver
USBCDC_DIR		= $(ROOT)/lib/STM32_USB_Device_Library/Class/cdc
STARTUP_DIR		= $(ROOT)/lib/startup
BIN_DIR	 		= obj

#################################
# Source Files
#################################
VPATH 	 := $(VPATH):$(ROOT)
LDSCRIPT := $(STARTUP_DIR)/stm32_flash_f405_1024k.ld
ASOURCES := $(STARTUP_DIR)/startup_stm32f40_41xxx.s

VPATH	 := $(VAPTH):$(STDPERIPH_DIR)/src
VPATH 	 := $(VPATH):$(SRC_DIR)
VPATH 	 := $(VPATH):$(SRC_DIR)/vcp
VPATH	 := $(VPATH):$(USBCORE_DIR)/src:$(USBOTG_DIR)/src:$(USBCDC_DIR)/src
VPATH	 := $(VPATH):$(CMSIS_DIR)/CM4/CoreSupport:$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx

CMSIS_SRC	 	= $(notdir $(wildcard $(CMSIS_DIR)/CM4/CoreSupport/*.c \
				  $(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx/*.c))
STDPERIPH_SRC	= $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
USBCORE_SRC 	= $(notdir $(wildcard $(USBCORE_DIR)/src/*.c))
USBOTG_SRC 		= $(notdir $(wildcard $(USBOTG_DIR)/src/*.c))
USBCDC_SRC 		= $(notdir $(wildcard $(USBCDC_DIR)/src/*.c))
VCP_SRC 		= $(notdir $(wildcard $(SRC_DIR)/vcp/*.c))
PROCESSOR_SRC 	= $(notdir $(wildcard $(SRC_DIR)/*.c))

EXCLUDES = usb_bsp_template.c \
		   usb_conf_template.c \
		   usb_hcd_int.c \
		   usb_hcd.c \
		   usb_otg.c \
		   usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))
USBOTG_SRC := $(filter-out ${EXCLUDES}, $(USBOTG_SRC))


CSOURCES =  $(CMSIS_SRC) \
			$(STDPERIPH_SRC) \
			$(USBCORE_SRC) \
			$(USBOTG_SRC) \
			$(USBCDC_SRC) \
			$(VCP_SRC) \
			$(PROCESSOR_SRC) \
			main.cpp

CXXSOURCES  = 	$(SRC_DIR)/drv_i2c.cpp \
				$(SRC_DIR)/drv_led.cpp

INCLUDE_DIRS = 	$(SRC_DIR) \
				$(ROOT)/inc \
	   			$(STDPERIPH_DIR)/inc \
	   			$(USBOTG_DIR)/inc \
	   			$(USBCORE_DIR)/inc \
	   			$(USBCDC_DIR)/inc \
	   			$(CMSIS_DIR)/CM4/CoreSupport \
	   			$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx \
	   			$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx/Include \
	   			$(SRC_DIR)/vcp

#################################
# Object List
#################################
OBJECTS	=$(addsuffix .o,$(addprefix $(BIN_DIR)/$(TARGET)/,$(basename $(ASOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(BIN_DIR)/$(TARGET)/,$(basename $(CSOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(BIN_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))

#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(TARGET).elf
TARGET_HEX=$(BIN_DIR)/$(TARGET).hex

#################################
# Debug Config
#################################
#$(info ***** Building with Debug Symbols *****)
DEBUG_FLAGS = -ggdb3
OPTIMIZE 	= -Og

#################################
# Flags
#################################
MCFLAGS=-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
# DEBUG = -DDebug -ggdb3
DEFS=-DSTM32F40_41xxx -D__CORTEX_M4 -D__FPU_PRESENT -DWORDS_STACK_SIZE=200 -DUSE_STDPERIPH_DRIVER
CFLAGS=-c $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(DEBUG_FLAGS) $(addprefix -I,$(INCLUDE_DIRS)) -std=c99 -DTARGET_$(BOARD)
CXXFLAGS=-c $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(DEBUG_FLAGS) $(addprefix -I,$(INCLUDE_DIRS)) -std=c++11 -U__STRICT_ANSI__ -DTARGET_$(BOARD)
LDFLAGS =-T $(LDSCRIPT) $(MCFLAGS) -lm -nostartfiles -lc --specs=rdimon.specs $(ARCH_FLAGS)  $(LTO_FLAGS)  $(DEBUG_FLAGS) -static  -Wl,-gc-sections

#################################
# Build
#################################
$(TARGET_HEX): $(TARGET_ELF)
	$(CP) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

$(BIN_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CXX) -c -o $@ $(CXXFLAGS) $<

$(BIN_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

$(BIN_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<


##################################
# Recipes
#################################
.PHONY: flash clean

clean:
	rm -f $(OBJECTS) $(TARGET_ELF) $(TARGET_HEX) $(BIN_DIR)/output.map

flash: $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 921600 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 921600 $(SERIAL_DEVICE)
