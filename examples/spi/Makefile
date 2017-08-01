###############################################################################
# Generic Makefile Template for C/C++ for use with STM32 Microcontrollers
#
# Copyright (c) 2016 - James Jackson
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the <organization> nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TARGET = blink

BOARD ?= REVO

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/ttyUSB0

#################################
# GNU ARM Embedded Toolchain
#################################
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
NM=arm-none-eabi-nm
SIZE=arm-none-eabi-size
A2L=arm-none-eabi-addr2line

#################################
# Working directories
#################################
AIRBOURNE_DIR 	= ../../
ROOT		 	= .
SRC_DIR		 	= $(ROOT)
CMSIS_DIR	 	= $(AIRBOURNE_DIR)/lib/CMSIS
MATH_DIR	 	= $(AIRBOURNE_DIR)/lib/math
STDPERIPH_DIR	= $(AIRBOURNE_DIR)/lib/STM32F4xx_StdPeriph_Driver
USBCORE_DIR	= $(AIRBOURNE_DIR)/lib/STM32_USB_Device_Library/Core
USBOTG_DIR	= $(AIRBOURNE_DIR)/lib/STM32_USB_OTG_Driver
USBCDC_DIR	= $(AIRBOURNE_DIR)/lib/STM32_USB_Device_Library/Class/cdc
VCP_DIR		= $(AIRBOURNE_DIR)/lib/vcp
OBJECT_DIR	= obj
BIN_DIR	 	= obj

#################################
# Source Files
#################################
VPATH := $(VPATH):$(AIRBOURNE_DIR)
LDSCRIPT = $(AIRBOURNE_DIR)processor/stm32_flash_f405_1024k.ld
ASOURCES = $(AIRBOURNE_DIR)processor/startup_stm32f40_41xxx.s

VPATH		:= $(SRC_DIR):$(SRC_DIR)/startups

# Search path and source files for the CMSIS sources
VPATH		:= 	$(VPATH):$(CMSIS_DIR)/CM4/CoreSupport:$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx
CMSIS_SRC	 = 	$(notdir $(wildcard $(CMSIS_DIR)/CM4/CoreSupport/*.c \
				$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx/*.c))

# Search path and source files for the ST stdperiph library and exclude files we don't need
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES = stm32f4xx_crc.c \
		stm32f4xx_can.c \
		stm32f4xx_fmc.c \
		stm32f4xx_fsmc.c \
		stm32f4xx_sai.c \
		stm32f4xx_cec.c \
		stm32f4xx_dsi.c \
		stm32f4xx_flash_ramfunc.c \
		stm32f4xx_fmpi2c.c \
		stm32f4xx_lptim.c \
		stm32f4xx_qspi.c
STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))


# Search path and source files for the USB libraries
VPATH		:= $(VPATH):$(USBCORE_DIR)/src:$(USBOTG_DIR)/src:$(USBCDC_DIR)/src
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/src/*.c))
USBOTG_SRC = $(notdir $(wildcard $(USBOTG_DIR)/src/*.c))
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/src/*.c))

# Add VCP source files
VPATH := $(VPATH):$(VCP_DIR)
VCP_SRC = $(notdir $(wildcard $(VCP_DIR)/*.c))

# Add processor-specific files
VPATH := $(VPATH):$(AIRBOURNE_DIR)/src
PROCESSOR_SRC = $(notdir $(wildcard $(AIRBOURNE_DIR)/src/*.c))

EXCLUDES	= usb_bsp_template.c \
			  usb_conf_template.c \
			  usb_hcd_int.c \
			  usb_hcd.c \
			  usb_otg.c \
			  usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))
USBOTG_SRC := $(filter-out ${EXCLUDES}, $(USBOTG_SRC))

# Search path and source files for the math library
VPATH		:= $(VPATH):$(MATH_DIR)
MATH_SRC	 = $(notdir $(wildcard $(MATH_DIR)/*.cpp))


# Make a list of source files and includes
CSOURCES =  $(CMSIS_SRC) \
	    	$(STDPERIPH_SRC) \
	    	$(USBCORE_SRC) \
	    	$(USBOTG_SRC) \
	    	$(USBCDC_SRC) \
                $(PROCESSOR_SRC) \
                $(VCP_SRC) \
                $(ROOT)/main.cpp


CXXSOURCES = $(AIRBOURNE_DIR)/src/gpio.cpp \
             $(AIRBOURNE_DIR)/src/VCP.cpp \
             $(AIRBOURNE_DIR)/src/spi.cpp \
             $(AIRBOURNE_DIR)/src/mpu6000_spi.cpp \
             $(MATH_SRC)

INCLUDE_DIRS	 = 	$(SRC_DIR) \
					$(AIRBOURNE_DIR)/include \
		   			$(MATH_DIR) \
		   			$(STDPERIPH_DIR)/inc \
		   			$(USBOTG_DIR)/inc \
		   			$(USBCORE_DIR)/inc \
		   			$(USBCDC_DIR)/inc \
		   			$(CMSIS_DIR)/CM4/CoreSupport \
		   			$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx \
		   			$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx/Include \
		   			$(VCP_DIR)




#################################
# Object List
#################################
OBJECTS=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(ASOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(CSOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))

#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(TARGET).elf
TARGET_HEX=$(BIN_DIR)/$(TARGET).hex

#################################
# Debug Config
#################################
# ifeq ($(DEBUG), GDB)
DEBUG_FLAGS = -ggdb3
OPTIMIZE = -Og
# else
# OPTIMIZE = -Os
# LTO_FLAGS = -flto -fuse-linker-plugin $(OPTIMIZE)
# endif

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

$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CXX) -c -o $@ $(CXXFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<


#################################
# Recipes
#################################
.PHONY: all flash clean

clean:
	rm -f $(OBJECTS) $(TARGET_ELF) $(TARGET_HEX) $(BIN_DIR)/output.map

flash: $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 921600 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 921600 $(SERIAL_DEVICE)



