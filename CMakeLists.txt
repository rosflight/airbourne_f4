
###############################################################################
# Copyright (c) 2020 - James Jackson
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

cmake_minimum_required(VERSION 3.10)
project(airbourne C CXX ASM)

# Project-Wide Settings
# include(cmake/Options.cmake)
include(cmake/ExampleRule.cmake)
include(cmake/CompileFlags.cmake)
include(cmake/ToolChain.cmake)

add_subdirectory(lib)

add_library(airbourne_core
    src/advanced.cpp
    src/backup_sram.cpp
    src/gpio.cpp
    src/led.cpp
    src/mpu6000.cpp
    src/pwm.cpp
    src/spi.cpp
    src/uart.cpp
    src/analog_digital_converter.cpp
    src/battery_monitor.cpp
    src/hmc5883l.cpp
    src/M25P16.cpp
    src/ms4525.cpp
    src/rc_ppm.cpp
    src/system.c
    src/ublox.cpp
    src/analog_pin.cpp
    src/eeprom.cpp
    src/i2c.cpp
    src/mb1242.cpp
    src/ms5611.cpp
    src/rc_sbus.cpp
    src/system_stm32f4xx.c
    src/vcp.cpp)
target_include_directories(airbourne_core PUBLIC include)
target_link_libraries(airbourne_core CMSIS StdPeriphLib usb)

add_subdirectory(examples)
