
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

# Generic is used for cross-compiled systems
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR STM32F4)

# avoid an error because CMake can't test this compiler
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# CMake finds the rest of the compile stuff automatically, since it is a GCC comiler
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# If we ever support more boards, this will need to be set by an option
set(BOARD REVO)

set(CMAKE_C_FLAGS_DEBUG "-ggdb3 -Og")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "-ggdb3")

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto -fuse-linker-plugin ${CMAKE_C_FLAGS_RELEASE}")

#set(CXX_STRICT_FLAGS "-std=c++11 -pedantic -pedantic-errors -Werror -Wall -Wextra \
#  -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 -Wlogical-op \
#  -Wmissing-include-dirs -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wswitch-default \
#  -Wundef -Wunused -Wvariadic-macros -Wctor-dtor-privacy -Wnoexcept -Wold-style-cast \
#  -Woverloaded-virtual -Wsign-promo -Wstrict-null-sentinel")

set(FILE_SIZE_FLAGS "-ffunction-sections -fdata-sections -fno-exceptions")
set(CXX_FILE_SIZE_FLAGS "${FILE_SIZE_FLAGS} -fno-rtti")

set(MC_FLAGS "-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
    -fsingle-precision-constant -Wdouble-promotion")
set(DEFS "-DSTM32F40_41xxx -D__CORTEX_M4 -D__FPU_PRESENT -DWORDS_STACK_SIZE=200 \
    -DUSE_STDPERIPH_DRIVER -DTARGET_${BOARD}")

set(CMAKE_C_FLAGS "${MC_FLAGS} ${DEFS} ${FILE_SIZE_FLAGS} -std=c99")
set(CMAKE_CXX_FLAGS "${MC_FLAGS} ${DEFS} ${CXX_FILE_SIZE_FLAGS} ${CXX_STRICT_FLAGS} \
    -std=c++11")
set(CMAKE_ASM_FLAGS ${CMAKE_C_FLAGS})

set(CMAKE_EXE_LINKER_FLAGS "${MC_FLAGS} -T${CMAKE_CURRENT_LIST_DIR}/../lib/startup/stm32f405.ld \
    -lm -lc --specs=nano.specs --specs=rdimon.specs ${ARCH_FLAGS}  ${LTO_FLAGS}  -static  \
    -Wl,-gc-sections")
