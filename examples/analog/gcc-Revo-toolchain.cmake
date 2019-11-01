cmake_minimum_required(VERSION 3.10)

message("LOADING STM32F4 TOOLCHAIN")
# Generic is used for sytems that can't run CMake
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
SET(CMAKE_SYSTEM_PROCESSOR STM32F4)

#avoid an error because CMake can't test this compiler
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

#CMake finds the rest of the compile stuff automatically, since it is a GCC comiler
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
SET(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(BOARD REVO)

set(DEBUG_FLAG "-ggdb3")
set(DEBUG_OPTIMIZE "-Og")
set(CMAKE_C_FLAGS_DEBUG "${DEBUG_FLAG} ${DEBUG_OPTIMIZE}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${DEBUG_FLAG}")

set(CMAKE_C_FLAGS_RELEASE "-g0 -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto -fuse-linker-plugin ${CMAKE_C_FLAGS_RELEASE}")

# I don't know why these were commented out in the original make files,
# but I did so here as well
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
# The linker script will have to be defined in the individual CMakeLists.txt
set(CMAKE_EXE_LINKER_FLAGS "${MC_FLAGS} -lm -lc --specs=nano.specs --specs=rdimon.specs \
    ${ARCH_FLAGS}  ${LTO_FLAGS}  -static  -Wl,-gc-sections")
