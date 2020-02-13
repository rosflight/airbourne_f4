set(MCFLAGS "-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
            -fsingle-precision-constant -Wdouble-promotion")
set(DEFS "-DSTM32F40_41xxx -D__CORTEX_M4 -D__FPU_PRESENT -DWORDS_STACK_SIZE=200 \
         -DUSE_STDPERIPH_DRIVER -DTARGET_${BOARD}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEFS} ${MCFLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} ${DEFS} ${MCFLAGS}")
## TODO: LTO flags
