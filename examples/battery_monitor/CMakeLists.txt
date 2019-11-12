cmake_minimum_required(VERSION 3.10)

project(Analog C CXX ASM)

set(DEBUG GDB)
set(SERIAL_DEVICE /dev/ttyACM0)

project(BatteryMonitor)

#set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
get_filename_component(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
get_filename_component(ROOT_DIR ${ROOT_DIR} DIRECTORY)
message(${ROOT_DIR})
set(USBCORE_DIR ${ROOT_DIR}/lib/STM32_USB_Device_Library/Core)
set(VCP_DIR ${ROOT_DIR}/lib/vcp)
set(STDPERIPH_DIR ${ROOT_DIR}/lib/STM32F4xx_StdPeriph_Driver)
set(CMSIS_DIR1 ${ROOT_DIR}/lib/CMSIS/CM4/CoreSupport)
set(CMSIS_DIR2 ${ROOT_DIR}/lib/CMSIS/CM4/DeviceSupport/ST/STM32F4xx)
set(USBCORE_DIR ${ROOT_DIR}/lib/STM32_USB_Device_Library/Core)
set(USBOTG_DIR ${ROOT_DIR}/lib/STM32_USB_OTG_Driver)
set(USBCDC_DIR ${ROOT_DIR}/lib/STM32_USB_Device_Library/Class/cdc)
set(STARTUP_DIR ${ROOT_DIR}/lib/startup)

include_directories(${ROOT_DIR}/include)
include_directories(${CMSIS_DIR1})
include_directories(${CMSIS_DIR2})
include_directories(${STDPERIPH_DIR}/inc/)
include_directories(${USBCDC_DIR}/inc)
include_directories(${USBCORE_DIR}/inc)
include_directories(${VCP_DIR})
include_directories(${USBOTG_DIR}/inc)
include_directories(${ROOT_DIR}/lib/printf)

file(GLOB USBCORE_SRC ${USBCORE_DIR}/src/*.c LIST_DIRECTORIES false)
#file(GLOB USBOTG_SRC ${USBOTG_DIR}/src/*.c LIST_DIRECTORIES false)
set(USBOTG_SRC
	"${USBOTG_DIR}/src/usb_core.c"
	"${USBOTG_DIR}/src/usb_dcd.c"
	"${USBOTG_DIR}/src/usb_dcd_int.c"
	)
#file(GLOB USBCDC_SRC ${USBCDC_DIR}/src/*.c LIST_DIRECTORIES false)
set(USBCDC_SRC ${USBCDC_DIR}/src/usbd_cdc_core.c)
file(GLOB VCP_SRC ${VCP_DIR}/*.c LIST_DIRECTORIES false)
#file(GLOB STDPERIPH_SRC ${STDPERIPH_DIR}/src/*.c LIST_DIRECTORIES false)
set(STDPERIPH_SRC
	${STDPERIPH_DIR}/src/misc.c
	${STDPERIPH_DIR}/src/stm32f4xx_adc.c
	${STDPERIPH_DIR}/src/stm32f4xx_cryp.c
	${STDPERIPH_DIR}/src/stm32f4xx_cryp_aes.c
	${STDPERIPH_DIR}/src/stm32f4xx_cryp_des.c
	${STDPERIPH_DIR}/src/stm32f4xx_cryp_tdes.c
	${STDPERIPH_DIR}/src/stm32f4xx_dac.c
	${STDPERIPH_DIR}/src/stm32f4xx_dbgmcu.c
	${STDPERIPH_DIR}/src/stm32f4xx_dcmi.c
	${STDPERIPH_DIR}/src/stm32f4xx_dma.c
	${STDPERIPH_DIR}/src/stm32f4xx_exti.c
	${STDPERIPH_DIR}/src/stm32f4xx_flash.c
	${STDPERIPH_DIR}/src/stm32f4xx_hash.c
	${STDPERIPH_DIR}/src/stm32f4xx_hash_md5.c
	${STDPERIPH_DIR}/src/stm32f4xx_hash_sha1.c
	${STDPERIPH_DIR}/src/stm32f4xx_i2c.c
	${STDPERIPH_DIR}/src/stm32f4xx_iwdg.c
	${STDPERIPH_DIR}/src/stm32f4xx_pwr.c
	${STDPERIPH_DIR}/src/stm32f4xx_rng.c
	${STDPERIPH_DIR}/src/stm32f4xx_rtc.c
	${STDPERIPH_DIR}/src/stm32f4xx_sdio.c
	${STDPERIPH_DIR}/src/stm32f4xx_spi.c
	${STDPERIPH_DIR}/src/stm32f4xx_syscfg.c
	${STDPERIPH_DIR}/src/stm32f4xx_tim.c
	${STDPERIPH_DIR}/src/stm32f4xx_usart.c
	${STDPERIPH_DIR}/src/stm32f4xx_wwdg.c
)
file(GLOB CMSIS_SRC1 ${CMSIS_DIR1}/*.c LIST_DIRECTORIES false)
file(GLOB CMSIS_SRC2 ${CMSIS_DIR2}/*.c LIST_DIRECTORIES false)

set(ASM_SCRIPT ${STARTUP_DIR}/stm32f405.s)

set(LINKER_SCRIPT ${STARTUP_DIR}/stm32f405.ld)
set(CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_SCRIPT} ${CMAKE_EXE_LINKER_FLAGS}")

set(ELF_TARGET ${PROJECT_NAME}.elf)
message(${ELF_TARGET})

# TODO add header files to this list, so as to recompile when they change
add_executable(${ELF_TARGET}
	main.cpp
	${ROOT_DIR}/src/vcp.cpp
        ${ROOT_DIR}/src/analog_digital_converter.cpp
        ${ROOT_DIR}/src/analog_pin.cpp
        ${ROOT_DIR}/src/battery_monitor.cpp
        ${ROOT_DIR}/src/system.c
	${ROOT_DIR}/src/system_stm32f4xx.c
	${ROOT_DIR}/src/led.cpp
	${ROOT_DIR}/src/gpio.cpp
	${ROOT_DIR}/src/i2c.cpp
	${ROOT_DIR}/src/hmc5883l.cpp
	${ROOT_DIR}/lib/printf/printf.cpp
	${USBCORE_SRC}
	${USBOTG_SRC}
	${USBCDC_SRC}
	${VCP_SRC}
	${STDPERIPH_SRC}
	${CMSIS_SRC1}
	${CMSIS_SRC2}
	${ASM_SCRIPT}
)
set_target_properties(${ELF_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})
