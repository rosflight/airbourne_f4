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

add_library(CMSIS CMSIS/CM4/DeviceSupport/ST/STM32F4xx/stm32f4xx_gpio.c
                  CMSIS/CM4/DeviceSupport/ST/STM32F4xx/stm32f4xx_rcc.c)
target_include_directories(CMSIS PUBLIC
    CMSIS/CM4/CoreSupport
    CMSIS/CM4/DeviceSupport/ST/STM32F4xx
    CMSIS/CM4/DeviceSupport/ST/STM32F4xx/Include
    STM32F4xx_StdPeriph_Driver/inc/)

add_library(StdPeriphLib
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
    STM32F4xx_StdPeriph_Driver/src/misc.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
    STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c)
target_link_libraries(StdPeriphLib CMSIS)
target_include_directories(StdPeriphLib PUBLIC
STM32F4xx_StdPeriph_Driver/inc)

add_library(usb
    STM32_USB_Device_Library/Core/src/usbd_req.c
    STM32_USB_Device_Library/Core/src/usbd_core.c
    STM32_USB_Device_Library/Core/src/usbd_ioreq.c
    STM32_USB_OTG_Driver/src/usb_dcd_int.c
    STM32_USB_OTG_Driver/src/usb_core.c
    STM32_USB_OTG_Driver/src/usb_dcd.c
    STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c
    vcp/usb_bsp.c
    vcp/usbd_usr.c
    vcp/stm32f4xx_it.c
    vcp/usbd_desc.c
    vcp/usbd_cdc_vcp.c)
target_include_directories(usb PUBLIC
    STM32_USB_Device_Library/Class/cdc/inc
    STM32_USB_Device_Library/Core/inc
    STM32_USB_OTG_Driver/inc/
    vcp
    ../include)
target_link_libraries(usb CMSIS StdPeriphLib)

add_library(printf printf/printf.cpp)
target_include_directories(printf PUBLIC printf)
