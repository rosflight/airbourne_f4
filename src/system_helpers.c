#include "system_helpers.h"
#include <ctype.h> // for NULL and intptr_t
#include "stm32f4xx_gpio.h"

IRQn_Type system_helpers_dma_stream_to_dma_irqn(DMA_Stream_TypeDef* stream) {
    if (stream == NULL) {
        return 0;
    }

    IRQn_Type result = 0;
    switch ((intptr_t)stream) {
        case (intptr_t)DMA1_Stream0:
            result = DMA1_Stream0_IRQn;
            break;
        case (intptr_t)DMA1_Stream1:
            result = DMA1_Stream1_IRQn;
            break;
        case (intptr_t)DMA1_Stream2:
            result = DMA1_Stream2_IRQn;
            break;
        case (intptr_t)DMA1_Stream3:
            result = DMA1_Stream3_IRQn;
            break;
        case (intptr_t)DMA1_Stream4:
            result = DMA1_Stream4_IRQn;
            break;
        case (intptr_t)DMA1_Stream5:
            result = DMA1_Stream5_IRQn;
            break;
        case (intptr_t)DMA1_Stream6:
            result = DMA1_Stream6_IRQn;
            break;
        case (intptr_t)DMA1_Stream7:
            return DMA1_Stream7_IRQn;
            break;
        case (intptr_t)DMA2_Stream0:
            result = DMA2_Stream0_IRQn;
            break;
        case (intptr_t)DMA2_Stream1:
            result = DMA2_Stream1_IRQn;
            break;
        case (intptr_t)DMA2_Stream2:
            result = DMA2_Stream2_IRQn;
            break;
        case (intptr_t)DMA2_Stream3:
            result = DMA2_Stream3_IRQn;
            break;
        case (intptr_t)DMA2_Stream4:
            result = DMA2_Stream4_IRQn;
            break;
        case (intptr_t)DMA2_Stream5:
            result = DMA2_Stream5_IRQn;
            break;
        case (intptr_t)DMA2_Stream6:
            result = DMA2_Stream6_IRQn;
            break;
        case (intptr_t)DMA2_Stream7:
            result = DMA2_Stream7_IRQn;
            break;
    }
    return result;
}

uint16_t system_helpers_gpio_pin_to_gpio_pinsource(uint32_t gpio_pin) {
    uint16_t result = GPIO_PIN_SOURCE_INVALID;
    switch (gpio_pin) {
        case GPIO_Pin_0:
            result = GPIO_PinSource0;
            break;
        case GPIO_Pin_1:
            result = GPIO_PinSource1;
            break;
        case GPIO_Pin_2:
            result = GPIO_PinSource2;
            break;
        case GPIO_Pin_3:
            result = GPIO_PinSource3;
            break;
        case GPIO_Pin_4:
            result = GPIO_PinSource4;
            break;
        case GPIO_Pin_5:
            result = GPIO_PinSource5;
            break;
        case GPIO_Pin_6:
            result = GPIO_PinSource6;
            break;
        case GPIO_Pin_7:
            result = GPIO_PinSource7;
            break;
        case GPIO_Pin_8:
            result = GPIO_PinSource8;
            break;
        case GPIO_Pin_9:
            result = GPIO_PinSource9;
            break;
        case GPIO_Pin_10:
            result = GPIO_PinSource10;
            break;
        case GPIO_Pin_11:
            result = GPIO_PinSource11;
            break;
        case GPIO_Pin_12:
            result = GPIO_PinSource12;
            break;
        case GPIO_Pin_13:
            result = GPIO_PinSource13;
            break;
        case GPIO_Pin_14:
            result = GPIO_PinSource14;
            break;
        case GPIO_Pin_15:
            result = GPIO_PinSource15;
            break;
    }
    return result;
}

uint8_t system_helpers_gpio_af_tim_from_tim(TIM_TypeDef* tim) {
    if (tim == NULL) {
        return GPIO_TIM_AF_INVALID;
    }

    uint8_t result = GPIO_TIM_AF_INVALID;

    switch ((intptr_t)tim) {
        case (intptr_t)TIM1:
            result = GPIO_AF_TIM1;
            break;
        case (intptr_t)TIM2:
            result = GPIO_AF_TIM2;
            break;
        case (intptr_t)TIM3:
            result = GPIO_AF_TIM3;
            break;
        case (intptr_t)TIM4:
            result = GPIO_AF_TIM4;
            break;
        case (intptr_t)TIM5:
            result = GPIO_AF_TIM5;
            break;
        case (intptr_t)TIM8:
            result = GPIO_AF_TIM8;
            break;
        case (intptr_t)TIM9:
            result = GPIO_AF_TIM9;
            break;
        case (intptr_t)TIM10:
            result = GPIO_AF_TIM10;
            break;
        case (intptr_t)TIM11:
            result = GPIO_AF_TIM11;
            break;
    }
    return result;
}

uint8_t system_helpers_dma_stream_interrupt_flag_offset(DMA_Stream_TypeDef* stream) {
     if (stream == NULL) {
        return UINT8_MAX;
    }

    uint8_t result = UINT8_MAX;
    switch ((intptr_t)stream) {
        case (intptr_t)DMA1_Stream0:
        case (intptr_t)DMA2_Stream0:
        case (intptr_t)DMA1_Stream4:
        case (intptr_t)DMA2_Stream4:
            result = 0;
            break;
        case (intptr_t)DMA1_Stream1:
        case (intptr_t)DMA2_Stream1:
        case (intptr_t)DMA1_Stream5:
        case (intptr_t)DMA2_Stream5:
            result = 6;
            break;
        case (intptr_t)DMA1_Stream2:
        case (intptr_t)DMA2_Stream2:
        case (intptr_t)DMA1_Stream6:
        case (intptr_t)DMA2_Stream6:
            result = 16;
            break;
        case (intptr_t)DMA1_Stream3:
        case (intptr_t)DMA2_Stream3:
        case (intptr_t)DMA1_Stream7:
        case (intptr_t)DMA2_Stream7:
            result = 22;
            break;
    }
    return result;
}