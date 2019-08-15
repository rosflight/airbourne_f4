#ifndef _SYSTEM_HELPERS_H_
#define _SYSTEM_HELPERS_H_

#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_PIN_SOURCE_INVALID 0xFFFF
#define GPIO_TIM_AF_INVALID UINT8_MAX

/**
 * @brief system_helpers_dma_stream_to_dma_irqn
 *
 * Given a DMA_Stream, find its matching IRQn_Type aka interrupt number
 *
 * @param stream
 *      The DMA_stream to find the corresponding interrupt number for
 * @return IRQn_Type
 *      The interrupt number for the given stream. Or zero if failed to 
 *      find an interrupt number or stream was NULL
 */
IRQn_Type system_helpers_dma_stream_to_dma_irqn(DMA_Stream_TypeDef* stream);

/**
 * @brief system_helpers_gpio_pin_to_gpio_pinsource
 *
 * Given a GPIO_Pin, find its matching GPIO_PinSource
 *
 * @param gpio_pin
 *      The GPIO_pin to find the corresponding pin source for
 * @return uint16_t
 *      The corresponding pin source for the given GPIO pin, or
 *      @ref GPIO_PIN_SOURCE_INVALID if it failed to find a match
 */
uint16_t system_helpers_gpio_pin_to_gpio_pinsource(uint32_t gpio_pin);

/**
 * @brief system_helpers_gpio_af_tim_from_tim
 *
 * Given a TIM pointer find its matching GPIO_AF mode
 *
 * @param tim
 *      The TIM to find the corresponding GPIO_AF mode for
 * @return uint8_t
 *      The corresponding GPIO_AF for the given TIm, or
 *      @ref GPIO_TIM_AF_INVALID if no match was found or if tim is NULL
 */
uint8_t system_helpers_gpio_af_tim_from_tim(TIM_TypeDef* tim);

/**
 * @brief system_helpers_dma_stream_interrupt_clear_offset
 *
 * Given a DMA stream, find the shift required in a DMA interrupt
 * register (either LISR, LIFCR, HISR or HIFCR) to get to that stream's
 * interrupt flags.
 * ie: - Stream2 requires a left shift of 16 bits in LISR or LIFCR to access
 *       its flags
 *     - Stream5 requires a left shift of 6 bits in HISR or HIFCR to access 
 *       its flags
 *
 * @param stream
 *      The stream to find the corresponding interrupt offset for
 * @return uint8_t
 *      The interrupt register offset for the given DMA stream, or UINT8_MAX
 *      on failure.     
 */
uint8_t system_helpers_dma_stream_interrupt_flag_offset(DMA_Stream_TypeDef* stream);

#ifdef __cplusplus
}
#endif

#endif /* _SYSTEM_HELPERS_H_ */