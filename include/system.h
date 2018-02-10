
/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SYSTEM_H
#define SYSTEM_H

#define ARM_MATH_CM4

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <stdarg.h>

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

typedef struct {
	SPI_TypeDef* dev;
	GPIO_TypeDef* GPIO;
	uint8_t SCK_PinSource;
	uint16_t SCK_Pin;
	uint8_t MOSI_PinSource;
	uint16_t MOSI_Pin;
	uint8_t MISO_PinSource;
	uint16_t MISO_Pin;
	uint8_t GPIO_AF;
	IRQn_Type DMA_IRQn;
	DMA_Stream_TypeDef* Tx_DMA_Stream;
	DMA_Stream_TypeDef* Rx_DMA_Stream;
	uint32_t DMA_Channel;
	uint32_t Tx_DMA_TCIF;
	uint32_t Rx_DMA_TCIF;
} spi_hardware_struct_t;

typedef struct {
	I2C_TypeDef* dev;
	uint32_t I2C_ClockSpeed;
	IRQn_Type I2C_EV_IRQn;
	IRQn_Type I2C_ER_IRQn;
	GPIO_TypeDef* GPIO;
	uint8_t GPIO_AF;
	uint8_t SCL_PinSource;
	uint16_t SCL_Pin;
	uint8_t SDA_PinSource;
	uint16_t SDA_Pin;
	DMA_Stream_TypeDef* DMA_Stream;
	uint32_t DMA_Channel;
	IRQn_Type DMA_IRQn;
	uint32_t DMA_TCIF;
} i2c_hardware_struct_t;

typedef struct {
	GPIO_TypeDef* GPIO;
	uint16_t GPIO_Pin;
	uint8_t GPIO_PinSource;
	TIM_TypeDef* TIM;
	uint8_t TIM_Channel;
	uint8_t GIPO_AF_TIM;
	IRQn_Type TIM_IRQn;
	uint16_t TIM_IT_CC;
} pwm_hardware_struct_t;

typedef struct {
	USART_TypeDef* dev;
	GPIO_TypeDef* GPIO;
	uint16_t Rx_Pin;
	uint16_t Tx_Pin;
	uint8_t Rx_PinSource;
	uint8_t Tx_PinSource;
	uint8_t GPIO_AF;
	IRQn_Type USART_IRQn;
	IRQn_Type Rx_DMA_IRQn;
	IRQn_Type Tx_DMA_IRQn;
	DMA_Stream_TypeDef* Rx_DMA_Stream;
	DMA_Stream_TypeDef* Tx_DMA_Stream;
	uint32_t DMA_CHannel;
	uint32_t DMA_Rx_IT_Bit;
	uint32_t DMA_Tx_IT_Bit;
} uart_hardware_struct_t;

#ifdef __cplusplus
extern "C" {
#endif

void systemInit(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint64_t micros(void);
uint32_t millis(void);

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(void);

#ifdef __cplusplus
}
#endif

#endif //SYSTEM_H
