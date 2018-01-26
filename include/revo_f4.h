
#ifndef REVO_F4_H
#define REVO_F4_H

#include "system.h"

///
/// This file defines the hardware configuration used in the C++ abstraction layer
/// All chip-specific configuration should happen here
///

/////////////////////// UART CONFIG ///////////////////////
#define NUM_UART 3
//typedef struct {
//	USART_TypeDef* dev;
//	GPIO_TypeDef* GPIO;
//	uint16_t Rx_Pin;
//	uint16_t Tx_Pin;
//	uint8_t Rx_PinSource;
//	uint8_t Tx_PinSource;
//	uint8_t GPIO_AF;
//	IRQn_Type USART_IRQn;
//	IRQn_Type Rx_DMA_IRQn;
//	IRQn_Type Tx_DMA_IRQn;
//	DMA_Stream_TypeDef* Rx_DMA_Stream;
//	DMA_Stream_TypeDef* Tx_DMA_Stream;
//	uint32_t DMA_CHannel;
//	uint32_t DMA_Rx_IT_Bit;
//	uint32_t DMA_Tx_IT_Bit;
//} uart_hardware_struct_t;
const uart_hardware_struct_t uart_config[NUM_UART] =
{
  {USART1, GPIOA, GPIO_Pin_10, GPIO_Pin_9, GPIO_PinSource10, GPIO_PinSource9,
   GPIO_AF_USART1, USART1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream7_IRQn, DMA2_Stream2,
   DMA2_Stream7, DMA_Channel_4, DMA_IT_TCIF2, DMA_IT_TCIF7},
  {USART2, GPIOA, GPIO_Pin_10, GPIO_Pin_9, GPIO_PinSource10, GPIO_PinSource9,
   GPIO_AF_USART2, USART2_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream5,
   DMA1_Stream6, DMA_Channel_5, DMA_IT_TCIF5, DMA_IT_TCIF6},
  {USART3, GPIOB, GPIO_Pin_10, GPIO_Pin_9, GPIO_PinSource10, GPIO_PinSource9,
   GPIO_AF_USART3, USART3_IRQn, DMA1_Stream1_IRQn, DMA1_Stream3_IRQn, DMA1_Stream1,
   DMA1_Stream3, DMA_Channel_5, DMA_IT_TCIF1, DMA_IT_TCIF3},
};


/////////////////////// SPI CONFIG ///////////////////////
#define NUM_SPI 3
//typedef struct {
//	SPI_TypeDef* dev;
//	GPIO_TypeDef* GPIO;
//	uint8_t SCK_PinSource;
//	uint16_t SCK_Pin;
//	uint8_t MOSI_PinSource;
//	uint16_t MOSI_Pin;
//	uint8_t MISO_PinSource;
//	uint16_t MISO_Pin;
//	uint8_t GPIO_AF;
//	IRQn_Type DMA_IRQn;
//	DMA_Stream_TypeDef* Tx_DMA_Stream;
//	DMA_Stream_TypeDef* Rx_DMA_Stream;
//	uint32_t DMA_Channel;
//	uint32_t Tx_DMA_TCIF;
//	uint32_t Rx_DMA_TCIF;
//} spi_hardware_struct_t;
const spi_hardware_struct_t spi_config[NUM_SPI] =
{
  {
  	.dev = SPI1,
   	.GPIO = GPIOA,
   	.SCK_PinSource = GPIO_PinSource5,
   	.SCK_Pin = GPIO_Pin_5,
   	.MOSI_PinSource = GPIO_PinSource6,
   	.MOSI_Pin = GPIO_Pin_6,
   	.MISO_PinSource = GPIO_PinSource7,
   	.MISO_Pin = GPIO_Pin_7,
   	.GPIO_AF = GPIO_AF_SPI1,
   	.DMA_IRQn = DMA2_Stream3_IRQn,
   	.Tx_DMA_Stream = DMA2_Stream3,
   	.Rx_DMA_Stream = DMA2_Stream2,
   	.DMA_Channel = DMA_Channel_3,
    .Tx_DMA_TCIF = DMA_FLAG_TCIF3,
    .Rx_DMA_TCIF = DMA_FLAG_TCIF2
   }, {	
  	.dev = SPI2, 
  	.GPIO = GPIOB, 
  	.SCK_PinSource = GPIO_PinSource13, 
  	.SCK_Pin = GPIO_Pin_13, 
  	.MOSI_PinSource = GPIO_PinSource15, 
  	.MOSI_Pin = GPIO_Pin_15,
   	.MISO_PinSource = GPIO_PinSource14, 
   	.MISO_Pin = GPIO_Pin_14, 
   	.GPIO_AF = GPIO_AF_SPI2, 
   	.DMA_IRQn = DMA1_Stream4_IRQn, 
   	.Tx_DMA_Stream = DMA1_Stream4,
   	.Rx_DMA_Stream = DMA1_Stream3, 
   	.DMA_Channel = DMA_Channel_0, 
    .Tx_DMA_TCIF = DMA_FLAG_TCIF4,
    .Rx_DMA_TCIF = DMA_FLAG_TCIF3
   }, {
  	.dev = SPI3, 
  	.GPIO = GPIOC, 
  	.SCK_PinSource = GPIO_PinSource10, 
  	.SCK_Pin = GPIO_Pin_10, 
  	.MOSI_PinSource = GPIO_PinSource12, 
  	.MOSI_Pin = GPIO_Pin_12,
  	.MISO_PinSource = GPIO_PinSource11, 
  	.MISO_Pin = GPIO_Pin_11, 
  	.GPIO_AF = GPIO_AF_SPI3, 
  	.DMA_IRQn = DMA1_Stream5_IRQn, 
  	.Tx_DMA_Stream = DMA1_Stream5,
  	.Rx_DMA_Stream = DMA1_Stream2, 
  	.DMA_Channel = DMA_Channel_0, 
    .Tx_DMA_TCIF = DMA_FLAG_TCIF5,
    .Rx_DMA_TCIF = DMA_FLAG_TCIF2}
};
#define MPU6000_SPI 0
#define MPU6000_CS_GPIO GPIOA
#define MPU6000_CS_PIN GPIO_Pin_4

#define FLASH_SPI 2
#define FLASH_CS_GPIO GPIOB
#define FLASH_CS_PIN GPIO_Pin_3

/////////////////////// I2C CONFIG ///////////////////////
#define NUM_I2C 2
const i2c_hardware_struct_t i2c_config[NUM_I2C] = {
  {
    .dev = I2C1,
    .I2C_ClockSpeed = 400000,
    .I2C_EV_IRQn = I2C1_EV_IRQn,
    .I2C_ER_IRQn = I2C1_ER_IRQn,
    .GPIO = GPIOB,
    .GPIO_AF = GPIO_AF_I2C1,
    .SCL_PinSource = GPIO_PinSource8,
    .SCL_Pin = GPIO_Pin_8,
    .SDA_PinSource = GPIO_PinSource9,
    .SDA_Pin = GPIO_Pin_9,
    .DMA_Stream = DMA1_Stream0,
    .DMA_Channel = DMA_Channel_1,
    .DMA_IRQn = DMA1_Stream0_IRQn,
    .DMA_TCIF = DMA_FLAG_TCIF0
  },

  {
    .dev = I2C2,
    .I2C_ClockSpeed = 400000,
    .I2C_EV_IRQn = I2C2_EV_IRQn,
    .I2C_ER_IRQn = I2C2_ER_IRQn,
    .GPIO = GPIOB,
    .GPIO_AF = GPIO_AF_I2C2,
    .SCL_PinSource = GPIO_PinSource10,
    .SCL_Pin = GPIO_Pin_10,
    .SDA_PinSource = GPIO_PinSource11,
    .SDA_Pin = GPIO_Pin_11,
    .DMA_Stream = DMA1_Stream2,
    .DMA_Channel = DMA_Channel_7,
    .DMA_IRQn = DMA1_Stream2_IRQn,
    .DMA_TCIF = DMA_FLAG_TCIF2
  }
};
#define MAG_I2C 0
#define BARO_I2C 0
#define EXTERNAL_I2C 1

/////////////////////// LED CONFIG ///////////////////////
#define LED1_GPIO	GPIOB
#define LED1_PIN	GPIO_Pin_4
#define LED2_GPIO	GPIOB
#define LED2_PIN	GPIO_Pin_5

/////////////////////// PWM CONFIG ///////////////////////
#define PWM_NUM_CHANNELS 13
#define PWM_NUM_OUTPUTS 11
//typedef struct {
//	GPIO_TypeDef* GPIO;
//	uint16_t GPIO_Pin;
//	uint8_t GPIO_PinSource;
//	TIM_TypeDef* TIM;
//	uint8_t TIM_Channel;
//	uint8_t GIPO_AF_TIM;
//	IRQn_Type TIM_IRQn;
//	uint16_t TIM_IT_CC;
//} pwm_hardware_struct_t;
const pwm_hardware_struct_t pwm_config[PWM_NUM_CHANNELS] =
{
  {GPIOB, GPIO_Pin_0,  GPIO_PinSource0,  TIM3,  TIM_Channel_3, GPIO_AF_TIM3, TIM3_IRQn, TIM_IT_CC3, }, // PWM1
  {GPIOB, GPIO_Pin_1,  GPIO_PinSource1,  TIM3,  TIM_Channel_4, GPIO_AF_TIM3, TIM3_IRQn, TIM_IT_CC4, }, // PWM2
  {GPIOA, GPIO_Pin_3,  GPIO_PinSource3,  TIM9,  TIM_Channel_2, GPIO_AF_TIM9, TIM1_BRK_TIM9_IRQn, TIM_IT_CC2, }, // PWM3
  {GPIOA, GPIO_Pin_2,  GPIO_PinSource2,  TIM2,  TIM_Channel_3, GPIO_AF_TIM2, TIM2_IRQn, TIM_IT_CC3, }, // PWM4
  {GPIOA, GPIO_Pin_1,  GPIO_PinSource1,  TIM5,  TIM_Channel_2, GPIO_AF_TIM5, TIM5_IRQn, TIM_IT_CC2, }, // PWM5
  {GPIOA, GPIO_Pin_0,  GPIO_PinSource0,  TIM5,  TIM_Channel_1, GPIO_AF_TIM5, TIM5_IRQn, TIM_IT_CC1, }, // PWM6
  {GPIOC, GPIO_Pin_9,  GPIO_PinSource9,  TIM8,  TIM_Channel_4, GPIO_AF_TIM8, TIM8_CC_IRQn, TIM_IT_CC4, }, // RC 6 (Flexi-10)
  {GPIOC, GPIO_Pin_8,  GPIO_PinSource8,  TIM8,  TIM_Channel_3, GPIO_AF_TIM8, TIM8_CC_IRQn, TIM_IT_CC3, }, // RC 5 (Flexi-9)
  {GPIOC, GPIO_Pin_7,  GPIO_PinSource7,  TIM8,  TIM_Channel_2, GPIO_AF_TIM8, TIM8_CC_IRQn, TIM_IT_CC2, }, // RC 4 (Flexi-8)
  {GPIOC, GPIO_Pin_6,  GPIO_PinSource6,  TIM8,  TIM_Channel_1, GPIO_AF_TIM8, TIM8_CC_IRQn, TIM_IT_CC1, }, // RC 3 (Flexi-7)
  {GPIOB, GPIO_Pin_15, GPIO_PinSource15, TIM12, TIM_Channel_2, GPIO_AF_TIM12, TIM8_BRK_TIM12_IRQn, TIM_IT_CC2, }, // RC2 (Flexi-6)
  {GPIOB, GPIO_Pin_14, GPIO_PinSource14, TIM12, TIM_Channel_1, GPIO_AF_TIM12, TIM8_BRK_TIM12_IRQn, TIM_IT_CC1, }, // RC1 (Flexi-5) // Used for PPM RC
  {GPIOA, GPIO_Pin_8,  GPIO_PinSource8,  TIM1,  TIM_Channel_1, GPIO_AF_TIM1, TIM1_CC_IRQn, TIM_IT_CC1, }, // Buzzer
};
#define RC_PPM_PIN 11
#define PPM_RC_IQRHandler TIM8_BRK_TIM12_IRQHandler

#endif // REVO_F4_H
