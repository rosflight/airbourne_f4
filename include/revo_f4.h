
#ifndef REVO_F4_H
#define REVO_F4_H

#include "system.h"

///
/// This file defines the hardware configuration used in the C++ abstraction layer
/// All chip-specific configuration should happen here
///

/////////////////////// UART CONFIG ///////////////////////
#define NUM_UART 1
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
   GPIO_AF_USART1, USART1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream7_IRQn, DMA2_Stream2,
   DMA2_Stream7, DMA_Channel_4, DMA_IT_TCIF2, DMA_IT_TCIF7},
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
  {SPI1, GPIOA, GPIO_PinSource5, GPIO_Pin_5, GPIO_PinSource6, GPIO_Pin_6,
   GPIO_PinSource7, GPIO_Pin_7, GPIO_AF_SPI1, DMA2_Stream3_IRQn, DMA2_Stream3,
   DMA2_Stream2, DMA_Channel_3, DMA_FLAG_TCIF2, DMA_FLAG_TCIF3},

  {SPI2, GPIOB, GPIO_PinSource13, GPIO_Pin_13, GPIO_PinSource15, GPIO_Pin_15,
   GPIO_PinSource14, GPIO_Pin_14, GPIO_AF_SPI2, DMA1_Stream4_IRQn, DMA1_Stream4,
   DMA1_Stream3, DMA_Channel_0, DMA_FLAG_TCIF3, DMA_FLAG_TCIF4},

  {SPI3, GPIOC, GPIO_PinSource10, GPIO_Pin_10, GPIO_PinSource12, GPIO_Pin_12,
  GPIO_PinSource11, GPIO_Pin_11, GPIO_AF_SPI3, DMA1_Stream5_IRQn, DMA1_Stream5,
  DMA1_Stream2, DMA_Channel_0, DMA_FLAG_TCIF2, DMA_FLAG_TCIF5}
};
#define MPU6000_SPI 0
#define MPU6000_CS_GPIO GPIOA
#define MPU6000_CS_PIN GPIO_Pin_4


/////////////////////// LED CONFIG ///////////////////////
#define LED1_GPIO	GPIOB
#define LED1_PIN	GPIO_Pin_4
#define LED2_GPIO	GPIOB
#define LED2_PIN	GPIO_Pin_5

/////////////////////// PWM CONFIG ///////////////////////
#define PWM_NUM_CHANNELS 13
#define PWM_NUM_OUTPUTS 11
//typedef struct {
//	GPIO_TypeDef* gpio;
//	uint16_t gpio_pin;
//	uint8_t gpio_pin_source;
//	TIM_TypeDef* tim;
//	uint8_t tim_channel;
//	uint8_t tim_af_config;
//} pwm_hardware_struct_t;
const pwm_hardware_struct_t pwm_config[PWM_NUM_CHANNELS] =
{
  {GPIOB, GPIO_Pin_0,  GPIO_PinSource0,  TIM3,  TIM_Channel_3, GPIO_AF_TIM3}, // PWM1
  {GPIOB, GPIO_Pin_1,  GPIO_PinSource1,  TIM3,  TIM_Channel_4, GPIO_AF_TIM3}, // PWM2
  {GPIOA, GPIO_Pin_3,  GPIO_PinSource3,  TIM9,  TIM_Channel_2, GPIO_AF_TIM9}, // PWM3
  {GPIOA, GPIO_Pin_2,  GPIO_PinSource2,  TIM2,  TIM_Channel_3, GPIO_AF_TIM2}, // PWM4
  {GPIOA, GPIO_Pin_1,  GPIO_PinSource1,  TIM5,  TIM_Channel_2, GPIO_AF_TIM5}, // PWM5
  {GPIOA, GPIO_Pin_0,  GPIO_PinSource0,  TIM5,  TIM_Channel_1, GPIO_AF_TIM5}, // PWM6
  {GPIOB, GPIO_Pin_14, GPIO_PinSource14, TIM12, TIM_Channel_1, GPIO_AF_TIM12}, // RC1 (Flexi-5) // Used for PPM RC
  {GPIOB, GPIO_Pin_15, GPIO_PinSource15, TIM12, TIM_Channel_1, GPIO_AF_TIM12}, // RC2 (Flexi-6)
  {GPIOC, GPIO_Pin_6,  GPIO_PinSource6,  TIM8,  TIM_Channel_1, GPIO_AF_TIM8}, // RC 3 (Flexi-7)
  {GPIOC, GPIO_Pin_7,  GPIO_PinSource7,  TIM8,  TIM_Channel_2, GPIO_AF_TIM8}, // RC 4 (Flexi-8)
  {GPIOC, GPIO_Pin_8,  GPIO_PinSource8,  TIM8,  TIM_Channel_3, GPIO_AF_TIM8}, // RC 5 (Flexi-9)
  {GPIOC, GPIO_Pin_9,  GPIO_PinSource9,  TIM8,  TIM_Channel_4, GPIO_AF_TIM8}, // RC 6 (Flexi-10)
  {GPIOA, GPIO_Pin_8,  GPIO_PinSource8,  TIM1,  TIM_Channel_1, GPIO_AF_TIM1}, // Buzzer
};

/////////////////////// PPM RC CONFIG ///////////////////////
#define PPM_RC_GPIO GPIOB
#define PPM_RC_GPIO_Pin GPIO_Pin_0
#define PPM_RC_TIM TIM3
#define PPM_RC_IRQn TIM3_IRQn
#define PPM_RC_Channel TIM_Channel_3
#define PPM_RC_TIM_IT TIM_IT_CC3
#define PPM_RC_IQRHandler TIM3_CC_IRQHandler


/////////////////////// I2C CONFIG ///////////////////////
#define HMC5883L_I2C		I2C1
#define MS5611_I2C      I2C1

#endif // REVO_F4_H
