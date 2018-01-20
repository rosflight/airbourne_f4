
#ifndef REVO_F4_H
#define REVO_F4_H

#include "system.h"

///
/// This file defines the hardware configuration used in the C++ abstraction layer
/// All chip-specific configuration should happen here
///

/////////////////////// SERIAL CONFIG ///////////////////////


/////////////////////// SPI CONFIG ///////////////////////
#define NUM_SPI 1
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
  {SPI1, GPIOA, GPIO_PinSource5, GPIO_Pin_5, GPIO_PinSource6, GPIO_Pin_6, GPIO_PinSource7, GPIO_Pin_7, \
   GPIO_AF_SPI1, DMA2_Stream3_IRQn, DMA2_Stream3, DMA2_Stream2, DMA_Channel_3, DMA_FLAG_TCIF2, DMA_FLAG_TCIF3},
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
