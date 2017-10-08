
#ifndef REVO_F4_H
#define REVO_F4_H

#include "system.h"

#ifndef STM32F40_41xxx
#define STM32F40_41xxx
#endif

/////////////////////// SERIAL CONFIG ///////////////////////
#define VCP_GPIO GPIOA
#define VCP_GPIO_RX_PIN GPIO_Pin_11
#define VCP_GPIO_TX_PIN GPIO_Pin_12

/////////////////////// SPI CONFIG ///////////////////////
#define MPU6000_SPI SPI1

#define SPI1_GPIO			 GPIOA
#define SPI1_NSS_PIN		 GPIO_Pin_4
#define SPI1_NSS_PIN_SOURCE  GPIO_PinSource4 
#define SPI1_SCK_PIN         GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE  GPIO_PinSource5
#define SPI1_MISO_PIN        GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE GPIO_PinSource6
#define SPI1_MOSI_PIN        GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE GPIO_PinSource7

/////////////////////// LED CONFIG ///////////////////////
#define LED1_GPIO	GPIOB
#define LED1_PIN	GPIO_Pin_4
#define LED2_GPIO	GPIOB
#define LED2_PIN	GPIO_Pin_5

/////////////////////// PWM CONFIG ///////////////////////
#define PWM_MAX_US 2000
#define PWM_MIN_US 1000
#define PWM_NUM_CHANNELS 13
#define PWM_NUM_OUTPUTS 10
#define PWM_NUM_RC_INPUTS 8
const pwm_hardware_struct_t pwm_config[PWM_NUM_CHANNELS] =
{
  {GPIOB, GPIO_Pin_0, GPIO_PinSource0, TIM3, TIM_Channel_3, GPIO_AF_TIM3}, // PWM1
  {GPIOB, GPIO_Pin_1, GPIO_PinSource1, TIM3, TIM_Channel_4, GPIO_AF_TIM3}, // PWM2
  {GPIOA, GPIO_Pin_3, GPIO_PinSource3, TIM9, TIM_Channel_2, GPIO_AF_TIM9}, // PWM3
  {GPIOA, GPIO_Pin_2, GPIO_PinSource2, TIM2, TIM_Channel_3, GPIO_AF_TIM2}, // PWM4
  {GPIOA, GPIO_Pin_1, GPIO_PinSource1, TIM5, TIM_Channel_2, GPIO_AF_TIM5}, // PWM5
  {GPIOA, GPIO_Pin_0, GPIO_PinSource0, TIM5, TIM_Channel_1, GPIO_AF_TIM5}, // PWM6
  {GPIOB, GPIO_Pin_14, GPIO_PinSource14, TIM12, TIM_Channel_1, GPIO_AF_TIM12}, // RC1 (Flexi-5) // Used for PPM RC
  {GPIOB, GPIO_Pin_15, GPIO_PinSource15, TIM12, TIM_Channel_1, GPIO_AF_TIM12}, // RC2 (Flexi-6)
  {GPIOC, GPIO_Pin_6, GPIO_PinSource6, TIM8, TIM_Channel_1, GPIO_AF_TIM8}, // RC 3 (Flexi-7)
  {GPIOC, GPIO_Pin_7, GPIO_PinSource7, TIM8, TIM_Channel_2, GPIO_AF_TIM8}, // RC 4 (Flexi-8)
  {GPIOC, GPIO_Pin_8, GPIO_PinSource8, TIM8, TIM_Channel_3, GPIO_AF_TIM8}, // RC 5 (Flexi-9)
  {GPIOC, GPIO_Pin_9, GPIO_PinSource9, TIM8, TIM_Channel_4, GPIO_AF_TIM8}, // RC 6 (Flexi-10)
  {GPIOA, GPIO_Pin_8, GPIO_PinSource8, TIM1, TIM_Channel_1, GPIO_AF_TIM1}, // Buzzer
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
#define I2C_TIMEOUT_US		30000



#endif // REVO_F4_H
