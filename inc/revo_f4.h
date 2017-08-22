
#ifndef REVO_F4_H
#define REVO_F4_H

#include "system.h"

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

#define LED1_GPIO	GPIOB
#define LED1_PIN	GPIO_Pin_4

#define LED2_GPIO	GPIOB
#define LED2_PIN	GPIO_Pin_5

#define NUM_PWM_OUTPUTS 6
const pwm_hardware_struct_t pwm_hardware[NUM_PWM_OUTPUTS] = 
{
	{GPIOB, GPIO_Pin_0, GPIO_PinSource0, TIM3, TIM_Channel_3, GPIO_AF_TIM3},
	{GPIOB, GPIO_Pin_1, GPIO_PinSource1, TIM3, TIM_Channel_4, GPIO_AF_TIM3},
	{GPIOA, GPIO_Pin_3, GPIO_PinSource3, TIM9, TIM_Channel_2, GPIO_AF_TIM9},
	{GPIOA, GPIO_Pin_2, GPIO_PinSource2, TIM2, TIM_Channel_3, GPIO_AF_TIM2},
	{GPIOA, GPIO_Pin_1, GPIO_PinSource1, TIM5, TIM_Channel_2, GPIO_AF_TIM5},
	{GPIOA, GPIO_Pin_0, GPIO_PinSource0, TIM5, TIM_Channel_1, GPIO_AF_TIM5}
};

#endif // REVO_F4_H