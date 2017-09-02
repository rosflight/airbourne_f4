
#ifndef REVO_F4_H
#define REVO_F4_H

#include "system.h"

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
#define PWM_MAX_US		2000
#define PWM_MIN_US		1000
#define PWM_NUM_OUTPUTS 6
const pwm_hardware_struct_t pwm_config[PWM_NUM_OUTPUTS] =
{
	{GPIOB, GPIO_Pin_0, GPIO_PinSource0, TIM3, TIM_Channel_3, GPIO_AF_TIM3},
	{GPIOB, GPIO_Pin_1, GPIO_PinSource1, TIM3, TIM_Channel_4, GPIO_AF_TIM3},
	{GPIOA, GPIO_Pin_3, GPIO_PinSource3, TIM9, TIM_Channel_2, GPIO_AF_TIM9},
	{GPIOA, GPIO_Pin_2, GPIO_PinSource2, TIM2, TIM_Channel_3, GPIO_AF_TIM2},
	{GPIOA, GPIO_Pin_1, GPIO_PinSource1, TIM5, TIM_Channel_2, GPIO_AF_TIM5},
	{GPIOA, GPIO_Pin_0, GPIO_PinSource0, TIM5, TIM_Channel_1, GPIO_AF_TIM5}
};

/////////////////////// I2C CONFIG ///////////////////////
#define HMC5883L_I2C		I2C1
#define I2C_TIMEOUT_US		30000

#define I2C1_GPIO			GPIOB
#define I2C1_SCL_PIN		GPIO_Pin_8
#define I2C1_SCL_PIN_SOURCE GPIO_PinSource8
#define I2C1_SDA_PIN		GPIO_Pin_9
#define I2C1_SDA_PIN_SOURCE GPIO_PinSource9

#define I2C2_GPIO			GPIOB
#define I2C2_SCL_PIN		GPIO_Pin_10
#define I2C2_SCL_PIN_SOURCE GPIO_PinSource10
#define I2C2_SDA_PIN		GPIO_Pin_11
#define I2C2_SDA_PIN_SOURCE GPIO_PinSource11

#endif // REVO_F4_H
