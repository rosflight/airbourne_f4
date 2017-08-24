#include "drv_i2c.h"

I2C::I2C(I2C_TypeDef *I2C) 
{
	dev = I2C;
	//enable peripheral clocks as we need them
	if (dev == I2C1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	}
	else if (dev == I2C2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	}
	init();
}

void I2C::init(void)
{
	GPIO_InitTypeDef gpio_init_struct;
	I2C_InitTypeDef	 i2c_init_struct;
	NVIC_InitTypeDef nvic_init_struct;

	if (dev == I2C1)
	{
		unstick(); //unstick will properly initialize pins

		//configure the gpio pins
		GPIO_PinAFConfig(I2C1_GPIO, I2C1_SCL_PIN_SOURCE, GPIO_AF_I2C1);
		GPIO_PinAFConfig(I2C1_GPIO, I2C1_SDA_PIN_SOURCE, GPIO_AF_I2C1);
	}
	else if (dev == I2C2)
	{
		unstick(); //unstick will properly initialize pins

		//configure the gpio pins
		GPIO_PinAFConfig(I2C2_GPIO, I2C2_SCL_PIN_SOURCE, GPIO_AF_I2C2);
		GPIO_PinAFConfig(I2C2_GPIO, I2C2_SDA_PIN_SOURCE, GPIO_AF_I2C2);
	}

	//initialze the i2c itself
	if (dev == I2C1 || dev == I2C2)
	{
		I2C_DeInit(dev);

		I2C_StructInit(&i2c_init_struct);
		i2c_init_struct.I2C_ClockSpeed 			= 400000;
		i2c_init_struct.I2C_Mode 				= I2C_Mode_I2C;
		i2c_init_struct.I2C_DutyCycle 			= I2C_DutyCycle_2;
		i2c_init_struct.I2C_OwnAddress1 		= 0; 					//The first device own address
		i2c_init_struct.I2C_Ack 				= I2C_Ack_Disable;
		i2c_init_struct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

		I2C_Init(dev, &i2c_init_struct);
		I2C_Cmd(dev, ENABLE);
	}
}

void I2C::unstick() 
{
	GPIO_InitTypeDef gpio_init_struct;

	gpio_init_struct.GPIO_Mode 	= GPIO_Mode_OUT;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_OType = GPIO_OType_OD;
	gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;

	if (dev == I2C1)
	{
		gpio_init_struct.GPIO_Pin = I2C1_SDA_PIN | I2C1_SCL_PIN;
		GPIO_Init(I2C1_GPIO, &gpio_init_struct);

		GPIO_SetBits(I2C1_GPIO, I2C1_SDA_PIN | I2C1_SCL_PIN);

		for (int i = 0; i < 8; ++i)
		{
			//wait for potential clock stretching to finish before proceeding
			//play it safe by delaying 3us which would put clk@ 333.33 kHz
			while(!GPIO_ReadInputDataBit(I2C1_GPIO, I2C1_SCL_PIN))
				delayMicroseconds(3);

			//Toggle clk at 333.33 kHz
			GPIO_ResetBits(I2C1_GPIO, I2C1_SCL_PIN);
			delayMicroseconds(3);
			GPIO_SetBits(I2C1_GPIO, I2C1_SCL_PIN);
			delayMicroseconds(3);
		}

		//Generate start condition
		GPIO_ResetBits(I2C1_GPIO, I2C1_SDA_PIN);
		delayMicroseconds(3);                                        
		GPIO_ResetBits(I2C1_GPIO, I2C1_SCL_PIN);
		delayMicroseconds(3);     

		//Generate stop condition                                   
		GPIO_SetBits(I2C1_GPIO, I2C1_SCL_PIN);
		delayMicroseconds(3);                                        
		GPIO_SetBits(I2C1_GPIO, I2C1_SDA_PIN);
		delayMicroseconds(3);

		//reset back to normal:
		gpio_init_struct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(I2Cq_GPIO, &gpio_init_struct);
	}
	else if (dev == I2C2)
	{
		gpio_init_struct.GPIO_Pin = I2C2_SDA_PIN | I2C2_SCL_PIN;
		GPIO_Init(I2C2_GPIO, &gpio_init_struct);

		GPIO_SetBits(I2C2_GPIO, I2C2_SDA_PIN | I2C2_SCL_PIN);

		for (int i = 0; i < 8; ++i)
		{
			//wait for potential clock stretching to finish before proceeding
			//play it safe by delaying 3us which would put clk @ 333.33 kHz
			while(!GPIO_ReadInputDataBit(I2C2_GPIO, I2C2_SCL_PIN))
				delayMicroseconds(3);

			//Toggle clk at 333.33 kHz
			GPIO_ResetBits(I2C2_GPIO, I2C2_SCL_PIN);
			delayMicroseconds(3);
			GPIO_SetBits(I2C2_GPIO, I2C2_SCL_PIN);
			delayMicroseconds(3);
		}

		GPIO_ResetBits(I2C2_GPIO, I2C2_SDA_PIN);
		delayMicroseconds(3);
		GPIO_ResetBits(I2C2_GPIO, I2C2_SCL_PIN);
		delayMicroseconds(3);    

		GPIO_SetBits(I2C2_GPIO, I2C2_SCL_PIN);
		delayMicroseconds(3);
		GPIO_SetBits(I2C2_GPIO, I2C2_SDA_PIN);
		delayMicroseconds(3);

		//reset back to normal:
		gpio_init_struct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(I2C2_GPIO, &gpio_init_struct);
	}
}

bool I2C::read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	addr_ = addr << 1;
	reg_  = reg;
	len_  = len;

}

void I2C::handle_hardware_failure()
{
	error_count_++;
	init(); //unstick and reinitialize the hardware
}
