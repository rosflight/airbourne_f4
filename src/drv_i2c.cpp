#include "drv_i2c.h"

I2C::I2C(I2C_TypeDef *I2C) {
	GPIO_InitTypeDef gpio_init_struct;
	I2C_InitTypeDef	 i2c_init_struct;

	if (I2C == I2C1)
	{
		dev = I2C;
		unstick();
	}
	else if (I2C == I2C2)
	{
		dev = I2C;
		unstick();
	}

}

void I2C::unstick() {
	GPIO_InitTypeDef gpio_init_struct;

	if (dev == I2C1)
	{
		/* code */
	}
	else if (I2C == I2C2)
	{
		/* code */
	}
}