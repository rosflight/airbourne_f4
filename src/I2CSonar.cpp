#include "I2CSonar.h"

I2CSonar::I2CSonar(I2C &i2c)
{
	this->i2c=i2c;
	new_data=false;
	last_value=0;
}
I2CSonar::async_update()
{
	this->i2c.write(DEFAULT_ADDRESS, DEFAULT_REGISTER, 1, READ_COMMAND, callBack);
}

