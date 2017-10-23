#include "I2CSonar.h"

I2CSonar::I2CSonar(I2C &i2c) : i2c(i2c)
{
	new_data=false;
	value=0;
	last_update=millis()-UPDATE_WAIT_MILLIS;//this makes it so that async_update can be called right away
	ready_to_read=false;
}
void I2CSonar::async_update()
{
	uint64_t now=millis();
	if(now>this->last_update+UPDATE_WAIT_MILLIS)
	{
		if(this->ready_to_read)
			this->i2c.write(DEFAULT_ADDRESS, DEFAULT_REGISTER, 1, READ_COMMAND, this->cb_start_read);
		else
			this->i2c.read(DEFAULT_ADDRESS, DEFAULT_REGISTER, 2, this->buffer, this->cb_finished_read, false);
	}

}
float I2CSonar::async_read()
{
	if(this->new_data)
	{
		uint16_t centimeters=this->buffer[1]<<8|buffer[0];//Convert to a single number
		//Calibration from BreezySTM32 by Simon D. Levy 
		this->value=(1.071*(float)centimeters+3.103)/100.0;
	}
	return this->value;
}
void I2CSonar::cb_start_read()
{

}
void I2CSonar::cb_finished_read()
{
	this->new_data=true;
}
