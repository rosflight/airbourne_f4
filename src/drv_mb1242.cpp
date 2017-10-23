#include <functional>
#include "drv_mb1242.h"

I2CSonar::I2CSonar (I2C& i2cIn) : i2c(i2cIn)
{
  new_data=0;
  value=0;
  last_update=millis()-UPDATE_WAIT_MILLIS;//this makes it so that async_update can be called right away
    ready_to_read=1;
}
void I2CSonar::async_update()
{
  uint64_t now=millis();
  if (now>last_update+UPDATE_WAIT_MILLIS)
  {
        last_update=now;
    if (ready_to_read)
      i2c.write(DEFAULT_ADDRESS, DEFAULT_REGISTER, READ_COMMAND, std::bind(&I2CSonar::cb_start_read,this));
    else
      i2c.read(DEFAULT_ADDRESS, DEFAULT_REGISTER, 2, buffer, std::bind(&I2CSonar::cb_finished_read,this), true);
  }

}
float I2CSonar::async_read()
{
  this->async_update();
  if (new_data)
  {
    uint16_t centimeters=buffer[1]<<8|buffer[0];//Convert to a single number
    //Calibration from BreezySTM32 by Simon D. Levy 
    value=(1.071*(float)centimeters+3.103)/100.0;
  }
  return value;
}
void I2CSonar::cb_start_read()
{
    ready_to_read=0;
}
void I2CSonar::cb_finished_read()
{
    new_data=1;
    ready_to_read=1;
}
