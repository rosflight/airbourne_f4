#include <functional>
#include "drv_mb1242.h"
#define MB1242_RAW

I2CSonar::I2CSonar (I2C *i2cIn)
{
  i2c=i2cIn;
  new_data=false;
  value=0;
  last_update=millis();
  ready_to_ping=1;
}
//Tries to either start a measurement, or read it from the sensor
void I2CSonar::async_update()
{
  uint64_t now=millis();
  if (now>last_update+UPDATE_WAIT_MILLIS)
  {
    last_update=now;
    /*if(ready_to_ping)
    {
        i2c->write(DEFAULT_ADDRESS, 0xFF,PING_COMMAND);
        cb_start_read();
    }*/
   /* else
    {
        i2c->read(DEFAULT_ADDRESS,DEFAULT_REGISTER,buffer);
        cb_finished_read();
    }*/
    if (ready_to_ping)
      i2c->write(DEFAULT_ADDRESS, DEFAULT_REGISTER, PING_COMMAND, std::bind(&I2CSonar::cb_start_read,this));
    else
    {
      //Something isn't working with the callback. Maybe this will work?
      i2c->read(DEFAULT_ADDRESS, DEFAULT_REGISTER, 2, buffer, std::bind(&I2CSonar::cb_finished_read,this));
      /*delay(100);
      cb_finished_read();*/
    }
  }

}
//Returns the most recent reading, also calling async_update
float I2CSonar::async_read()
{
  this->async_update();
  if (new_data)
  {
    uint16_t centimeters=buffer[1]<<8|buffer[0];//Convert to a single number
    //Calibration from BreezySTM32 by Simon D. Levy
#ifdef MB1242_RAW
    value=(float)centimeters*.01;
#else
    value=(1.071*(float)centimeters+3.103)/100.0;
#endif
    new_data=false;
  }
  return value;
}
//callback after the measure command has been sent to the sensor
void I2CSonar::cb_start_read()
{
    ready_to_ping=false;
}
//callback after reading from the sensor has finished
void I2CSonar::cb_finished_read()
{
    new_data=true;
    ready_to_ping=true;
}
