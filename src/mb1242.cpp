#include <functional>

#include "mb1242.h"

I2CSonar::I2CSonar (I2C *i2cIn)
{
  i2c_=i2cIn;
  new_data_=false;
  value_=0;
  last_update_=millis();
  ready_to_ping_=true;
}
//Tries to either start a measurement, or read it from the sensor
//Does nothing if it has done something in the last UPDATE_WAIT_MILLIS ms
//Feel free to call it more often, though.
void I2CSonar::async_update()
{
  uint64_t now=millis();
  if (now>last_update_+UPDATE_WAIT_MILLIS)
  {
    last_update_=now;
    if (ready_to_ping_)
      i2c_->write(DEFAULT_ADDRESS, DEFAULT_REGISTER, PING_COMMAND, std::bind(&I2CSonar::cb_start_read,this));
    else
      i2c_->read(DEFAULT_ADDRESS, DEFAULT_REGISTER, 2, buffer_, std::bind(&I2CSonar::cb_finished_read,this));
  }
}
//Returns the most recent reading
//It is during this method that the reading is converted to meters, as well
//If there has not yet been a successful reading, returns 0
float I2CSonar::async_read()
{
  if (new_data_)
  {
    uint16_t centimeters=buffer_[0]<<8|buffer_[1];//Convert to a single number
#ifdef MB1242_RAW
    value=(float)centimeters*.01;
#else
    //Calibration from BreezySTM32 by Simon D. Levy
    value_=(1.071*(float)centimeters+3.103)/100.0;
#endif
    new_data_=false;
  }
  return value_;
}
//callback after the measure command has been sent to the sensor
void I2CSonar::cb_start_read()
{
    ready_to_ping_=false;
}
//callback after reading from the sensor has finished
void I2CSonar::cb_finished_read()
{
    new_data_=true;
    ready_to_ping_=true;
}
