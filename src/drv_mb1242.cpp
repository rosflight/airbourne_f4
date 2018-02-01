#include <functional>
#include "drv_mb1242.h"

MB1242::MB1242 (I2C& _i2c) : i2c_(_i2c)
{
  new_data_ = 0;
  value_ = 0;
  last_update_ = 0;
  ready_to_read_ = 1;
  sensor_present_ = false;
}


void MB1242::async_update()
{
  uint64_t now = millis();
  int8_t success;
  if (now > last_update_ + UPDATE_WAIT_MILLIS)
  {
    last_update_ = now;
    if (ready_to_read_)
      success = i2c_.write(DEFAULT_ADDRESS, DEFAULT_REGISTER, READ_COMMAND, std::bind(&MB1242::cb_start_read,this), true);
    else
      success = i2c_.read(DEFAULT_ADDRESS, DEFAULT_REGISTER, 2, buffer_, std::bind(&MB1242::cb_finished_read,this), true);
  }
}


float MB1242::async_read()
{
  if (new_data_)
  {
    uint16_t centimeters=buffer_[1] << 8 | buffer_[0];  //Convert to a single number
    //Calibration from BreezySTM32 by Simon D. Levy
    value_ = (1.071 * (float)centimeters + 3.103) / 100.0;
  }
  return value_;
}


void MB1242::cb_start_read()
{
  ready_to_read_ = 0;
  sensor_present_ = true;
}


void MB1242::cb_finished_read()
{
  new_data_ = 1;
  ready_to_read_ = 1;
}
