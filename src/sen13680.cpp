#include "sen13680.h"

SEN13680* LidarPtr;

void _Lidar_cb(uint8_t result)
{
  LidarPtr->cb(result);
}

SEN13680::SEN13680() {}

void SEN13680::init(I2C *_i2c)
{
  i2c_ = _i2c;
  LidarPtr = this;
  // check for device SEN13680_DEFAULT_ADDRESS and set
  // to read fast and noisy if it's there
  bool success = true;
  if (i2c_->write(SEN13680_DEFAULT_ADDRESS, 0xFF, 0x00))
  {
    // Set the time between measurements (0x45).  0x04 means 250 Hz
    success &= i2c_->write(SEN13680_DEFAULT_ADDRESS, 0x45, 0x04);
    delay(10);
    // Set the mode pin to default setting
    success &= i2c_->write(SEN13680_DEFAULT_ADDRESS, 0x04, 0x21);
    delay(10);
    // Set the number of measurements to be taken (continuous)
    success &= i2c_->write(SEN13680_DEFAULT_ADDRESS, 0x11, 0xFF);
    delay(10);
    // Initiate Reading
    success &= i2c_->write(SEN13680_DEFAULT_ADDRESS, 0x00, 0x04);
    delay(10);
    
    last_callback_ms_ = millis();
    sensor_present_ = true;
    state_ = START_READ;
    update();
  }
  
  if (!success)
  {
    value_ = -1.0;
    sensor_present_ = false;
  }
  next_update_ms_ = 0;
}

void SEN13680::update()
{
  if (millis() > next_update_ms_)
  {
    // save current time
    next_update_ms_ = millis() + 100;

    // Request and read a lidar measurement
    // Lidar Lite is stupid, and needs a stop signal before the second start signal
    // in an I2C read.  So, I'm writing the address to nowhere, stopping, starting again
    // and then reading from nowhere
    state_ = START_READ;    
    i2c_->write(SEN13680_DEFAULT_ADDRESS, 0xFF, SEN13680_READ_REGISTER, &_Lidar_cb);
  }
}

bool SEN13680::present()
{
  if (millis() > last_callback_ms_ + 500)
  {
    sensor_present_ = false;
  }
  return sensor_present_;
}

float SEN13680::read()
{
  if (new_data_)
  {
    new_data_ = false;
    value_ = (float)((int16_t)(buffer_[0] << 8) + buffer_[1])/100.0;
  }
  return value_;
}

void SEN13680::cb(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
  {
    sensor_present_ = true;
    last_callback_ms_ = millis();
    switch (state_)
    {
    case START_READ:
      // immediately start the read
      state_ = READ;
      i2c_->read(SEN13680_DEFAULT_ADDRESS, 0xFF, 2, buffer_, &_Lidar_cb);
      break;
    case READ:
      next_update_ms_ = millis() + 20;
      new_data_ = true;
      state_ = START_READ;
      break;
    }
  }
}

