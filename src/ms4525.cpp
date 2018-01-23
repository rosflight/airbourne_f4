#include "ms4525.h"

MS4525::MS4525(){}

bool MS4525::init(I2C *_i2c)
{
  i2c_ = _i2c;
  sensor_present_ = false;
  uint8_t buf[1];
  sensor_present_ |= i2c_->read(ADDR, 0xFF, buf);
  return sensor_present_;
}

bool MS4525::present()
{
  return sensor_present_;
}

void MS4525::update()
{
  uint32_t now_ms = millis();

  if (now_ms > next_update_ms_)
  {
    i2c_->read(ADDR, 0xFF, 4, buf_, std::bind(&MS4525::read_cb, this));
    next_update_ms_ += 100;
  }
}

void MS4525::read_cb()
{
  new_data_ = true;
  sensor_present_ = true;
  next_update_ms_ += 20;
}

void MS4525::read(float &differential_pressure, float &temp)
{
  if (new_data_)
  {
    uint8_t status = (buf_[0] & 0xC0) >> 6;
    if(status == 0x00) // good data packet
    {
      int16_t raw_diff_pressure = 0x3FFF & ((buf_[0] << 8) + buf_[1]);
      int16_t raw_temp = ( 0xFFE0 & ((buf_[2] << 8) + buf_[3])) >> 5;
      // Convert to Pa and K
      diff_press_ = -(((float)raw_diff_pressure - 1638.3f) / 6553.2f - 1.0f) * 6894.757;
      temp_ = (0.097703957f * raw_temp)  + 223.0; // K
    }
    new_data_ = false;
  }
  differential_pressure = diff_press_;
  temp = temp_;
}

