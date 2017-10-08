#include "hmc5883l.h"

HMC5883L::HMC5883L(I2C* i2c_drv) {
  i2c_ = i2c_drv;
}

bool HMC5883L::init() {
  // Wait for the chip to power up
  while (millis() < 500);

  // Detect Magnetometer
  uint8_t byte = 0;
  if(!i2c_->read(HMC58X3_ADDR, HMC58X3_ID1, &byte))
  {
    return false;
  }
  else
  {
    // Configure HMC5833L
    i2c_->write(HMC58X3_ADDR, HMC58X3_CRA, HMC58X3_CRA_DO_75 | HMC58X3_CRA_NO_AVG | HMC58X3_CRA_MEAS_MODE_NORMAL ); // 75 Hz Measurement, no bias, no averaging
    i2c_->write(HMC58X3_ADDR, HMC58X3_CRB, HMC58X3_CRB_GN_390); // 390 LSB/Gauss
    i2c_->write(HMC58X3_ADDR, HMC58X3_MODE, HMC58X3_MODE_CONTINUOUS); // Continuous Measurement Mode

    // Start a measurement transfer
    last_update_ms_ = 0;
    update();

    return true;
  }
}

void HMC5883L::update()
{
  uint32_t now_ms = millis();
  if (now_ms > last_update_ms_ + 10)
  {
//    std::function<void(void)> callback_fn =
    i2c_->read(HMC58X3_ADDR, HMC58X3_DATA, 6, i2c_buf_, std::bind(&HMC5883L::convert, this));
    last_update_ms_ = now_ms;
  }
}

void HMC5883L::convert(void)
{
  data_[0] = (float)((int16_t)((i2c_buf_[0] << 8) | i2c_buf_[1]));
  data_[1] = (float)((int16_t)((i2c_buf_[2] << 8) | i2c_buf_[3]));
  data_[2] = (float)((int16_t)((i2c_buf_[4] << 8) | i2c_buf_[5]));
}

bool HMC5883L::read(float (&mag_data)[3])
{
  mag_data[0] = data_[0];
  mag_data[1] = data_[1];
  mag_data[2] = data_[2];

  //if the mag's ADC over or underflows, then the data register is given the value of -4096
  //the data register can also be assigned -4096 if there's a math overflow during bias calculation
  return mag_data[0] != -4096 && mag_data[1] != -4096 && mag_data[2] != -4096;
}
