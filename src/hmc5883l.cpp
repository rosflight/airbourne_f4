#include "hmc5883l.h"

HMC5883L::HMC5883L(I2C* i2c_drv) {
  i2c = i2c_drv;
}

bool HMC5883L::init() {
  // Detect Magnetometer
  uint8_t byte = 0;
  if(!i2c->read(HMC58X3_ADDR, HMC58X3_ID1, &byte))
  {
    return false;
  }
  else if( byte != 0x48)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool HMC5883L::read(float (&mag_data)[3]) {
  uint8_t raw[6];
  i2c->read(HMC58X3_ADDR, HMC58X3_DATA, 6, raw);

  mag_data[0] = (float)((int16_t)((raw[0] << 8) | raw[1]));
  mag_data[1] = (float)((int16_t)((raw[2] << 8) | raw[3]));
  mag_data[2] = (float)((int16_t)((raw[4] << 8) | raw[5]));

  //if the mag's ADC over or underflows, then the data register is given the value of -4096
  //the data register can also be assigned -4096 if there's a math overflow during bias calculation
  return mag_data[0] != -4096 && mag_data[1] != -4096 && mag_data[2] != -4096;
}
