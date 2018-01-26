#include "system.h"
#include "i2c.h"

class MS4525
{
public:
  MS4525();

  bool init(I2C* _i2c);
  bool present();
  void update();
  void read(float *differential_pressure, float *temp);

  void read_cb();

private:
  static const uint8_t ADDR = 0x28;

  I2C* i2c_;
  uint8_t buf_[4];
  float diff_press_;
  float temp_;
  uint32_t next_update_ms_;
  bool new_data_;
  bool sensor_present_;
};
