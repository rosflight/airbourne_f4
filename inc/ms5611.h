#ifndef MS5611_H
#define MS5611_H

#include "system.h"
#include "drv_i2c.h"


class MS5611
{
private:

  enum : uint8_t
  {
    RESET    = 0x1E, // ADC reset command
    ADC_READ = 0x00, // ADC read command
    ADC_CONV = 0x40, // ADC conversion command
    ADC_D1   = 0x00, // ADC D1 conversion
    ADC_D2   = 0x10, // ADC D2 conversion
    ADC_256  = 0x00, // ADC OSR=256
    ADC_512  = 0x02, // ADC OSR=512
    ADC_1024 = 0x04, // ADC OSR=1024
    ADC_2048 = 0x06, // ADC OSR=2048
    ADC_4096 = 0x08, // ADC OSR=4096
    PROM_RD  = 0xA0 // Prom read command
  };

  enum : uint8_t
  {
    START_TEMP = 0,
    READ_TEMP = 1,
    START_PRESS = 2,
    READ_PRESS = 3,
  };

  static const uint8_t ADDR = 0x77;

  void reset();
  void read_prom();
  int8_t calc_crc();
  void read_pres_mess();
  void read_temp_mess();
  void start_temp_meas();
  void start_pres_meas();
  void convert();

  I2C* i2c_;
  uint8_t pres_buf_[3];
  uint8_t temp_buf_[3];
  uint32_t pres_raw_;
  uint32_t temp_raw_;
  float pressure_;
  float temperature_;
  uint16_t prom[8];
  uint32_t next_update_ms_;

public:
  MS5611(I2C* _i2c);
  bool init();
  void update();
  void read();

  void temp_read_cb();
  void pres_read_cb();



};

#endif // MS5611_H
