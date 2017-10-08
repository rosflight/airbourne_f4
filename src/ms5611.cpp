#include "ms5611.h"

void MS5611::reset()
{
  i2c_->write(ADDR, RESET, 1);
  delayMicroseconds(2800);
}

void MS5611::read_prom()
{
  uint8_t buf[2];
  for (int i = 0; i < 8; i++)
  {
    i2c_->read(ADDR, PROM_RD + i*2, 2, buf, nullptr, true);
    prom[i] = (uint16_t)(buf[0] << 8 | buf[1]);
  }
}

int8_t MS5611::calc_crc()
{
  uint32_t res = 0;
  uint8_t crc = prom[7] & 0xF;
  prom[7] &= 0xFF00;

  bool blank = true;

  for (int i = 0; i < 16; i++) {
    if (prom[i >> 1]) {
      blank = false;
    }
    if (i & 1)
      res ^= ((prom[i >> 1]) & 0x00FF);
    else
      res ^= (prom[i >> 1] >> 8);
    for (int j = 8; j > 0; j--) {
      if (res & 0x8000)
        res ^= 0x1800;
      res <<= 1;
    }
  }
  prom[7] |= crc;
  if (!blank && crc == ((res >> 12) & 0xF))
    return 0;

  return -1;
}

void MS5611::start_temp_meas()
{
  i2c_->write(ADDR, ADC_CONV + ADC_D2 + ADC_4096, 1);
}

void MS5611::start_pres_meas()
{
  i2c_->write(ADDR, ADC_CONV + ADC_D1 + ADC_4096, 1);
}

void MS5611::read_pres_mess()
{
  i2c_->read(ADDR, ADC_READ, 3, pres_buf_, std::bind(&MS5611::pres_read_cb, this));
}

void MS5611::read_temp_mess()
{
  i2c_->read(ADDR, ADC_READ, 3, temp_buf_, std::bind(&MS5611::temp_read_cb, this));
}

void MS5611::temp_read_cb()
{
  temp_raw_ = (temp_buf_[0] << 16) | (temp_buf_[1] << 8) | temp_buf_[2];
}
void MS5611::pres_read_cb()
{
  pres_raw_ = (pres_buf_[0] << 16) | (pres_buf_[1] << 8) | pres_buf_[2];
}


MS5611::MS5611(I2C *_i2c)
{
  i2c_ = _i2c;
}

bool MS5611::init()
{
  while (millis() < 10);  // wait for chip to power on

  uint8_t byte;
  i2c_->write(0, 0, 0);
  delay(1);
  uint8_t ack = i2c_->read(ADDR, PROM_RD, &byte);
  if (!ack)
  {
    return false;
  }

  reset();

  // Read the PROM
  read_prom();

  // Check crc
  if (calc_crc() != 0)
    return false;

  return true;
}

void MS5611::update(){}
void MS5611::read(){}
