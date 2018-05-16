/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ms5611.h"


bool MS5611::init(I2C* _i2c)
{
  i2c_ = _i2c;
  baro_present_ = false;
  while (millis() < 10);  // wait for chip to power on

  uint8_t byte;
  i2c_->write(0, 0, 0);
  delay(1);
  uint8_t ack = i2c_->read(ADDR, PROM_RD, &byte);
  if (!ack)
  {
    baro_present_ = false;
    return false;
  }

  reset();

  // Read the PROM
  read_prom();

    // Check crc
  if (calc_crc() != 0)
    return false;

  next_update_ms_ = 0;
  state_ = START_TEMP;
  new_data_ = false;

  baro_present_ = true;

  return true;
}

bool MS5611::present()
{
  return baro_present_;
}

void MS5611::update()
{
  uint32_t now_ms = millis();

  if (now_ms > next_update_ms_)
  {
    next_update_ms_ += 100;

    switch (state_)
    {
    case START_TEMP:
      if (start_temp_meas())
        state_ = READ_TEMP;
      break;
    case READ_TEMP:
      if (read_temp_mess())
        state_ = START_PRESS;
      break;
    case START_PRESS:
      if (start_pres_meas())
        state_ = READ_PRESS;
      break;
    case READ_PRESS:
      if (read_pres_mess())
        state_ = START_TEMP;
      break;
    default:
      state_ = START_TEMP;
      break;
    }
  }
  if (new_data_)
  {
    convert();
  }
}


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
    prom[i] = static_cast<uint16_t>(buf[0] << 8 | buf[1]);
  }
}

int8_t MS5611::calc_crc()
{
  uint32_t res = 0;
  uint8_t crc = prom[7] & 0xF;
  prom[7] &= 0xFF00;

  bool blank = true;

  for (int i = 0; i < 16; i++)
  {
    if (prom[i >> 1])
    {
      blank = false;
    }
    if (i & 1)
      res ^= ((prom[i >> 1]) & 0x00FF);
    else
      res ^= (prom[i >> 1] >> 8);
    for (int j = 8; j > 0; j--)
    {
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

void MS5611::convert()
{
  int32_t press = 0;
  int64_t temp = 0;
  int64_t delta = 0;
  if(pres_raw_ > 9085466 * 2 / 3 && temp_raw_ > 0)
  {
    int64_t dT = static_cast<int64_t>(temp_raw_) - (static_cast<int64_t>(prom[5]) << 8);
    int64_t off = (static_cast<int64_t>(prom[2]) << 16) + ((static_cast<int64_t>(prom[4]) * dT) >> 7);
    int64_t sens = (static_cast<int64_t>(prom[1]) << 15) + ((static_cast<int64_t>(prom[3]) * dT) >> 8);
    temp = 2000 + ((dT * static_cast<int64_t>(prom[6])) >> 23);

    // temperature lower than 20degC
    if (temp < 2000)
    {
      delta = temp - 2000;
      delta = 5 * delta * delta;
      off -= delta >> 1;
      sens -= delta >> 2;

      // temperature lower than -15degC
      if (temp < -1500)
      {
        delta = temp + 1500;
        delta = delta * delta;
        off -= 7 * delta;
        sens -= (11 * delta) >> 1;
      }

      temp -= ((dT * dT) >> 31);
    }
    press = (((static_cast<uint64_t>(pres_raw_) * sens) >> 21) - off) >> 15;

    pressure_ = static_cast<float>(press); // Pa
    temperature_ = static_cast<float>(temp) / 100.0 + 273.0; // K
  }
  new_data_ = false;
}

bool MS5611::start_temp_meas()
{
  return i2c_->write(ADDR, ADC_CONV + ADC_D2 + ADC_4096, 1, std::bind(&MS5611::temp_start_cb, this)) > 0;
}

bool MS5611::start_pres_meas()
{
  return i2c_->write(ADDR, ADC_CONV + ADC_D1 + ADC_4096, 1, std::bind(&MS5611::pres_start_cb, this)) > 0;
}

bool MS5611::read_pres_mess()
{
  return i2c_->read(ADDR, ADC_READ, 3, pres_buf_, std::bind(&MS5611::pres_read_cb, this)) > 0;
}

bool MS5611::read_temp_mess()
{
  return (i2c_->read(ADDR, ADC_READ, 3, temp_buf_, std::bind(&MS5611::temp_read_cb, this)) > 0);
}

void MS5611::temp_read_cb()
{
  temp_raw_ = (temp_buf_[0] << 16) | (temp_buf_[1] << 8) | temp_buf_[2];
  next_update_ms_ = millis() + 15;
  new_data_ = true;
}

void MS5611::pres_read_cb()
{
  pres_raw_ = (pres_buf_[0] << 16) | (pres_buf_[1] << 8) | pres_buf_[2];
  next_update_ms_ = millis() + 15;
  new_data_ = true;
}

void MS5611::temp_start_cb()
{
  next_update_ms_ = millis() + 15;
}

void MS5611::pres_start_cb()
{
  next_update_ms_ = millis() + 15;
}

void MS5611::read(float * press, float* temp)
{
  (*press) = pressure_;
  (*temp) = temperature_;
}
