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

MS5611* baro_ptr;
static void cb(int8_t result);

#define REBOOT_PERIOD_MS 1000 * 60 * 30 // reboot the device every 30 minutes

bool MS5611::init(I2C* _i2c)
{
  baro_ptr = this;
  i2c_ = _i2c;
  baro_present_ = false;
  while (millis() < 10);  // wait for chip to power on
  
  next_update_ms_ = millis();
  last_update_ms_ = millis();
  
  // I'm not sure why this is required, but the barometer doesn't respond otherwise
  // It doesn't have to be 0x00 either, pretty much anything works in my experience
  i2c_->checkPresent(0x00);

  if (i2c_->checkPresent(ADDR) == I2C::RESULT_SUCCESS)
  {
    baro_present_ = true;
  }
  else
  {
    baro_present_ = false;
    return false;
  }

  reset();

  // Read the PROM (try a couple times if it fails)
  bool got_valid_prom = false;
  for (int i = 0; i < 5; i++)
  {
    if (read_prom() == true)
    {
      if (calc_crc() != 0)
        continue;
      else
      {
        got_valid_prom = true;
        break;
      }
    }
  }

  if (got_valid_prom)
  {
    state_ = START_TEMP;
    new_data_ = false;
    baro_present_ = true;
    next_reboot_ms_ = REBOOT_PERIOD_MS;
    return true;
  }
  else
  {
    return false;
  }
}

bool MS5611::present()
{
  if (baro_present_ && (millis() > last_update_ms_ + 200))
    baro_present_ = false;
  return baro_present_;
}

void MS5611::update()
{
  uint32_t now_ms = millis();
  
  if (now_ms < next_update_ms_)
      return;

  next_update_ms_ += 100;

  if (!baro_present_)
  {
      callback_type_ = CB_PRESENT;
      i2c_->checkPresent(ADDR, &cb);
  }
  else
  {
    switch (state_)
    {
    case START_TEMP:
      start_temp_meas();
      break;
    case READ_TEMP:
      read_temp_meas();
      break;
    case START_PRESS:
      start_pres_meas();
      break;
    case READ_PRESS:
      read_pres_meas();
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
  i2c_->write(ADDR, RESET);
  delayMicroseconds(2800);
}

bool MS5611::read_prom()
{
  uint8_t buf[2] = {0, 0};
  
  // Read 128 bit PROM 2 bytes at a time
  for (int i = 0; i < 8; i++)
  {
    if ((i2c_->write(ADDR, PROM_RD | (i<<1)) == I2C::RESULT_SUCCESS)
        && (i2c_->read(ADDR, buf, 2) == I2C::RESULT_SUCCESS))
    {
      prom[i] = static_cast<uint16_t>(buf[0] << 8 | buf[1]);
    }
    else
    {
      // didn't work, reset and try again
      reset();
      i2c_->checkPresent(0x00);
      delay(3);
      return false;
    }
  }
  return true;
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
  int32_t temp = 0;
  int64_t delta = 0;
  temp_raw_ = (temp_buf_[0] << 16) | (temp_buf_[1] << 8) | temp_buf_[2];
  pres_raw_ = (pres_buf_[0] << 16) | (pres_buf_[1] << 8) | pres_buf_[2];
  if(pres_raw_ > 9085466 * 2 / 3 && temp_raw_ > 0)
  {
    int32_t dT = temp_raw_ - (static_cast<int32_t>(prom[5]) << 8);
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
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_TEMP_START;
  return i2c_->write(ADDR, ADC_CONV | ADC_D2 | ADC_4096, &cb) == I2C::RESULT_SUCCESS;
}

bool MS5611::start_pres_meas()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_PRES_START;
  return i2c_->write(ADDR, ADC_CONV | ADC_D1 | ADC_4096, &cb) == I2C::RESULT_SUCCESS;
}

bool MS5611::read_adc(uint8_t* read_buf)
{
  uint8_t byte = ADC_READ;
  i2c_->clearLog();
  return (i2c_->write(ADDR, &byte, 1) == I2C::RESULT_SUCCESS
          && i2c_->read(ADDR, read_buf, 3, &cb) == I2C::RESULT_SUCCESS);
}

bool MS5611::read_pres_meas()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_PRES_READ;

  return read_adc(pres_buf_);
}

bool MS5611::read_temp_meas()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_TEMP_READ;

  return read_adc(temp_buf_);
}

void MS5611::temp_read_cb(uint8_t result)
{
  (void) result;
  state_ = START_PRESS;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  new_data_ = true;
}

void MS5611::pres_read_cb(uint8_t result)
{
  (void) result;
  state_ = START_TEMP;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  new_data_ = true;
}

void MS5611::temp_start_cb(uint8_t result)
{
  (void) result;
  state_ = READ_TEMP;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 9;
}

void MS5611::pres_start_cb(uint8_t result)
{
  (void) result;
  state_ = READ_PRESS;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 9;
}

void MS5611::read(float * press, float* temp)
{
  (*press) = pressure_;
  (*temp) = temperature_;
}

void MS5611::master_cb(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
    baro_present_ = true;
  switch (callback_type_)
  {
  case CB_TEMP_READ:
    temp_read_cb(result);
    break;
  case CB_PRES_READ:
    pres_read_cb(result);
    break;
  case CB_TEMP_START:
    temp_start_cb(result);
    break;
  case CB_PRES_START:
    pres_start_cb(result);
    break;
  default:
    state_ = START_TEMP;
    break;
  }
}

void cb(int8_t result)
{
  baro_ptr->master_cb(result);
}
