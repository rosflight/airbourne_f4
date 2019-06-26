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


#include "tfmini.h"

using namespace i2c2;

void cb(int8_t status);
TFMini* TFMini_Ptr;

TFMini::TFMini()
{

}

bool TFMini::init(I2C *_i2c)
{
    i2c_ = _i2c;
    TFMini_Ptr = this;
    while (millis() < 10); // wait for bootup
    next_update_ms_ = 0;
    last_update_ms_ = millis();

    present_ = (i2c_->checkPresent(ADDR) == I2C::RESULT_SUCCESS);
    if (!present_)
      return false;

    reset();
    setParameter(SET_DETECTION_PATTERN, DET_PATTERN_AUTO);
//    setParameter(SET_RANGE_MODE, RANGE_MODE_MIDDLE);

    return true;
}

void TFMini::reset()
{
  i2c_->write(ADDR, RESET);
  delay(3);
}

void TFMini::setParameter(uint16_t cmd, uint8_t val)
{
  uint8_t data[3] = {(uint8_t)(cmd >> 16), (uint8_t)(cmd & 0xFF), 0x01};

  i2c_->clearLog();
  i2c_->addJob(I2C::TaskType::START);
  i2c_->addJob(I2C::TaskType::WRITE_MODE, ADDR);
  i2c_->addJob(I2C::TaskType::WRITE, 0, i2c_->copyToWriteBuf(data,3), 3);
  i2c_->addJob(I2C::TaskType::START);
  i2c_->addJob(I2C::TaskType::WRITE_MODE, ADDR);
  i2c_->addJob(I2C::TaskType::WRITE, 0, i2c_->copyToWriteBuf(val), 1);
  i2c_->addJob(I2C::TaskType::STOP);
}

void TFMini::read_cb()
{
  last_update_ms_ = millis();
  new_data_ = true;
}

bool TFMini::present()
{
  if (present_ && (millis() - last_update_ms_) < 100)
  {
    return true;
  }
  else
  {
    present_ = false;
    return false;
  }
}

void TFMini::read()
{
  uint8_t data[3] = {0x01, 0x02, 0x07};
  i2c_->clearLog();
  i2c_->waitForJob();
  i2c_->addJob(I2C::TaskType::START);
  i2c_->addJob(I2C::TaskType::WRITE_MODE, ADDR);
  i2c_->addJob(I2C::TaskType::WRITE, 0, i2c_->copyToWriteBuf(data,3), 3);
  i2c_->addJob(I2C::TaskType::START);
  i2c_->addJob(I2C::TaskType::READ_MODE, ADDR);
  i2c_->addJob(I2C::TaskType::READ, 0, packet_.buf, 7);
  i2c_->addJob(I2C::TaskType::STOP);
  while (i2c_->checkBusy())
  {

  }
}

bool TFMini::update()
{
  if (millis() > next_update_ms_)
  {
    read();
  }

}



void cb(int8_t status)
{
  (void)status;
  TFMini_Ptr->read_cb();
}
