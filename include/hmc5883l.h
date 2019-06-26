/*
 * Copyright (c) 2017, James Jackson and Tyler Miller
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

#pragma once

#include "system.h"
#include "i2c.h"

class HMC5883L
{
  enum : uint8_t
  {
    ADDR = 0x1E,
    CRA = 0x00,
    CRB = 0x01,
    MODE = 0x02,
    DATA = 0x03,
    STATUS = 0x09,
    ID1 = 0x0A,
    ID2 = 0x0B,
    ID3 = 0x0C,

    CRA_NO_AVG = 0x00,
    CRA_AVG_2_MEAS = 0x20,
    CRA_AVG_4_MEAS = 0x40,
    CRA_AVG_8_MEAS = 0x60,

    CRA_DO_0_75 = 0x00,
    CRA_DO_1_5 = 0x04,
    CRA_DO_3 = 0x08,
    CRA_DO_7_5 = 0x0C,
    CRA_DO_15 = 0x10,
    CRA_DO_30 = 0x14,
    CRA_DO_75 = 0x18,

    CRA_MEAS_MODE_NORMAL = 0x00,
    CRA_MEAS_MODE_POS_BIAS = 0x01,
    CRA_MEAS_MODE_NEG_BIAS = 0x02,

    CRB_GN_1370 = 0x00,
    CRB_GN_1090 = 0x20,
    CRB_GN_820 = 0x40,
    CRB_GN_660 = 0x60,
    CRB_GN_440 = 0x80,
    CRB_GN_390 = 0xA0,
    CRB_GN_330 = 0xC0,
    CRB_GN_230 = 0xE0,

    MODE_HS = 0x80,
    MODE_CONTINUOUS = 0x00,
    MODE_SINGLE = 0x01,
    MODE_IDLE = 0x02,

    SR_LOCK = 0x02,
    SR_RDY = 0x01,

  };

  static constexpr size_t TIMEOUT = 30000;

public:
  bool init(I2C* i2c_drv);
  void update();
  bool read(float mag_data[]);
  bool present();
  void cb(int8_t result);

  bool configure(uint8_t reg, uint8_t val);

private:
  I2C* i2c_;
  uint8_t i2c_buf_[6];
  volatile float data_[3];
  uint32_t last_update_ms_;
  uint32_t next_update_ms_;
  bool mag_present_;
};
