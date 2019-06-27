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


#include <stdint.h>

#include "i2c.h"

class TFMini
{
  enum : uint8_t
  {
    ADDR = 0x10,
    RESET = 0x06
  };

  enum : uint16_t
  {
    READ = 0x0102,

    SET_RANGE_MODE  = 0x0050,
    SET_DETECTION_PATTERN = 0x0051,
    SET_SET_RANGE_LIMIT = 0x0055,
    SET_RANGE_OUTPUT_LIMIT1 = 0x0056,
    SET_RANGE_OUTPUT_LIMIT2 = 0x0057,
    SET_LOWER_STRENGTH_LIMIT1 = 0x0058,
    SET_LOWER_STRENGTH_LIMIT2 = 0x0059,
    SET_UPPER_STRENGTH_LIMIT1 = 0x005A,
    SET_UPPER_STRENGTH_LIMIT2 = 0x005B,
    SET_OUTPUT_SIGNAL_MAX1 = 0x005C,
    SET_OUTPUT_SIGNAL_MAX2 = 0x005D,
    SET_DISTANCE_UNIT = 0x0066,

    CMD_SLAVE_ADDR = 0x0026,
    SET_TRIGGER_MODE = 0x0027,

    CMD_EXTERNAL_TRIGGER = 0x0100,
    CMD_RESET = 0x0070
  };

  enum : uint8_t
  {
    RANGE_MODE_SHORT = 0x00,
    RANGE_MODE_MIDDLE = 0x03,
    RANGE_MODE_LONG = 0x07,

    DET_PATTERN_AUTO = 0x00,
    DET_PATTERN_FIXED = 0x01,

    RANGE_LIMIT_DISABLED = 0x00,
    RANGE_LIMIT_ENABLED = 0x01,

    DISTANCE_UNIT_MM = 0x00,
    DISTANCE_UNIT_CM = 0x01,

    TRIGGER_MDOE_AUTO = 0x00,
    TRIGGER_MODE_EXTERNAL = 0x01,

    EXTERNAL_TRIGGER_CODE = 0x01,
    RESET_CODE = 0x02,
  };

public:
  TFMini();
  bool init(I2C* _i2c);
  bool present();
  bool update();
  inline float distance() { return distance_; } // returns distance in meters
  inline uint16_t strength() { return strength_; }

  // public so we can trampoline from IRQ
  void read_cb(int8_t status);

private:
  static constexpr size_t UPDATE_RATE_MS = 20;
  void reset();
  void do_read();
  void convert();
  void check_present();

  float distance_;
  uint16_t strength_;

  union
  {
    uint8_t buf[7];
    struct
    {
      uint8_t trig;
      uint8_t res;
      uint16_t dist;
      uint16_t strength;
      uint8_t mode;
    } data;
  } packet_;


  uint32_t last_update_ms_;
  uint32_t next_update_ms_;
  bool new_data_;
  bool present_;

  void setParameter(uint16_t cmd, uint8_t val);

  I2C* i2c_;

};
