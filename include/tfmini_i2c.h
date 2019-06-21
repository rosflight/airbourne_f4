/*
 * Copyright (c) 2019, Daniel Koch
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

#ifndef TFMINI_I2C_H
#define TFMINI_I2C_H

#include <cstdint>

#include "i2c.h"

// datasheet: https://cdn.sparkfun.com/assets/d/9/e/c/d/TFmini-I__C-Product_Manual_V1.1_EN.pdf
#define TFMINII2C_DEFAULT_ADDRESS 0x10
#define TFMINII2C_READ_RANGE_REGISTER_H 0x01
#define TFMINII2C_READ_RANGE_REGISTER_L 0x02
#define TFMINII2C_READ_RANGE_DATA_LENGTH 0x07

#define TFMINII2C_UPDATE_WAIT_MILLIS 10 // minimum time between calls of update that actually do something

class TFminiI2C
{
private:
  float value_;         // the latest reading from the sensor
  bool new_data_;       // whether or not new data is ready to be returned
  bool sensor_present_; // flag for whether we have received data from the sensor
  I2C *i2c_;            // the i2c object used for communication

  bool ready_to_ping_;
  uint32_t last_update_ms_;
  uint32_t last_callback_ms_;

  // union for receiving and decoding range data from the sensor
  union {
    uint8_t buffer[7];             // buffer for receiving data
    struct __attribute__((packed)) // struct for decoding buffered data
    {
      uint8_t trigger_done; // 0x01: results of current frame, 0x00: results of previous frame
      uint8_t reserved;     // 0x00
      uint16_t dist;        // distance value in centimeters (0-1200)
      uint16_t strength;    // strength of return signal (0-3000). When between 20 and 2000, distance value is credible
      uint8_t mode;         // ranging gear information. 0x00: short distance, 0x03: middle distance, 0x07: long distance
    } data;
  } range_data_buffer_;

  uint8_t buffer_[7]; // for receiving data from the sensor

public:
  TFminiI2C();
  void init(I2C *i2c);
  bool present();
  float read(); // Returns the most recent reading, converted to meters, or 0 if there is none
  void update();

  // Callbacks. For internal use only, but public so the I2C peripheral can call them
  void cb_start_read(uint8_t result);    // callback after the measure command has been sent to the sensor
  void cb_finished_read(uint8_t result); // callback after reading from the sensor has finished
};

#endif // TFMINI_I2C_H