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


#include "rc_sbus.h"
#include <functional>

void RC_SBUS::init(GPIO* inv_pin, UART *uart)
{
  uart_ = uart;
  inv_pin_ = inv_pin;

  uart_->set_mode(100000, UART::MODE_8E2);
  uart_->register_rx_callback(std::bind(&RC_SBUS::read_cb, this, std::placeholders::_1));

  // turn on the serial inverter
  inv_pin_->write(GPIO::HIGH);
}

uint32_t RC_SBUS::read(uint8_t channel)
{
  return raw_[channel];
}

bool RC_SBUS::lost()
{
  return millis() < last_pulse_ms_ + 100 && failsafe_status_ == SBUS_SIGNAL_OK;
}

void RC_SBUS::decode_buffer()
{
  last_pulse_ms_ = millis();

  // process actual sbus data
  raw_[0] = ( (buffer_[1] & 0xFF) << 3 ) | (buffer_[2] >> 5);
  raw_[1] = ( (buffer_[2] & 0x1F) << 6 ) | (buffer_[3] >> 2);
  raw_[2] = ( (buffer_[3] & 0x03) << 9 ) | (buffer_[4] << 1) | (buffer_[5] >> 7);
  raw_[3] = ( (buffer_[5] & 0x7F) << 4 ) | (buffer_[6] >> 4);
  raw_[4] = ( (buffer_[6] & 0x0F) << 7 ) | (buffer_[7] >> 1);
  raw_[5] = ( (buffer_[7] & 0x01) << 10 ) | (buffer_[8] << 2) | (buffer_[9] >> 6);
  raw_[6] = ( (buffer_[9] & 0x3F) << 5 ) | (buffer_[10] >> 3);
  raw_[7] = ( (buffer_[10] & 0x07) << 8 | buffer_[11]);

  raw_[8] = ( (buffer_[12] & 0xFF) << 3 ) | (buffer_[13] >> 5);
  raw_[9] = ( (buffer_[13] & 0x1F) << 6 ) | (buffer_[14] >> 2);
  raw_[10] = ( (buffer_[14] & 0x03) << 9 ) | (buffer_[15] << 1) | (buffer_[16] >> 7);
  raw_[11] = ( (buffer_[16] & 0x7F) << 4 ) | (buffer_[17] >> 4);
  raw_[12] = ( (buffer_[17] & 0x0F) << 7 ) | (buffer_[18] >> 1);
  raw_[13] = ( (buffer_[18] & 0x01) << 10 ) | (buffer_[19] << 2) | (buffer_[20] >> 6);
  raw_[14] = ( (buffer_[20] & 0x3F) << 5 ) | (buffer_[21] >> 3);
  raw_[15] = ( (buffer_[21] & 0x07) << 8 | buffer_[22]);

  // Digitall Channel 1
  if (buffer_[23] & (1<<0))
    raw_[16] = 1;
  else
    raw_[16] = 0;

  // Digital Channel 2
  if (buffer_[23] & (1<<1))
    raw_[17] = 1;
  else
    raw_[17] = 0;

  // Failsafe
  failsafe_status_ = SBUS_SIGNAL_OK;
  if (buffer_[23] & (1<<2))
    failsafe_status_ = SBUS_SIGNAL_LOST;
  if (buffer_[23] & (1<<3))
    failsafe_status_ = SBUS_SIGNAL_FAILSAFE;
}

void RC_SBUS::read_cb(uint8_t byte)
{
  switch (buffer_pos_)
  {
  case 0:
    if (byte == START_BYTE)
    {
      buffer_[buffer_pos_] = byte;
      buffer_pos_++;
    }
    break;

  case 24:
    decode_buffer();
    buffer_pos_ = 0;
    break;

  default:
    buffer_[buffer_pos_] = byte;
    buffer_pos_++;
    break;
  }
}

