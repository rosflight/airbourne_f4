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

  decode_buffer();
}

uint32_t RC_SBUS::read(uint8_t channel)
{
    return raw_[channel];
//  return buffer_[channel];
}

bool RC_SBUS::lost()
{
  return millis() < last_pulse_ms_ + 100 && failsafe_status_ == SBUS_SIGNAL_OK;
}

void RC_SBUS::decode_buffer()
{
  last_pulse_ms_ = millis();

  // process actual sbus data
  raw_[0]  = sbus_data_.data.chan0;
  raw_[1]  = sbus_data_.data.chan1;
  raw_[2]  = sbus_data_.data.chan2;
  raw_[3]  = sbus_data_.data.chan3;
  raw_[4]  = sbus_data_.data.chan4;
  raw_[5]  = sbus_data_.data.chan5;
  raw_[6]  = sbus_data_.data.chan6;
  raw_[7]  = sbus_data_.data.chan7;
  raw_[8]  = sbus_data_.data.chan8;
  raw_[9]  = sbus_data_.data.chan9;
  raw_[10] = sbus_data_.data.chan10;
  raw_[11] = sbus_data_.data.chan11;
  raw_[12] = sbus_data_.data.chan12;
  raw_[13] = sbus_data_.data.chan13;
  raw_[14] = sbus_data_.data.chan14;
  raw_[15] = sbus_data_.data.chan15;

  // Digital Channel 1
  if (sbus_data_.array[23] & (1<<0))
    raw_[16] = 2000;
  else
    raw_[16] = 1000;

  // Digital Channel 2
  if (sbus_data_.array[23] & (1<<1))
    raw_[17] = 2000;
  else
    raw_[17] = 1000;

  // Failsafe
  failsafe_status_ = SBUS_SIGNAL_OK;
  if (sbus_data_.array[23] & (1<<2))
    failsafe_status_ = SBUS_SIGNAL_LOST;
  if (sbus_data_.array[23] & (1<<3))
    failsafe_status_ = SBUS_SIGNAL_FAILSAFE;
}

void RC_SBUS::read_cb(uint8_t byte)
{
  // If there has been a 4ms gap in the stream, then assume we are at the start of the packet
  uint32_t now = millis();
  if (now >= last_pulse_ms_ + 5)
  {
    buffer_pos_ = 0;
  }
  last_pulse_ms_ = now;

  if (buffer_pos_ == 0 && byte != START_BYTE)
  {
    // this is an error, wait for the right start byte
  }
  else
  {
    // Load the bytes into our buffer
    sbus_data_.array[buffer_pos_] = byte;
    buffer_pos_++;
  }

  if (buffer_pos_ == 25)
  {
    // If we have a complete packet, decode
    decode_buffer();
    buffer_pos_ = 0;
  }
}

