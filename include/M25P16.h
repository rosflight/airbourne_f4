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

#include "system.h"
#include "spi.h"
#include "gpio.h"

class M25P16
{
private:
  SPI* spi_;
  GPIO cs_;
  uint32_t current_page_;
  uint32_t current_position_;
  uint32_t config_size_;
  uint32_t num_pages_for_config_;

  static const uint8_t WRITE_ENABLE = 0x06;
  static const uint8_t WRITE_DISABLE = 0x04;
  static const uint8_t READ_IDENTIFICATION = 0x9F;
  static const uint8_t READ_IDENTIFICATION2 = 0x9E;
  static const uint8_t READ_STATUS = 0x05;
  static const uint8_t WRITE_STATUS = 0x01;
  static const uint8_t READ_DATA = 0x03;
  static const uint8_t READ_DATA_HIGH_SPEED = 0x0B;
  static const uint8_t PAGE_PROGRAM = 0x02;
  static const uint8_t SECTOR_ERASE = 0xD8;
  static const uint8_t BULK_ERASE = 0xC7;
  static const uint8_t DEEP_POWER_DOWN = 0xB9;
  static const uint8_t RELEASE_DEEP_POWER_DOWN = 0xAB;
  static const uint8_t STATUS_WEL_BIT = 0x02;
  static const uint8_t STATUS_WIP_BIT = 0x01;
  static const uint8_t STATUS_BLOCK_PROTECT_BITS = 0x1C;
  static const uint8_t STATUS_SRWD_BIT = 0x80;

  uint8_t write_buffer_[260];

  uint8_t get_status();

public:
  M25P16();
  void init(SPI *_spi);
  bool read_config(uint8_t* data, uint32_t len);
  bool write_config(uint8_t* data, uint32_t len);
  void write_page(uint8_t* data);
  void read(uint8_t* data, uint8_t len);
};
