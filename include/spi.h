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

#pragma once

#include "gpio.h"
#include "revo_f4.h"

class SPI
{
public:
  void init(const spi_hardware_struct_t* conf);
  void set_divisor(uint16_t new_divisor);

  void enable(GPIO& cs);
  void disable(GPIO& cs);

  bool transfer(uint8_t* out_data, uint32_t num_bytes, uint8_t* in_data, GPIO* cs = NULL, void (*cb)(void) = NULL);
  bool write(const uint8_t* out_data, uint32_t num_bytes, GPIO* cs = NULL);
  uint8_t transfer_byte(uint8_t data, GPIO* cs = NULL);

  void transfer_complete_cb();
  inline bool is_busy() { return busy_; }

private:
  void perform_transfer();
  uint8_t* in_buffer_ptr_;
  const uint8_t* out_buffer_ptr_;
  uint32_t num_bytes_;

  const spi_hardware_struct_t* c_;
  GPIO mosi_;
  GPIO miso_;
  GPIO sck_;

  DMA_InitTypeDef DMA_InitStructure_;

  uint32_t errors_ = 0;
  GPIO* cs_;
  volatile bool busy_ = false;
  void (*transfer_cb_)(void) = NULL;
};
