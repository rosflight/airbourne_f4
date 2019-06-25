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

#include <stdint.h>

#include "revo_f4.h"

#include "gpio.h"

class I2C
{
public:

  void init(const i2c_hardware_struct_t *c);
  
  enum : int8_t
  {
    RESULT_ERROR = 0,
    RESULT_SUCCESS = 1,
    RESULT_BUSY = -1
  };
  
  int8_t read(uint8_t addr, uint8_t reg, uint8_t num_bytes, uint8_t* data, void(*callback)(uint8_t) = nullptr, bool blocking = false);
  int8_t write(uint8_t addr, uint8_t reg, uint8_t* data, size_t len,
               void(*callback)(uint8_t), bool blocking = false, bool disable_stop=false);

  // Single-byte read/write for configuring devices
  int8_t write(uint8_t addr, uint8_t reg, uint8_t data);
  int8_t read(uint8_t addr, uint8_t reg, uint8_t *data);
  
  inline uint16_t num_errors() { return error_count_; }

private:

  // [SR2 << 8 | SR1] Bits
  enum {
    SB = 0x0001,
    ADDR = 0x0002,
    BTF = 0x0004,
    ADD10 = 0x0008,
    STOPF = 0x0010,
    RES1 = 0x0020,
    RXNE = 0x0040,
    TXE = 0x0080,
    BERR = 0x0100,
    ARLO = 0x0200,
    AF = 0x0400,
    OVR = 0x0800,
    PEC_ERR = 0x1000,
    RES2 = 0x2000,
    TIMEOUT = 0x4000,
    SMB_ALERT = 0x8000,
    MSL = 0x1 << 16,
    BUSY = 0x2 << 16,
    TRA = 0x4 << 16,
    RES3 = 0x8 << 16,
    GEN_CALL = 0x10 << 16,
    SMBDE_FAULT = 0x20 << 16,
    DUALF = 0x40 << 16,
  };

  typedef enum
  {
    IDLE,
    READING,
    WRITING
  } current_status_t;

  static constexpr size_t BUFFER_LEN = 25;

  void unstick();
  void hardware_failure();
  bool check_busy();
  void handle_hardware_failure();
  void (*cb_)(uint8_t);
  uint64_t last_event_us_;
  GPIO scl_;
  GPIO sda_;

  uint16_t error_count_ = 0;

  //Variables for current job:
  volatile current_status_t current_status_;
  volatile uint8_t return_code_;
  bool subaddress_sent_ = false;
  size_t idx_ = 0;

  bool disable_stop_;
  volatile uint8_t addr_;
  volatile uint8_t reg_;
  volatile uint8_t len_;
  uint8_t data_[BUFFER_LEN];
  uint32_t last_event = 0;

  DMA_InitTypeDef  DMA_InitStructure_;
  const i2c_hardware_struct_t *c_;

  //interrupt handlers
public:
  void handle_error(); // Interrupts have to be public because they are trampolined from the pointers
  void handle_event();
  void transfer_complete_cb();
};

//global i2c ptrs used by the event interrupts
extern I2C* I2C1_Ptr;
extern I2C* I2C2_Ptr;
extern I2C* I2C3_Ptr;
