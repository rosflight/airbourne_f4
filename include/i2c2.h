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

namespace i2c2
{

class I2C
{
  enum
    {
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

public:
  enum : int8_t
  {
    RESULT_ERROR = 0,
    RESULT_SUCCESS = 1,
    RESULT_BUSY = -1
  };

  enum class TaskType : uint8_t
  {
    START,
    WRITE_MODE,
    READ_MODE,
    WRITE,
    READ,
    STOP,
    DONE,
  };

private:
  struct Task
  {
    TaskType task;
    uint8_t addr;
    uint8_t* data;
    size_t len;
    void (*cb)(int8_t);
  };

  // microseconds to wait for transfer to complete before adding a new one
  static constexpr uint64_t tout = 200;
  static constexpr uint64_t tout_block = 20000;
  static constexpr uint64_t tout_reset = 20000;

  static constexpr size_t TASK_BUFFER_SIZE = 25;
  Task tasks_[TASK_BUFFER_SIZE];
  size_t task_head_ = 0;
  size_t task_tail_ = 0;
  size_t task_idx_ = 0;

  static constexpr size_t WRITE_BUFFER_SIZE = 50;
  uint8_t write_buffer_[WRITE_BUFFER_SIZE];
  size_t wb_head_ = 0;
  volatile size_t write_idx_;

  uint32_t last_event_;
  uint32_t expected_event_;
  volatile bool busy_;
  size_t num_errors_;
  int8_t return_code_;
  uint64_t last_event_us_;
  bool timeout_;

  // hardware
  DMA_InitTypeDef  DMA_InitStructure_;
  const i2c_hardware_struct_t* c_;
  GPIO scl_;
  GPIO sda_;

public:
  I2C();
  void init(const i2c_hardware_struct_t *c);
  void addJob(TaskType type, uint8_t addr=0, uint8_t* data=nullptr, size_t len=1, void(*cb)(int8_t)=nullptr);
  inline bool checkBusy() { return busy_; }

  Task& currentTask();
  Task& nextTask();
  Task& prevTask();
  void advanceTask();

  void clearLog();
  int8_t checkPresent(uint8_t addr);
  int8_t checkPresent(uint8_t addr, void(*cb)(int8_t));

  int8_t write(uint8_t addr, uint8_t data);
  int8_t write(uint8_t addr, uint8_t reg, uint8_t data);
  int8_t write(uint8_t addr, uint8_t data, void(*cb)(int8_t));
  int8_t write(uint8_t addr, uint8_t* data, size_t len);
  int8_t read(uint8_t addr, uint8_t reg, uint8_t* data);

  int8_t read(uint8_t addr, uint8_t* data, size_t len);
  int8_t read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);
  int8_t read(uint8_t addr, uint8_t* data, size_t len, void(*cb)(int8_t));
  int8_t read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len, void(*cb)(int8_t));

  uint8_t* copyToWriteBuf(uint8_t byte);
  uint8_t* copyToWriteBuf(uint8_t* data, size_t len);
  uint8_t* getWriteBufferData(uint8_t* begin, size_t write_idx);

  bool waitForJob();
  void handleError();
  void handleEvent();


private:
  bool handleJobs();
  void unstick();


};

}
