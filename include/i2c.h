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

#include <cstdint>

#include "revo_f4.h"

#include "gpio.h"


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
  };

private:

  struct Task
  {
    TaskType type;
    union {
      uint8_t addr; // READ/WRITE
      struct {
        size_t buffer_idx;
        size_t len;
      } write; // WRITE
      struct {
        uint8_t* dst;
        size_t len;
      } read; // READ
      void (*cb)(int8_t); // STOP
    } data;
  };

  class TaskQueue
  {
  public:
    inline const Task* current() const { return len_ > 0 ? buffer_ + tail_ : nullptr; }
    inline Task* last_staged() { return stage_offset_ > 0 ? buffer_ + ((head_ + stage_offset_ - 1) % TASK_BUFFER_SIZE) : nullptr; }

    bool stage_push();
    void finalize_push();
    void cancel_push();
    void pop();

  private:
    static constexpr size_t TASK_BUFFER_SIZE = 25;

    Task buffer_[TASK_BUFFER_SIZE];
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t len_ = 0;

    size_t stage_offset_ = 0;
  };

  class WriteQueue
  {
  public:
    bool stage_push(const uint8_t *src, size_t len, size_t *start);
    void finalize_push();
    void cancel_push();
    void move_to_index(size_t index);
    uint8_t consume_byte();

  private:
    static constexpr size_t WRITE_BUFFER_SIZE = 50;

    uint8_t buffer_[WRITE_BUFFER_SIZE];
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t len_ = 0;

    size_t stage_offset_ = 0;
  };

  // microseconds to wait for transfer to complete before adding a new one
  static constexpr uint64_t tout = 200;
  static constexpr uint64_t tout_block = 20000;
  static constexpr uint64_t tout_reset = 20000;

  static constexpr size_t TASK_BUFFER_SIZE = 25;
  TaskQueue task_queue_;

  static constexpr size_t WRITE_BUFFER_SIZE = 50;
  WriteQueue write_queue_;

  uint32_t last_event_;
  uint32_t expected_event_;
  volatile bool busy_;
  size_t num_errors_;
  int8_t return_code_;
  uint64_t last_event_us_;
  bool timeout_;

  Task const * current_task_;
  size_t current_write_idx_;

  bool add_job_in_progress_ = false;
  bool add_job_success_;

  // hardware
  DMA_InitTypeDef  DMA_InitStructure_;
  const i2c_hardware_struct_t* c_;
  GPIO scl_;
  GPIO sda_;

public:
  I2C();
  // Initializes the NVIC, I2C and DMA
  void init(const i2c_hardware_struct_t *c);
  size_t num_errors() const { return num_errors_; }

  // Advanced API
  bool beginJob();
  bool addTaskStart();
  bool addTaskWriteMode(uint8_t addr);
  bool addTaskReadMode(uint8_t addr);
  inline bool addTaskWrite(uint8_t data) { return addTaskWrite(&data, 1); }
  bool addTaskWrite(uint8_t *src, size_t len);
  bool addTaskRead(uint8_t *dst, size_t len);
  bool addTaskStop(void(*cb)(int8_t) = nullptr);
  bool finalizeJob();

  // Helper Abstractions (these call addJob in the right order and with the right arguments)
  // functions with callbacks are asynchronous and return RESULT_SUCCESS if the job was queued properly
  // and the return code is forwarded through the callback.  Functions without a callback
  // are blocking and return the result immediately.
  int8_t checkPresent(uint8_t addr, void(*cb)(int8_t)=nullptr);

  int8_t read(uint8_t addr, uint8_t reg, uint8_t* data, void(*cb)(int8_t)=nullptr);
  int8_t read(uint8_t addr, uint8_t* data, size_t len, void(*cb)(int8_t)=nullptr);
  int8_t read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len, void(*cb)(int8_t)=nullptr);

  int8_t write(uint8_t addr, uint8_t data, void(*cb)(int8_t)=nullptr);
  int8_t write(uint8_t addr, uint8_t reg, uint8_t data, void(*cb)(int8_t)=nullptr);
  int8_t write(uint8_t addr, uint8_t* data, size_t len, void(*cb)(int8_t)=nullptr);
  int8_t write(uint8_t addr, uint8_t reg, uint8_t* data, size_t len, void(*cb)(int8_t)=nullptr);

  // These need to be public so the trampolines can land on them
  void handleError();
  void handleEvent();

  // This clears the internal trace log
  void clearLog();

private:
  void handleTask();
  bool advanceTask();
  void unstick();
  inline bool checkBusy() const { return busy_; }
};
