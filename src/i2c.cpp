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

#include <cstdlib>
#include <cstring>

#include "i2c.h"

//global i2c ptrs used by the event interrupts
I2C* I2C1_Ptr;
I2C* I2C2_Ptr;
I2C* I2C3_Ptr;



#define if_timeout(cond, us) \
do {\
  timeout_ = false;\
  uint64_t start = micros();\
  while((cond) && micros() < start + us);\
}while(0);\
if (cond)

#define waitToFinish \
do {\
  if_timeout(checkBusy(), tout_block)\
  {\
    log_line;\
    num_errors_++;\
    return RESULT_ERROR;\
  }\
} while(0)


#ifndef NDEBUG
class DebugHistory
{
  uint32_t history_[60];
  uint32_t head_ = 0;

public:
  void add_event(uint32_t event)
  {
    history_[head_] = event;
    head_ = (head_ + 1) % 60;
  }
  void clear()
  {
    memset(history_, 0, sizeof(history_));
    head_ = 0;
  }
};
DebugHistory event_history_;
DebugHistory interrupt_history_;
#define log_line event_history_.add_event(__LINE__)
#define clear_log event_history_.clear()
#else
#define log_line
#define clear_log
#endif

bool I2C::TaskQueue::stage_push()
{
  if (len_ + stage_offset_ == TASK_BUFFER_SIZE)
    return false;

  ++stage_offset_;
  return true;
}

void I2C::TaskQueue::finalize_push()
{
  head_ = (head_ + stage_offset_) % TASK_BUFFER_SIZE;
  len_ += stage_offset_;
  stage_offset_ = 0;
}

void I2C::TaskQueue::cancel_push()
{
  stage_offset_ = 0;
}

void I2C::TaskQueue::pop()
{
  if (len_ > 0)
  {
    tail_ = (tail_ + 1) % TASK_BUFFER_SIZE;
    --len_;
  }
}

bool I2C::WriteQueue::stage_push(const uint8_t *src, size_t len, size_t *start)
{
  if (len_ + stage_offset_ + len >= WRITE_BUFFER_SIZE)
    return false;

  size_t start_offset = (head_ + stage_offset_) % WRITE_BUFFER_SIZE;
  *start = start_offset;

  // copy at most to the end of the buffer
  size_t first_batch = (WRITE_BUFFER_SIZE - start_offset) < len ? WRITE_BUFFER_SIZE - start_offset : len;
  memcpy(buffer_ + start_offset, src, first_batch);

  // copy any remaining to the beginning of the buffer
  if (first_batch < len)
  {
    memcpy(buffer_, src + first_batch, len - first_batch);
  }

  stage_offset_ += len;

  return true;
}

void I2C::WriteQueue::finalize_push()
{
  head_ = (head_ + stage_offset_) % WRITE_BUFFER_SIZE;
  len_ += stage_offset_;
  stage_offset_ = 0;
}

void I2C::WriteQueue::cancel_push()
{
  stage_offset_ = 0;
}

void I2C::WriteQueue::move_to_index(size_t index)
{
  // TODO there are a lot of invalid cases I'm not checking
  if (index < tail_)
  {
    len_ -= (WRITE_BUFFER_SIZE - tail_) + index;
  }
  else
  {
    len_ -= index - tail_;
  }

  tail_ = index;
}

uint8_t I2C::WriteQueue::consume_byte()
{
  if (len_ > 0)
  {
    uint8_t data = buffer_[tail_];
    tail_ = (tail_ + 1) % WRITE_BUFFER_SIZE;
    --len_;
    return data;
  }
  else
  {
    return 0;
  }
}

I2C::I2C()
{}

void I2C::init(const i2c_hardware_struct_t *c)
{
  c_ = c;

  if (c->dev == I2C1)
    I2C1_Ptr = this;

  if (c->dev == I2C2)
    I2C2_Ptr = this;
 if (c->dev == I2C3)
    I2C3_Ptr = this;

  GPIO_PinAFConfig(c->GPIO, c->SCL_PinSource, c->GPIO_AF);
  GPIO_PinAFConfig(c->GPIO, c->SDA_PinSource, c->GPIO_AF);
  scl_.init(c->GPIO, c->SCL_Pin, GPIO::PERIPH_IN_OUT);
  sda_.init(c->GPIO, c->SDA_Pin, GPIO::PERIPH_IN_OUT);

  //initialize the i2c itself
  I2C_DeInit(c->dev);

  I2C_InitTypeDef I2C_InitStructure;
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_ClockSpeed = c->I2C_ClockSpeed;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0; //The first device address
  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(c->dev, &I2C_InitStructure);

  // Interrupts
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = c->I2C_EV_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  // I2C Event Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = c->I2C_ER_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  // DMA Event Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = c->DMA_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  DMA_Cmd(c->DMA_Stream, DISABLE);
  DMA_DeInit(c->DMA_Stream);
  DMA_InitStructure_.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure_.DMA_FIFOThreshold = DMA_FIFOThreshold_Full ;
  DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure_.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure_.DMA_Channel = c->DMA_Channel;

  DMA_InitStructure_.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(c->dev->DR));
  DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure_.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Init(c_->DMA_Stream, &DMA_InitStructure_);

  unstick();
  last_event_us_ = micros();
  I2C_Cmd(c->dev, ENABLE);
  log_line;

  num_errors_ = 0;
  busy_ = false;
}

void I2C::clearLog()
{
  clear_log;
}

bool I2C::beginJob()
{
  if (add_job_in_progress_)
    return false;

  add_job_in_progress_ = true;
  add_job_success_ = true;
  return true;
}

bool I2C::addTaskStart()
{
  if (!add_job_in_progress_)
    return false;

  if (task_queue_.stage_push())
  {
    task_queue_.last_staged()->type = TaskType::START;
    return true;
  }
  else
  {
    add_job_success_ = false;
    return false;
  }
}

bool I2C::addTaskWriteMode(uint8_t addr)
{
  if (!add_job_in_progress_)
    return false;

  if (task_queue_.stage_push())
  {
    Task* staged = task_queue_.last_staged();
    staged->type = TaskType::WRITE_MODE;
    staged->data.addr = addr;
    return true;
  }
  else
  {
    add_job_success_ = false;
    return false;
  }
}

bool I2C::addTaskReadMode(uint8_t addr)
{
  if (!add_job_in_progress_)
    return false;

  if (task_queue_.stage_push())
  {
    Task *staged = task_queue_.last_staged();
    staged->type = TaskType::READ_MODE;
    staged->data.addr = addr;
    return true;
  }
  else
  {
    add_job_success_ = false;
    return false;
  }
}

bool I2C::addTaskWrite(uint8_t *src, size_t len)
{
  if (!add_job_in_progress_)
    return false;

  if (task_queue_.stage_push())
  {
    Task *staged = task_queue_.last_staged();
    staged->type = TaskType::WRITE;
    if (!write_queue_.stage_push(src, len, &staged->data.write.buffer_idx))
    {
      add_job_success_ = false;
      return false;
    }
    staged->data.write.len = len;
    return true;
  }
  else
  {
    add_job_success_ = false;
    return false;
  }
}

bool I2C::addTaskRead(uint8_t *dst, size_t len)
{
  if (!add_job_in_progress_)
    return false;

  if (task_queue_.stage_push())
  {
    Task *staged = task_queue_.last_staged();
    staged->type = TaskType::READ;
    staged->data.read.dst = dst;
    staged->data.read.len = len;
    return true;
  }
  else
  {
    add_job_success_ = false;
    return false;
  }
}

bool I2C::addTaskStop(void (*cb)(int8_t))
{
  if (!add_job_in_progress_)
    return false;

  if (task_queue_.stage_push())
  {
    Task *staged = task_queue_.last_staged();
    staged->type = TaskType::STOP;
    staged->data.cb = cb;
    return true;
  }
  else
  {
    add_job_success_ = false;
    return false;
  }
}

bool I2C::finalizeJob()
{
  if (!add_job_in_progress_)
    return false;

  add_job_in_progress_ = false;

  if (add_job_success_)
  {
    task_queue_.finalize_push();
    write_queue_.finalize_push();

    if (!busy_)
    {
      log_line;
      current_task_ = task_queue_.current();
      handleTask();
    }

    return true;
  }
  else
  {
    task_queue_.cancel_push();
    write_queue_.cancel_push();
    return false;
  }
}

bool I2C::advanceTask()
{
  log_line;
  task_queue_.pop();
  current_task_ = task_queue_.current();

  if (current_task_ && current_task_->type == TaskType::WRITE)
  {
    log_line;
    write_queue_.move_to_index(current_task_->data.write.buffer_idx);
    current_write_idx_ = 0;
  }

  return current_task_ != nullptr;
}

void I2C::handleTask()
{
  log_line;
  busy_ = true;
  bool task_complete = true;
  bool start_next_task = false;

  switch (current_task_->type)
  {
  case TaskType::START:
    log_line;
    expected_event_ = I2C_EVENT_MASTER_MODE_SELECT;
    I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
    I2C_Cmd(c_->dev, ENABLE);
    I2C_GenerateSTART(c_->dev, ENABLE);
    break;

  case TaskType::WRITE_MODE:
    log_line;
    I2C_Send7bitAddress(c_->dev, current_task_->data.addr << 1, I2C_Direction_Transmitter);
    expected_event_ = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
    break;

  case TaskType::READ_MODE:
    log_line;
    I2C_AcknowledgeConfig(c_->dev, ENABLE);
    I2C_DMALastTransferCmd(c_->dev, ENABLE);
    I2C_Send7bitAddress(c_->dev, current_task_->data.addr << 1, I2C_Direction_Receiver);
    expected_event_ = I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;
    break;

  case TaskType::WRITE:
    log_line;
    I2C_SendData(c_->dev, write_queue_.consume_byte());
    ++current_write_idx_;
    if (current_write_idx_ < current_task_->data.write.len)
    {
      log_line;
      I2C_SendData(c_->dev, write_queue_.consume_byte());
      ++current_write_idx_;
    }
    expected_event_ = I2C_EVENT_MASTER_BYTE_TRANSMITTED;
    task_complete = (current_write_idx_ == current_task_->data.write.len);
    break;

  case TaskType::READ:
    log_line;
    c_->DMA_Stream->M0AR = reinterpret_cast<uint32_t>(current_task_->data.read.dst);
    DMA_SetCurrDataCounter(c_->DMA_Stream, current_task_->data.read.len);
    I2C_DMACmd(c_->dev, ENABLE);
    DMA_ITConfig(c_->DMA_Stream, DMA_IT_TC, ENABLE);
    DMA_Cmd(c_->DMA_Stream, ENABLE);
    expected_event_ = RXNE & BTF;
    break;

  case TaskType::STOP:
    log_line;
    I2C_GenerateSTOP(c_->dev, ENABLE);
    I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
    I2C_Cmd(c_->dev, DISABLE);
    expected_event_ = 0x00;
    if (current_task_->data.cb)
      current_task_->data.cb(return_code_);
    start_next_task = true;
  }

  if (task_complete)
  {
    log_line;
    busy_ = advanceTask();

    if (!busy_)
    {
      log_line;
      I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    }
    else if (start_next_task)
    {
      log_line;
      handleTask();
    }
  }

  log_line;
}

void I2C::handleEvent()
{
  last_event_ = I2C_GetLastEvent(c_->dev);
  log_line;
  if ((last_event_ & expected_event_) == expected_event_)
  {
    last_event_us_ = micros();
    log_line;
    handleTask();
  }
}

void I2C::handleError()
{
  log_line;
  return_code_ = RESULT_ERROR;
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

  while (advanceTask() && current_task_->type != TaskType::STOP)
  {
    log_line;
  }

  //reset errors
  if (c_->dev->SR1 & AF)
  {
    log_line;
    c_->dev->SR1 &= ~AF;
  }
  if (c_->dev->SR1 & BERR)
  {
    log_line;
    c_->dev->SR1 &= ~BERR;
  }

  if (current_task_) // run the STOP task if there is one
  {
    log_line;
    handleTask();
  }
  else // otherwise add a STOP task and run it
  {
    log_line;
    beginJob();
    addTaskStop();
    if (finalizeJob())
    {
      log_line;
      current_task_ = task_queue_.current();
      handleTask();
    }
  }
}

void I2C::unstick()
{
  I2C_Cmd(c_->dev, DISABLE);

  I2C_ClearFlag(c_->dev, I2C_FLAG_BUSY);

  // Turn off the interrupts
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

  //reset errors
  I2C_ClearFlag(c_->dev, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);

  scl_.write(GPIO::HIGH);
  sda_.write(GPIO::HIGH);
  scl_.set_mode(GPIO::OUTPUT, GPIO::HIGH);
  sda_.set_mode(GPIO::OUTPUT, GPIO::HIGH);

  delayMicroseconds(10);

  static const int cyc = 200;

  // clock out some bits
  for (int i = 0; i < 18; ++i)
  {
    for (int i = 0; i < cyc; i++); // delay
    scl_.toggle();
  }
  delayMicroseconds(1);

  // send a start condition
  scl_.write(GPIO::LOW);
  for (int i = 0; i < cyc; i++); // delay
  sda_.write(GPIO::LOW);
  for (int i = 0; i < cyc; i++); // delay

  // then a stop
  scl_.write(GPIO::HIGH);
  for (int i = 0; i < cyc; i++); // delay
  sda_.write(GPIO::HIGH);
  for (int i = 0; i < cyc; i++); // delay

  // turn things back on
  scl_.set_mode(GPIO::PERIPH_IN_OUT);
  sda_.set_mode(GPIO::PERIPH_IN_OUT);
  I2C_Cmd(c_->dev, ENABLE);

  last_event_us_ = micros();
  log_line;
}

/// Helper Abstractions
int8_t I2C::checkPresent(uint8_t addr, void (*cb)(int8_t))
{
  clear_log;
  log_line;
  bool blocking = (cb==nullptr);
  if (blocking)
    waitToFinish;

  uint8_t dummy_byte = 0x00;

  beginJob();
  addTaskStart();
  addTaskWriteMode(addr);
  addTaskWrite(dummy_byte);
  addTaskStop(cb);
  if (finalizeJob())
  {
    return_code_ = RESULT_SUCCESS;
    if (blocking)
      waitToFinish;
    return return_code_;
  }
  else
    return RESULT_ERROR;
}

int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t* data, void(*cb)(int8_t))
{
  return read(addr, reg, data, 1, cb);
}

int8_t I2C::read(uint8_t addr, uint8_t *data, size_t len, void (*cb)(int8_t))
{
  return read(addr, 0xFF, data, len, cb);
}

int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len, void (*cb)(int8_t))
{
    clear_log;
    log_line;
    bool blocking = (cb == nullptr);

    if_timeout(blocking && checkBusy(), tout_block)
    {
      log_line;
      num_errors_++;
      return RESULT_ERROR;
    }

    beginJob();
    addTaskStart();
    if (reg != 0xFF)
    {
      addTaskWriteMode(addr);
      addTaskWrite(reg);
      addTaskStart();
    }
    addTaskReadMode(addr);
    addTaskRead(data, len);
    addTaskStop(cb);

    if (finalizeJob())
        return_code_ = RESULT_SUCCESS;
    else
        return RESULT_ERROR;

    if_timeout(blocking && checkBusy(), tout_block)
    {
      log_line;
      num_errors_++;
      return RESULT_ERROR;
    }
    return return_code_;
}

int8_t I2C::write(uint8_t addr, uint8_t reg, uint8_t data, void(*cb)(int8_t))
{
  write(addr, reg, &data, 1, cb);
}

int8_t I2C::write(uint8_t addr, uint8_t* data, size_t len, void(*cb)(int8_t))
{
  write(addr, 0xFF, data, len, cb);
}

int8_t I2C::write(uint8_t addr, uint8_t data, void(*cb)(int8_t))
{
  write(addr, 0xFF, &data, 1, cb);
}

int8_t I2C::write(uint8_t addr, uint8_t reg, uint8_t *data, size_t len, void (*cb)(int8_t))
{
    clear_log;
    log_line;
    bool blocking = (cb == nullptr);

    if_timeout(blocking && checkBusy(), tout_block)
    {
      log_line;
      num_errors_++;
      return RESULT_ERROR;
    }

    beginJob();
    addTaskStart();
    addTaskWriteMode(addr);
    if (reg != 0xFF)
      addTaskWrite(reg);
    addTaskWrite(data, len);
    addTaskStop(cb);

    if (finalizeJob())
        return_code_ = RESULT_SUCCESS;
    else
        return RESULT_ERROR;

    if_timeout(blocking && checkBusy(), tout_block)
    {
      log_line;
      num_errors_++;
      return RESULT_ERROR;
    }
    return return_code_;
}


// Trampolines for C->C++ linkage
extern "C"
{

// C-based IRQ functions (defined in the startup script)
void DMA1_Stream2_IRQHandler(void)
{
  log_line;
  if (DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2))
  {
    log_line;
    DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
    I2C_DMACmd(I2C2, DISABLE);
    DMA_Cmd(DMA1_Stream2, DISABLE);
    I2C2_Ptr->handleEvent();
  }
}

void DMA1_Stream0_IRQHandler(void)
{
  log_line;
  if (DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0))
  {
    log_line;
    DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
    I2C_DMACmd(I2C1, DISABLE);
    DMA_Cmd(DMA1_Stream0, DISABLE);
    I2C1_Ptr->handleEvent();
  }
}

void I2C1_ER_IRQHandler(void) {
  I2C1_Ptr->handleError();
}

void I2C1_EV_IRQHandler(void) {
  I2C1_Ptr->handleEvent();
}

void I2C2_ER_IRQHandler(void) {
  I2C2_Ptr->handleError();
}

void I2C2_EV_IRQHandler(void) {
  I2C2_Ptr->handleEvent();
}

void I2C3_ER_IRQHandler(void) {
  I2C3_Ptr->handleError();
}

void I2C3_EV_IRQHandler(void) {
  I2C3_Ptr->handleEvent();
}

}
