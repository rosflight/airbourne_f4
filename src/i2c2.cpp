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


#include "i2c2.h"

namespace i2c2
{

#define while_check(cond, result) \
{\
  int32_t timeout_var = 200; \
  while ((cond) && timeout_var) \
  timeout_var--; \
  if (!timeout_var) \
{ \
  handle_hardware_failure();\
  result = RESULT_ERROR; \
}\
}

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
  }
};
DebugHistory event_history_;
DebugHistory interrupt_history_;
#define log_line event_history_.add_event(__LINE__)
#else
#define log_line
#endif

//global i2c ptrs used by the event interrupts
I2C* I2C1_Ptr;
I2C* I2C2_Ptr;
I2C* I2C3_Ptr;

I2C::I2C()
{
  memset(tasks_, 0, sizeof(tasks_));
}

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

  last_event_us_ = micros();
  I2C_Cmd(c->dev, ENABLE);
  log_line;
}


void I2C::addJob(task_type_t type, uint8_t addr, uint8_t *data, size_t len, void (*cb)(uint8_t))
{
  tasks_[task_head_].task = type;
  tasks_[task_head_].addr = addr;
  tasks_[task_head_].data = data;
  tasks_[task_head_].len = len;
  tasks_[task_head_].cb = cb;
  task_head_ = (task_head_ + 1) % TASK_BUFFER_SIZE;

  if (!checkBusy())
  {
    handleJobs();
  }
}

I2C::Task& I2C::currentTask()
{
  return tasks_[task_idx_];
}

I2C::Task& I2C::nextTask()
{
  size_t tmp = (task_idx_ + 1) % TASK_BUFFER_SIZE;
  return tasks_[tmp];
}

I2C::Task& I2C::prevTask()
{
  size_t tmp = (task_idx_ - 1 + TASK_BUFFER_SIZE) % TASK_BUFFER_SIZE;
  return tasks_[tmp];
}

void I2C::advanceTask()
{
  task_idx_ = (task_idx_ + 1) % TASK_BUFFER_SIZE;
}

void I2C::read(uint8_t addr, uint8_t reg, uint8_t* data)
{
  addJob(START);
  addJob(WRITE_MODE, addr);
  addJob(WRITE, 0, copyToWriteBuf(reg), 1);
  addJob(START);
  addJob(READ_MODE, addr);
  addJob(READ, 0, data, 1);
  addJob(STOP);
  return_code_ = RESULT_SUCCESS;

  while (checkBusy())
  {
    // wait for read to complete
  }

  return return_code_;
}

void I2C::read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len, bool blocking, void (*cb)(uint8_t))
{
  addJob(START);
  addJob(WRITE_MODE, addr);
  addJob(WRITE, 0, copyToWriteBuf(reg), 1);
  addJob(START);
  addJob(READ_MODE, addr);
  addJob(READ, 0, data, len);
  addJob(STOP, 0, 0, 0, cb);
}

void I2C::write(uint8_t addr, uint8_t reg, uint8_t data)
{
  addJob(START);
  addJob(WRITE_MODE, addr);
  addJob(WRITE, 0, copyToWriteBuf(reg), 1);
  addJob(START);
  addJob(WRITE_MODE, addr);
  addJob(WRITE, 0, copyToWriteBuf(data), 1);
  addJob(STOP);

  return_code_ = RESULT_SUCCESS;

  while (checkBusy())
  {
    // wait for read to complete
  }

  return return_code_;
}


uint8_t* I2C::copyToWriteBuf(uint8_t byte)
{
  write_buffer_[wb_head_] = byte;
  uint8_t* tmp =  write_buffer_ + wb_head_;
  wb_head_ = (wb_head_ + 1) % wb_head_;
  return tmp;
}

bool I2C::handleJobs()
{
  if (task_idx_ == task_head_)
  {
    busy_ = false;
    return true;
  }

  Task& task(currentTask());
  busy_ = true;
  bool done = true;
  switch (type)
  {
    case START:
      I2C_GenerateSTART(c_->dev, ENABLE);
      I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
      expected_event_ = I2C_EVENT_MASTER_MODE_SELECT;
      idx_ = 0;
      break;

    case WRITE_MODE:
      I2C_Send7bitAddress(c_->dev, task.addr, I2C_Direction_Transmitter);
      expected_event_ = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
      break;

    case READ_MODE:
      I2C_Send7bitAddress(c_->dev, task.addr, I2C_Direction_Receiver);
      expected_event_ = I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;
      break;

    case WRITE:
      I2C_SendData(c_->dev, task.data[idx_++]);
      if (idx_ < task.len)
        I2C_SendData(c_->dev, task.data[idx_++]);
      expected_event_ = I2C_EVENT_MASTER_BYTE_TRANSMITTED;
      done = (idx_ == task.len);
      break;

    case READ:
      DMA_DeInit(c_->DMA_Stream);
      DMA_InitStructure_.DMA_BufferSize = static_cast<uint16_t>(len_);
      DMA_InitStructure_.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>(data);
      DMA_Init(c_->DMA_Stream, &DMA_InitStructure_);
      DMA_SetCurrDataCounter(c_->DMA_Stream, task.len);
      I2C_DMACmd(c_->dev, ENABLE);
      DMA_ITConfig(c_->DMA_Stream, DMA_IT_TC, ENABLE);
      DMA_Cmd(c_->DMA_Stream, ENABLE);
      expected_event_ = I2C_EVENT_MASTER_BYTE_TRANSMITTED;
      break;

    case STOP:
      I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
      I2C_GenerateSTOP(c_->dev, ENABLE);
      if (task.cb)
        cb(RESULT_SUCCESS);
      expected_event_ = 0;
      break;

    default:
      num_errors_++;
      I2C_GenerateSTOP(c_->dev, ENABLE);
      expected_event_ = 0;
      break;
  }

  if (done)
    advanceTask();

  return false;
}

void I2C::handleEvent()
{
  uint32_t last_event = I2C_GetLastEvent(c_->dev);
  log_line;
  if (last_event != expected_event_)
  {
    log_line;
    if (handleJobs(currentTask()))
    {
      log_line;
      advanceTask();
    }
  }
  else
  {
    log_line;
    handleError();
  }
}

void I2C::handleError()
{
  log_line;
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
  while (currentTask().task != STOP)
  {
    log_line;
    advanceTask();
  }
  log_line;
  handleJobs(currentTask());
  advanceTask();

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
}


}

// Trampolines for C->C++ linkage
extern "C"
{

// C-based IRQ functions (defined in the startup script)
void DMA1_Stream2_IRQHandler(void)
{

  if (DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2))
  {
    // Clear the DMA transmission complete flag
    DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
    // Turn off the DMA
    I2C_DMACmd(I2C2, DISABLE);
    DMA_Cmd(DMA1_Stream2, DISABLE);
    I2C2_Ptr->handleEvent();
  }
}

void DMA1_Stream0_IRQHandler(void)
{
  if (DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0))
  {
    // Clear the DMA transmission complete flag
    DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
    // Turn off the DMA
    I2C_DMACmd(I2C1, DISABLE);
    DMA_Cmd(DMA1_Stream0, DISABLE);
    I2C1_Ptr->handleEvent();
  }
}

void I2C1_ER_IRQHandler(void) {
  I2C1_Ptr->handle_error();
}

void I2C1_EV_IRQHandler(void) {
  I2C1_Ptr->handle_event();
}

void I2C2_ER_IRQHandler(void) {
  I2C2_Ptr->handle_error();
}

void I2C2_EV_IRQHandler(void) {
  I2C2_Ptr->handle_event();
}

void I2C3_ER_IRQHandler(void) {
  I2C3_Ptr->handle_error();
}

void I2C3_EV_IRQHandler(void) {
  I2C3_Ptr->handle_event();
}

}

