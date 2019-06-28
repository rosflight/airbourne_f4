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

I2C::I2C()
{
  memset(tasks_, 0, sizeof(tasks_));
  memset(write_buffer_, 0xFF, sizeof(write_buffer_));
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
  DMA_Init(c_->DMA_Stream, &DMA_InitStructure_);

  unstick();
  last_event_us_ = micros();
  I2C_Cmd(c->dev, ENABLE);
  log_line;

  busy_ = false;
  task_head_ = 0;
}

void I2C::clearLog()
{
  clear_log;
}

bool I2C::addJob(TaskType type, uint8_t addr, uint8_t *data, size_t len, void (*cb)(int8_t))
{
  // If we have gone all the way around the buffer and the current task still hasn't been handled,
  // then we are smashing on the task buffer, return false
  if (task_head_ == task_idx_ && !currentTask().handled_)
    return false;

  if (type == TaskType::WRITE)
  {
    // create a local copy, in case the data-to-be-written goes out of scope
    uint8_t* local_data = copyToWriteBuf(data, len);
    if (local_data == nullptr)
      return false;  // ran out of room
    tasks_[task_head_].data = local_data;
  }
  else
  {
    tasks_[task_head_].data = data;
  }
  tasks_[task_head_].task = type;
  tasks_[task_head_].addr = addr << 1;
  tasks_[task_head_].len = len;
  tasks_[task_head_].cb = cb;
  tasks_[task_head_].handled_ = false;
  task_head_ = (task_head_ + 1) % TASK_BUFFER_SIZE;
  /// TODO: check we aren't smashing on tasks
  /// TODO: check we aren't smashing on write buffer if writing

  if (!checkBusy())
  {
    handleJobs();
  }
  return true;
}

I2C::Task& I2C::currentTask()
{
  return tasks_[task_idx_];
}


void I2C::advanceTask()
{
  task_idx_ = (task_idx_ + 1) % TASK_BUFFER_SIZE;
}

int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t* data)
{
  clear_log;
  log_line;
  waitToFinish;

  if (addJob(TaskType::START)
    && addJob(TaskType::WRITE_MODE, addr)
    && addJob(TaskType::WRITE, 0, &reg, 1)
    && addJob(TaskType::START)
    && addJob(TaskType::READ_MODE, addr)
    && addJob(TaskType::READ, 0, data, 1)
    && addJob(TaskType::STOP))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  waitToFinish;
  return return_code_;
}

int8_t I2C::read(uint8_t addr, uint8_t* data, size_t len)
{
  clear_log;
  log_line;
  waitToFinish;

  if (addJob(TaskType::START)
    && addJob(TaskType::READ_MODE, addr)
    && addJob(TaskType::READ, 0, data, len)
    && addJob(TaskType::STOP))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  waitToFinish;
  return return_code_;
}

bool I2C::waitForJob()
{
  if_timeout(I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), tout)
  {
    if (micros() - last_event_us_ > tout_reset)
    {
      unstick();
    }
    log_line;
    num_errors_++;
    return RESULT_ERROR;
  }
  return RESULT_SUCCESS;
}

int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len, void (*cb)(int8_t))
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  if (addJob(TaskType::START)
    && addJob(TaskType::WRITE_MODE, addr)
    && addJob(TaskType::WRITE, 0, &reg, 1)
    && addJob(TaskType::START)
    && addJob(TaskType::READ_MODE, addr)
    && addJob(TaskType::READ, 0, data, len)
    && addJob(TaskType::STOP, 0, 0, 0, cb))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  log_line;
  return return_code_;
}

int8_t I2C::read(uint8_t addr, uint8_t *data, size_t len, void (*cb)(int8_t))
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  if (addJob(TaskType::START)
    && addJob(TaskType::READ_MODE, addr)
    && addJob(TaskType::READ, 0, data, len)
    && addJob(TaskType::STOP, 0, 0, 0, cb))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  log_line;
  return return_code_;
}

int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  if (addJob(TaskType::START)
    && addJob(TaskType::WRITE_MODE, addr)
    && addJob(TaskType::WRITE, 0, &reg, 1)
    && addJob(TaskType::START)
    && addJob(TaskType::READ_MODE, addr)
    && addJob(TaskType::READ, 0, data, len)
    && addJob(TaskType::STOP))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  waitToFinish;
  return return_code_;
}

int8_t I2C::write(uint8_t addr, uint8_t reg, uint8_t data)
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  if (addJob(TaskType::START)
    && addJob(TaskType::WRITE_MODE, addr)
    && addJob(TaskType::WRITE, 0, &reg, 1)
    && addJob(TaskType::WRITE, 0, &data, 1)
    && addJob(TaskType::STOP))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  waitToFinish;
  return return_code_;
}

int8_t I2C::write(uint8_t addr, uint8_t data)
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  if (addJob(TaskType::START)
    && addJob(TaskType::WRITE_MODE, addr)
    && addJob(TaskType::WRITE, 0, &data, 1)
    && addJob(TaskType::STOP))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  waitToFinish;
  return return_code_;
}

int8_t I2C::write(uint8_t addr, uint8_t* data, size_t len)
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  if (addJob(TaskType::START)
    && addJob(TaskType::WRITE_MODE, addr)
    && addJob(TaskType::WRITE, 0, data, len)
    && addJob(TaskType::STOP))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  waitToFinish;
  return return_code_;
}

int8_t I2C::write(uint8_t addr, uint8_t data, void(*cb)(int8_t))
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  if (addJob(TaskType::START)
    && addJob(TaskType::WRITE_MODE, addr)
    && addJob(TaskType::WRITE, 0, &data, 1)
    && addJob(TaskType::STOP, 0, 0, 0, cb))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;
  return return_code_;
}

int8_t I2C::checkPresent(uint8_t addr, void (*cb)(int8_t))
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  uint8_t dummy_byte = 0x00;
  if (addJob(TaskType::START)
    && addJob(TaskType::WRITE_MODE, addr)
    && addJob(TaskType::WRITE, 0, &dummy_byte, 1)
    && addJob(TaskType::STOP, 0, 0, 0, cb))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;
  return return_code_;
}

int8_t I2C::checkPresent(uint8_t addr)
{
  clear_log;
  log_line;
  if (waitForJob() == RESULT_ERROR)
    return RESULT_ERROR;

  uint8_t dummy_byte = 0x00;
  if (addJob(TaskType::START)
   && addJob(TaskType::WRITE_MODE, addr)
   && addJob(TaskType::WRITE, 0, &dummy_byte, 1)
   && addJob(TaskType::STOP))
    return_code_ = RESULT_SUCCESS;
  else
    return RESULT_ERROR;

  if_timeout(checkBusy(), tout_block)
  {
    log_line;
    num_errors_++;
    return RESULT_ERROR;
  }
  log_line;
  return return_code_;
}

uint8_t* I2C::copyToWriteBuf(uint8_t* data, size_t len)
{
  size_t remaining = (wb_head_ >= wb_tail_) ? WRITE_BUFFER_SIZE - (wb_head_ - wb_tail_):
                                              wb_tail_ - wb_head_;
  if (remaining < len)
    return nullptr;

  log_line;
  uint8_t* tmp =  write_buffer_ + wb_head_;
  for (size_t i = 0; i < len; i++)
  {
    write_buffer_[wb_head_] = data[i];
    wb_head_ = (wb_head_ + 1) % WRITE_BUFFER_SIZE;
  }
  return tmp;
}

uint8_t* I2C::getWriteBufferData(uint8_t *begin, size_t write_idx)
{
  log_line;
  uint8_t* data = begin + write_idx;
  if (data >= write_buffer_+WRITE_BUFFER_SIZE)
    data -= WRITE_BUFFER_SIZE;
  wb_tail_ = (wb_tail_ + 1) % WRITE_BUFFER_SIZE;
  return data;
}

bool I2C::handleJobs()
{
  log_line;
  if (task_idx_ == task_head_)
  {
    log_line;
    busy_ = false;
    I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    return true;
  }

  log_line;
  Task& task(currentTask());
  busy_ = true;
  bool done = true;
  bool keep_going = false;
  switch (task.task)
  {
  case TaskType::START:
    log_line;
    expected_event_ = I2C_EVENT_MASTER_MODE_SELECT;
    write_idx_ = 0;
    I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
    I2C_Cmd(c_->dev, ENABLE);
    I2C_GenerateSTART(c_->dev, ENABLE);
    break;

  case TaskType::WRITE_MODE:
    log_line;
    I2C_Send7bitAddress(c_->dev, task.addr, I2C_Direction_Transmitter);
    expected_event_ = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
    break;

  case TaskType::READ_MODE:
    log_line;
    I2C_AcknowledgeConfig(c_->dev, ENABLE);
    I2C_DMALastTransferCmd(c_->dev, ENABLE);
    I2C_Send7bitAddress(c_->dev, task.addr, I2C_Direction_Receiver);
    expected_event_ = I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;
    break;

  case TaskType::WRITE:
    log_line;
    I2C_SendData(c_->dev, *getWriteBufferData(task.data, write_idx_++));
    if (write_idx_ < task.len)
      I2C_SendData(c_->dev, *getWriteBufferData(task.data, write_idx_++));
    expected_event_ = I2C_EVENT_MASTER_BYTE_TRANSMITTED;
    done = (write_idx_ == task.len);
    break;

  case TaskType::READ:
    log_line;
    c_->DMA_Stream->M0AR = reinterpret_cast<uint32_t>(task.data);
    DMA_SetCurrDataCounter(c_->DMA_Stream, task.len);
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
    if (task.cb)
      task.cb(return_code_);
    if (task_idx_ == task_head_)
    {
      log_line;
      busy_ = false;
    }
    else
    {
      log_line;
      keep_going = true;
    }
  }

  if (done)
  {
    log_line;
    task.handled_ = true;
    advanceTask();
  }

  if (keep_going)
  {
    log_line;
    handleJobs();
  }

  return false;
}

void I2C::handleEvent()
{
  last_event_ = I2C_GetLastEvent(c_->dev);
  log_line;
  if ((last_event_ & expected_event_) == expected_event_)
  {
    last_event_us_ = micros();
    log_line;
    if (handleJobs())
      log_line;
  }
}

void I2C::handleError()
{
  log_line;
  return_code_ = RESULT_ERROR;
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
  while (currentTask().task != TaskType::STOP)
  {
    log_line;
    advanceTask();
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

  log_line;
  handleJobs();
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
