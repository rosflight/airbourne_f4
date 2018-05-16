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

#define while_check(cond, result) \
  {\
    int32_t timeout_var = 5000; \
    while ((cond) && timeout_var) \
      timeout_var--; \
    if (!timeout_var) \
    { \
      handle_hardware_failure();\
      result = ERROR; \
    }\
  }

//global i2c ptrs used by the event interrupts
I2C* I2C1_Ptr;
I2C* I2C2_Ptr;
I2C* I2C3_Ptr;

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

  unstick(); //unstick will properly initialize pins

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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
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

  I2C_Cmd(c->dev, ENABLE);
}

void I2C::unstick()
{
  I2C_Cmd(c_->dev, DISABLE);
  
  I2C_ClearFlag(c_->dev, I2C_FLAG_BUSY);
  
  // Turn off the interrupts
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
  
  //reset errors
  I2C_ClearFlag(c_->dev, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);
    
  scl_.set_mode(GPIO::OUTPUT);
  sda_.set_mode(GPIO::OUTPUT);

  scl_.write(GPIO::HIGH);
  sda_.write(GPIO::HIGH);

  // clock out some bits
  for (int i = 0; i < 8; ++i)
  {
    delayMicroseconds(1);
    scl_.toggle();
  }

  // send a start condition
  sda_.write(GPIO::LOW);
  delayMicroseconds(1);
  scl_.write(GPIO::LOW);
  delayMicroseconds(1);

  // then a stop
  scl_.write(GPIO::HIGH);
  delayMicroseconds(1);
  sda_.write(GPIO::HIGH);
  delayMicroseconds(1);

  // turn things back on
  scl_.set_mode(GPIO::PERIPH_IN_OUT);
  sda_.set_mode(GPIO::PERIPH_IN_OUT);
  I2C_Cmd(c_->dev, ENABLE);
  
  current_status_ = IDLE;  
}


int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t num_bytes, uint8_t* data, std::function<void(void)> callback, bool blocking)
{
  if (current_status_ != IDLE)
    return BUSY;
  current_status_ = READING;
  addr_ = addr << 1;
  cb_ = callback;
  reg_ = reg;
  subaddress_sent_ = (reg_ == 0xFF);
  len_ = num_bytes;
  done_ = false;

  DMA_DeInit(c_->DMA_Stream);
  DMA_InitStructure_.DMA_BufferSize = static_cast<uint16_t>(len_);
  DMA_InitStructure_.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>(data);
  DMA_Init(c_->DMA_Stream, &DMA_InitStructure_);

  I2C_Cmd(c_->dev, ENABLE);
  bool result = SUCCESS;

  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), result);

  // If we don't need to send the subaddress, then go ahead and spool up the DMA NACK
  if (subaddress_sent_)
  {
    I2C_AcknowledgeConfig(c_->dev, ENABLE);
    I2C_DMALastTransferCmd(c_->dev, ENABLE);
  }
  I2C_GenerateSTART(c_->dev, ENABLE);

  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

  if (blocking)
  {
    while(current_status_ != IDLE);
  }
  return result;
}


void I2C::transfer_complete_cb()
{
  current_status_ = IDLE;
  if (cb_)
    cb_();
}


// blocking, single register read (for configuring devices)
int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t *data)
{
  if (current_status_ != IDLE)
    return BUSY;
  int8_t return_code = SUCCESS;

  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code);

  I2C_Cmd(c_->dev, ENABLE);
  if (reg != 0xFF)
  {
    I2C_GenerateSTART(c_->dev, ENABLE);
    while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_MODE_SELECT), return_code);
    I2C_Send7bitAddress(c_->dev, addr << 1, I2C_Direction_Transmitter);
    uint32_t timeout = 500;
    while (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --timeout != 0);
    if (timeout != 0 || return_code != ERROR)
    {
      I2C_GenerateSTOP(c_->dev, ENABLE);
      I2C_Cmd(c_->dev, DISABLE);
      return ERROR;
    }
    I2C_Cmd(c_->dev, ENABLE);
    I2C_SendData(c_->dev, reg);
    while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code);
  }

  // Read the byte
  I2C_AcknowledgeConfig(c_->dev, DISABLE);
  I2C_GenerateSTART(c_->dev, ENABLE);
  while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_MODE_SELECT), return_code);
  I2C_Cmd(c_->dev, ENABLE);
  I2C_Send7bitAddress(c_->dev, addr << 1, I2C_Direction_Receiver);
  uint32_t timeout = 500;
  while (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_BYTE_RECEIVED) && --timeout != 0);
  if (timeout != 0 && return_code != ERROR)
  {
    *data = I2C_ReceiveData(c_->dev);
  }
  I2C_GenerateSTOP(c_->dev, ENABLE);
  I2C_Cmd(c_->dev, DISABLE);

  return return_code;
}

// asynchronous write, for commanding adc conversions
int8_t I2C::write(uint8_t addr, uint8_t reg, uint8_t data, std::function<void(void)> callback, bool blocking)
{
  if (current_status_ != IDLE)
    return BUSY;

  current_status_ = WRITING;
  addr_ = addr << 1;
  cb_ = callback;
  reg_ = reg;
  subaddress_sent_ = (reg_ == 0xFF);
  len_ = 1;
  done_ = false;
  data_ = data;

  I2C_Cmd(c_->dev, ENABLE);
  
  bool return_code = SUCCESS;

  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code);

  I2C_GenerateSTART(c_->dev, ENABLE);

  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

  if (blocking)
    while (current_status_ != IDLE);

  return return_code;
}

// blocking, single register write (for configuring devices)
int8_t I2C::write(uint8_t addr, uint8_t reg, uint8_t data)
{
  if (current_status_ != IDLE)
    return BUSY;
  bool return_code = SUCCESS;
  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code);
  I2C_Cmd(c_->dev, ENABLE);

  // start the transfer
  I2C_GenerateSTART(c_->dev, ENABLE);
  while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_MODE_SELECT), return_code);
  I2C_Send7bitAddress(c_->dev, addr << 1, I2C_Direction_Transmitter);
  uint32_t timeout = 500;
  while (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --timeout);
  if (!timeout)
  {
    I2C_GenerateSTOP(c_->dev, ENABLE);
    I2C_Cmd(c_->dev, DISABLE);
    return ERROR;
  }

  // Send the register
  if (reg != 0xFF)
  {
    I2C_SendData(c_->dev, reg);
    while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code);
  }

  // Write the byte with a NACK
  I2C_AcknowledgeConfig(c_->dev, DISABLE);
  I2C_SendData(c_->dev, data);
  while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code);
  I2C_GenerateSTOP(c_->dev, ENABLE  );
  I2C_Cmd(c_->dev, DISABLE);
  return return_code;

}

// if for some reason, a step in an I2C read or write fails, call this
void I2C::handle_hardware_failure() {
  error_count_++;
//  unstick(); //unstick and reinitialize the hardware
}


// This is the I2C_IT_ERR handler
bool I2C::handle_error()
{
  I2C_Cmd(c_->dev, DISABLE);
  bool return_code = SUCCESS;
  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code);

  // Turn off the interrupts
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

  //reset errors
  I2C_ClearFlag(c_->dev, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);
  current_status_ = IDLE;
  return true;
}

// This is the I2C_IT_EV handler
void I2C::handle_event()
{
  uint32_t last_event = I2C_GetLastEvent(c_->dev);

  // We just sent a byte
  if (last_event == I2C_EVENT_MASTER_BYTE_TRANSMITTED)
  {
    // If we are reading, then we just sent a subaddress and need to send
    // a repeated start, and enable the DMA NACK
    if (current_status_ == READING)
    {
      I2C_AcknowledgeConfig(c_->dev, ENABLE);
      I2C_DMALastTransferCmd(c_->dev, ENABLE);
      I2C_GenerateSTART(c_->dev, ENABLE);
    }
    // We are in write mode and are done, need to clean up
    else
    {
      I2C_GenerateSTOP(c_->dev, ENABLE);
      I2C_ITConfig(c_->dev, I2C_IT_EVT, DISABLE);
      transfer_complete_cb();
    }
  }

  // We just sent the address in write mode
  if (last_event == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
  {
    // We need to send the subaddress
    if (!subaddress_sent_)
    {
      I2C_SendData(c_->dev, reg_);
      subaddress_sent_ = true;
      if (current_status_ == WRITING)
      {
        I2C_SendData(c_->dev, data_);
        done_ = true;
      }
    }
    // We need to send our data (no subaddress)
    else
    {
      I2C_SendData(c_->dev, data_);
      done_ = true;
    }
  }

  // We are in receiving mode, preparing to receive the big DMA dump
  if (last_event == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
  {
    I2C_ITConfig(c_->dev, I2C_IT_EVT, DISABLE);
    DMA_SetCurrDataCounter(c_->DMA_Stream, len_);
    I2C_DMACmd(c_->dev, ENABLE);
    DMA_ITConfig(c_->DMA_Stream, DMA_IT_TC, ENABLE);
    DMA_Cmd(c_->DMA_Stream, ENABLE);
  }

  // Start just sent
  if (last_event == I2C_EVENT_MASTER_MODE_SELECT)
  {
    // we either don't need to send, or already sent the subaddress
    if (subaddress_sent_ && current_status_ == READING)
    {
      // Set up a receive
      I2C_Send7bitAddress(c_->dev, addr_, I2C_Direction_Receiver);
    }
    // We need to either send the subaddress or our datas
    else
    {
      // Set up a write
      I2C_Send7bitAddress(c_->dev, addr_, I2C_Direction_Transmitter);
    }
  }
}

extern "C"
{

// C-based IRQ functions (defined in the STD lib somewhere)
void DMA1_Stream2_IRQHandler(void)
{

  if (DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2))
  {
    /* Clear transmission complete flag */
    DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);

    I2C_DMACmd(I2C2, DISABLE);
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Disable DMA channel*/
    DMA_Cmd(DMA1_Stream2, DISABLE);

    I2C2_Ptr->transfer_complete_cb(); // TODO make this configurable
  }
}

void DMA1_Stream0_IRQHandler(void)
{
  if (DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0))
  {
    /* Clear transmission complete flag */
    DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);

    I2C_DMACmd(I2C1, DISABLE);
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    /* Disable DMA channel*/
    DMA_Cmd(DMA1_Stream0, DISABLE);

    I2C1_Ptr->transfer_complete_cb(); // TODO make this configurable
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
