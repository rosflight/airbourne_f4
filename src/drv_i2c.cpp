#include "drv_i2c.h"

#define while_check(cond) \
  {\
    int32_t timeout_var = 30000; \
    while ((cond) && timeout_var) \
      timeout_var--; \
    if (!timeout_var) \
    { \
      handle_hardware_failure();\
      return false; \
    }\
  }

//global i2c ptrs used by the event interrupts
I2C* I2C1_Ptr;
I2C* I2C2_Ptr;

I2C::I2C(I2C_TypeDef *I2C) {
  dev = I2C;
  //enable peripheral clocks as we need them
  if (dev == I2C1)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    //configure the gpio pins
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    sda_.init(GPIOB, GPIO_Pin_9, GPIO::PERIPH_IN_OUT);
    scl_.init(GPIOB, GPIO_Pin_8, GPIO::PERIPH_IN_OUT);
    I2C1_Ptr = this;
    DMA_stream_ = DMA1_Stream0;
    DMA_channel_ = DMA_Channel_1;
    DMA_Stream_TCFLAG_ = DMA_FLAG_TCIF0;
    DMA_IRQn_ = DMA1_Stream0_IRQn;
    I2C_EV_IRQn_ = I2C1_EV_IRQn;
    I2C_ER_IRQn_ = I2C1_ER_IRQn;
  }
  else if (dev == I2C2)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    //configure the gpio pins
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
    sda_.init(GPIOB, GPIO_Pin_11, GPIO::PERIPH_IN_OUT);
    scl_.init(GPIOB, GPIO_Pin_10, GPIO::PERIPH_IN_OUT);
    I2C2_Ptr = this;
    DMA_stream_ = DMA1_Stream2;
    DMA_channel_ = DMA_Channel_7;
    DMA_Stream_TCFLAG_ = DMA_FLAG_TCIF2;
    DMA_IRQn_ = DMA1_Stream2_IRQn;
    I2C_EV_IRQn_ = I2C2_EV_IRQn;
    I2C_ER_IRQn_ = I2C2_ER_IRQn;
  }

  debug_.init(GPIOB, GPIO_Pin_0, GPIO::OUTPUT);

  unstick(); //unstick will properly initialize pins

  //initialze the i2c itself
  I2C_DeInit(dev);

  I2C_InitTypeDef I2C_InitStructure;
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_ClockSpeed = 400000;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0; //The first device address
  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(dev, &I2C_InitStructure);

  // Interrupts
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = I2C_EV_IRQn_;
  NVIC_Init(&NVIC_InitStructure);

  // I2C Event Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = I2C_ER_IRQn_;
  NVIC_Init(&NVIC_InitStructure);

  // DMA Event Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA_IRQn_;
  NVIC_Init(&NVIC_InitStructure);

  DMA_Cmd(DMA_stream_, DISABLE);
  DMA_DeInit(DMA_stream_);
  DMA_InitStructure_.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure_.DMA_FIFOThreshold = DMA_FIFOThreshold_Full ;
  DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure_.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure_.DMA_Channel = DMA_channel_;

  DMA_InitStructure_.DMA_PeripheralBaseAddr = (uint32_t)(&(dev->DR));
  DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure_.DMA_DIR = DMA_DIR_PeripheralToMemory;

  I2C_Cmd(dev, ENABLE);
}

void I2C::unstick()
{
  debug_.write(GPIO::HIGH);
  scl_.set_mode(GPIO::OUTPUT);
  sda_.set_mode(GPIO::OUTPUT);

  scl_.write(GPIO::HIGH);
  sda_.write(GPIO::HIGH);

  for (int i = 0; i < 8; ++i)
  {
    delayMicroseconds(3);
    scl_.toggle();
    if ( i == 4)
      debug_.write(GPIO::LOW);
  }

  sda_.write(GPIO::LOW);
  delayMicroseconds(3);
  scl_.write(GPIO::LOW);
  delayMicroseconds(3);



  scl_.write(GPIO::HIGH);
  delayMicroseconds(3);
  sda_.write(GPIO::HIGH);
  delayMicroseconds(3);

  scl_.set_mode(GPIO::PERIPH_IN_OUT);
  sda_.set_mode(GPIO::PERIPH_IN_OUT);
}


bool I2C::read(uint8_t addr, uint8_t reg, uint8_t num_bytes, uint8_t* data, std::function<void(void)> callback)
{
  debug_.write(GPIO::HIGH);
  busy_ = true;
  addr_ = addr << 1;
  cb_ = callback;
  reg_ = reg;
  subaddress_sent_ = (reg_ == 0xFF);
  len_ = num_bytes;

  DMA_DeInit(DMA_stream_);
  DMA_InitStructure_.DMA_BufferSize = (uint16_t)(len_);
  DMA_InitStructure_.DMA_Memory0BaseAddr = (uint32_t) data;
  DMA_Init(DMA_stream_, &DMA_InitStructure_);

  I2C_Cmd(dev, ENABLE);

  while_check (I2C_GetFlagStatus(dev, I2C_FLAG_BUSY));

  I2C_GenerateSTART(dev, ENABLE);

  I2C_ITConfig(dev, I2C_IT_EVT, ENABLE);

  debug_.write(GPIO::LOW);
  return true;
}


void I2C::transfer_complete_cb()
{
  debug_.write(GPIO::HIGH);
  busy_ = false;
  if (cb_ != NULL);
    cb_();
  debug_.write(GPIO::LOW);
}


// blocking, single register read (for configuring devices)
bool I2C::read(uint8_t addr, uint8_t reg, uint8_t *data)
{
  debug_.write(GPIO::HIGH);
  while_check (I2C_GetFlagStatus(dev, I2C_FLAG_BUSY));

  I2C_Cmd(dev, ENABLE);
  if (reg != 0xFF)
  {
    I2C_GenerateSTART(dev, ENABLE);
    while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(dev, addr << 1, I2C_Direction_Transmitter);
    while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_Cmd(dev, ENABLE);
    I2C_SendData(dev, reg);
    while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  }

  // Read the byte
  I2C_AcknowledgeConfig(dev, DISABLE);
  I2C_GenerateSTART(dev, ENABLE);
  while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Cmd(dev, ENABLE);
  I2C_Send7bitAddress(dev, addr << 1, I2C_Direction_Receiver);
  while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_BYTE_RECEIVED));
  *data = I2C_ReceiveData(dev);
  I2C_GenerateSTOP(dev, ENABLE  );
  I2C_Cmd(dev, DISABLE);

  debug_.write(GPIO::LOW);

  return true;

}

// blocking, single register write (for configuring devices)
bool I2C::write(uint8_t addr, uint8_t reg, uint8_t data)
{
  debug_.write(GPIO::HIGH);
  while (I2C_GetFlagStatus(dev, I2C_FLAG_BUSY));
  I2C_Cmd(dev, ENABLE);

  // start the transfer
  I2C_GenerateSTART(dev, ENABLE);
  while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(dev, addr << 1, I2C_Direction_Transmitter);
  while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_Cmd(dev, ENABLE);

  // Send the register
  if (reg != 0xFF)
  {
    I2C_SendData(dev, reg);
    while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  }

  // Write the byte with a NACK
  I2C_AcknowledgeConfig(dev, DISABLE);
  I2C_SendData(dev, data);
  while_check (!I2C_CheckEvent(dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTOP(dev, ENABLE  );
  I2C_Cmd(dev, DISABLE);

  debug_.write(GPIO::LOW);
  return true;

}

// if for some reason, a step in an I2C read or write fails, call this
void I2C::handle_hardware_failure() {
  error_count_++;
  unstick(); //unstick and reinitialize the hardware
}


// This is the I2C_IT_ERR handler
bool I2C::handle_error()
{
  I2C_Cmd(dev, DISABLE);
  while_check (I2C_GetFlagStatus(dev, I2C_FLAG_BUSY));
  I2C_ITConfig(dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE); // Disable EVT and ERR interrupts while bus inactive. They'll be reenabled

  //reset errors
  I2C_ClearFlag(dev, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);
  busy_ = false;
}

// This is the I2C_IT_EV handler
bool I2C::handle_event()
{
  debug_.write(GPIO::HIGH);
  uint32_t last_event = I2C_GetLastEvent(dev);
  if (last_event == I2C_EVENT_MASTER_BYTE_TRANSMITTED)
  {
    I2C_AcknowledgeConfig(dev, ENABLE);
    I2C_DMALastTransferCmd(dev, ENABLE);

    I2C_GenerateSTART(dev, ENABLE);
  }

  // We just sent the address in write mode, preparing to send the subaddress
  if (last_event == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
  {
      I2C_SendData(dev, reg_);
      subaddress_sent_ = true;
  }

  // We are in receiving mode, preparing to receive the big DMA dump
  if (last_event == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
  {
    I2C_ITConfig(dev, I2C_IT_EVT, DISABLE);
    DMA_SetCurrDataCounter(DMA_stream_, len_);
    I2C_DMACmd(dev, ENABLE);
    DMA_ITConfig(DMA_stream_, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA_stream_, ENABLE);
  }

  // Start just sent
  if (last_event == I2C_EVENT_MASTER_MODE_SELECT)
  {
    // we either don't need to send, or already sent the subaddress
    if (subaddress_sent_)
    {
      // Set up a receive
      I2C_Send7bitAddress(dev, addr_, I2C_Direction_Receiver);
    }
    // We need to write a subaddress
    else
    {
      // Set up a write
      I2C_Send7bitAddress(dev, addr_, I2C_Direction_Transmitter);
    }
  }
  debug_.write(GPIO::LOW);
}

extern "C"
{

// C-based IRQ functions (defined in the STD lib somewhere)


void DMA1_Stream2_IRQHandler(void)
{

  if (DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2))
  {
    I2C2_Ptr->debug_.write(GPIO::HIGH);
    /* Clear transmission complete flag */
    DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);

    I2C_DMACmd(I2C2, DISABLE);
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Disable DMA channel*/
    DMA_Cmd(DMA1_Stream2, DISABLE);

    I2C2_Ptr->transfer_complete_cb();
    I2C2_Ptr->debug_.write(GPIO::LOW);
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

    I2C1_Ptr->transfer_complete_cb();
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

}
