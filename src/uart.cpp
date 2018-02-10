#include "uart.h"

UART* UART1Ptr = NULL;

UART::UART()
{}


void UART::init(const uart_hardware_struct_t* conf, uint32_t baudrate)
{
  c_ = conf;

  rx_pin_.init(c_->GPIO, c_->Rx_Pin, GPIO::PERIPH_IN_OUT);
  tx_pin_.init(c_->GPIO, c_->Tx_Pin, GPIO::PERIPH_IN_OUT);
  GPIO_PinAFConfig(c_->GPIO, c_->Rx_PinSource, c_->GPIO_AF);
  GPIO_PinAFConfig(c_->GPIO, c_->Tx_PinSource, c_->GPIO_AF);

  if (c_->dev == USART1)
  {
    UART1Ptr = this;
  }

  init_UART(baudrate);
  init_DMA();
  init_NVIC();

  receive_CB_ = nullptr;
}

void UART::init_UART(uint32_t baudrate)
{
  // Configure the device
  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = baudrate;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(c_->dev, &USART_InitStruct);

  // Throw interrupts on byte receive
  USART_ITConfig(c_->dev, USART_IT_RXNE, ENABLE);
  USART_Cmd(c_->dev, ENABLE);
}

void UART::init_DMA()
{
  DMA_InitTypeDef DMA_InitStructure;

  // Common DMA Configuration
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(c_->dev->DR));
  DMA_InitStructure.DMA_Channel = c_->DMA_Channel;

  // Configure the Tx DMA
  DMA_DeInit(c_->Tx_DMA_Stream);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = TX_BUFFER_SIZE;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) tx_buffer_;
  DMA_Init(c_->Tx_DMA_Stream, &DMA_InitStructure);

  // Configure the Rx DMA
  DMA_DeInit(c_->Rx_DMA_Stream);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = RX_BUFFER_SIZE;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) rx_buffer_;
  DMA_Init(c_->Rx_DMA_Stream, &DMA_InitStructure);

  // Turn on the Rx DMA Stream
  DMA_Cmd(c_->Rx_DMA_Stream, ENABLE);

  //  Hook up the DMA to the uart
  USART_DMACmd(c_->dev, USART_DMAReq_Tx, ENABLE);
  USART_DMACmd(c_->dev, USART_DMAReq_Rx, ENABLE);

  // Turn on the transfer complete interrupt source from the DMA
  DMA_ITConfig(c_->Tx_DMA_Stream, DMA_IT_TC, ENABLE);
  DMA_ITConfig(c_->Rx_DMA_Stream, DMA_IT_TC, ENABLE);

  // Initialize the Circular Buffers
  // set the buffer pointers to where the DMA is starting (starts at 256 and counts down)
  rx_buffer_tail_ = DMA_GetCurrDataCounter(c_->Rx_DMA_Stream);
  rx_buffer_head_ = rx_buffer_tail_;
  tx_buffer_head_ = 0;
  tx_buffer_tail_ = 0;

  memset(rx_buffer_, 0, RX_BUFFER_SIZE);
  memset(tx_buffer_, 0, TX_BUFFER_SIZE);
}


void UART::init_NVIC()
{
  // Configure the Interrupt
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = c_->USART_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = c_->Tx_DMA_IRQn;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = c_->Rx_DMA_IRQn;
  NVIC_Init(&NVIC_InitStruct);
}


void UART::write(uint8_t* ch, uint8_t len)
{
  // Put Data on the tx_buffer
  for (int i = 0; i < len ; i++)
  {
    tx_buffer_[tx_buffer_head_] = ch[i];
    tx_buffer_head_ = (tx_buffer_head_ + 1) % TX_BUFFER_SIZE;
  }

  if (DMA_GetCmdStatus(c_->Tx_DMA_Stream) == DISABLE)
  {
    startDMA();
  }
}

void UART::startDMA()
{
  // Set the start of the transmission to the oldest data
  c_->Tx_DMA_Stream->M0AR = (uint32_t)&tx_buffer_[tx_buffer_tail_];
  if(tx_buffer_head_ > tx_buffer_tail_)
  {
    // Set the length of the transmission to the data on the buffer
    // if contiguous, this is easy
    DMA_SetCurrDataCounter(c_->Tx_DMA_Stream, tx_buffer_head_ - tx_buffer_tail_);
    tx_buffer_tail_ = tx_buffer_head_;
  }
  else
  {
    // We will have to send the data in two groups, first the tail,
    // then the head we will do later
    DMA_SetCurrDataCounter(c_->Tx_DMA_Stream, TX_BUFFER_SIZE - tx_buffer_tail_);
    tx_buffer_tail_ = 0;
  }
  // Start the Transmission
  DMA_Cmd(c_->Tx_DMA_Stream, ENABLE);
}

uint8_t UART::read_byte()
{
  uint8_t byte = 0;
  // pull the next byte off the array
  // (the head counts down, because CNTR counts down)
  if(rx_buffer_head_ != rx_buffer_tail_)
  {
    // read a new byte and decrement the tail
    byte = rx_buffer_[RX_BUFFER_SIZE - rx_buffer_tail_];
    if(--rx_buffer_tail_ == 0)
    {
      // wrap to the top if at the bottom
      rx_buffer_tail_ = RX_BUFFER_SIZE;
    }
  }
  return byte;
}


void UART::put_byte(uint8_t ch)
{
  write(&ch, 1);
}

uint32_t UART::rx_bytes_waiting()
{
  // Remember, the DMA CNDTR counts down
  rx_buffer_head_ = DMA_GetCurrDataCounter(c_->Rx_DMA_Stream);
  if (rx_buffer_head_ < rx_buffer_tail_)
  {
    // Easy, becasue it's contiguous
    return rx_buffer_tail_ - rx_buffer_head_;
  }
  else if (rx_buffer_head_ > rx_buffer_tail_)
  {
    // Add the parts on either end of the buffer
    // I'm pretty sure this is wrong
    return rx_buffer_tail_ + RX_BUFFER_SIZE - rx_buffer_head_;
  }
  else
  {
    return 0;
  }
}

uint32_t UART::tx_bytes_free()
{
  tx_buffer_head_ = DMA_GetCurrDataCounter(c_->Tx_DMA_Stream);
  if (tx_buffer_head_ > tx_buffer_tail_)
  {
    return rx_buffer_head_ - rx_buffer_tail_;
  }
  else if (tx_buffer_head_ < tx_buffer_tail_)
  {
    // Add the parts on either end of the buffer
    // I'm pretty sure this is wrong
    return tx_buffer_head_ + RX_BUFFER_SIZE - rx_buffer_tail_;
  }
  else
  {
    return 0;
  }
}

bool UART::set_baud_rate(uint32_t baud)
{
  init_UART(baud);
}

bool UART::tx_buffer_empty()
{
  return tx_buffer_head_ == tx_buffer_tail_;
}

bool UART::flush()
{
  uint32_t timeout = 10000;
  while (!tx_buffer_empty() && --timeout);
  if (timeout)
    return true;
  else
    return false;
}

void UART::DMA_Rx_IRQ_callback()
{
  // DMA took care of putting the data on the buffer
  // Just call the callback until we have not more data
  // Update the head position from the DMA
  rx_buffer_head_ =  DMA_GetCurrDataCounter(c_->Rx_DMA_Stream);
  if(receive_CB_ != nullptr)
  {
    while(rx_buffer_head_ != rx_buffer_tail_)
    {
      // read a new byte and decrement the tail
      uint8_t byte = rx_buffer_[RX_BUFFER_SIZE - rx_buffer_tail_];
      receive_CB_(byte);
      if(rx_buffer_tail_-- == 0)
      {
        // wrap to the top if at the bottom
        rx_buffer_tail_ = RX_BUFFER_SIZE;
      }
    }
  }
}

void UART::DMA_Tx_IRQ_callback()
{
  // If there is more data to be sent
  if(tx_buffer_head_ != tx_buffer_tail_)
  {
    startDMA();
  }
}

void UART::register_rx_callback(std::function<void(uint8_t)> cb)
{
  receive_CB_ = cb;
}

void UART::unregister_rx_callback()
{
  receive_CB_ = nullptr;
}

extern "C"
{

void USART1_IRQHandler (void)
{
  UART1Ptr->DMA_Rx_IRQ_callback();
}

void DMA2_Stream5_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
  {
    DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
    UART1Ptr->DMA_Rx_IRQ_callback();
  }
}

void DMA2_Stream7_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
  {
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
    DMA_Cmd(DMA2_Stream7, DISABLE);
    UART1Ptr->DMA_Tx_IRQ_callback();
  }
}

}
