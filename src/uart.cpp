#include "uart.h"

UART* UART1Ptr = NULL;

UART::UART(USART_TypeDef *_uart)
{
  dev_ = _uart;

  if (dev_ == USART1)
  {
    rx_pin_.init(GPIOA, GPIO_Pin_9, GPIO::PERIPH_IN);
    rx_pin_.init(GPIOA, GPIO_Pin_10, GPIO::PERIPH_IN);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    UARTIRQ_ = USART1_IRQn;
    RxDMAIRQ_ = DMA2_Stream2_IRQn;
    TxDMAIRQ_ = DMA2_Stream7_IRQn;
    DMA_Channel_ = DMA_Channel_4;
    UART1Ptr = this;
  }

  init_UART(115200);
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
  USART_Init(dev_, &USART_InitStruct);

  // Throw interrupts on byte receive
  USART_ITConfig(dev_, USART_IT_RXNE, ENABLE);
  USART_Cmd(dev_, ENABLE);
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
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(dev_->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_;

  // Configure the Tx DMA
  DMA_DeInit(Tx_DMA_Stream_);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) tx_buffer_;
  DMA_Init(Tx_DMA_Stream_, &DMA_InitStructure);

  // Configure the Rx DMA
  DMA_DeInit(Rx_DMA_Stream_);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) rx_buffer_;
  DMA_Init(Rx_DMA_Stream_, &DMA_InitStructure);

  //  Hook up the DMA to the uart
  USART_DMACmd(dev_ , USART_DMAReq_Tx, ENABLE);
  USART_DMACmd(dev_, USART_DMAReq_Rx, ENABLE);

  // Turn on the transfer complete interrupt source from the DMA
  DMA_ITConfig(Tx_DMA_Stream_, DMA_IT_TC, ENABLE);
  DMA_ITConfig(Rx_DMA_Stream_, DMA_IT_TC, ENABLE);

  // Turn on the DMA
  DMA_Cmd(Rx_DMA_Stream_, ENABLE);
  DMA_Cmd(Tx_DMA_Stream_, ENABLE);

  // set the buffer pointers to where the DMA is starting (starts at 256 and counts down)
  rx_buffer_tail_ = DMA_GetCurrDataCounter(Rx_DMA_Stream_);
  rx_buffer_head_ = DMA_GetCurrDataCounter(Rx_DMA_Stream_);

  // Initialize the Circular Buffers
  rx_buffer_head_ = RX_BUFFER_SIZE; // DMA counts down on receive
  rx_buffer_tail_ = RX_BUFFER_SIZE;
  tx_buffer_head_ = 0;
  tx_buffer_tail_ = 0;
}


void UART::init_NVIC()
{
  // Configure the Interrupt
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = UARTIRQ_;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = TxDMAIRQ_;
  NVIC_Init(&NVIC_InitStruct);

  //  NVIC_InitStruct.NVIC_IRQChannel = RxDMAIRQ_;
  //  NVIC_Init(&NVIC_InitStruct);
}


void UART::write(uint8_t* ch, uint8_t len)
{
  // Put Data on the tx_buffer
  for (int i = 0; i < len ; i++)
  {
    tx_buffer_[tx_buffer_head_] = ch[i];
    tx_buffer_head_ = (tx_buffer_head_ + 1) % TX_BUFFER_SIZE;
  }

  if (DMA_GetCmdStatus(Tx_DMA_Stream_) == ENABLE)
  {
    startDMA();
  }
}

void UART::startDMA()
{
  // Set the start of the transmission to the oldest data
  Tx_DMA_Stream_->M0AR = (uint32_t)&tx_buffer_[tx_buffer_tail_];
  if(tx_buffer_head_ > tx_buffer_tail_)
  {
    // Set the length of the transmission to the data on the buffer
    // if contiguous, this is easy
    DMA_SetCurrDataCounter(Tx_DMA_Stream_, tx_buffer_head_ - tx_buffer_tail_);
    tx_buffer_tail_ = tx_buffer_head_;
  }
  else
  {
    // We will have to send the data in two groups, first the tail,
    // then the head we will do later
    DMA_SetCurrDataCounter(Tx_DMA_Stream_, TX_BUFFER_SIZE - tx_buffer_tail_);
    tx_buffer_tail_ = 0;
  }
  // Start the Transmission
  DMA_Cmd(Tx_DMA_Stream_, ENABLE);
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
  //  while (!USART_GetFlagStatus())
}

uint32_t UART::rx_bytes_waiting()
{
  // Remember, the DMA CNDTR counts down
  rx_buffer_head_ = DMA_GetCurrDataCounter(Rx_DMA_Stream_);
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
  tx_buffer_head_ = DMA_GetCurrDataCounter(Tx_DMA_Stream_);
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

bool UART::set_mode(uint8_t mode){}

bool UART::flush()
{
  uint32_t timeout = 10000;
  while (!tx_buffer_empty() && --timeout);
  if (timeout)
    return true;
  else
    return false;
}

void UART::register_rx_callback(std::function<void(uint8_t)> cb)
{
  receive_CB_ = cb;
}

void UART::unregister_rx_callback()
{
  receive_CB_ = nullptr;
}
