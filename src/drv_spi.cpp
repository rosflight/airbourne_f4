#include "drv_spi.h"


SPI* SPIptr;

SPI::SPI(SPI_TypeDef *SPI) {

  GPIO_InitTypeDef gpio_init_struct;
  SPI_InitTypeDef  spi_init_struct;

  if (SPI == SPI1)
  {
    SPIptr = this;
    // Configure the Select Pin
    nss_.init(GPIOA, GPIO_Pin_4, GPIO::OUTPUT);

    disable();

    // Set the AF configuration for the other pins
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    // Initialize other pins
    sck_.init(GPIOA, GPIO_Pin_5, GPIO::PERIPH_OUT);
    miso_.init(GPIOA, GPIO_Pin_6, GPIO::PERIPH_OUT);
    mosi_.init(GPIOA, GPIO_Pin_7, GPIO::PERIPH_OUT);

    dev = SPI1;

    // Set up the SPI peripheral
    SPI_I2S_DeInit(dev);
    spi_init_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi_init_struct.SPI_Mode = SPI_Mode_Master;
    spi_init_struct.SPI_DataSize = SPI_DataSize_8b;
    spi_init_struct.SPI_CPOL = SPI_CPOL_High;
    spi_init_struct.SPI_CPHA = SPI_CPHA_2Edge;
    spi_init_struct.SPI_NSS = SPI_NSS_Soft;
    spi_init_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // 42/64 = 0.65625 MHz SPI Clock
    spi_init_struct.SPI_FirstBit = SPI_FirstBit_MSB;
    spi_init_struct.SPI_CRCPolynomial = 7;
    SPI_Init(dev, &spi_init_struct);
    SPI_CalculateCRC(dev, DISABLE);
    SPI_Cmd(dev, ENABLE);

    // Wait for any transfers to clear (this should be really short if at all)
    while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_ReceiveData(dev); //dummy read if needed

    DMA_InitStructure_.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    DMA_InitStructure_.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure_.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    DMA_InitStructure_.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
    DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure_.DMA_Priority = DMA_Priority_High;

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_Init(&NVIC_InitStruct);
  }
}

void SPI::set_divisor(uint16_t new_divisor) {
  SPI_Cmd(dev, DISABLE);

  const uint16_t clearBRP = 0xFFC7;

  uint16_t temp = dev->CR1;

  switch(new_divisor) {
  case 2:
    temp &= clearBRP;
    temp |= SPI_BaudRatePrescaler_2;
    break;
  case 4:
    temp &= clearBRP;
    temp |= SPI_BaudRatePrescaler_4;
    break;
  case 8:
    temp &= clearBRP;
    temp |= SPI_BaudRatePrescaler_8;
    break;
  case 16:
    temp &= clearBRP;
    temp |= SPI_BaudRatePrescaler_16;
    break;
  case 32:
    temp &= clearBRP;
    temp |= SPI_BaudRatePrescaler_32;
    break;
  case 64:
    temp &= clearBRP;
    temp |= SPI_BaudRatePrescaler_64;
    break;
  case 128:
    temp &= clearBRP;
    temp |= SPI_BaudRatePrescaler_128;
    break;
  case 256:
    temp &= clearBRP;
    temp |= SPI_BaudRatePrescaler_256;
    break;
  }
  dev->CR1 = temp;

  SPI_Cmd(dev, ENABLE);
}

void SPI::register_complete_cb(void (*cb)())
{
  transfer_cb_ = cb;
}

void SPI::enable() {
  nss_.write(GPIO::HIGH);
}

void SPI::disable() {
  nss_.write(GPIO::LOW);
}

uint8_t SPI::transfer_byte(uint8_t data)
{
  uint8_t byte = data;
  uint16_t spiTimeout;

  spiTimeout = 0x1000;

  while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET)
  {
    if ((spiTimeout--) == 0)
      return false;
  }

  SPI_I2S_SendData(dev, data);

  spiTimeout = 0x1000;

  while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if ((spiTimeout--) == 0)
      return false;
  }
  // Pack received data into the same array
  data = (uint8_t)SPI_I2S_ReceiveData(dev);

  return byte;
}

bool SPI::transfer(uint8_t* out_data, uint8_t num_bytes, uint8_t* in_data)
{
  busy_ = true;

  // Configure the DMA
  DMA_DeInit(DMA2_Stream3); //SPI1_TX_DMA_STREAM
  DMA_DeInit(DMA2_Stream2); //SPI1_RX_DMA_STREAM

  DMA_InitStructure_.DMA_BufferSize = (uint16_t)(num_bytes); // we receive 14 bytes

  /* Configure Tx DMA */
  DMA_InitStructure_.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure_.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure_.DMA_Memory0BaseAddr = (uint32_t) out_data;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure_);

  /* Configure Rx DMA */
  DMA_InitStructure_.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure_.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure_.DMA_Memory0BaseAddr = (uint32_t) in_data;
  DMA_Init(DMA2_Stream2, &DMA_InitStructure_);

  //  Configure the Interrupt
  DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);

  enable();

  DMA_Cmd(DMA2_Stream3, ENABLE); /* Enable the DMA SPI TX Stream */
  DMA_Cmd(DMA2_Stream2, ENABLE); /* Enable the DMA SPI RX Stream */

  /* Enable the SPI Rx/Tx DMA request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

  SPI_Cmd(SPI1, ENABLE);
}



void SPI::transfer_complete_cb()
{
  disable();
  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
  DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);

  DMA_Cmd(DMA2_Stream3, DISABLE);
  DMA_Cmd(DMA2_Stream2, DISABLE);

  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

  SPI_Cmd(SPI1, DISABLE);

  busy_ = false;
  if (transfer_cb_)
    transfer_cb_();
}

extern "C"
{

void DMA2_Stream3_IRQHandler()
{
  if (DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3))
  {
    DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);
    SPIptr->transfer_complete_cb();
  }
}

}
