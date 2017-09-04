#include "drv_spi.h"


SPI::SPI(SPI_TypeDef *SPI) {

  GPIO_InitTypeDef gpio_init_struct;
  SPI_InitTypeDef  spi_init_struct;

  if (SPI == SPI1)
  {
    // Configure the Select Pin
    nss_.init(SPI1_GPIO, SPI1_NSS_PIN, GPIO::OUTPUT);

    disable();

    // Set the AF configuration for the other pins
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MOSI_PIN_SOURCE, GPIO_AF_SPI1);

    // Initialize other pins
    sck_.init(SPI1_GPIO, SPI1_SCK_PIN, GPIO::PERIPH_OUT);
    miso_.init(SPI1_GPIO, SPI1_MISO_PIN, GPIO::PERIPH_OUT);
    mosi_.init(SPI1_GPIO, SPI1_MOSI_PIN, GPIO::PERIPH_OUT);

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

    // Wait for any transfers to clear (this should be really short if at all)
    while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_ReceiveData(dev); //dummy read if needed
  }
}

void SPI::set_divisor(uint16_t new_divisor) {

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
  transfer(&byte, 1);
  return byte;
}

bool SPI::transfer(uint8_t* data, uint8_t num_bytes)
{
  uint16_t spiTimeout;

  spiTimeout = 0x1000;

  uint8_t rx_array[num_bytes];


  DMA_DeInit(DMA2_Stream2); // SPI1 Rx is DMA2 Stream 2 Channel 3
  DMA_DeInit(DMA2_Stream3); // SPI1 Tx is DMA2 Stream 3 Channel 3

  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)(num_bytes); // 1 byte for command, 14 for data
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;

  /* Configure Tx DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) data;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);

  /* Configure Rx DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &data[0];
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);

  DMA_Cmd(DMA2_Stream3, ENABLE); /* Enable the DMA SPI TX Stream */
  DMA_Cmd(DMA2_Stream2, ENABLE); /* Enable the DMA SPI RX Stream */

  /* Enable the SPI Rx/Tx DMA request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

  SPI_Cmd(SPI1, ENABLE);

  while (DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3)==RESET)
  {
    if ((spiTimeout--) == 0)
      return false;
  }
  while (DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2)==RESET);
  {
    if ((spiTimeout--) == 0)
      return false;
  }

  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
  DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);

  DMA_Cmd(DMA2_Stream3, DISABLE);
  DMA_Cmd(DMA2_Stream2, DISABLE);

  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

  SPI_Cmd(SPI1, DISABLE);

  return true;
}
