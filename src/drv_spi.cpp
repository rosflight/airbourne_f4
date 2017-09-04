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
    SPI_Cmd(dev, ENABLE);

    // Wait for any transfers to clear (this should be really short if at all)
    while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_ReceiveData(dev); //dummy read if needed

    // Configure the DMA
    DMA_DeInit(DMA2_Stream3); //SPI1_TX_DMA_STREAM
    DMA_DeInit(DMA2_Stream2); //SPI1_RX_DMA_STREAM

    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_BufferSize = (uint16_t)(14 + 3); // we receive 14 bytes

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
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) tx_buffer_;
    DMA_Init(DMA2_Stream3, &DMA_InitStructure);

    /* Configure Rx DMA */
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) rx_buffer_;
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

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

  for (uint8_t i = 0; i < num_bytes; i++)
  {
    while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET)
    {
      if ((spiTimeout--) == 0)
        return false;
    }

    SPI_I2S_SendData(dev, data[i]);

    spiTimeout = 0x1000;

    while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE) == RESET)
    {
      if ((spiTimeout--) == 0)
        return false;
    }
    // Pack received data into the same array
    data[i] = (uint8_t)SPI_I2S_ReceiveData(dev);
  }
}
