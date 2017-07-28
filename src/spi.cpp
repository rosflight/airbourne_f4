#include "spi.h"

SPI::SPI(spi_configuration_t config)
{
  // Turn on the RCC for the SPI peripheral
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  // Initialize the GPIO Pins
  if (config.nss_pin)
  {
    using_nss = true;
    nss_.init(config.GPIO, config.nss_pin, GPIO::OUTPUT);
  }
  else
  {
    using_nss = false;
  }
  sck_.init(config.GPIO, config.sck_pin, GPIO::PERIPH_IN_OUT);
  miso_.init(config.GPIO, config.miso_pin, GPIO::PERIPH_IN_OUT);
  mosi_.init(config.GPIO, config.mosi_pin, GPIO::PERIPH_IN_OUT);

  // This may, or may not be necessary
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

  dev = config.dev;
  leading_edge = config.leading_edge;

  SPI_I2S_DeInit(dev);

  SPI_InitTypeDef spi_init_struct;
  SPI_StructInit(&spi_init_struct);
  spi_init_struct.SPI_Mode = SPI_Mode_Master;
  spi_init_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi_init_struct.SPI_DataSize = SPI_DataSize_8b;
  spi_init_struct.SPI_NSS = SPI_NSS_Soft;
  spi_init_struct.SPI_FirstBit = SPI_FirstBit_MSB;
  spi_init_struct.SPI_CRCPolynomial = 7;
  spi_init_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;

  SPI_Init(dev, &spi_init_struct);
  SPI_Cmd(dev, ENABLE);

  if (using_nss)
  {
    nss_.write(GPIO::HIGH);
  }
}

void SPI::set_divisor(uint16_t divisor)
{
  SPI_Cmd(dev, DISABLE);

  uint16_t temp = dev->CR1;
  temp &= 0xFFC7;
  switch(divisor)
  {
  case 2:
    temp |= SPI_BaudRatePrescaler_2;
    break;
  case 4:
    temp |= SPI_BaudRatePrescaler_4;
    break;
  case 8:
    temp |= SPI_BaudRatePrescaler_8;
    break;
  case 16:
    temp |= SPI_BaudRatePrescaler_16;
    break;
  case 32:
    temp |= SPI_BaudRatePrescaler_32;
    break;
  case 64:
    temp |= SPI_BaudRatePrescaler_64;
    break;
  case 128:
    temp |= SPI_BaudRatePrescaler_128;
    break;
  case 256:
    temp |= SPI_BaudRatePrescaler_256;
    break;
  }
  dev->CR1 = temp;

  SPI_Cmd(dev, ENABLE);
}

uint32_t SPI::get_error_count()
{
  return error_count;
}

bool SPI::is_busy()
{
  return SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET ||
         SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_BSY) == SET;
}

bool SPI::transfer(uint8_t *out, uint32_t len, uint8_t *in)
{
  uint16_t timeout;
  for(int i = 0; i < len; i++)
  {
    timeout = 1000;
    // Wait for any sends to finish
    while(SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET)
    {
      if (--timeout == 0)
        return false;
    }

    if(out)
    {
      // Send the new data
      SPI_I2S_SendData(dev, out[i]);
    }

    timeout = 1000;

    // Wait for any reads to finish
    while(SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE) == RESET)
    {
      if (--timeout == 0)
        return false;
    }
    uint8_t b = SPI_I2S_ReceiveData(dev);
    if(in)
    {
      in[i] = b;
    }
  }
  return true;
}

void SPI::set_ss_low()
{
  nss_.write(GPIO::LOW);
}

void SPI::set_ss_high()
{
  nss_.write(GPIO::HIGH);
}

