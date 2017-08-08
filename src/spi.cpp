#include "spi.h"

SPI::SPI(spi_configuration_t config)
{
  // RCC for the SPI peripheral
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  // RCC for GPIOA
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  if (config.nss_pin)
  {
    //by default on STM32 the nss pin toggles between bytes, which isnt ideal
    //get around this be setting it up as an output and manually toggling it whenever
    //we transfer stuff
    using_nss = true;
    GPIO_InitTypeDef nss_init_struct;
    nss_init_struct.GPIO_Pin   = config.nss_pin;
    nss_init_struct.GPIO_Mode  = GPIO_Mode_OUT;
    nss_init_struct.GPIO_Speed = GPIO_Low_Speed;
    nss_init_struct.GPIO_OType = GPIO_OType_OD; //open drain
    nss_init_struct.GPIO_PuPd  = GPIO_PuPd_NOPULL; //nss pin connected to pu resistor external to chip
    //nss_init_struct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(config.GPIO, &nss_init_struct);
  }
  else {
    using_nss = false;//true;  
  }
  
  //if these pin sources could work with config struct, that'd be nice
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  
  //gpio config
  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.GPIO_Pin   = config.sck_pin|config.miso_pin|config.mosi_pin;
  gpio_init_struct.GPIO_Mode  = GPIO_Mode_AF;
  gpio_init_struct.GPIO_Speed = GPIO_Low_Speed; //2Mhz
  gpio_init_struct.GPIO_OType = GPIO_OType_PP; 
  gpio_init_struct.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(config.GPIO, &gpio_init_struct);

  //spi config
  dev = config.dev;
  leading_edge = config.leading_edge;

  SPI_I2S_DeInit(dev);

  SPI_InitTypeDef spi_init_struct;
  SPI_StructInit(&spi_init_struct);
  spi_init_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi_init_struct.SPI_Mode = SPI_Mode_Master;
  
  //8b is already the default from SPI_StructInit
  //spi_init_struct.SPI_DataSize = SPI_DataSize_8b;
  //I think this should be SPI_NSS_Hard(which is default from struct init)
  //spi_init_struct.SPI_NSS = SPI_NSS_Hard;
  //CPOL Low, CPHA 1 edge (ie clk held low, sampled at rising edge)
  spi_init_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //still not sure what to set this to
  spi_init_struct.SPI_FirstBit = SPI_FirstBit_MSB;
  spi_init_struct.SPI_CRCPolynomial = 7;
  

  SPI_Init(dev, &spi_init_struct);
  SPI_Cmd(dev, ENABLE);
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

