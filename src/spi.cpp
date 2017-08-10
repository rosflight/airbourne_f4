#include "spi.h"

SPI::SPI(spi_configuration_t config)
{
  // RCC for the SPI peripheral
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  // RCC for GPIOA
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_DeInit(config.GPIO);

  if (config.nss_pin)
  {
    //by default on STM32 the nss pin toggles between bytes, which isnt ideal
    //get around this be setting it up as an output and manually toggling it whenever
    //we transfer stuff
    using_nss = true;
    GPIO_InitTypeDef nss_init_struct;
    nss_init_struct.GPIO_Pin   = config.nss_pin;
    nss_init_struct.GPIO_Mode  = GPIO_Mode_OUT;
    nss_init_struct.GPIO_Speed = GPIO_Fast_Speed;
    nss_init_struct.GPIO_OType = GPIO_OType_PP; 
    nss_init_struct.GPIO_PuPd  = GPIO_PuPd_UP; //nss pin connected to pu resistor external to chip

    nss_ = GPIO(config.GPIO, &nss_init_struct, GPIO::OUTPUT);
    nss_.write(GPIO::HIGH);
  }
  else {
    using_nss = false;
  }
  
  //if these pin sources could work with config struct, that'd be nice
  GPIO_PinAFConfig(config.GPIO, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(config.GPIO, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(config.GPIO, GPIO_PinSource7, GPIO_AF_SPI1);
  
  //gpio config
  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.GPIO_Mode  = GPIO_Mode_AF;
  gpio_init_struct.GPIO_Speed = GPIO_Fast_Speed;
  gpio_init_struct.GPIO_OType = GPIO_OType_PP; 
  gpio_init_struct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  gpio_init_struct.GPIO_Pin   = config.sck_pin;
  GPIO_Init(config.GPIO, &gpio_init_struct);

  gpio_init_struct.GPIO_Pin   = config.miso_pin;
  GPIO_Init(config.GPIO, &gpio_init_struct);

  gpio_init_struct.GPIO_Pin   = config.mosi_pin;
  GPIO_Init(config.GPIO, &gpio_init_struct);

  //spi config
  dev = config.dev;
  leading_edge = config.leading_edge;

  SPI_I2S_DeInit(dev);

  SPI_InitTypeDef spi_init_struct;
  SPI_StructInit(&spi_init_struct);
  spi_init_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi_init_struct.SPI_Mode = SPI_Mode_Master;
  spi_init_struct.SPI_DataSize = SPI_DataSize_8b;
  spi_init_struct.SPI_CPOL = SPI_CPOL_Low;
  spi_init_struct.SPI_CPHA = SPI_CPHA_2Edge;
  spi_init_struct.SPI_NSS = SPI_NSS_Soft;
  spi_init_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  spi_init_struct.SPI_FirstBit = SPI_FirstBit_MSB;
  //spi_init_struct.SPI_CRCPolynomial = 7;
  

  SPI_Init(dev, &spi_init_struct);

  //delayMicroseconds(100);
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

void SPI::send(uint8_t address, uint8_t data){
  set_ss_low();

  //uint16_t to_send = ((uint16_t)address << 8) | data;
  while(!SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(dev, address);
  delayMicroseconds(10);
  while(!SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(dev);//receive to clear the rxne flag

  //while(!(dev->SR & SPI_I2S_FLAG_TXE));
  while(!SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(dev, data);
  while(!SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(dev);//receive to clear the rxne flag

  set_ss_high();
}

void SPI::receive(uint8_t start_address, uint8_t length, uint8_t *data_out) {
  set_ss_low();
  start_address = 0x80 | start_address;//setting msb to 1 indicates read op

  for (uint8_t i = 0; i < length; ++i)
  {
    uint8_t request_address = start_address + i;
    while(!SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE)); 
    SPI_I2S_SendData(dev, request_address);
    while(!SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE));
    delayMicroseconds(10);
    SPI_I2S_ReceiveData(dev);

    while(!SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(dev, 0x00); //Dummy byte to generate clock
    while(!SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE));
    delayMicroseconds(10);
    data_out[i] = SPI_I2S_ReceiveData(dev); //Clear RXNE bit
  }

  set_ss_high();
}

bool SPI::transfer(uint8_t *out, uint32_t len, uint8_t *in)
{
  uint16_t timeout;
  for(int i = 0; i < len; i++)
  {
    //timeout = 1000;
    // Wait for any sends to finish
    while(SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET)
    {
      //if (--timeout == 0)//how do we know each of these would be a ms?
      //  return false;
    }

    if(out)
    {
      // Send the new data
      SPI_I2S_SendData(dev, out[i]);
    }

    //timeout = 1000;

    // Wait for any reads to finish
    /*while(SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE) == RESET)
    {
      //if (--timeout == 0)
      //  return false;
    }*/
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
  if (using_nss)
    nss_.write(GPIO::LOW);
}

void SPI::set_ss_high()
{
  if (using_nss)
    nss_.write(GPIO::HIGH);
}

