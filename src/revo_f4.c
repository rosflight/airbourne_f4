#include "system.h"

LED_configuration_t led_config[NUM_LED] = {{GPIOB, GPIO_Pin_5}};

// From Serial.h UART = 0, VCP = 1
serial_configuration_t serial_config[NUM_SERIAL_CONNECTIONS] =
{
  {1, GPIOA, GPIO_Pin_11, GPIO_Pin_12}
};

spi_configuration_t spi_config[NUM_SPI_CONNECTIONS] =
{
  {SPI1, GPIOA, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, RCC_APB2Periph_SPI1,
   DMA2, DMA_Channel_3, DMA1_Stream0_IRQn, DMA1_Stream3_IRQn, true}
};
