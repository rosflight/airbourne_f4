#include "drv_spi.h"


SPI::SPI(SPI_TypeDef *SPI) {
	
	GPIO_InitTypeDef gpio_init_struct;
	SPI_InitTypeDef  spi_init_struct;

	if (SPI == SPI1)
	{

		gpio_init_struct.GPIO_Pin 	= SPI1_NSS_PIN;
		gpio_init_struct.GPIO_Mode 	= GPIO_Mode_OUT;
		gpio_init_struct.GPIO_Speed	= GPIO_Speed_50MHz;
		gpio_init_struct.GPIO_OType = GPIO_OType_PP;
		gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;

		nss_gpio = SPI1_GPIO;
		nss_pin = SPI1_NSS_PIN;

		GPIO_Init(SPI1_GPIO, &gpio_init_struct);

		disable();

		GPIO_PinAFConfig(SPI1_GPIO, SPI1_SCK_PIN_SOURCE, GPIO_AF_SPI1);
		GPIO_PinAFConfig(SPI1_GPIO, SPI1_MISO_PIN_SOURCE, GPIO_AF_SPI1);
		GPIO_PinAFConfig(SPI1_GPIO, SPI1_MOSI_PIN_SOURCE, GPIO_AF_SPI1);

		gpio_init_struct.GPIO_Pin 	= SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
		gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF;
		gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_init_struct.GPIO_OType = GPIO_OType_PP;
		gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;

		GPIO_Init(SPI1_GPIO, &gpio_init_struct);

		SPI_I2S_DeInit(SPI1);

		spi_init_struct.SPI_Direction 			= SPI_Direction_2Lines_FullDuplex;
		spi_init_struct.SPI_Mode 				= SPI_Mode_Master;
		spi_init_struct.SPI_DataSize 			= SPI_DataSize_8b;
		spi_init_struct.SPI_CPOL 				= SPI_CPOL_High;
		spi_init_struct.SPI_CPHA 				= SPI_CPHA_2Edge;
		spi_init_struct.SPI_NSS 				= SPI_NSS_Soft;
		spi_init_struct.SPI_BaudRatePrescaler 	= SPI_BaudRatePrescaler_64;  // 42/64 = 0.65625 MHz SPI Clock
		spi_init_struct.SPI_FirstBit 			= SPI_FirstBit_MSB;
		spi_init_struct.SPI_CRCPolynomial 		= 7;

		SPI_Init(SPI1, &spi_init_struct);
		SPI_CalculateCRC(SPI1, DISABLE);
		SPI_Cmd(SPI1, ENABLE);
		
		dev = SPI1;

		while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET);
  		SPI_I2S_ReceiveData(dev); //dummy read if needed
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
	GPIO_ResetBits(nss_gpio, nss_pin);
}

void SPI::disable() {
	GPIO_SetBits(nss_gpio, nss_pin);
}

uint8_t SPI::transfer(uint8_t data) {
    uint16_t spiTimeout;

    spiTimeout = 0x1000;

    while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return(0);

    SPI_I2S_SendData(dev, data);

    spiTimeout = 0x1000;

    while (SPI_I2S_GetFlagStatus(dev, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
    	    return(0);

    return((uint8_t)SPI_I2S_ReceiveData(dev));
}