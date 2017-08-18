#ifndef DRV_SPI_H
#define DRV_SPI_H

#include "system.h"

class SPI {
public:
	SPI(SPI_TypeDef *SPI);

	void set_divisor(uint16_t new_divisor);
	void enable();
	void disable();
	uint8_t transfer(uint8_t data);

private:
	GPIO_TypeDef*	nss_gpio;
	uint16_t		nss_pin; 
	SPI_TypeDef*	dev;
};


#endif // DRV_SPI_H
