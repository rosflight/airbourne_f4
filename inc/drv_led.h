#ifndef DRV_LED_H
#define DRV_LED_H

#include "system.h"

class LED {
public:

	/**
	 * @param gpio_port  GPIO port to use (ie GPIOA, GPIOB, etc)
	 * @param pin 		 The pin within this port to use (ie: GPIO_Pin_5)
	 */
	LED(GPIO_TypeDef *gpio_port, uint16_t pin);

	void toggle();

	void on();

	void off();

private:
	uint16_t pin_;
	GPIO_TypeDef *port_;
};

#endif // DRV_LED_H