#ifndef DRV_LED_H
#define DRV_LED_H

#include "revo_f4.h"
#include "gpio.h"

class LED : public GPIO
{
public:

  /**
   * @param gpio_port  GPIO port to use (ie GPIOA, GPIOB, etc)
   * @param pin  The pin within this port to use (ie: GPIO_Pin_5)
   */
  void init(GPIO_TypeDef *gpio_port, uint16_t pin);

  void on();
  void off();
};

#endif // DRV_LED_H
