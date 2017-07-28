
#include "stm32f4xx.h"
#include "system.h"
#include "gpio.h"

int main()
{
  systemInit();

  GPIO LED(led_config[0].GPIO, led_config[0].pin, GPIO::OUTPUT);

  while(1)
  {
    delay(200);
    LED.toggle();
  }
}
