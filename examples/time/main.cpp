#include "revo_f4.h"

#include "drv_spi.h"
#include "mpu6000.h"
#include "drv_led.h"
#include "vcp.h"
#include "printf.h"

VCP* uartPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}

int main() {

  systemInit();

  VCP vcp;
  uartPtr = &vcp;

  init_printf(NULL, _putc);

  LED warn(LED1_GPIO, LED1_PIN);
  LED info(LED2_GPIO, LED2_PIN);

  GPIO test;
  test.init(GPIOB, GPIO_Pin_0, GPIO::OUTPUT);

  info.on();

  int i = 0;
  int delays[4] = {2000, 2000, 2000, 10000};
  uint64_t next_pulse = 0;
  while(1)
  {
    // To check the timing, just hook up a saelae or a oscilloscope to PWM 1 out.
    if (micros() > next_pulse)
    {
      next_pulse += delays[i++ % 4];
      test.toggle();
    }
  }
}
