#include "revo_f4.h"

#include "spi.h"
#include "M25P16.h"
#include "led.h"
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

//  VCP vcp;
//  vcp.init();
//  uartPtr = &vcp;

//  init_printf(NULL, _putc);

  LED warn;
  warn.init(LED1_GPIO, LED1_PIN);
  LED info;
  info.init(LED2_GPIO, LED2_PIN);

  SPI spi;
  spi.init(&spi_config[FLASH_SPI]);

  M25P16 flash;
  flash.init(&spi);
  while(1)
  {
    info.toggle();
    delay(10);
  }
}
