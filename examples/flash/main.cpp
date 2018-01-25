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

typedef struct
{
  uint8_t magic_BE = 0xBE;
  uint8_t big_array[500];
  uint8_t magic_AC = 0xAC;
  uint8_t big_array2[500];
  uint8_t magic_D3 = 0xD3;
  uint8_t crc;
} config_t;

int main()
{

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

  // Create a gigantic "config file"
  config_t config_file;

  while(1)
  {
    info.toggle();
    flash.write_config((uint8_t*)&config_file, sizeof(config_t));
    delay(10);
  }
}
