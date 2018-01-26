#include "revo_f4.h"

#include "spi.h"
#include "M25P16.h"
#include "led.h"
#include "vcp.h"
#include "printf.h"

#include <cstdlib>

VCP vcp;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    vcp.put_byte(c);
}

typedef struct
{
  uint8_t magic_BE = 0xBE;
  uint8_t big_array[2048];
  uint8_t magic_AC = 0xAC;
  uint8_t big_array2[2048];
  uint8_t magic_D3 = 0xD3;
  uint8_t crc;
} config_t;



int main()
{
  static config_t config_file;
  static uint8_t config_buffer[sizeof(config_t)];

  systemInit();

  vcp.init();
  init_printf(NULL, _putc);

  config_file.magic_AC = 0xAC;
  config_file.magic_BE = 0xBE;
  config_file.magic_D3 = 0xD3;

  for (int i = 0; i < 2048; i++)
  {
    config_file.big_array[i] = (uint8_t)std::rand();
    config_file.big_array2[i] = (uint8_t)std::rand();
  }

  LED warn;
  warn.init(LED1_GPIO, LED1_PIN);
  LED info;
  info.init(LED2_GPIO, LED2_PIN);

  SPI spi;
  spi.init(&spi_config[FLASH_SPI]);

  M25P16 flash;
  flash.init(&spi);

  // calculate crc
  uint8_t crc = 0;
  for (uint8_t* p = (uint8_t*)&config_file; p < (uint8_t*)&config_file + sizeof(config_file); p++)
  {
    crc ^= *p;
  }
  config_file.crc = crc;

  info.on();

  bool success = false;

  // write the config to flash
  flash.write_config((uint8_t*)&config_file, sizeof(config_t));

  // Read config from flash
  flash.read_config(config_buffer, sizeof(config_t));

  // See if it is valid
  config_t* config_ptr = (config_t*) config_buffer;

  // Calculate crc of new data
  crc = 0;
  for (uint8_t* p = (uint8_t*) config_ptr; p < (uint8_t*)config_ptr + sizeof(config_file); p++)
  {
    crc ^= *p;
  }

  if (config_ptr->magic_AC == 0xAC &&
      config_ptr->magic_BE == 0xBE &&
      config_ptr->magic_D3 == 0xD3 &&
      crc == 0)
  {
    warn.on();
    success = true;
  }
  else
  {
    warn.off();
    success = false;
  }

  while(1)
  {
    uint32_t size = sizeof(config_file);
    info.toggle();
    delay(1000);
    if (success)
      printf("successfully wrote, read and validated %d.%dKB worth of data\n", size/1000, size%1000);
    else
      printf("failed\n");
  }
}

