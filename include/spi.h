#ifndef SPI_H
#define SPI_H

#include "system.h"
#include "gpio.h"

class SPI
{
public:
  enum
  {
    INITIALIZATION = 256,
    SLOW           = 128, //00.65625 MHz
    STANDARD       = 8,   //10.50000 MHz
    FAST           = 4,   //21.00000 MHz
    ULTRAFAST      = 2    //42.00000 MHz
  };
  SPI(spi_configuration_t config);
  void init();

  bool transfer(uint8_t* out, uint32_t len, uint8_t* in);
  bool is_busy();

  uint32_t get_error_count();
  void set_divisor(uint16_t divisor);
  void set_ss_low();
  void set_ss_high();

private:
  GPIO nss_;
  GPIO sck_;
  GPIO miso_;
  GPIO mosi_;

  SPI_TypeDef* dev;

  bool leading_edge;
  bool using_nss;
  uint16_t divisor;
  uint32_t error_count;
};

#endif // SPI_H
