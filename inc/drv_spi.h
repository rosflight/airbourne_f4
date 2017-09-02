/**
 * STM32F4xx SPI driver for OpenPilot REVO
 *
 * Adapted from https://github.com/jihlein/AQ32Plus/blob/master/src/drv/drv_spi.h
 * 			and https://github.com/superjax/airbourne/blob/f4/f4/src/spi.cpp
 * @author len0rd
 * @since 2017-08-04
 */
#pragma once

#include "revo_f4.h"

class SPI {
public:
  SPI(SPI_TypeDef *SPI);

  void set_divisor(uint16_t new_divisor);
  void enable();
  void disable();
  uint8_t transfer(uint8_t data);

private:
  GPIO_TypeDef*	nss_gpio;
  uint16_t nss_pin;
  SPI_TypeDef*	dev;
};
