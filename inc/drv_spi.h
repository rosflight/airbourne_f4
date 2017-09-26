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

#include "gpio.h"

class SPI {
public:
  SPI(SPI_TypeDef *SPI);

  void set_divisor(uint16_t new_divisor);
  void enable();
  void disable();
  bool transfer(uint8_t *data, uint8_t num_bytes);
  uint8_t transfer_byte(uint8_t data);

private:
  SPI_TypeDef*	dev;
  GPIO mosi_;
  GPIO miso_;
  GPIO sck_;
  GPIO nss_;

  uint8_t tx_buffer_[14];
  uint8_t rx_buffer_[14];
  uint8_t tx_buffer_index_ = 0;
  uint8_t rx_buffer_index_ = 0;
};
