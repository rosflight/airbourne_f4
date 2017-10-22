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

class SPI
{

public:

  void init(SPI_TypeDef *SPI);

  void set_divisor(uint16_t new_divisor);
  void enable();
  void disable();
  bool transfer(uint8_t *out_data, uint8_t num_bytes, uint8_t* in_data);
  uint8_t transfer_byte(uint8_t data);
  void transfer_complete_cb();
  void register_complete_cb(void (*cb)(void));

private:
  bool busy_ = false;
  SPI_TypeDef*	dev;
  GPIO mosi_;
  GPIO miso_;
  GPIO sck_;
  GPIO nss_;

  DMA_InitTypeDef DMA_InitStructure_;

  void (*transfer_cb_)(void) = NULL;
};
