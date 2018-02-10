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

  void init(const spi_hardware_struct_t *conf);
  void set_divisor(uint16_t new_divisor);

  void enable(GPIO& cs);
  void disable(GPIO& cs);

  bool transfer(uint8_t *out_data, uint32_t num_bytes, uint8_t* in_data, GPIO* cs = NULL, void (*cb)(void) = NULL);
  uint8_t transfer_byte(uint8_t data, GPIO* cs = NULL);

  void transfer_complete_cb();
  inline bool is_busy() {return busy_;}

private:
  const spi_hardware_struct_t* c_;
  GPIO mosi_;
  GPIO miso_;
  GPIO sck_;

  DMA_InitTypeDef DMA_InitStructure_;

  uint32_t errors_ = 0;
  GPIO* cs_;
  bool busy_ = false;
  void (*transfer_cb_)(void) = NULL;
};
