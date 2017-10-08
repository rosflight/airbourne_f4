/**
 * STM32F4xx I2C driver for OpenPilot REVO
 *
 * Adapted from https://github.com/jihlein/AQ32Plus/blob/master/src/drv/drv_i2c.h
 * 			and https://github.com/superjax/airbourne/blob/master/include/i2c.h
 * @author len0rd
 * @since 2017-08-23
 */
#pragma once

#include <functional>

#include "revo_f4.h"

#include "gpio.h"

class I2C {
private:
  void handle_hardware_failure();

  I2C_TypeDef* dev_;

  GPIO scl_;
  GPIO sda_;

  uint16_t error_count_ = 0;

  //Variables for current job:
  bool busy_  = false;
  bool subaddress_sent_ = false;

  volatile uint8_t  addr_;
  volatile uint8_t  reg_;
  volatile uint8_t  len_;

  DMA_InitTypeDef  DMA_InitStructure_;
  DMA_Stream_TypeDef* DMA_stream_;
  uint32_t DMA_channel_;
  uint32_t DMA_Stream_TCFLAG_;
  IRQn_Type DMA_IRQn_;
  IRQn_Type I2C_EV_IRQn_;
  IRQn_Type I2C_ER_IRQn_;

public:
  std::function<void(void)> cb_;
  I2C(I2C_TypeDef *I2C);

  void init();
  void unstick();
  void hardware_failure();
  int8_t read(uint8_t addr, uint8_t reg, uint8_t num_bytes, uint8_t* data, std::function<void(void)> callback, bool blocking = false);

  int8_t write(uint8_t addr, uint8_t reg, uint8_t data);
  int8_t read(uint8_t addr, uint8_t reg, uint8_t *data);

  inline uint16_t num_errors() { return error_count_; }

  //interrupt handlers
  bool handle_error();
  bool handle_event();
  void transfer_complete_cb();
};

//global i2c ptrs used by the event interrupts
extern I2C* I2CDev_1Ptr;
extern I2C* I2C2_Ptr;
