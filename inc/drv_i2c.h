/**
 * STM32F4xx I2C driver for OpenPilot REVO
 *
 * Adapted from https://github.com/jihlein/AQ32Plus/blob/master/src/drv/drv_i2c.h
 * 			and https://github.com/superjax/airbourne/blob/master/include/i2c.h
 * @author len0rd
 * @since 2017-08-23
 */
#pragma once

#include "revo_f4.h"

#include "gpio.h"

class I2C {
private:
  void handle_hardware_failure();

  I2C_TypeDef* dev;

  GPIO scl_;
  GPIO sda_;

  uint16_t error_count_ = 0;

  //Variables for current job:
  bool reading_;
  bool subaddress_sent_;
  volatile bool busy_  = false;
  volatile bool error_ = false;
  uint8_t index_;

  volatile uint8_t  addr_;
  volatile uint8_t  reg_;
  volatile uint8_t  len_;
  volatile uint8_t* data_buffer_;

  DMA_InitTypeDef  DMA_InitStructure_;

public:
  I2C(I2C_TypeDef *I2C);

  void init();
  void unstick();
  bool write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
  bool read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
  void DMA_Read(uint8_t addr, uint8_t reg, uint8_t num_bytes, uint8_t* data);

  bool write(uint8_t addr, uint8_t reg, uint8_t data);
  bool read(uint8_t addr, uint8_t reg, uint8_t *data);

  inline uint16_t num_errors() { return error_count_; }

  //interrupt handlers
  void handle_error();
  void handle_event();
  void transfer_complete_cb();
};

//global i2c ptrs used by the event interrupts
extern I2C* I2CDev_1Ptr;
extern I2C* I2CDev_2Ptr;
