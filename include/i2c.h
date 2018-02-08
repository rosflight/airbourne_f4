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
#include <stdint.h>
//For testing only
#include <string.h>
//end testing only

#include "revo_f4.h"

#include "gpio.h"

class I2C
{
private:
  void handle_hardware_failure();

  enum : int8_t
  {
    ERROR = 0,
    SUCCESS = 1,
    BUSY = -1
  };

  typedef enum
  {
    IDLE,
    READING,
    WRITING
  } current_status_t;

  GPIO scl_;
  GPIO sda_;

  uint16_t error_count_ = 0;

  //Variables for current job:
  current_status_t current_status_;
  bool subaddress_sent_ = false;
  bool done_ = false;

  volatile uint8_t  addr_;
  volatile uint8_t  reg_;
  volatile uint8_t  len_;
  volatile uint8_t data_;

  DMA_InitTypeDef  DMA_InitStructure_;

  const i2c_hardware_struct_t *c_;

public:
  std::string log;
  std::function<void(void)> cb_;

  void init(const i2c_hardware_struct_t *c);
  void unstick();
  void hardware_failure();
  int8_t read(uint8_t addr, uint8_t reg, uint8_t num_bytes, uint8_t* data, std::function<void(void)> callback, bool blocking = false);
  int8_t write(uint8_t addr, uint8_t reg, uint8_t data, std::function<void(void)> callback);

  int8_t write(uint8_t addr, uint8_t reg, uint8_t data);
  int8_t read(uint8_t addr, uint8_t reg, uint8_t *data);

  inline uint16_t num_errors() { return error_count_; }

  //interrupt handlers
  bool handle_error();
  bool handle_event();
  void transfer_complete_cb();
};

//global i2c ptrs used by the event interrupts
extern I2C* I2C1_Ptr;
extern I2C* I2C2_Ptr;
extern I2C* I2C3_Ptr;
