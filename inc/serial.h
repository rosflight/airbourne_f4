#ifndef SERIAL_CLASS_H
#define SERIAL_CLASS_H

#include <stdint.h>
#include <functional>
#include "gpio.h"

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64


class Serial
{
public:
  enum
  {
    POLLING = 0x00,
    INTERRUPT = 0x01,
    DMA_TX = 0x02,
    DMA_RX = 0x04
  };

  enum
  {
    UART = 0,
    VCP = 1
  };
  Serial(){}
  virtual void write(uint8_t*ch, uint8_t len) = 0;
  virtual uint32_t rx_bytes_waiting() = 0;
  virtual uint32_t tx_bytes_free() = 0;
  virtual uint8_t read_byte() = 0;
  virtual bool set_baud_rate(uint32_t baud) = 0;
  virtual bool tx_buffer_empty() = 0;
  virtual void put_byte(uint8_t ch) = 0;
  virtual bool flush() = 0;
  virtual void register_rx_callback(std::function<void(uint8_t)> cb) = 0;
  virtual void unregister_rx_callback();

protected:
  GPIO tx_pin_;
  GPIO rx_pin_;

  uint8_t mode_;

  std::function<void(uint8_t)> receive_CB_;

};

#endif // SERIAL CLASS_H
