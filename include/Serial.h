#ifndef SERIAL_CLASS_H
#define SERIAL_CLASS_H

#include <stdint.h>
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
    SERIAL_NOT_INVERTED  = 0 << 0,
    SERIAL_INVERTED      = 1 << 0,
    SERIAL_STOPBITS_1    = 0 << 1,
    SERIAL_STOPBITS_2    = 1 << 1,
    SERIAL_PARITY_NO     = 0 << 2,
    SERIAL_PARITY_EVEN   = 1 << 2,
    SERIAL_UNIDIR        = 0 << 3,
    SERIAL_BIDIR         = 1 << 3,

    /*
       * Note on SERIAL_BIDIR_PP
       * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
       * is lost and the first data byte will be lost by a framing error.
       * To ensure the first start bit to be sent, prepend a zero byte (0x00)
       * to actual data bytes.
       */
    SERIAL_BIDIR_OD      = 0 << 4,
    SERIAL_BIDIR_PP      = 1 << 4
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
  virtual bool set_mode(uint8_t mode) = 0;
  virtual void put_byte(uint8_t ch) = 0;
  virtual bool flush() = 0;
  virtual void begin_write() = 0;
  virtual void end_write() = 0;
  virtual void register_rx_callback(void (*rx_callback_ptr)(uint8_t data)) = 0;

protected:
  GPIO tx_pin_;
  GPIO rx_pin_;

  uint8_t mode;
  uint8_t options;
  uint32_t baudrate;
  uint8_t rx_buffer[RX_BUFFER_SIZE];
  uint8_t tx_buffer[TX_BUFFER_SIZE];

  uint8_t tx_buffer_head;
  uint8_t tx_buffer_tail;
  uint8_t rx_buffer_head;
  uint8_t rx_buffer_tail;

  void (*rx_callback)(uint8_t data);

};

#endif // SERIAL CLASS_H
