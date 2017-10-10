#ifndef VCP_CLASS_H
#define VCP_CLASS_H

#include "revo_f4.h"

#include "serial.h"
#include "gpio.h"

extern "C" {
#include "stm32f4xx_conf.h"
#include "usbd_cdc_core.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "usbd_ioreq.h"
}


class VCP //: Serial
{
public:
  VCP();
  void write(uint8_t*ch, uint8_t len);
  uint32_t rx_bytes_waiting();
  uint32_t tx_bytes_free();
  uint8_t read_byte();
  bool set_baud_rate(uint32_t baud);
  bool tx_buffer_empty();
  bool set_mode(uint8_t mode_);
  void put_byte(uint8_t ch);
  bool flush();
  void begin_write();
  void end_write();
  void register_rx_callback(std::function<void(uint8_t)> cb);
  void unregister_rx_callback();
  bool in_bulk_mode();

  std::function<void(uint8_t)> receive_CB_;

private:

  void send_disconnect_signal();

  GPIO rx_pin_;
  GPIO tx_pin_;

  uint8_t bulk_mode_buffer[64];
  uint8_t bulk_mode_buffer_index;
  bool bulk_mode;
};

#endif 
