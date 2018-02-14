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


class VCP : public Serial
{
public:
  void init();
  virtual void write( uint8_t *ch, uint8_t len) override;
  uint32_t rx_bytes_waiting() override;
  uint32_t tx_bytes_free() override;
  uint8_t read_byte() override;
  bool set_baud_rate(uint32_t baud) override;
  bool tx_buffer_empty() override;
  bool set_mode(uint8_t mode_);
  void put_byte(uint8_t ch) override;
  bool flush() override;
  void begin_write();
  void end_write();
  void register_rx_callback(std::function<void(uint8_t)>) override;
  void unregister_rx_callback() override;
  bool in_bulk_mode();

private:

  void send_disconnect_signal();

  uint8_t bulk_mode_buffer[64];
  uint8_t bulk_mode_buffer_index;
  bool bulk_mode;

  GPIO rx_pin_;
  GPIO tx_pin_;
};

#endif 
