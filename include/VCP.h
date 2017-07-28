#include "system.h"
#include "Serial.h"

extern "C" {
#include "stm32f4xx_conf.h"
#include "usbd_cdc_core.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "usbd_ioreq.h"
}


class VCP : Serial
{
public:
  VCP(serial_configuration_t config);
  void write(uint8_t*ch, uint8_t len);
  uint32_t rx_bytes_waiting();
  uint32_t tx_bytes_free();
  uint8_t read_byte();
  bool set_baud_rate(uint32_t baud);
  bool tx_buffer_empty();
  bool set_mode(uint8_t mode);
  void put_byte(uint8_t ch);
  bool flush();
  void begin_write();
  void end_write();
  void register_rx_callback(void (*rx_callback_ptr)(uint8_t data));
  bool in_bulk_mode();

private:

  void send_disconnect_signal();

  uint8_t bulk_mode_buffer[64];
  uint8_t bulk_mode_buffer_index;
  bool bulk_mode;
};
