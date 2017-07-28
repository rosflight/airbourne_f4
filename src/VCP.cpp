#include "VCP.h"

#define USB_TIMEOUT  50

VCP::VCP(serial_configuration_t config)
{
  // Initialize the GPIOs for the pins
  rx_pin_.init(config.GPIO, config.rx_pin, GPIO::PERIPH_IN_OUT);
  tx_pin_.init(config.GPIO, config.tx_pin, GPIO::PERIPH_IN_OUT);

  send_disconnect_signal();

  USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
}

void VCP::write(uint8_t*ch, uint8_t len)
{
  uint32_t start = millis();
  while (len > 0)
  {
    uint32_t num_bytes_sent = CDC_Send_DATA(ch, len);
    len -= num_bytes_sent;
    ch += num_bytes_sent;

    if (millis() > start + USB_TIMEOUT)
      break;
  }
}


uint32_t VCP::rx_bytes_waiting()
{
  return CDC_Receive_BytesAvailable();
}


uint32_t VCP::tx_bytes_free()
{
  return CDC_Send_FreeBytes();
}


uint8_t VCP::read_byte()
{
  uint8_t data;

  if (CDC_Receive_DATA(&data, 1))
    return data;
  else
    return 0;
}


bool VCP::set_baud_rate(uint32_t baud){}

bool VCP::tx_buffer_empty()
{
  return true;
}

bool VCP::set_mode(uint8_t mode)
{
  (void)mode;
}

void VCP::put_byte(uint8_t ch)
{
  CDC_Send_DATA(&ch, 1);
}

bool VCP::flush()
{
  CDC_flush();
}
void VCP::begin_write(){}
void VCP::end_write(){}


void VCP::register_rx_callback(void (*rx_callback_ptr)(uint8_t data))
{
  rx_callback = rx_callback_ptr;
  Register_CDC_RxCallback(rx_callback_ptr);
}


bool VCP::in_bulk_mode()
{
  return false;
}


void VCP::send_disconnect_signal()
{
  tx_pin_.set_mode(GPIO::OUTPUT);
  tx_pin_.write(GPIO::LOW);
  delay(200);
  tx_pin_.write(GPIO::HIGH);
  tx_pin_.set_mode(GPIO::PERIPH_IN_OUT);
}
