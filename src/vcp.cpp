/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vcp.h"

#define USB_TIMEOUT  50

static VCP* vcpPtr = nullptr;

void vcp_rx_callback(uint8_t byte)
{
  if (vcpPtr->cb_)
    vcpPtr->cb_(byte);
}

void VCP::init()
{
  // Initialize the GPIOs for the pins
  rx_pin_.init(GPIOA, GPIO_Pin_11, GPIO::PERIPH_IN_OUT);
  tx_pin_.init(GPIOA, GPIO_Pin_12, GPIO::PERIPH_IN_OUT);

  send_disconnect_signal();

  USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
  vcpPtr = this;
}

void VCP::write(const uint8_t*ch, uint8_t len)
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

bool VCP::tx_buffer_empty()
{
  return CDC_Send_FreeBytes() > 0;
}

void VCP::put_byte(uint8_t ch)
{
  CDC_Send_DATA(&ch, 1);
}

bool VCP::flush()
{
  CDC_flush();
  return true;
}


void VCP::register_rx_callback(std::function<void(uint8_t)> cb)
{
  cb_ = cb;
  Register_CDC_RxCallback(&vcp_rx_callback);
}

void VCP::unregister_rx_callback()
{
  cb_ = NULL;
}


void VCP::send_disconnect_signal()
{
  tx_pin_.set_mode(GPIO::OUTPUT);
  tx_pin_.write(GPIO::LOW);
  delay(200);
  tx_pin_.write(GPIO::HIGH);
  tx_pin_.set_mode(GPIO::PERIPH_IN_OUT);
}
