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

#ifndef UART_H
#define UART_H

// from serial.h
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64
#include <functional>

#include "system.h"

#include "serial.h"
#include "gpio.h"

class UART : public Serial
{
public:
  UART();
  void init(const uart_hardware_struct_t *conf, uint32_t baudrate_);

  void write(const uint8_t*ch, uint8_t len) override;
  uint32_t rx_bytes_waiting() override;
  uint32_t tx_bytes_free() override;
  uint8_t read_byte() override;
  bool set_baud_rate(uint32_t baud) override;
  bool tx_buffer_empty() override;
  void put_byte(uint8_t ch) override;
  bool flush() override;
  void register_rx_callback(std::function<void(uint8_t)> cb) override;
  void unregister_rx_callback() override;

  void DMA_Tx_IRQ_callback();
  void DMA_Rx_IRQ_callback();
  void USART_IRQ_callback();

private:
  void init_UART(uint32_t baudrate_);
  void init_DMA();
  void init_NVIC();
  void startDMA();

  const uart_hardware_struct_t* c_;

  uint32_t baudrate_;
  uint8_t rx_buffer_[RX_BUFFER_SIZE];
  uint8_t tx_buffer_[TX_BUFFER_SIZE];
  uint16_t rx_buffer_head_;
  uint16_t rx_buffer_tail_;
  uint16_t tx_buffer_head_;
  uint16_t tx_buffer_tail_;
  GPIO rx_pin_;
  GPIO tx_pin_;
  //std::function<void(uint8_t)> receive_CB_;
};

#endif // UART_H
