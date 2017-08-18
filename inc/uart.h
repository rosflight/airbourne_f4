/*
 * uart.h - A UART Class for use on the naze32
 * Copyright (c) 2016 James Jackson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UART_H
#define UART_H

#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256

#include "stm32f10x_conf.h"

#include "board.h"
#include "serialport.h"
#include "gpio.h"

class UART : public Serial
{
public:
  UART();

  enum
  {
    MODE_IT,
    MODE_DMA_TX,
    MODE_DMA_RX,
    MODE_DMA_TX_RX,
  };

  void init(int index, uint32_t baudrate, uint8_t mode);
  void put_byte(uint8_t* ch, uint32_t len);
  uint8_t read_byte();
  bool bytes_waiting();
  uint32_t num_bytes_waiting();
  void IRQ_callback();
  void startDMA();
  void register_receive_CB(void (*CB)(uint8_t c));
  void unregister_receive_CB();

private:
  void USART_Configuration();
  void DMA_Configuration();
  void NVIC_Configuration();

  uint32_t baudrate_;
  uint8_t rx_buffer_[RX_BUFFER_SIZE];
  uint8_t tx_buffer_[RX_BUFFER_SIZE];
  uint16_t rx_buffer_head_;
  uint16_t rx_buffer_tail_;
  uint16_t tx_buffer_head_;
  uint16_t tx_buffer_tail_;
  uint16_t rx_DMA_read_index_;
  uint32_t rx_DMA_pos_;
  USART_TypeDef* UART_Base_addr_;
  GPIO rx_gpio_;
  GPIO tx_gpio_;
  DMA_Channel_TypeDef* Tx_DMA_Channel_;
  DMA_Channel_TypeDef* Rx_DMA_Channel_;
  bool DMA_Tx_;
  bool DMA_Rx_;
  IRQn_Type TxDMAIRQ_;
  IRQn_Type RxDMAIRQ_;
  IRQn_Type UARTIRQ_;
  uint32_t DMA_TX_IT_BIT_;
  uint32_t DMA_RX_IT_BIT_;
  void (*receive_CB_)(uint8_t c);
};


// Pointers to the initialized classes, so we can send the IRQ routines to them
extern UART* UART1Ptr;
extern UART* UART2Ptr;
extern UART* UART3Ptr;

#endif // UART_H
