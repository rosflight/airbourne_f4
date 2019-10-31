/*
 * Copyright (c) 2019, James Jackson
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE  for (int i = 0; i < sizeof(received); ++i)
            {
              vcp.put_byte(received.buf[i]);
            }
            got_message = false;
            received_head = 0;
            memset(received.buf, 0, sizeof(received));OSSIBILITY OF SUCH DAMAGE.
 */


#include "system.h"
#include "vcp.h"
#include "led.h"

LED led1;

// Message definition (This needs to match desktop/pingpong_driver.cc)
char str[] = { "Sup dudes, I'm transmitting data!!" };
struct Message
{
    uint8_t magic_aa;
    uint64_t t0;
    uint64_t t1;
    char string[sizeof(str)];
} __attribute__((packed));

union Buffer 
{
    uint8_t buf[sizeof(Message)];
    Message msg;
};
Buffer received;

VCP* uartPtr = NULL;


// Simple serial protocol parsing
bool got_message = false;
uint8_t received_head = 0;
bool looking_for_start = true;
void callback(uint8_t byte)
{
    // Look for the start byte
    if (looking_for_start && byte != 0xAA)
        return;
    else if (byte == 0xAA)
        looking_for_start = false;

    // Got too many bytes
    if (1 + received_head > sizeof(Message))
    {
        received_head = 0;
        return;
    }

    received.buf[received_head++] = byte;
    
    // See if we have the whole message
    if (received_head == sizeof(Message))
    {
        led1.toggle();
        got_message = true;
        received.msg.t1 = micros();
        looking_for_start = true;
    }
}

int main()
{
    systemInit();

    led1.init(LED1_GPIO, LED1_PIN);
    led1.off();

    VCP vcp;
    vcp.init();
    uartPtr = &vcp;
    vcp.register_rx_callback(&callback);

    while(1)
    {
        uint8_t bytes = vcp.rx_bytes_waiting();
        for (int i = 0; i < bytes; ++i)
        {
            callback(vcp.read_byte());
        }

        if (got_message)
        {
            for (int i = 0; i < sizeof(received); ++i)
            {
                vcp.put_byte(received.buf[i]);
            }
            got_message = false;
            received_head = 0;
            memset(received.buf, 0, sizeof(received));
        }
    }
}
