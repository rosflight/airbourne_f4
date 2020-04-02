/*
 * Copyright (c) 2019, James Jackson and Trey Henrichsen
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <csignal>
#include <cstring>

#include "async_comm/serial.h"

using namespace std::chrono;

// Exit on Ctrl+C 
bool stop = false;
void inthand(int signum)
{
    stop = true;
}


// Here I'm defining a simple serial protocol so we can sync up  between the
// F4 and the desktop
char str[] = { "Sup dudes, I'm transmitting data!!" };
struct Message
{
    uint8_t magic_aa;           // Start byte is 0xAA (0b10101010)
    uint64_t t0;                // Timestamp of when it was sent  (us since driver start)
    uint64_t t1;                // Timestamp of F4 (us since F4 start)
    char string[sizeof(str)];   // Payload
} __attribute__((packed));      // Don't align members at 32-bit boundary

union Buffer                    // A trick for quick unpacking of the struct
{
    uint8_t buf[sizeof(Message)];
    Message msg;
};

Buffer received;
Buffer sent;

uint8_t received_head = 0;      // Keeps track of where we are in reading the message back
milliseconds timeout(10);      // how long to wait before declaring a failure
std::chrono::time_point<std::chrono::high_resolution_clock> t0;  // when the test started
double alpha = 0.99;            // Low-pass filter on dt_us so it's actually readable

uint64_t dt_us = 0;             // how long it takes for a round trip
bool got_message = false;       // Flag of whether we actually received what we were expecting
void callback(const uint8_t* bytes, size_t size)
{
    // Got too many bytes
    if (size + received_head > sizeof(Message))
        return;

    // Read in the bytes
    for (int i = 0; i < size; ++i)
    {
        received.buf[received_head++] = bytes[i];
    }
    
    // See if we have the whole message
    if (received_head == sizeof(Message) && strcmp(sent.msg.string, received.msg.string)==0)
    {
        auto now = high_resolution_clock::now();
        uint64_t us = duration_cast<microseconds>(now - t0).count();
        if (dt_us == 0)  // First time, don't LPF
            dt_us =  (us - received.msg.t0);
        else // 0-th order LPF
            dt_us = alpha * dt_us + (1.0-alpha) * (us - received.msg.t0);
        got_message = true; // Signal we have the message to the main loop
    }
}

// Reset the received message
void reset_received()
{
    memset(received.buf, sizeof(received), 0);
    received_head = 0;
    got_message = false;
}

int main(int argc, char** argv)
{
    // initialize
    char* port;
    if (argc < 2) 
    {
        std::printf("USAGE: %s PORT\n", argv[0]);
        return 1;
    }
    else
    {
        std::printf("Using port %s\n", argv[1]);
        port = argv[1];
    }

    // open serial port
    async_comm::Serial serial(port, 115200);
    serial.register_receive_callback(&callback);

    if (!serial.init()) 
    {
        std::printf("Failed to initialize serial port\n");
        return 2;
    }

    // look for Ctrl+C and quit
    signal(SIGINT, inthand);

    t0 = high_resolution_clock::now();

    int message_count = 0;
    int print_throttle = 100;  // Don't spam too much
    while (!stop) 
    {
        // Start a new transfer.  Record the time in the message and ship it off
        reset_received();
        sent.msg.magic_aa = 0xAA;
        auto start = high_resolution_clock::now();
        sent.msg.t0 = duration_cast<microseconds>(start - t0).count();
        serial.send_bytes(sent.buf, sizeof(sent));

        // Wait for the message to come back
        auto now = start;
        while (!got_message && (now - start) < timeout) 
        {
            now = high_resolution_clock::now();
            std::this_thread::sleep_for(milliseconds(5));
        }

        if (!got_message)
        {
            printf("Failed to get message!\n");
        }
        else 
        {
            if ((message_count++ % print_throttle) == 0)
                printf("Got message %d.  Took %dus\n", message_count, dt_us);
        }
    }

    return 0;
}