/*
 * Copyright (c) 2018, James Jackson
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

#include "i2c.h"

#define SEN13680_DEFAULT_ADDRESS 0x62
#define SEN13680_READ_REGISTER 0x8F

class SEN13680
{
public:
    SEN13680();
    void init(I2C *_i2c);
    bool present();
    float read(); 
    void update();
    void cb(uint8_t result);
    
private:
    typedef enum
    {
      START_READ,
      READ
    } state_t;
    state_t state_;
    uint32_t next_update_ms_; // The last time that async_update was called
    uint32_t last_callback_ms_; // The last time the sensor responded
    float value_; // the latest reading from the sensor
    bool new_data_; // Whether or not new data is ready to be returned
    I2C *i2c_; // The i2c object used for communication
    bool ready_to_ping_; // Whether the sensor is ready to make another measurement
    uint8_t buffer_[2]; // for recieving data from the sensor
    bool sensor_present_; // Flag of whether we have received data from the sensor
};
