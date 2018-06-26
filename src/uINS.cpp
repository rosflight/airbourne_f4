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

#include "uINS.h"

#include <cmath>
uINS* uINSPtr;

void _read_cb(uint8_t byte)
{
  uINSPtr->read_cb(byte);
}

uINS::uINS(){}

void uINS::init(UART* _dev)
{
  uINSPtr = this;
  uart_ = _dev;
  
  // Speed up the UART becuase the uINS sends a lot of data
  uart_->set_mode(3000000, UART::MODE_8N1);
  uart_->register_rx_callback(_read_cb);
  
  // Initialize the uINS parser
  comm_.buffer = message_buffer_;
  comm_.bufferSize = sizeof(message_buffer_);
  is_comm_init(&comm_);
  successfully_configured_ = get_flash_config();
  if (!successfully_configured_)
    return;
  
  uint32_t messageSize;

  // Make sure the navigation rate is right, if it's not, then we need to change and reset it.
  if (flash_.startupNavDtMs != nav_dt_ms_)
  {
    messageSize = is_comm_set_data(&comm_, DID_FLASH_CONFIG, offsetof(nvm_flash_cfg_t, startupNavDtMs), sizeof(uint32_t), (void*)&nav_dt_ms_);
    uart_->write(message_buffer_, messageSize);
    reset();
    
    // Re-request flash config to confirm change
    get_flash_config();
    successfully_configured_ = (flash_.startupNavDtMs == nav_dt_ms_);
  }
  
  if (successfully_configured_)
  {
    // stop all broadcasts
    uint32_t messageSize = is_comm_stop_broadcasts(&comm_);
    uart_->write(message_buffer_, messageSize);
    
    // Configure the dynamic model
    set_flash_config<int>(8, offsetof(nvm_flash_cfg_t, insDynModel));
    
    // Configure data streams
    uint32_t rmcBits = RMC_BITS_GPS_NAV | RMC_BITS_STROBE_IN_TIME | RMC_BITS_DUAL_IMU | 
                       RMC_BITS_BAROMETER | RMC_BITS_MAGNETOMETER1 | RMC_BITS_GPS_NAV |
                       RMC_BITS_INS1 | RMC_BITS_INS2;
    messageSize = is_comm_get_data_rmc(&comm_, rmcBits);
    uart_->write(message_buffer_, messageSize);
  }
}

bool uINS::present()
{
  return successfully_configured_;
}

void uINS::update()
{
  // Try re-connecting if it didn't work before
  if (!successfully_configured_)
    init(uart_);
}

void uINS::read_INS(float ned[], float uvw[], float q[], uint32_t *time_ms)
{
  ned[0] = ins1_.ned[0];
  ned[1] = ins1_.ned[1];
  ned[2] = ins1_.ned[2];
  
  uvw[0] = ins2_.uvw[0];
  uvw[1] = ins2_.uvw[1];
  uvw[2] = ins2_.uvw[2];
  
  q[0] = ins2_.qn2b[0];
  q[1] = ins2_.qn2b[1];
  q[2] = ins2_.qn2b[2];
  q[3] = ins2_.qn2b[3];
  
  *time_ms = system_time_from_tow(ins2_.timeOfWeek);
}

void uINS::read_IMU(float pqr[], float acc[], uint32_t *time_ms)
{
  pqr[0] = imu_.I[0].pqr[0];
  pqr[1] = imu_.I[0].pqr[1];
  pqr[2] = imu_.I[0].pqr[2];
  
  acc[0] = imu_.I[0].acc[0];
  acc[1] = imu_.I[0].acc[1];
  acc[2] = imu_.I[0].acc[2];
  *time_ms = system_time_from_start_time(imu_.time);
}

void uINS::read_other_sensors(float mag[], float *baro, uint32_t *time_ms)
{
  mag[0] = mag_.mag[0];
  mag[1] = mag_.mag[1];
  mag[2] = mag_.mag[2];
  
  *baro = baro_.bar;
  *time_ms = system_time_from_start_time(mag_.time);
}


#define CALLBACK(DID, T, mem_) { \
  case DID: \
    mem_ = *((T*)message_buffer_); \
    break; \
}

void uINS::read_cb(uint8_t byte)
{
  uint32_t message_type = is_comm_parse(&comm_, byte);
  if (message_type != DID_NULL)
  {
    switch (message_type)
    {
    // We got a valid message!  Process it
    CALLBACK(DID_FLASH_CONFIG, nvm_flash_cfg_t, flash_);
    CALLBACK(DID_INS_1, ins_1_t, ins1_);
    CALLBACK(DID_INS_2, ins_2_t, ins2_);
    CALLBACK(DID_DUAL_IMU, dual_imu_t, imu_);
    CALLBACK(DID_GPS_NAV, gps_nav_t, gps_);
    CALLBACK(DID_MAGNETOMETER_1, magnetometer_t, mag_);
    CALLBACK(DID_BAROMETER, barometer_t, baro_);
    CALLBACK(DID_PREINTEGRATED_IMU, preintegrated_imu_t, preint_imu_);
    CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_);   
    default:
      // Unhandled message
      break;
    }
  }
}

void uINS::set_current_pos_as_refLLa()
{
  set_vector_flash_config<double>(ins2_.lla, 3, offsetof(nvm_flash_cfg_t, refLla));
}

bool uINS::perform_multi_mag_cal()
{
  uint32_t multi_axis_command = 0;
  int messageSize = is_comm_set_data(&comm_, DID_MAG_CAL, offsetof(mag_cal_t, enMagRecal), sizeof(uint32_t), &multi_axis_command);
  uart_->write(message_buffer_, messageSize);  
}
  
bool uINS::get_flash_config() 
{
  got_flash_config = false;
  int messageSize = is_comm_get_data(&comm_, DID_FLASH_CONFIG, 0, 0, 0);
  uart_->write(message_buffer_, messageSize);
  
  // wait for flash config message to confirm connection
  uint32_t start_ms = millis();
  while (!flash_.startupNavDtMs && (millis() - start_ms) <= 1000) {}
  
  return ((millis() - start_ms) < 1000);
}


void uINS::reset() 
{
  // send reset command
  uint32_t reset_command = 99;
  int messageSize = is_comm_set_data(&comm_, DID_CONFIG, offsetof(config_t, system), sizeof(uint32_t), &reset_command);
  uart_->write(message_buffer_, messageSize);
} 

template<typename T> void uINS::set_vector_flash_config(T * value, uint32_t len, uint32_t offset) 
{
  int messageSize = is_comm_set_data(&comm_, DID_FLASH_CONFIG, offset, sizeof(T)*len, value);
  uart_->write(message_buffer_, messageSize);
}

template<typename T> void uINS::set_flash_config(T value, uint32_t offset) 
{
  int messageSize = is_comm_set_data(&comm_, DID_FLASH_CONFIG, offset, sizeof(T), &value);
  uart_->write(message_buffer_, messageSize);
}

// Time synchronization Functions
uint32_t uINS::system_time_from_week_and_tow(const uint32_t week, const uint32_t timeOfWeekMs) 
{
  (void)week;
  // If we have a fix, use GPS time
  return system_time_from_start_time(timeOfWeekMs - (uint32_t)(gps_.towOffset*1e3L));
}

uint32_t uINS::system_time_from_start_time(const uint32_t time_ms) 
{
  if (!got_first_message_)
  {
    got_first_message_ = true;
    INS_local_offset_ms_ = millis() - time_ms;
  }
  else
  {
    int32_t y_offset = millis() - time_ms;
    // If we are way off, then just directly upate
    if (abs(y_offset - INS_local_offset_ms_) > 100)
      INS_local_offset_ms_ = y_offset;
    else
      INS_local_offset_ms_ = round(0.005 * y_offset + 0.995 * INS_local_offset_ms_);
  }
  return INS_local_offset_ms_ + time_ms;
}

uint32_t uINS::system_time_from_tow(const double tow) 
{
  return uINS::system_time_from_start_time(tow*1e3L - gps_.towOffset*1e3L);
}



