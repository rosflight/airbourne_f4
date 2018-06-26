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

#ifndef UINS_H
#define UINS_H

#include "uart.h"
#include "inertialsense_serial_protocol/data_sets.h"
#include "inertialsense_serial_protocol/ISComm.h"

#ifndef UINS_BUFFER_SIZE
#define UINS_BUFFER_SIZE 256
#endif

class uINS
{
public:
  uINS();
  void init(UART* _dev);
  bool perform_multi_mag_cal();
  void update();
  bool present();
  void read_cb(uint8_t byte);
  void read_INS(float ned[3], float uvw[3], float q[4], uint32_t *time_ms);
  void read_IMU(float pqr[3], float acc[3], uint32_t* time_ms);
  void read_other_sensors(float mag[3], float* baro, uint32_t* time_ms);
  void reset(); 
  bool new_imu_data();
  bool got_fix();
  void set_current_pos_as_refLLa();
  uint32_t time_skew_count();
  
  
private:
  UART* uart_;
  is_comm_instance_t comm_;
  uint8_t message_buffer_[UINS_BUFFER_SIZE];
  bool got_flash_config = false;
  bool successfully_configured_ = false;
  
  uint32_t nav_dt_ms_ = 4;
  uint32_t time_skew_ = 0;
  
  bool new_imu_ = 0;
  
  bool get_flash_config();

  
  template<typename T> void set_vector_flash_config(T* value, uint32_t size, uint32_t offset);
  template<typename T> void set_flash_config(T value, uint32_t offset);
  
  // Local copies of data
  nvm_flash_cfg_t flash_ = {};
  ins_1_t ins1_ = {};
  ins_2_t ins2_ = {};
  dual_imu_t imu_ = {};
  gps_nav_t gps_ = {};
  magnetometer_t mag_ = {};
  barometer_t baro_ = {};
  preintegrated_imu_t preint_imu_ = {};
  strobe_in_time_t strobe_ = {};
  
  // Time synchronization Functions
  uint32_t system_time_from_week_and_tow(const uint32_t week, const uint32_t timeOfWeekMs);
  uint32_t system_time_from_start_time(const uint32_t time_ms);
  uint32_t system_time_from_tow(const double tow);
  int32_t INS_local_offset_ms_ = 0.0; // Current estimate of the uINS start time in ROS time seconds (possibly negative)
  bool got_first_message_ = false; // Flag to capture first uINS start time guess
};

#endif
