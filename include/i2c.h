/*
   drv_i2c.h :  I^2C support for STM32F103CB

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_i2c.h

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <cstddef>

#include "board.h"
#include "gpio.h"

// Temporary
#define I2C2_DMA_CHANNEL_TX DMA1_Channel4
#define I2C2_DMA_CHANNEL_RX DMA1_Channel5

enum {
    I2C_JOB_DEFAULT,
    I2C_JOB_QUEUED,
    I2C_JOB_BUSY,
    I2C_JOB_COMPLETE,
    I2C_JOB_ERROR
};

typedef struct i2cJob{
    uint8_t type;
    uint8_t addr;
    uint8_t reg;
    uint8_t* data;
    uint8_t length;
    struct i2cJob* next_job;
    volatile uint8_t* status;
    void (*CB)(void);
} i2cJob_t;

class I2C
{

public:
    enum{
        READ = 0x01,
        WRITE = 0x02,
        POLLING = 0x04,
        IT = 0x08,
        DMA = 0x10
    };
    void init(uint8_t index);
    uint16_t get_error_count(void);

    // Interrupt Handlers (called by the IRQ functions, so they have to be public)
    void error_handler(void);
    void event_handler(void);


    // Blocking function calls
    bool write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
    bool write_byte(uint8_t addr, uint8_t reg, uint8_t data);
    bool read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);


    // ===================================================================
    // Asynchronous I2C handler
    // To use this, queue up a job, and create a callback you want called when the job is finished
    // You can track progress of the job using the status pointer.  Otherwise, functions the same
    // as the blocking versions.
    //
    // This uses a circular buffer to stage jobs for the I2C peripheral.  The buffer is, by default, 64 jobs
    // long (I2C_BUFFER_SIZE), with a maximum size of 256. I hope you never queue up that many jobs, because
    // that will take a long time to process However, if you were to reach the limit, it would then start
    // ignoring new jobs until there was space on the buffer.
    //
    // For an example of how to use, check out mpu6050_request_read_temp - the non-blocking way to read
    // the accelerometer
    void queue_job_IT(uint8_t type, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, volatile uint8_t *status, void (*CB)(void));
    bool perform_I2C_job(uint8_t type, uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len, volatile uint8_t* status, void (*CB)(void));


private:

    void DMA_Config();

    // Connections to the hardware
    I2C_TypeDef *I2Cx_ = NULL;
    uint8_t I2Cx_index_;
    uint8_t ev_irq_;
    uint8_t er_irq_;

    // GPIO Pins
    GPIO SDA_;
    GPIO SCL_;

    // DMA Configuration
    DMA_InitTypeDef DMA_InitStruct_;

    bool start_I2C_job();

    // I2C Circular Buffer Variables
    i2cJob_t job_buffer_[I2C_BUFFER_SIZE];
    volatile uint8_t buffer_head_;
    volatile uint8_t buffer_tail_;
    volatile uint8_t buffer_count_;
    void init_buffer(void);
    void job_handler(void);
    bool hardware_failure(void);
    void unstick(void);

    volatile uint16_t error_count_ = 0;

    // Status Variables for the interrupt routine calls
    volatile bool error_ = false;
    volatile bool busy_;
    bool subaddress_sent_;
    bool final_stop_;
    uint8_t index_;
    volatile uint8_t addr_;
    volatile uint8_t reg_;
    volatile uint8_t len_;
    volatile uint8_t operation_;
    volatile uint8_t* data_buffer_;
    volatile uint8_t *job_status_;

    uint64_t async_watchdog_start_time_us_;

    void (*complete_CB_)(void);

    // Non-Blocking I2C Functions
    bool read_IT(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf, volatile uint8_t* status_, void (*CB)(void));
    bool write_IT(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *bu_, volatile uint8_t* status, void (*CB)(void));
};

extern I2C* I2CDev_1Ptr;
extern I2C* I2CDev_2Ptr;
