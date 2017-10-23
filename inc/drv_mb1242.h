#ifndef I2C_SONAR_H
#define I2C_SONAR_H
#include <cstdint>
#include "drv_i2c.h"

#define DEFAULT_ADDRESS 112
#define DEFAULT_REGISTER 0xFF
#define READ_COMMAND 81
#define UPDATE_WAIT_MILLIS 100


static uint32_t last_update;
static float value;
static bool new_data;
static I2C* i2c;
static const uint8_t read_command=READ_COMMAND;
static bool ready_to_read;
static uint8_t buffer[2];

void mb1242_init(I2C& i2c);
float mb1242_async_read();
void mb1242_async_update();
void cb_start_read();
void cb_finished_read();


#endif
