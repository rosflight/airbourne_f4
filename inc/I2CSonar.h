#ifndef I2C_SONAR_H
#define I2C_SONAR_H
#include <cstdint>
#include "drv_i2c.h"

#define DEFAULT_ADDRESS 112
#define DEFAULT_REGISTER 0xFF
#define READ_COMMAND 81
#define UPDATE_WAIT_MILLIS 25


class I2CSonar
{
private:
	uint32_t last_update;
	float value;
	bool new_data;
	I2C i2c;
	const uint8_t read_command=READ_COMMAND;
    bool ready_to_read=true;
	uint8_t buffer[2];
public:
	I2CSonar(I2C &i2c);
	float async_read();
	void async_update();
	void cb_start_read();
	void cb_finished_read();
};


#endif
