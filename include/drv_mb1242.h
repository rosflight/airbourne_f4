#ifndef I2C_SONAR_H
#define I2C_SONAR_H
#include <cstdint>
#include "i2c.h"

#define DEFAULT_ADDRESS 112
#define DEFAULT_REGISTER 0xFF
#define READ_COMMAND 81
#define UPDATE_WAIT_MILLIS 100


class MB1242
{
private:
	uint32_t last_update_;
	float value_;
	bool new_data_;
	I2C i2c_;
	const uint8_t read_command_=READ_COMMAND;
	bool ready_to_read_;
	uint8_t buffer_[2];
	bool sensor_present_;

public:
	MB1242 (I2C& i2c_);
	inline bool present() {return sensor_present_;}
	float async_read();
	void async_update();
	void cb_start_read();
	void cb_finished_read();
};


#endif
