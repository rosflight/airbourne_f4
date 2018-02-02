#ifndef I2C_SONAR_H
#define I2C_SONAR_H
#include <cstdint>
#include "i2c.h"

#define DEFAULT_ADDRESS 112
#define DEFAULT_REGISTER 0xFF
#define PING_COMMAND 81
#define UPDATE_WAIT_MILLIS 100 //minimum time between calls of async_update that actually do something


class I2CSonar
{
private:
    uint32_t last_update;//The last time that async_update was called
    float value;//the latest reading from the sensor
    bool new_data;//Whether or not new data is ready to be returned
    I2C i2c;//The i2c object used for communication
    bool ready_to_ping;//Whether the sensor is ready to make another measurement
    uint8_t buffer[2];//for recieving data from the sensor
public:
	I2CSonar (I2C& i2c);
    float async_read();//Returns the most recent reading, also calling async_update
    void async_update();//Tries to either start a measurement, or read it from the sensor
    void cb_start_read();//callback after the measure command has been sent to the sensor
    void cb_finished_read();//callback after reading from the sensor has finished
};


#endif
