#ifndef BAROMETER_H
#define BAROMETER_H

#include "i2c.h"

class Barometer
{
public:
    Barometer() {}
    bool init(I2C* I2CDev) {}
    void update(){}
    void read(float* pressure, float* temperature){}
};

#endif // BAROMETER_H
