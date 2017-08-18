#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "vector3.h"

class Magnetometer
{
public:
    Magnetometer(){}
    bool init(I2C *I2CDev){}
    void read(vector3 *mag){}
    bool new_data(){}
    void update(){}
};

#endif // MAGNETOMETER_H
