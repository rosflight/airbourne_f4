#ifndef HMC5883L_H
#define HMC5883L_H

#include "vector3.h"
#include "i2c.h"
#include "gpio.h"

#define HMC58X3_ADDR 0x1E
#define HMC58X3_CRA 0x00
#define HMC58X3_CRB 0x01
#define HMC58X3_MODE 0x02
#define HMC58X3_DATA 0x03
#define HMC58X3_STATUS 0x0A
#define HMC58X3_ID1 0x0B
#define HMC58X3_ID2 0x0C
#define HMC58X3_ID3 0x0D

#define HMC58X3_CRA_NO_AVG 0x00
#define HMC58X3_CRA_AVG_2_MEAS 0x20
#define HMC58X3_CRA_AVG_4_MEAS 0x40
#define HMC58X3_CRA_AVG_8_MEAS 0x60

#define HMC58X3_CRA_DO_0_75 0x00
#define HMC58X3_CRA_DO_1_5 0x04
#define HMC58X3_CRA_DO_3 0x08
#define HMC58X3_CRA_DO_7_5 0x0C
#define HMC58X3_CRA_DO_15 0x10
#define HMC58X3_CRA_DO_30 0x14
#define HMC58X3_CRA_DO_75 0x18

#define HMC58X3_CRA_MEAS_MODE_NORMAL 0x00
#define HMC58X3_CRA_MEAS_MODE_POS_BIAS 0x01
#define HMC58X3_CRA_MEAS_MODE_NEG_BIAS 0x02

#define HMC58X3_CRB_GN_1370 0x00
#define HMC58X3_CRB_GN_1090 0x20
#define HMC58X3_CRB_GN_820 0x40
#define HMC58X3_CRB_GN_660 0x60
#define HMC58X3_CRB_GN_440 0x80
#define HMC58X3_CRB_GN_390 0xA0
#define HMC58X3_CRB_GN_330 0xC0
#define HMC58X3_CRB_GN_230 0xE0

#define HMC58X3_MODE_HS 0x80
#define HMC58X3_MODE_CONTINUOUS 0x00
#define HMC58X3_MODE_SINGLE 0x01
#define HMC58X3_MODE_IDLE 0x02

#define HMC58X3_SR_LOCK 0x02
#define HMC58X3_SR_RDY 0x01

#define HMC58X3_TIMEOUT 30000

#include "magnetometer.h"

class HMC5883L : public Magnetometer
{
public:
    HMC5883L();

    bool init(I2C *I2CDev);
    void update();
    void read(vector3 *mag);
    bool new_data();

    void data_read_CB();

private:

    void configure_DRDY_interrupt();
    I2C* i2c;

    GPIO DRDY_;

    void blocking_read(vector3 *data);

    vector3 gain_;

    bool new_data_;

    int16_t raw_data_[3];
    uint8_t data_buffer_[6];
};

extern HMC5883L* HMC5883LPtr;

#endif // HMC5883L_H
