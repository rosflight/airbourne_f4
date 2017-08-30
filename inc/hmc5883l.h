/**
 * HMC5883L sensor driver for OpenPilot REVO
 *
 * Adapted from https://github.com/jihlein/AQ32Plus/blob/master/src/sensors/hmc5883.h
 * 			and https://github.com/superjax/airbourne/blob/master/include/hmc5883l.h
 * @author len0rd
 * @since 2017-08-30
 */
#pragma once

#include "system.h"
#include "drv_i2c.h"

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

class HMC5883L {
public:
	HMC5883L(I2C* i2c_drv);

	bool init();
	void read_mag(int16_t (&mag_data)[3]);

private:
	I2C* i2c;
};