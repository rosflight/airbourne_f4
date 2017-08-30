#include "hmc5883l.h"

HMC5883L::HMC5883L(I2C* i2c_drv) {
	i2c = i2c_drv;
}

bool HMC5883L::init() {
	// Detect Magnetometer
	uint8_t byte = 0;
	if(!i2c->read(HMC58X3_ADDR, HMC58X3_ID1, &byte)) {
		return false;
	}
	else if( byte != 0x34) {
		return false;
	}

	// Configure HMC5833L
    i2c->write_byte(HMC58X3_ADDR, HMC58X3_CRA,  HMC58X3_CRA_DO_75  | 
    											HMC58X3_CRA_NO_AVG | 
    											HMC58X3_CRA_MEAS_MODE_NORMAL ); // 75 Hz Measurement, no bias, no averaging
    delay(20);

    i2c->write_byte(HMC58X3_ADDR, HMC58X3_CRB, HMC58X3_CRB_GN_390); // 390 LSB/Gauss
    delay(20);

    i2c->write_byte(HMC58X3_ADDR, HMC58X3_MODE, HMC58X3_MODE_CONTINUOUS); // Continuous Measurement Mode
    delay(20);

    return true;
}