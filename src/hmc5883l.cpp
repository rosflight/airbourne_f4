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
	else if( byte != 0x48) {
		return false;
	}

	// Configure HMC5833L
    i2c->write(HMC58X3_ADDR, HMC58X3_CRA,  HMC58X3_CRA_DO_75  | //these may need some tweaking
												HMC58X3_CRA_NO_AVG | 
												HMC58X3_CRA_MEAS_MODE_NORMAL ); // 75 Hz Measurement, no bias, no averaging
	delay(20);
	//is there a reason not to use the default 1090 here?
    i2c->write(HMC58X3_ADDR, HMC58X3_CRB, HMC58X3_CRB_GN_390); // 390 LSB/Gauss
	delay(20);
    i2c->write(HMC58X3_ADDR, HMC58X3_MODE, HMC58X3_MODE_CONTINUOUS); // Continuous Measurement Mode
	delay(20);

	uint32_t timeout = HMC58X3_TIMEOUT;
	byte = 0;
	do { //wait until data is ready
		delay(1);
		i2c->read(HMC58X3_ADDR, HMC58X3_STATUS, &byte);
    } while((byte == 0x00 || byte == 0x10) && --timeout > 0); //documentation indicates bit 4 (0x1) is a dont-care in SR
	if (timeout == 0) {
		return false;
	}
	int16_t mag[3];
	read(mag);

	timeout = HMC58X3_TIMEOUT;
	byte = 0;
	do { //wait until data is ready
		delay(1);
		i2c->read(HMC58X3_ADDR, HMC58X3_STATUS, &byte);
    } while((byte == 0x00 || byte == 0x10) && --timeout > 0); //documentation indicates bit 4 (0x1) is a dont-care in SR
	if (timeout == 0) {
		return false;
	}
	//this time our read should be using the proper gain
	read(mag);

	return true;
}

bool HMC5883L::read(int16_t (&mag_data)[3]) {
    uint8_t raw[6];
	for (int i = 0; i < 6; i++) {
		raw[i] = 0;
	}
    i2c->read(HMC58X3_ADDR, HMC58X3_DATA, 6, raw);

	mag_data[0] = (int16_t)((raw[0] << 8) | raw[1]);
	mag_data[1] = (int16_t)((raw[2] << 8) | raw[3]);
	mag_data[2] = (int16_t)((raw[4] << 8) | raw[5]);

	//if the mag's ADC over or underflows, then the data register is given the value of -4096
	//the data register can also be assigned -4096 if there's a math overflow during bias calculation
	return mag_data[0] != -4096 && mag_data[1] != -4096 && mag_data[2] != -4096;
}
