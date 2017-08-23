#ifndef DRV_I2C_H
#define DRV_I2C_H

#include "system.h"

class I2C {
public:
	I2C(I2C_TypeDef *I2C);

	void unstick();
	void write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
	
private:
	I2C_TypeDef* dev;
};

#endif //DRV_I2C_H