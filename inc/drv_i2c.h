#ifndef DRV_I2C_H
#define DRV_I2C_H

#include "system.h"

class I2C {
public:
	I2C(I2C_TypeDef *I2C);

	void unstick();
	void write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
	void read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
	
private:
	void handle_hardware_failure();
	void init();

	I2C_TypeDef* dev;

	uint16_t error_count_ = 0;

	//Variables for current job:
	volatile uint8_t  addr_;
	volatile uint8_t  reg_;
	volatile uint8_t  len_;
	volatile uint8_t* data_buffer_;
};

#endif //DRV_I2C_H