#include "drv_led.h"

LED::LED(GPIO_TypeDef *gpio_port, uint16_t pin) {
	GPIO_InitTypeDef gpio_init_struct;
    GPIO_StructInit(&gpio_init_struct);
    
    gpio_init_struct.GPIO_Pin 	= pin;
    gpio_init_struct.GPIO_Mode 	= GPIO_Mode_OUT;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;
    gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;

    port_ 	= gpio_port;
    pin_ 	= pin;

    GPIO_Init(gpio_port, &gpio_init_struct);

    off();
}

void LED::toggle() {
	if (GPIO_ReadOutputDataBit(port_, pin_))
		GPIO_ResetBits(port_, pin_);
	else
		GPIO_SetBits(port_, pin_);
}

void LED::on() {
    GPIO_ResetBits(port_, pin_);
}

void LED::off() {
	GPIO_SetBits(port_, pin_);
}


