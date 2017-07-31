#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>

#include "barometer.h"
#include "i2c.h"

#define MS5611_ADDR                0x77

#define MS5611_RESET               0x1E // ADC reset command
#define MS5611_ADC_READ            0x00 // ADC read command
#define MS5611_ADC_CONV            0x40 // ADC conversion command
#define MS5611_ADC_D1              0x00 // ADC D1 conversion
#define MS5611_ADC_D2              0x10 // ADC D2 conversion
#define MS5611_ADC_256             0x00 // ADC OSR=256
#define MS5611_ADC_512             0x02 // ADC OSR=512
#define MS5611_ADC_1024            0x04 // ADC OSR=1024
#define MS5611_ADC_2048            0x06 // ADC OSR=2048
#define MS5611_ADC_4096            0x08 // ADC OSR=4096
#define MS5611_PROM_RD             0xA0 // Prom read command
#define MS5611_PROM_LEN            8


class MS5611 : Barometer
{
public:
    MS5611();
    bool init(I2C *I2CDev);
    void update();
    void read(float *pressure, float *temperature);
    void read_CB(void);
    void measurement_request_CB(void);

private:

    enum
    {
        READ_PRESSURE,
        READ_TEMPERATURE
    };
    I2C* i2c;

    uint32_t raw_temp_;
    uint32_t raw_pressure_;
    uint32_t pressure_interval_us = 100000;
    uint32_t temp_interval_us = 10000;
    uint16_t PROM_[MS5611_PROM_LEN];
    uint8_t OSR_ = MS5611_ADC_4096;

    void convert_to_SI();
    void reset(void);
    bool read_prom();
    int8_t crc_check(uint16_t *prom);
    uint32_t read_ADC(void);
    void start_temp_measurement(void);
    void read_temp_measurement(void);
    void start_pressure_measurement(void);
    void read_pressure_measurement(void);

    float temperature_SI_;
    float pressure_SI_;
    uint32_t next_update_time_us;
    uint8_t current_state_;
    uint8_t pressure_buffer_[3];
    uint8_t temp_buffer_[3];

};

extern MS5611* BaroPtr;

#endif // MS5611_H
