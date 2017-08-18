#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include "string.h"

#include "board.h"

#define EEPROM_BUFFER_SIZE 512

class EEPROM
{
public:
    EEPROM();

    bool read(uint8_t* data, uint32_t len);
    bool write(const uint8_t *data, uint32_t len);

private:

    bool is_valid();

    typedef struct
    {
        uint8_t magic_ef;
        uint8_t crc;
    } eeprom_footer_t;

    typedef struct
    {
        uint32_t version;
        uint8_t magic_be;
        uint16_t size;
    } eeprom_header_t;

    eeprom_footer_t footer_;
    eeprom_header_t header_;

};

#endif // EEPROM_H
