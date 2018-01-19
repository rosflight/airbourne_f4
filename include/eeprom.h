#ifndef F4_EEPROM_H
#define F4_EEPROM_H

//The start of sector 11 in the flash memory
//This is used as persistant memory

#define FLASH_SECTOR11_START (void*)(0x080E0000)


#include <stdint.h>

void memory_init();
bool memory_read(void * dest, uint8_t len);
bool memory_write(const void * src, uint8_t len);
bool flash_erase();
bool flash_write( const void* flash_loc, void const* data, uint8_t len);

#endif
