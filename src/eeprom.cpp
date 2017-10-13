#include "string.h"
#include "stm32f4xx_flash.h"

#include "eeprom.h"

void memory_init()
{
  return;//nothing needed
}
//Reads the data stored in flash sector 11
bool memory_read(void * dest, uint8_t len)
{
  memcpy(dest, FLASH_SECTOR11_START, len);
}
//writes to the 11th sector, which is used for persistant storage
bool memory_write(const void* src, uint8_t len)
{
  flash_write(FLASH_SECTOR11_START, src,len);
}
//Reads from the specified location. Note that there is no error checking
bool flash_read(void* flash_loc,void* dest, uint8_t len)
{
  memcpy(dest, flash_loc, len);
  return true;
}
//Erases the 11th sector, which is used for persistent storage
bool flash_erase()
{
  FLASH_Unlock();
  FLASH_EraseSector(FLASH_Sector_11,VoltageRange_3);
}
//writes data to the given location. Note that there is no error checking
bool flash_write(void const* flash_loc, void const* data, uint8_t len)
{
  flash_erase();
  FLASH->CR|=FLASH_CR_PG;
  uint32_t* dataPtr = (uint32_t*)data;
  uint32_t flash_address=(uint32_t)flash_loc;
  for (uint32_t i=0;i<len;i+=4)
    FLASH_ProgramWord(flash_address+i,*(dataPtr++));
  return true;
}
