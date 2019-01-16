#include "backup_sram.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include <cstring>

void backup_sram_init()
{
    static_assert(sizeof(backup_data_t)%4==0,"Error with debug_info_t: Size is not multiple of 32 bits");
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
#pragma GCC diagnostic pop
    PWR_BackupRegulatorCmd(ENABLE);
}
void backup_sram_write(const backup_data_t& data)
{
    PWR_BackupAccessCmd(ENABLE);
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    std::memcpy(reinterpret_cast<void*>(BKPSRAM_BASE),&data,sizeof(backup_data_t));
#pragma GCC diagnostic pop
    PWR_BackupAccessCmd(DISABLE);
}
backup_data_t backup_sram_read()
{
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    backup_data_t data = *(reinterpret_cast<backup_data_t*>(BKPSRAM_BASE));
#pragma GCC diagnostic pop
    return data;
}
bool check_backup_checksum(const backup_data_t& data)
{
    return (data.checksum==generate_backup_checksum(data));
}
uint32_t generate_backup_checksum(const backup_data_t& data)
{
    const uint32_t* pointer = reinterpret_cast<const uint32_t*>(&data);
    uint32_t checksum = 0;
    for(uint8_t i=0;i<(sizeof (backup_data_t)/4)-1;i++)
        checksum^=*(pointer+i);
    return checksum;
}
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //Remove this, too
void test_backup_sram(){
  backup_sram_init();
  backup_data_t read_data = backup_sram_read();
  backup_data_t write_data = {};
  write_data.error_code = 0xabcde;
  if(check_backup_checksum(read_data))
  {
  }
  write_data.checksum=generate_backup_checksum(write_data);
  backup_sram_write(write_data);
}
