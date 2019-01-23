#include <cstring> //For memcpy
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "backup_sram.h"

void backup_sram_init()
{
    static_assert(sizeof(rosflight_firmware::BackupData)%4==0,"Error with debug_info_t: Size is not multiple of 32 bits");
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
#pragma GCC diagnostic pop
    PWR_BackupRegulatorCmd(ENABLE);
}
void backup_sram_write(const rosflight_firmware::BackupData& data)
{
    PWR_BackupAccessCmd(ENABLE);
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    std::memcpy(reinterpret_cast<void*>(BKPSRAM_BASE),&data,sizeof(rosflight_firmware::BackupData));
#pragma GCC diagnostic pop
    PWR_BackupAccessCmd(DISABLE);
}
rosflight_firmware::BackupData do_first_read() {
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    rosflight_firmware::BackupData data = *(reinterpret_cast<rosflight_firmware::BackupData*>(BKPSRAM_BASE));
#pragma GCC diagnostic pop
#pragma GCC diagnostic push //Ignore blank fields in struct
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    rosflight_firmware::BackupData blank_data = {0};
#pragma GCC diagnostic pop
    if(!check_backup_checksum(data))
        data=blank_data;
    backup_sram_write(blank_data);
    return data;
}
rosflight_firmware::BackupData backup_sram_read()
{
    static rosflight_firmware::BackupData data = do_first_read();
    return data;
}
bool check_backup_checksum(const rosflight_firmware::BackupData& data)
{
    return (data.checksum==generate_backup_checksum(data));
}
uint32_t generate_backup_checksum(const rosflight_firmware::BackupData& data)
{
    const uint32_t* pointer = reinterpret_cast<const uint32_t*>(&data);
    uint32_t checksum = 0;
    for(uint8_t i=0;i<(sizeof (rosflight_firmware::BackupData)/4)-1;i++)
        checksum^=*(pointer+i);
    return checksum;
}
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //Remove this, too
void test_backup_sram(){
  backup_sram_init();
  rosflight_firmware::BackupData read_data = backup_sram_read();
  rosflight_firmware::BackupData write_data = {};
  write_data.error_code = 0xabcde;
  if(check_backup_checksum(read_data))
  {
  }
  write_data.checksum=generate_backup_checksum(write_data);
  backup_sram_write(write_data);
}
