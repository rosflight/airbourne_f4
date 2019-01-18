#include "backup_sram.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include <cstring>

void backup_sram::init()
{
    static_assert(sizeof(rosflight_firmware::backup_data_t)%4==0,"Error with debug_info_t: Size is not multiple of 32 bits");
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
#pragma GCC diagnostic pop
    PWR_BackupRegulatorCmd(ENABLE);

}
void backup_sram::write(const rosflight_firmware::backup_data_t& write_data)
{
    PWR_BackupAccessCmd(ENABLE);
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    std::memcpy(reinterpret_cast<void*>(BKPSRAM_BASE),&write_data,sizeof(rosflight_firmware::backup_data_t));
#pragma GCC diagnostic pop
    PWR_BackupAccessCmd(DISABLE);
}
rosflight_firmware::backup_data_t backup_sram::do_first_read()
{
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    static rosflight_firmware::backup_data_t read_data = *(reinterpret_cast<rosflight_firmware::backup_data_t*>(BKPSRAM_BASE));
#pragma GCC diagnostic pop
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    if(!is_valid(read_data))
    {
        read_data={0};
        this->valid_data=false;
    }
    else
        this->valid_data=true;
#pragma GCC diagnostic pop
    return read_data;
}
bool backup_sram::has_data() {
   return this->valid_data;
}
bool backup_sram::is_valid(const rosflight_firmware::backup_data_t& data)
{
    return (data.checksum==generate_backup_checksum(data));
}
uint32_t backup_sram::generate_backup_checksum(const rosflight_firmware::backup_data_t& data)
{
    const uint32_t* pointer = reinterpret_cast<const uint32_t*>(&data);
    uint32_t checksum = 0;
    for(uint8_t i=0;i<(sizeof (rosflight_firmware::backup_data_t)/4)-1;i++)
        checksum^=*(pointer+i);
    return checksum;
}
uint32_t generate_backup_checksum(const rosflight_firmware::backup_data_t& data)
{
    return backup_sram::generate_backup_checksum(data);
}
void backup_sram_write(const rosflight_firmware::backup_data_t& data)
{
    backup_sram::get_instance().write(data);
}
#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //Remove this, too
/*void test_backup_sram(){
  backup_sram_init();
  rosflight_firmware::backup_data_t read_data = backup_sram_read();
  rosflight_firmware::backup_data_t write_data = {};
  write_data.error_code = 0xabcde;
  if(check_backup_checksum(read_data))
  {
  }
  write_data.checksum=generate_backup_checksum(write_data);
  backup_sram_write(write_data);
}*/
