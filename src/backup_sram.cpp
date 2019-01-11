#include "backup_sram.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include <cstring>

void backup_sram_init()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
    PWR_BackupRegulatorCmd(ENABLE);
}
void backup_sram_write(const backup_data_t& data)
{
    PWR_BackupAccessCmd(ENABLE);
    std::memcpy(reinterpret_cast<void*>(BKPSRAM_BASE),&data,sizeof(backup_data_t));
    PWR_BackupAccessCmd(DISABLE);
}
backup_data_t backup_sram_read()
{
    backup_data_t data = *(reinterpret_cast<backup_data_t*>(BKPSRAM_BASE));
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
