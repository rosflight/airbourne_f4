#ifndef BACKUP_SRAM_H
#define BACKUP_SRAM_H

#include <stdint.h>
struct debug_info_t{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
};
struct backup_data_t{
    uint32_t error_code;
    debug_info_t debug_info;
    uint32_t reset_count;
    uint32_t checksum; //With the current implementation of the checksum, this must go last
};
void backup_sram_init();
void backup_sram_write(const backup_data_t&);
backup_data_t backup_sram_read();
bool check_backup_checksum(const backup_data_t&);
uint32_t generate_backup_checksum(const backup_data_t&);
#endif // BACKUP_SRAM_H
