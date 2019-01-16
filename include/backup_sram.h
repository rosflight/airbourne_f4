#ifndef BACKUP_SRAM_H
#define BACKUP_SRAM_H

#include <stdint.h>
#include "board.h"

void backup_sram_init();
void backup_sram_write(const rosflight_firmware::backup_data_t&);
rosflight_firmware::backup_data_t backup_sram_read();
bool check_backup_checksum(const rosflight_firmware::backup_data_t&);
uint32_t generate_backup_checksum(const rosflight_firmware::backup_data_t&);
void test_backup_sram();
#endif // BACKUP_SRAM_H
