#ifndef BACKUP_SRAM_H
#define BACKUP_SRAM_H

#include <stdint.h>
#include "board.h"

#ifdef __cplusplus
class backup_sram {
public:
    void init();
    void write(const rosflight_firmware::backup_data_t&);
    rosflight_firmware::backup_data_t read();
    bool has_data();
    static uint32_t generate_backup_checksum(const rosflight_firmware::backup_data_t&);
    static backup_sram& get_instance() {return backup_sram::instance;}
private:
    backup_sram();
    rosflight_firmware::backup_data_t do_first_read();
    rosflight_firmware::backup_data_t data;
    bool valid_data=false;
    static bool is_valid(const rosflight_firmware::backup_data_t&);
    static backup_sram instance;
};

extern "C" {
#endif // __cplusplus

void backup_sram_write(const rosflight_firmware::backup_data_t&);
uint32_t generate_backup_checksum(const rosflight_firmware::backup_data_t&);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // BACKUP_SRAM_H
