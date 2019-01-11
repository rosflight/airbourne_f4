/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include "system.h"
#include "pwm.h"
#include "led.h"
#include "stm32f4xx_pwr.h"

#include "revo_f4.h"

struct backup_data {
    uint32_t reset_count;
    uint32_t answer;
    uint32_t checksum;
};

void init_backup_sram(){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
    PWR_BackupRegulatorCmd(ENABLE);

}
void write_backup_sram(backup_data& data){
    PWR_BackupAccessCmd(ENABLE);
    memcpy(reinterpret_cast<void*>(BKPSRAM_BASE),&data,sizeof(backup_data));
    PWR_BackupAccessCmd(DISABLE);
}
backup_data read_backup_sram(){
    backup_data data = *(reinterpret_cast<backup_data*>(BKPSRAM_BASE));
    return data;
}
uint32_t checksum_backup_data(const backup_data& data)
{
    return data.reset_count ^ data.answer;
}
bool check_backup_data(const backup_data& data)
{
    return(checksum_backup_data(data)==data.checksum);
}
void restart(){
    NVIC_SystemReset();
}
int main() {
	systemInit();
    init_backup_sram();
    backup_data read_data = read_backup_sram();
    uint32_t reset_count = 0;
    if(check_backup_data(read_data))
        reset_count = read_data.reset_count;
    backup_data write_data;
    write_data.answer=42;
    write_data.reset_count=++reset_count;
    write_data.checksum=checksum_backup_data(write_data);
    write_backup_sram(write_data);
    delay(300);
    restart();
}


