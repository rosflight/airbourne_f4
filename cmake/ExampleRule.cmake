
###############################################################################
# Copyright (c) 2020 - James Jackson
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the <organization> nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set(STARTUP_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/../lib/startup/stm32f405.s)

set(OBJ_COPY arm-none-eabi-objcopy)

function(add_example name)
    add_executable(${name}.elf ${ARGN} ${STARTUP_SCRIPT})
    target_link_libraries(${name}.elf airbourne_core printf)

    # The bin and hex targets get built every time
    add_custom_target(${name}.hex ALL
                       COMMAND ${OBJ_COPY} -O ihex --set-start 0x8000000 ${name}.elf ${name}.hex)
    add_dependencies(${name}.hex ${name}.elf)
    add_custom_target(${name}.bin ALL
                      COMMAND ${OBJ_COPY} -I ihex -O binary ${name}.hex ${name}.bin)
    add_dependencies(${name}.bin ${name}.hex)


    # This creates the rule so you can call `make flash_blink` to flash the blink example
    add_custom_target(flash_${name}
                      COMMAND dfu-util -a 0 -s  0x08000000 -D ${name}.bin)
    add_dependencies(flash_${name} ${name}.bin)
endfunction()
