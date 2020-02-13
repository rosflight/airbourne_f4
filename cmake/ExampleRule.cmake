set(STARTUP_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/../lib/startup/stm32f405.s)

set(OBJ_COPY arm-none-eabi-objcopy)

function(add_example name)
    message("Creating example ${name} from ${ARGN}")
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
