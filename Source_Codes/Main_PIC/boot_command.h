#ifndef BOOT_COMMAND_H
#define BOOT_COMMAND_H

#include <definitions.h>
#include <flash_memory.h>
#include <flash_memory_fn.h>
#include <scheduler.h>

void boot_commands_write()
{
    uint8_t* bc_ptr = (uint8_t*)boot_commands;
    flash_erase_pages(&spi_port_MAIN_FM, BOOT_COMMANDS_ADDR, BOOT_COMMANDS_ADDR + sizeof(boot_commands));
    flash_transfer_data_from_ram(&spi_port_MAIN_FM, BOOT_COMMANDS_ADDR, bc_ptr, sizeof(boot_commands));
}

void boot_commands_read()
{
    uint8_t* bc_ptr = (uint8_t*)boot_commands;
    flash_transfer_data_to_ram(&spi_port_MAIN_FM, BOOT_COMMANDS_ADDR, bc_ptr, sizeof(boot_commands));
}

void boot_commands_clear_nth(uint8_t n)
{
    if (n < BOOT_COMMANDS_MAX)
        boot_commands[n].time = TIME_T_MAX;
}

void boot_commands_clear_all()
{
    for (uint8_t i = 0; i < BOOT_COMMANDS_MAX; i++) {
        boot_commands_clear_nth(i);
    }
}

uint8_t boot_commands_add(boot_command bc)
{
    for (uint8_t i = 0; i < BOOT_COMMANDS_MAX; i++) {
        if (boot_commands[i].time == TIME_T_MAX) {
            memcpy(&boot_commands[i], &bc, sizeof(boot_command));
            return i;
        }
    }
    return -1;
}

void boot_commands_schedule()
{
    for (uint8_t i = 0; i < BOOT_COMMANDS_MAX; i++) {
        if(boot_commands[i].time > 0 && boot_commands[i].time < TIME_T_MAX)
            vschedule(current_time + boot_commands[i].time, boot_commands[i].command);
    }
}

#endif /* BOOT_COMMAND_H */
