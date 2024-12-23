#ifndef FLASH_MEMORY_FN_H
#define FLASH_MEMORY_FN_H

#include "definitions.h"
#include "flash_memory.h"

// Erase all sectors from a given flash memory (spi_port).
void flash_erase_all_sectors(spi_fn* spi_functions)
{
    uint32_t address = 0;
    for (uint16_t i = 0; i < MEMORY_N_SECTORS; i++) {
        flash_erase(spi_functions, address, ERASE_SECTOR);
        address = address + MEMORY_SECTOR_SIZE;
    }
}

// Transfer data from the flash memory (spi_port) to an uart device (uart_port), starting from a given
// address (page_address) and with a given total lenth (nbytes).
void flash_transfer_data_to_uart(spi_fn* from_spi_functions, uart_fn* to_uart_port, uint32_t page_address, uint32_t nbytes)
{
    uint8_t data;
    for (uint32_t i = 0; i < nbytes; i++) {
        data = flash_read(from_spi_functions, page_address);
        to_uart_port->put_char(data);
        page_address++;
    }
}

// Transfer data from one flash memory (from_spi_port) to another (to_spi_port), with different to and from page
// addresses and configurable length (nbytes). The same flash memory can be used as source and destination.
void flash_transfer_data_to_flash(spi_fn* from_spi_functions, uint32_t from_address, spi_fn* to_spi_functions, uint32_t to_address, uint32_t nbytes)
{
    uint8_t data;
    for (uint32_t i = 0; i < nbytes; i++) {
        data = flash_read(from_spi_functions, from_address);
        flash_write(to_spi_functions, to_address, data);
        from_address++;
        to_address++;
    }
}

// Transfer data from ram to flash memory (to_spi_port) with configurable length (nbytes).
void flash_transfer_data_from_ram(spi_fn* to_spi_functions, uint32_t to_address, uint8_t* data, uint32_t nbytes)
{
    for (uint32_t i = 0; i < nbytes; i++) {
        flash_write(to_spi_functions, to_address, data[i]);
        to_address++;
    }
}

// Transfer data to ram from flash memory (from_spi_port) with configurable length (nbytes).
void flash_transfer_data_to_ram(spi_fn* to_spi_functions, uint32_t from_address, uint8_t* data, uint32_t nbytes)
{
    for (uint32_t i = 0; i < nbytes; i++) {
        data[i] = flash_read(to_spi_functions, from_address);
        from_address++;
    }
}

// Erase memory pages around the desired addresses in memory spi_port.
// Bytes might be erased before start_address and after end_address due to page allignment.

void flash_erase_pages(spi_fn* to_spi_functions, uint32_t start_address, uint32_t end_address)
{
    for (uint32_t i = start_address; i < end_address; i += MEMORY_PAGE_SIZE) {
        flash_erase(to_spi_functions, i, ERASE_PAGE);
    }
}

// Erase memory sectors around the desired addresses in memory spi_port.
// Bytes might be erased before start_address and after end_address due to sector allignment.
void flash_erase_sectors(spi_fn* to_spi_functions, uint32_t start_address, uint32_t end_address)
{
    for (uint32_t i = start_address; i < end_address; i += MEMORY_SECTOR_SIZE) {
        flash_erase(to_spi_functions, i, ERASE_SECTOR);
    }
}

void flash_dump(spi_fn* spi_functions, uint32_t start, uint32_t end)
{
    uint8_t data;
    uint16_t empty_blocks = 0;
    for (uint32_t i = start; i < end; i++) {
        if (i % (TERMINAL_COLS / 2) == 0)
            fprintf(PC, "\r\n");
        data = flash_read(spi_functions, i);
        fprintf(PC, "%02X", data);
        if (data == 0xFF) {
            empty_blocks++;
        } else {
            empty_blocks = 0;
        }
        if (empty_blocks >= EMPTY_BLOCKS_LIMIT) {
            break;
        }
    }
}

// Initialize the flash control structure (stores address for flash memory operation)
void flash_initialize_flash_ctrl(uint32_t start,
    uint32_t end,
    uint32_t current,
    uint32_t packet_size,
    flash_ctrl* fmem)
{
    fmem->start = start;
    fmem->end = end - (end - start) % packet_size;
    fmem->current = current;
    fmem->delta = packet_size;
}

// Initialize the flash control structure (stores address for flash memory operation) based on existing address information from flash
void flash_initialize_flash_ctrl_from_memory_date_based(
    uint8_t sectors_per_day,
    int8_t deployment_counter,
    uint32_t candidate_address,
    uint32_t first_address,
    uint32_t telemetry_size,
    uint8_t update_date,
    flash_ctrl* fmem)
{
    
    if(deployment_counter < 0)
        deployment_counter = 0;

    uint32_t addr_start;
    uint16_t day_of_the_year;

    if(deployment_counter < 5){
        day_of_the_year = deployment_counter;
        addr_start = first_address + (uint32_t)deployment_counter * (uint32_t)sectors_per_day * MEMORY_SECTOR_SIZE;
    }
    else{
        if(update_date){
            struct_tm* local_time = localtime(&current_time);
            day_of_the_year = local_time->tm_yday; // struct_tm::tm_yday is the day of the year (from 0-365)
        } else{
            day_of_the_year = (candidate_address - first_address) / ((uint32_t)sectors_per_day * MEMORY_SECTOR_SIZE);
        }
        if (day_of_the_year > 366)
            day_of_the_year = 0;
        addr_start = first_address + (uint32_t)day_of_the_year * (uint32_t)sectors_per_day * MEMORY_SECTOR_SIZE;
    }
    uint32_t addr_end = addr_start + (uint32_t)sectors_per_day * MEMORY_SECTOR_SIZE;
    if(candidate_address >= addr_start && candidate_address < addr_end){
        flash_initialize_flash_ctrl(addr_start, addr_end, candidate_address, telemetry_size, fmem);
    } else {
        flash_initialize_flash_ctrl(addr_start, addr_end, addr_start, telemetry_size, fmem);
    }
    fprintf(PC, "\r\nDay of the year = %lu\r\n", day_of_the_year + 1);
    fprintf(PC, "Candidate telemetry address = 0x%8lX\r\n", candidate_address);
    fprintf(PC, "Initial telemetry address = 0x%8lX\r\n", addr_flags.flash_telemetry.start);
    fprintf(PC, "Current telemetry address = 0x%8lX\r\n", addr_flags.flash_telemetry.current);
    fprintf(PC, "Telemetry size = %u\r\n", addr_flags.flash_telemetry.delta);
    fprintf(PC, "Last telemetry address = 0x%8lX\r\n", addr_flags.flash_telemetry.end);
}

// Writes data according to a flash control structure (fctrl) to a flash memory in spi_port
void flash_cycle_write(spi_fn* spi_functions, uint8_t* data, flash_ctrl* fctrl)
{
    if (fctrl->current < fctrl->start || fctrl->current >= fctrl->end)
        fctrl->current = fctrl->start;
    uint32_t remaining_addressess = (MEMORY_PAGE_SIZE - (fctrl->current % MEMORY_PAGE_SIZE)) % MEMORY_PAGE_SIZE;
    if (remaining_addressess < fctrl->delta) {
        flash_erase(spi_functions, fctrl->current + remaining_addressess, ERASE_PAGE);
    }
    for (int i = 0; i < fctrl->delta; i++) {
        flash_write(spi_functions, fctrl->current, data[i]);
        fctrl->current++;
    }
}

// Recover last available address from a flash control structure (fctrl)
void flash_recover_last_addr(spi_fn* spi_functions, flash_ctrl* fctrl)
{
    uint8_t data;
    for (fctrl->current = fctrl->end - fctrl->delta; fctrl->current >= fctrl->start; fctrl->current -= fctrl->delta) {
        for (uint8_t i = 0; i < fctrl->delta; i++) {
            data = flash_read(spi_functions, fctrl->current + i);
            if (data != 0xFF) {
                fctrl->current += fctrl->delta;
                goto loop_end;
            }
        }
    }
loop_end:
    if (fctrl->current >= fctrl->end)
        fctrl->current = fctrl->start;
}

#endif // !FLASH_MEMORY_FN_H
