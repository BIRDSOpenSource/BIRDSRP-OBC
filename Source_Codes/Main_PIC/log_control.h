#ifndef LOG_CONTROL_H
#define LOG_CONTROL_H

#include <string.h>
#include "definitions.h"
#include "flash_memory.h"

// Adds a log entry to buffer in RAM
void log_add(log_entry e)
{
    if (log_index < MAX_LOGS_IN_RAM) {
        memcpy(&log_buffer[log_index], &e, sizeof(log_entry));
        log_index++;
    }
}

// Clears buffer in RAM
void log_clear()
{
    log_index = 0;
}

// Flushes RAM buffer to flash memory
void log_flush()
{
    for(uint8_t i=0; i<log_index; i++){
        uint8_t *ptr = (uint8_t*)&log_buffer[i];
        flash_cycle_write(&spi_port_COM_FM, ptr, &addr_flags.flash_log);
    }
    fprintf(PC, "Log: 0x%08lX => 0x%08lX", addr_flags.flash_log.current - log_index*sizeof(log_entry), addr_flags.flash_log.current);
    log_index = 0;
}

#endif /* LOG_CONTROL_H */
