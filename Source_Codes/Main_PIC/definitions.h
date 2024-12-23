#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "device.h"
#include <time.h>
#include <stdint.h>

uart_declare(PC);
uart_declare(COMM);
uart_declare(RST);
uart_declare(FAB);
uart_declare(PCIB);
uart_declare(ADCS);

spi_declare(MAIN_FM);
spi_declare(COM_FM);
spi_declare(MISSION_FM);

#ifndef EM
#warning "Building for FM"
#define SPACECRAFT_ID 0x55
#else
#warning "Building for EM"
#define SPACECRAFT_ID 0x56
#endif // !EM

#define BUFF_LENGTH 25        // Must accomodate messages of all UART sources that may be scheduled
#define MAX_LENGTH 108        // Maximum UART length
#define TERMINAL_COLS 80      // Number of columns in terminal
#define EMPTY_BLOCKS_LIMIT 64 // Stop after reading n bytes of empty data on flash

#define MSG_WILDCARD 0xFF // This has permission to run all commands

// OBC -> subsystem commands
#define MSG_OBC 0x0B

// Debug commands
#define MSG_PC 0xDB
#define MSG_LENGTH_PC 25
#define MSG_CHECKSUM_PC true

// Com PIC commands
#define MSG_COMM 0xC0
#define MSG_LENGTH_COMM 25
#define MSG_CHECKSUM_COMM true

// PCIB commands
#define MSG_PCIB 0xAB
#define MSG_LENGTH_PCIB 36
#define MSG_CHECKSUM_PCIB true

// Reset pic commands
#define MSG_RST 0xB0
#define MSG_LENGTH_RST 36
#define MSG_CHECKSUM_RST false

// EPS1 commands
#define MSG_FAB 0xE0
#define MSG_LENGTH_FAB 60
#define MSG_CHECKSUM_FAB false

// ADCS commands
#define MSG_ADCS 0xAD
#define MSG_LENGTH_ADCS 43
#define MSG_CHECKSUM_ADCS true

#define BOOT_FLAGS_ADDRESS 0x00000000
#define OBC_FLAGS_ADDRESS BOOT_FLAGS_ADDRESS + MEMORY_PAGE_SIZE
#define ADDR_FLAGS_ADDRESS OBC_FLAGS_ADDRESS + MEMORY_PAGE_SIZE
#define SCHEDULED_CMD_ADDRESS ADDR_FLAGS_ADDRESS + MEMORY_PAGE_SIZE

time_t current_time, previous_time;

#define T0 946684800        // earliest time possible for RTC (Jan 1st 2000, 00:00:00)
#define T_ANTENNA 946686630 // Antenna deployment time
#define Tn 2147483646       // latest time possible for RTC (Jan 19th 2038, 3:14:06)

typedef struct telemetry_time_str { // Stores the time when telemetry is received
    time_t reset_time;
    time_t fab_time;
    time_t pcib_time;
    time_t adcs_time;
    time_t com_time;
} telemetry_time_str;

typedef struct telemetry_str { // Stores the telemetry of the last period, before being written to flash
    uint8_t reset_time;
    uint8_t reset_message[MSG_LENGTH_RST - 12];
    uint8_t fab_time;
    uint8_t fab_message[MSG_LENGTH_FAB - 6];
    uint8_t pcib_time;
    uint8_t pcib_message[MSG_LENGTH_PCIB - 26];
    uint8_t adcs_time;
    uint8_t adcs_message[MSG_LENGTH_ADCS - 4];
    uint8_t com_time;
    uint16_t com_rssi;
    time_t obc_time;
    uint8_t master_footer[2];
} telemetry_str;

telemetry_str telemetry;
telemetry_time_str telemetry_time;

#define CW_PAGES 2
#define CW_LENGTH 6              // in bytes
uint8_t cw[CW_PAGES][CW_LENGTH]; // Stores the CW beacon string for the last 50s

// Flash memory addresses
#define FLASH_ADDR_START ADDR_FLAGS_ADDRESS
#define FLASH_ADDR_END SCHEDULED_CMD_ADDRESS
#define FLASH_ADDR_DELTA 8 // log and telemetry -- 4 bytes each

#define FLASH_LOG_START (1 * MEMORY_SECTOR_SIZE)
#define FLASH_LOG_END (17 * MEMORY_SECTOR_SIZE)
#define FLASH_LOG_DELTA 7 // time,origin,command,return

#define FLASH_TELEMETRY_START (17 * MEMORY_SECTOR_SIZE)
#define FLASH_TELEMETRY_END (749 * MEMORY_SECTOR_SIZE)
#define FLASH_TELEMETRY_DELTA sizeof(telemetry)
#define FLASH_TELEMETRY_SECTORS_PER_DAY 2

// Sectors 749 to 1517 (-1) are free to be allocated

#define FLASH_ADCS_HS_START (1517 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_HS_END (1524 * MEMORY_SECTOR_SIZE)

#define FLASH_ADCS_GPS_START (1524 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_GPS_END (1529 * MEMORY_SECTOR_SIZE)

#define FLASH_ADCS_TELEMETRY_START (1529 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_TELEMETRY_END (1785 * MEMORY_SECTOR_SIZE)

// From 1785-2048 there are 263 sectors of free space

typedef struct flash_ctrl {
    uint32_t start;
    uint32_t end;
    uint32_t current;
    uint8_t delta;
} flash_ctrl;

// Store the OBC boot variables
typedef struct bflags {
    uint8_t deployment_flag;
} bflags;

// Store the OBC variables
typedef struct oflags {
    int8_t leap_seconds;
    uint8_t adcs_on_off;
    uint8_t pcib_on_off;
    uint8_t gps_time_sync_state;
    int8_t adcs_initial_value;
    uint8_t cw_mode;                 // 0: never send CW; 1: follow ADCS; 2: Send CW always
    uint16_t heater_ref_temperature; // battery heater
    uint16_t heater_ref_voltage;     // battery heater
} oflags;

// Store the address variables
typedef struct aflags {
    flash_ctrl flash_addr;
    flash_ctrl flash_log;
    flash_ctrl flash_telemetry;
} aflags;

bflags boot_flags = { 0xFF };
oflags obc_flags = { 0, 1, 1, 0, 0, 2, 0xC3F, 0xC48 }; // Update to reference values
aflags addr_flags = { 0 };

void print_flags()
{
    fprintf(PC, "leap_seconds = %d\r\n", obc_flags.leap_seconds);
    fprintf(PC, "adcs_on_off = %d\r\n", obc_flags.adcs_on_off);
    fprintf(PC, "pcib_on_off = %d\r\n", obc_flags.pcib_on_off);
    fprintf(PC, "gps_time_sync_state = %d\r\n", obc_flags.gps_time_sync_state);
    fprintf(PC, "adcs_initial_value = %d\r\n", obc_flags.adcs_initial_value);
    fprintf(PC, "cw_mode = %d\r\n", obc_flags.cw_mode);
    fprintf(PC, "heater_ref_temperature = %02lX\r\n", obc_flags.heater_ref_temperature);
    fprintf(PC, "heater_ref_voltage = %02lX\r\n", obc_flags.heater_ref_voltage);
    fprintf(PC, "\r\n");
}

#define SCHEDULED_COMMANDS_MAX 32 // Requires 1600 bytes = 40 * (36 + 4)
#define TIME_T_MAX 0x7FFFFFFF

// A structure that defines a scheduled command.
typedef struct scheduled_command {
    time_t time; // Unix time (seconds after Jan 1st 1970)
    uint8_t command[BUFF_LENGTH];
} scheduled_command;

scheduled_command scheduled_commands[SCHEDULED_COMMANDS_MAX];

#define MAX_LOGS_IN_RAM 64

uint8_t memory_busy = false; // True if flash memory is used by COM

// A structure that defines a log entry.
typedef struct log_entry {
    time_t time;
    uint8_t origin;
    uint8_t command;
    uint8_t return_value;
} log_entry;

log_entry log_buffer[MAX_LOGS_IN_RAM]; // Stores logs in RAM before they are flushed to flash
int log_index = 0;                     // The index of current log in RAM

int clock_update = 0; // Periodical flag that indicates that periodic functions should run

inline void sync()
{
    while (!clock_update) { }
}

int uart_mux = 1;              // Used to chose which UART is being listened to
int response_rx = 0;           // Indicates a response was received
time_t reset_time = T0;        // Stores the time of last reset
int rst_clock_update = 0;      // Update local clock with reset pic
int rst_clock_updated = false; // Becomes true after reset pic clock sync
int adcs_mode = 12;            // ADCS desired mode (by OBC)

// Struct to hold a boot command.
typedef struct boot_command {
    time_t time; // Time to run the command after boot.
    uint8_t command[BUFF_LENGTH];
} boot_command;

#define BOOT_COMMANDS_MAX 8
#define BOOT_COMMANDS_ADDR 0 // Dedicated memory

boot_command boot_commands[BOOT_COMMANDS_MAX];

typedef struct xmodem_address {
    uint32_t source_address;
    uint32_t destination_address;
    uint32_t n_packets;
} xmodem_address;

#define STM32_SIZE 32
uint8_t stm32_command_uhf[STM32_SIZE];

#endif /* DEFINITIONS_H */
