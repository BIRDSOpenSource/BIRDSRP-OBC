#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <device.h>
#include <time.h>
#include <stdint.h>

// These are used only for PC simulation:
uart_declare(PC);
uart_declare(COMM);
uart_declare(RST);
uart_declare(FAB);
uart_declare(MSN);
spi_declare(MAIN_FM);
spi_declare(COM_FM);
spi_declare(MISSION_FM);

#ifndef EM
#warning "Building for FM"
#define SPACECRAFT_ID 0x53
#else
#warning "Building for EM"
#define SPACECRAFT_ID 0x54
#endif // !FM

#define BUFF_LENGTH 28            // Must accomodate messages of all UART sources that may be scheduled | OPERA: 2 + 32 - sizeof(time_t) - sizeof(uint16_t)
#define MAX_LENGTH MSG_LENGTH_FAB // Maximum UART length
#define TERMINAL_COLS 80          // Number of columns in terminal
#define EMPTY_BLOCKS_LIMIT 64     // Stop after reading n bytes of empty data on flash

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

// Reset PIC commands
#define MSG_RST 0xB0
#define MSG_LENGTH_RST 36
#define MSG_CHECKSUM_RST false

// EPS1 commands
#define MSG_FAB 0xE0
#define MSG_LENGTH_FAB 64
#define MSG_CHECKSUM_FAB false

// // SEL ZES commands
// #define MSG_SEL_ZES 0x2A
// #define MSG_LENGTH_SEL_ZES 128
// #define MSG_CHECKSUM_SEL_ZES false

// // TMCR1 commands
#define MSG_TMCR1 0x1C
#define MSG_LENGTH_TMCR1 36
#define MSG_CHECKSUM_TMCR1 true

// TMCR2 commands
#define MSG_TMCR2 0x1D
#define MSG_LENGTH_TMCR2 36
#define MSG_CHECKSUM_TMCR2 true

// ADCS commands
#define MSG_ADCS 0xAD
#define MSG_LENGTH_ADCS 44 // changed from 43 to 44
#define MSG_CHECKSUM_ADCS true

// OPERA commmands
#define MSG_OPERA 0xA1
#define MSG_LENGTH_OPERA 28
#define MSG_CHECKSUM_OPERA false

// MCPIC commands
#define MSG_MCPIC 0xCB
#define MSG_LENGTH_MCPIC 6
#define MSG_CHECKSUM_MCPIC true

// PCIB commands
#define MSG_PCIB 0xAB
#define MSG_LENGTH_PCIB 36
#define MSG_CHECKSUM_PCIB true

time_t current_time, previous_time;

#define T0 946684800        // earliest time possible for RTC (Jan 1st 2000, 00:00:00)
#define T_ANTENNA 946686630 // Antenna deployment time (30 min and 30 secs)
#define Tn 2147483646 // latest time possible for RTC (Jan 19th 2038, 3:14:06)

typedef struct telemetry_time_str { // Stores the time when telemetry is received
    time_t reset_time;
    time_t fab_time;
    time_t pcib_time;
    time_t msn_time;
    time_t adcs_time;
    time_t com_time;
} telemetry_time_str;

typedef struct telemetry_str { // Stores the telemetry of the last period, before being written to flash
    uint8_t reset_time;
    uint8_t reset_message[MSG_LENGTH_RST - 12];
    uint8_t fab_time;
    uint8_t fab_message[MSG_LENGTH_FAB - 2];
    uint8_t msn_time;
    uint8_t msn_message[MSG_LENGTH_MCPIC - 4];
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

typedef struct address_rotation {
    uint32_t flash_log_current;
    uint32_t flash_telemetry_current;
    uint32_t flash_sel_zes_current;
    uint32_t flash_sel_ref_current;
} address_rotation;

// =========================== MEMORY MAP =============================

#define BOOT_FLAGS_ADDRESS 0x00000000
#define OBC_FLAGS_ADDRESS BOOT_FLAGS_ADDRESS + MEMORY_PAGE_SIZE
#define ADDR_FLAGS_ADDRESS OBC_FLAGS_ADDRESS + MEMORY_PAGE_SIZE
#define SCHEDULED_CMD_ADDRESS ADDR_FLAGS_ADDRESS + MEMORY_PAGE_SIZE

#define FLASH_ADDR_START ADDR_FLAGS_ADDRESS
#define FLASH_ADDR_END SCHEDULED_CMD_ADDRESS
#define FLASH_ADDR_DELTA sizeof(address_rotation) // log, telemetry, SEL_ZES, SEL_REF -- 4 bytes each

#define FLASH_LOG_START (1 * MEMORY_SECTOR_SIZE)
#define FLASH_LOG_END (17 * MEMORY_SECTOR_SIZE)
#define FLASH_LOG_DELTA 7 // time,origin,command,return

#define FLASH_TELEMETRY_START (17 * MEMORY_SECTOR_SIZE)
#define FLASH_TELEMETRY_END (749 * MEMORY_SECTOR_SIZE)
#define FLASH_TELEMETRY_DELTA sizeof(telemetry)
#define FLASH_TELEMETRY_SECTORS_PER_DAY 2

#define FLASH_CAMERA_START (749 * MEMORY_SECTOR_SIZE)
#define FLASH_CAMERA_END (1005 * MEMORY_SECTOR_SIZE)

#define FLASH_SEL_ZES_START (1005 * MEMORY_SECTOR_SIZE)
#define FLASH_SEL_ZES_END (1133 * MEMORY_SECTOR_SIZE)
#define FLASH_SEL_ZES_DELTA 128

#define FLASH_SEL_REF_START (1133 * MEMORY_SECTOR_SIZE)
#define FLASH_SEL_REF_END (1261 * MEMORY_SECTOR_SIZE)
#define FLASH_SEL_REF_DELTA 128

#define FLASH_MAGNETOMETER_START (1261 * MEMORY_SECTOR_SIZE)
#define FLASH_MAGNETOMETER_END (1517 * MEMORY_SECTOR_SIZE)

#define FLASH_ADCS_HS_START (1517 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_HS_END (1524 * MEMORY_SECTOR_SIZE)

#define FLASH_ADCS_GPS_START (1524 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_GPS_END (1529 * MEMORY_SECTOR_SIZE)

#define FLASH_ADCS_TELEMETRY_START (1529 * MEMORY_SECTOR_SIZE)
#define FLASH_ADCS_TELEMETRY_END (1785 * MEMORY_SECTOR_SIZE)

#define FLASH_OPERA_START (1785 * MEMORY_SECTOR_SIZE)
#define FLASH_OPERA_END (2027 * MEMORY_SECTOR_SIZE)

#define FLASH_TMCR_B4_START (2027 * MEMORY_SECTOR_SIZE)
#define FLASH_TMCR_B4_END (2037 * MEMORY_SECTOR_SIZE)

#define FLASH_TMCR_N_START (2037 * MEMORY_SECTOR_SIZE)
#define FLASH_TMCR_N_END (2048 * MEMORY_SECTOR_SIZE)
// ====================================================================

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
    uint8_t MCP_on_off;
    uint8_t gps_time_sync_state;
    int8_t adcs_initial_value;
    uint8_t cw_mode;                 // 0: never send CW; 1: follow ADCS; 2: Send CW always
    uint16_t heater_ref_temperature; // battery heater
    uint16_t heater_ref_voltage;     // battery heater
    uint8_t opera_boot_duration;     // in seconds, check variable type when updating
    uint8_t relay_on_off;
    uint8_t camera_parameters[11]; // store parameters for camera capture command (0xC0CA)
} oflags;

// Store the address variables
typedef struct aflags {
    flash_ctrl flash_addr;
    flash_ctrl flash_log;
    flash_ctrl flash_telemetry;
    flash_ctrl flash_sel_zes;
    flash_ctrl flash_sel_ref;
} aflags;

// Initialize the structures with values and add comments for clarity
bflags boot_flags = {
    0xFF // deployment_flag
};

oflags obc_flags = {
    0,     // leap_seconds
    1,     // adcs_on_off
    1,     // MCP_on_off
    0,     // gps_time_sync_state
    0,     // adcs_initial_value
    2,     // cw_mode
    0xC3F, // heater_ref_temperature
    0xC48, // heater_ref_voltage
    120,   // opera_boot_duration
    1,     // relay_on_off
    { 0 }  // camera_parameters[15]
};

aflags addr_flags = { 0 };

void print_flags()
{
    fprintf(PC, "boot_flag = %02X\r\n", boot_flags.deployment_flag);                     // uint8_t
    fprintf(PC, "leap_seconds = %d\r\n", obc_flags.leap_seconds);                        // int8_t
    fprintf(PC, "adcs_on_off = %u\r\n", obc_flags.adcs_on_off);                          // uint8_t
    fprintf(PC, "MCP_on_off = %u\r\n", obc_flags.MCP_on_off);                            // uint8_t
    fprintf(PC, "gps_time_sync_state = %u\r\n", obc_flags.gps_time_sync_state);          // uint8_t
    fprintf(PC, "adcs_initial_value = %d\r\n", obc_flags.adcs_initial_value);            // int8_t
    fprintf(PC, "cw_mode = %u\r\n", obc_flags.cw_mode);                                  // uint8_t
    fprintf(PC, "heater_ref_temperature = %04lX\r\n", obc_flags.heater_ref_temperature); // uint16_t in hexadecimal
    fprintf(PC, "heater_ref_voltage = %04lX\r\n", obc_flags.heater_ref_voltage);         // uint16_t in hexadecimal
    fprintf(PC, "opera_boot_duration = %us\r\n", obc_flags.opera_boot_duration);         // uint8_t
    fprintf(PC, "relay_on_off = %u\r\n", obc_flags.relay_on_off);                        // uint8_t
    fprintf(PC, "camera_parameters = ");                                                 // uint8_t[]
    for(uint8_t i = 0; i < sizeof(obc_flags.camera_parameters); i++){
        fprintf(PC, "%02X", obc_flags.camera_parameters[i]);
    }
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

uint8_t verbose = false;     // True to set verbose mode
uint8_t memory_busy = false; // True if flash memory is used by COM
uint8_t mux_lock = false;    // True if long operation is locking the mux

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

typedef enum {
    mux_sel_zes = 0,
    mux_tmcr1 = 1,
    mux_adcs = 2,
    mux_tmcr2 = 3,
    mux_opera = 4,
    mux_mcpic = 5,
    mux_pcib = 6,
    mux_sel_ref = 7,
} cpld_mux_sel;

char* mux_str_list[] = {
    "ZES",
    "TM1",
    "ADC",
    "TM2",
    "OPE",
    "MCP",
    "CAM",
    "REF"
};

uint8_t mux_cpld_position = mux_mcpic;

// Change mission control pic MUX position
uint8_t mux_sel(cpld_mux_sel sel)
{
    if (mux_lock)
        return mux_cpld_position; // Do not change if locked

    // MUX_CPLD_SEL_0 is the MSB
    output_bit(MUX_CPLD_SEL_0, (sel >> 2) & 0x1);
    output_bit(MUX_CPLD_SEL_1, (sel >> 1) & 0x1);
    output_bit(MUX_CPLD_SEL_2, (sel >> 0) & 0x1);

#ifndef SIM
    if (sel == mux_sel_ref || sel == mux_sel_zes) {
        set_uart_speed(9600, MSN);
    } else {
        set_uart_speed(115200, MSN);
    }
#endif

    mux_cpld_position = sel;

    delay_ms(10);

    if (verbose)
        fprintf(PC, "|MUX changed to %s|", mux_str_list[mux_cpld_position]);

    return mux_cpld_position;
}

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

uint8_t adcs_raw_part_a[16] = { 0 }; // First part of raw ADCS command
uint8_t adcs_raw_part_b[16] = { 0 }; // Second part of raw ADCS command
uint8_t adcs_raw_part_a_crc0 = 0xFF; // CRC[0] of first part of raw ADCS command
uint8_t adcs_raw_part_b_crc0 = 0xFF; // CRC[0] of second part of raw ADCS command

enum {
    GA_part_a_size = 16, // First 4 bytes is mission time
    GA_part_b_size = 16
};

uint8_t opera_GA_part_a[GA_part_a_size] = { 0 };

uint8_t sband_tx_status = 0;
uint8_t rpi_status = 0;

#endif /* DEFINITIONS_H */
