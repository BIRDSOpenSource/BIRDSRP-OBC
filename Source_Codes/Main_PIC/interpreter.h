#ifndef INTERPRETER_H
#define INTERPRETER_H

#include "definitions.h"
#include "scheduler.h"
#include "crc16.h"
#include "libuart_fn.h"
#include "flash_memory.h"
#include "xmodem.h"
#include "log_control.h"
#include "boot_command.h"

// Interpreter: The procedures here are concerned with interpreting
// received commands and executing the appropriate commands.

// Definition of commands. Should follow the prototype: "uint8_t command_name(uint8_t *data)"
// Return value = 0 indicates that the command was successful
// Return value > 0 indicates that there was an error

// ============ Helper functions ============

void get_com_shared_fm_access()
{
    if (memory_busy) {
        scheduled_command_clear_specified_command(0xC0, 0x58); // Disable scheduled command to regain access to memory in the future
        output_low(MUX_SEL_COM_SHARED_FM);                     // Regain access to memory now
        memory_busy = 0;                                       // Now memory is free
    }
}

// Helper function to calculate OBC checksum
void checksum_obc(uint8_t* data, uint8_t size)
{
    uint8_t checksum = 0;
    for (uint8_t i = 1; i < size - 2; i++) {
        checksum ^= data[i];
    }
    data[size - 2] = checksum;
    data[size - 1] = data[0] + 1; // Footer
}

// Helper function to check if uplink is valid
uint8_t uplink_valid(uint8_t* buffer)
{
    const uint8_t cmd_length = 22;        // Extended packet length
    const uint8_t cmd_legacy_length = 14; // Legacy packet length

    struct packet {
        uint8_t packet_format_id;
        uint8_t satellite_id;
        uint8_t cmd_format_id;
    }* packet = (struct packet*)buffer;

    uint16_t cr, pk;                                                                 // these are the crc check variables
    if (packet->packet_format_id == 0x42 && packet->satellite_id == SPACECRAFT_ID) { // This packet is meant for CURTIS
        if (packet->cmd_format_id == 0xCC) {                                         // Extended KITSUNE format (22 bytes)
            cr = mk_crc(buffer, cmd_length - 2);
            pk = make16(buffer[cmd_length - 1], buffer[cmd_length - 2]);
        } else { // Herritage BIRDS format (14 bytes)
            cr = mk_crc(buffer, cmd_legacy_length - 2);
            pk = make16(buffer[cmd_legacy_length - 1], buffer[cmd_legacy_length - 2]);
        }
        if (cr == pk) { // CRC is good to go
            return 1;   // Packet is valid
        }
    }
    return 0; // Packet is invalid
}

// Helper function to send an acknowledge back to COM
void send_com_ack(uint8_t* data)
{
    uint8_t cmd[24] = { 0 };
    cmd[0] = 0x0B;
    cmd[1] = 0xAA;
    cmd[2] = 0xCC;
    memcpy(cmd + 3, data, 8);
    cmd[12] = 0x66;
    cmd[23] = 0x0C;
    for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], COMM);
    }
}

// Helper function to change reset time
void reset_pic_update_clock(time_t time)
{
    struct_tm* tstr = localtime(&time);
    struct rst_msg {
        uint8_t rst_command;
        uint8_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
    } msg;

    msg.rst_command = 0x70;
    msg.year = tstr->tm_year - 100;
    msg.month = tstr->tm_mon + 1;
    msg.day = tstr->tm_mday;
    msg.hour = tstr->tm_hour;
    msg.minute = tstr->tm_min;
    msg.second = tstr->tm_sec;

    uint8_t i;
    uint8_t cmd[36] = { 0 };
    cmd[0] = 0xB0;
    uint8_t* ptr = (uint8_t*)&msg;
    for (i = 0; i < sizeof(msg); i++) {
        cmd[i + 1] = ptr[i];
    }
    cmd[35] = 0xB1;
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }
    uart_print_pc_hex(cmd, sizeof(cmd));
}

// Helper function to print binary data
void print_binary16(uint16_t data)
{
    const uint8_t size = 16;
    for (uint8_t i = 1; i <= size; i++) {
        fprintf(PC, "%Lu", ((data >> (size - i)) & 1));
    }
    fputc('\r', PC);
    fputc('\n', PC);
}

// Helper funtion to initialize the telemetry array.
void initialize_telemetry()
{
    memset(&telemetry_time, 0, sizeof(telemetry_time));
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.master_footer[0] = 0xB0;
    telemetry.master_footer[1] = 0x0B;
}

// Convert gyro data 16 -> 8 bits (-128 to 127 with LSB = 1deg/s, with overflow protection)
int8_t gyro_to_cw(uint8_t msb, uint8_t lsb)
{
    int8_t gyro_cw;
    float gyro = ((int16_t)make16(msb, lsb)) * 8.75e-3;
    if (gyro > 127.) {
        gyro_cw = 127;
    } else if (gyro < -128.) {
        gyro_cw = -128;
    } else {
        if (gyro > 0.) {
            gyro_cw = (int8_t)(gyro + 0.5);
        } else {
            gyro_cw = (int8_t)(gyro - 0.5);
        }
    }
    return gyro_cw;
}

// Helper funtion to initialize the cw beacon array.
void build_cw()
{
    uint8_t sc = scheduled_command_count();
    uint8_t adcs_mode = telemetry.adcs_message[0];
    time_t time_after_reset = (current_time - reset_time) / 3600;
    uint8_t time_after_reset_ = time_after_reset > 24 ? 0x1F : time_after_reset;

    memset(&cw, 0, sizeof(cw)); // Erase old data.

    // Page 0
    cw[0][0] = (telemetry.reset_message[6] << 4) | (telemetry.reset_message[7] >> 4);      // Battery voltage
    cw[0][1] = (telemetry.fab_message[48] << 4) | (telemetry.fab_message[49] >> 4);        // Battery current, 12 -> 8 bit
    cw[0][2] = (telemetry.fab_message[50] << 4) | (telemetry.fab_message[51] >> 4);        // Battery temperature, 12 -> 8 bit
    cw[0][3] = (telemetry.fab_message[10] << 4) | (telemetry.fab_message[11] >> 4);        // CPLD temperature
    cw[0][4] = (make16(telemetry.fab_message[28], telemetry.fab_message[29]) > 0x229) << 7 // +X sun / no sun
        | (make16(telemetry.fab_message[30], telemetry.fab_message[31]) > 0x223) << 6      // -X sun / no sun
        | (make16(telemetry.fab_message[26], telemetry.fab_message[27]) > 0x2E8) << 5      // +Y sun / no sun
        | (make16(telemetry.fab_message[32], telemetry.fab_message[33]) > 0x03E) << 4      // -Y sun / no sun
        | (make16(telemetry.fab_message[36], telemetry.fab_message[37]) > 0x081) << 3      // +Z sun / no sun
        | (make16(telemetry.fab_message[34], telemetry.fab_message[35]) > 0x065) << 2      // -Z sun / no sun
        | (boot_flags.deployment_flag >= 5) << 1                                           // OBC mode
        | ((time_after_reset_ >> 4) & 0x1);                                                // Time after reset bit 5
    cw[0][5] = (time_after_reset_ & 0x0F) << 4                                             // Time after reset bits 0-4
        | (telemetry.fab_message[52] & 0x1) << 3                                           // Battery heater on/off
        | (telemetry.fab_message[53] & 0x1) << 2                                           // Kill switch Main PIC
        | ((telemetry.fab_message[53] & 0x10) >> 4) << 1                                   // Kill switch EPS PIC
        | 0x0;                                                                             // Format identifier

    // Page 1
    cw[1][0] = gyro_to_cw(telemetry.adcs_message[1], telemetry.adcs_message[2]); // Gyro X axis (deg/s), 16 -> 8 bits (-128 to 127 with LSB = 1deg/s)
    cw[1][1] = gyro_to_cw(telemetry.adcs_message[3], telemetry.adcs_message[4]); // Gyro Y axis (deg/s), 16 -> 8 bits (-128 to 127 with LSB = 1deg/s)
    cw[1][2] = gyro_to_cw(telemetry.adcs_message[5], telemetry.adcs_message[6]); // Gyro Z axis (deg/s), 16 -> 8 bits (-128 to 127 with LSB = 1deg/s)
    cw[1][3] = ((telemetry.adcs_message[37] >> 2) & 0x1) << 7                    // Magnetometer X axis sign bit
        | ((telemetry.adcs_message[37] >> 1) & 0x1) << 6                         // Magnetometer Y axis sign bit
        | (telemetry.adcs_message[37] & 0x1) << 5                                // Magnetometer Z axis sign bit
        | (adcs_mode < 8 ? adcs_mode : 7) << 2                                   // ADCS mode
        | ((sc < 4 ? sc : 3) & 0x03);                                            // No. of scheduled commands

    cw[1][4] = ((telemetry_time.reset_time > 0)
                   + (telemetry_time.fab_time > 0)
                   + (telemetry_time.pcib_time > 0)
                   + (telemetry_time.adcs_time > 0))
            << 5 // Number of subsystems communicating with OBC
        | 0;     // 5 bits free for assignment here

    cw[1][5] = 0 // 7 bits free for assignment here
        | 0x1;   // Format identifier

    fprintf(PC, "CW: 0x");
    uart_print_pc_hex_short(cw[0], sizeof(cw[0]));
    fprintf(PC, " 0x");
    uart_print_pc_hex_short(cw[1], sizeof(cw[1]));
    fputc(' ', PC);
}

// Helper function to save state to flash
void save_state(uint8_t current_command)
{
    get_com_shared_fm_access();

    // Save state of obc_flags:
    flash_erase_pages(&spi_port_COM_FM, OBC_FLAGS_ADDRESS, OBC_FLAGS_ADDRESS + sizeof(obc_flags));
    uint8_t* obc_flag_ptr = (uint8_t*)&obc_flags;
    flash_transfer_data_from_ram(
        &spi_port_COM_FM,
        OBC_FLAGS_ADDRESS,
        obc_flag_ptr,
        sizeof(obc_flags));

    // Disable the current command before saving
    for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
        if (scheduled_commands[i].command[0] == MSG_COMM && scheduled_commands[i].command[1] == current_command && scheduled_commands[i].time <= current_time) {
            scheduled_commands[i].time = TIME_T_MAX; // Disable the command from executing again (== reschedule it at infinity).
        }
    }

    // Save state of scheduled commands:
    flash_erase_pages(&spi_port_COM_FM, SCHEDULED_CMD_ADDRESS, SCHEDULED_CMD_ADDRESS + sizeof(scheduled_commands));
    uint8_t* cmd_ptr = (uint8_t*)scheduled_commands;
    flash_transfer_data_from_ram(&spi_port_COM_FM, SCHEDULED_CMD_ADDRESS, cmd_ptr, sizeof(scheduled_commands));
}

// ============ Commands for Telemetry request ============

// Request for reset telemetry
uint8_t command_request_reset(uint8_t* data)
{
    uart_clean(RST);
    uart_mux = 1;
    const uint8_t cmd[36] = {
        0xB0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB1
    };
    for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }
    return 0;
}

// Request for eps telemetry
uint8_t command_request_eps(uint8_t* data)
{
    uart_clean(FAB);
    uart_mux = 1;

    uint8_t i;
    enum { cmd_size = 6 };

    uint8_t cmd[cmd_size] = { 0 };

    cmd[0] = 0xE0;
    cmd[1] = 0x61;
    *(uint32_t*)&cmd[2] = current_time;

    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], FAB);
    }

    return 0;
}

// Request for pcib (ADB) telemetry
uint8_t command_request_pcib(uint8_t* data)
{
    struct request {
        uint8_t origin;
        uint8_t cmd;
        time_t obc_time;
        uint8_t data[MSG_LENGTH_PCIB - 8];
        uint8_t checksum;
        uint8_t footer;
    } request;

    uint8_t* cmd = (uint8_t*)&request;
    memset(cmd, 0, sizeof(request));

    request.origin = 0x0B;
    request.cmd = 0xAA;
    request.obc_time = current_time;

    uart_mux = 1;
    checksum_obc(cmd, sizeof(request));
    for (uint8_t i = 0; i < sizeof(request); i++) {
        fputc(cmd[i], PCIB);
    }
    return 0;
}

// request for adcs telemetry
uint8_t command_request_adcs(uint8_t* data)
{
    uart_clean(ADCS);
    uart_mux = 0;
    uint8_t adcs_command[MSG_LENGTH_ADCS] = { 0x0B };
    adcs_command[1] = 0xAB;
    *(time_t*)&adcs_command[2] = current_time;
    checksum_obc(adcs_command, sizeof(adcs_command));
    for (uint8_t i = 0; i < sizeof(adcs_command); i++) {
        fputc(adcs_command[i], ADCS);
    }
    // fprintf(PC, "ADCS REQ: ");
    // uart_print_pc_hex(adcs_command, MSG_LENGTH_ADCS);
    // fprintf(PC, "\r\n");
    return 0;
}

// ============ ADCS Commands ============

enum { // updated on 2023/05/01
    adcs_mode_tumbling = 0,
    adcs_mode_detumbling = 1,
    adcs_mode_detumble_sun_tracking_auto = 2,
    adcs_mode_sun_tracking = 3,
    adcs_mode_nadir_sband = 4,
    adcs_mode_nadir_camera = 5,
    adcs_mode_target = 6,
    adcs_mode_sun_tracking_quat = 7,
    adcs_mode_target_camera = 8,
    adcs_mode_horizon_camera = 9,
    adcs_mode_nadir_sband_pz = 10,
    adcs_mode_nadir_camera_pz = 11
};

// Helper function to send STM32 commands (up to 32-bytes)
void stm32_raw_command(uint8_t* data, uint8_t length, uint8_t tle)
{
    uint8_t adcs_command[MSG_LENGTH_ADCS] = { 0x0B, 0x06 };
    if (tle) {
        adcs_command[1] = 0x07;
    }
    memcpy(adcs_command + 2, data, length);
    checksum_obc(adcs_command, sizeof(adcs_command));
    for (uint8_t i = 0; i < sizeof(adcs_command); i++) {
        fputc(adcs_command[i], ADCS);
    }
    fprintf(PC, "STM RAW CMD: ");
    uart_print_pc_hex(adcs_command, sizeof(adcs_command));
}

// Change ADCS mode internally and externally
uint8_t command_adcs_mode(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t mode;
        uint8_t permanent; // when true, change mode permanently
    }* packet = (struct packet*)data;
    uint8_t adcs_command[3] = { 0x01 };
    adcs_mode = packet->mode;
    adcs_command[1] = packet->mode;
    adcs_command[2] = packet->permanent;
    stm32_raw_command(adcs_command, sizeof(adcs_command), 0);
    return packet->mode;
}

// Change ADCS default mode
uint8_t command_adcs_default_mode(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t new_mode;
    }* packet = (struct packet*)data;

    // Change stored vale in OBC
    obc_flags.adcs_initial_value = packet->new_mode;
    adcs_mode = obc_flags.adcs_initial_value;
    save_state(packet->command);

    // Change stored value in ADCS
    struct adcs_mode_st {
        uint8_t origin;
        uint8_t command;
        uint8_t mode;
        uint8_t permanent;
    } adcs_mode_st;
    adcs_mode_st.origin = MSG_COMM;
    adcs_mode_st.command = 0xAD;
    adcs_mode_st.mode = packet->new_mode;
    adcs_mode_st.permanent = true;
    vschedule(current_time, (uint8_t*)&adcs_mode_st); // ADCS change to selected mode

    return adcs_mode;
}

// Schedule ADCS mode (without coordinates)
uint8_t command_schedule_mode(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t adcs_mode;
        time_t mode_scheduled_time;
        uint16_t total_duration;
    }* packet = (struct packet*)data;

    if (packet->mode_scheduled_time == 0) { // debug case
        packet->mode_scheduled_time = current_time;
    }

    struct adcs_mode_st {
        uint8_t origin;
        uint8_t command;
        uint8_t mode;
        uint8_t permanent;
    } adcs_mode_st;

    adcs_mode_st.origin = MSG_COMM;
    adcs_mode_st.command = 0xAD;
    adcs_mode_st.mode = packet->adcs_mode;
    adcs_mode_st.permanent = false;
    vschedule(packet->mode_scheduled_time, (uint8_t*)&adcs_mode_st); // ADCS change to selected mode

    adcs_mode_st.mode = obc_flags.adcs_initial_value;
    vschedule(packet->mode_scheduled_time + packet->total_duration, (uint8_t*)&adcs_mode_st); // ADCS back to default mode

    return 0;
}

// Print the satellite flags to the debug line
uint8_t command_print_flags(uint8_t* data)
{
    print_flags();
    return 0;
}

// Send a raw command to ADCS
uint8_t command_adcs_raw(uint8_t* data)
{
    enum { adcs_cmd_size = 7 };

    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t adcs_command[adcs_cmd_size];
    }* packet = (struct packet*)data;

    uint8_t msg[MSG_LENGTH_ADCS] = { 0 };
    msg[0] = 0x0B;
    memcpy(msg + 1, packet->adcs_command, adcs_cmd_size);
    msg[MSG_LENGTH_ADCS - 1] = 0x0C;

    uart_print_pc_hex(msg, sizeof(msg));

    for (uint8_t i = 0; i < sizeof(msg); i++) {
        fputc(msg[i], ADCS);
    }

    uart_mux = 0;

    return 0;
}

uint8_t command_stm32_raw_8_16(uint8_t* data)
{
    enum { length = 16 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part[length];
    }* packet = (struct packet*)data;

    memset(stm32_command_uhf, 0, STM32_SIZE);
    memcpy(stm32_command_uhf, packet->part, length);
    stm32_raw_command(stm32_command_uhf, STM32_SIZE, 0);
    memset(stm32_command_uhf, 0, STM32_SIZE);

    fprintf(PC, "STM32 command (8/16)");

    return 0;
}

uint8_t command_stm32_raw_uhf32(uint8_t* data)
{
    enum { length = 16 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part[length];
    }* packet = (struct packet*)data;

    uint8_t part = ((packet->command & 0x0F) - 1);

    memcpy(stm32_command_uhf + length * part, packet->part, length);

    fprintf(PC, "STM32 command (32) part %d. ", part + 1);

    if (part == 1) {
        stm32_raw_command(stm32_command_uhf, STM32_SIZE, 0);
        memset(stm32_command_uhf, 0, STM32_SIZE);
    }

    return 0;
}

uint8_t command_stm32_raw_uhf32_tle(uint8_t* data)
{
    enum { length = 16 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part[length];
    }* packet = (struct packet*)data;

    uint8_t part = ((packet->command & 0x0F) - 1);

    memcpy(stm32_command_uhf + length * part, packet->part, length);

    fprintf(PC, "STM32 command (32) part %d. ", part + 1);

    if (part == 1) {
        stm32_raw_command(stm32_command_uhf, STM32_SIZE, true);
        memset(stm32_command_uhf, 0, STM32_SIZE);
    }

    return 0;
}

// Receive ADCS telemetry
uint8_t command_adcs_telemetry(uint8_t* data)
{
    response_rx = 1; // Received a reply
    uart_mux = 1;

    fprintf(PC, "ADCS: ");
    fprintf(PC, "Mode=%X | ", data[2]);
    uart_print_pc_hex(data, MSG_LENGTH_ADCS);

    telemetry_time.adcs_time = current_time;
    memcpy(telemetry.adcs_message, data + 2, sizeof(telemetry.adcs_message));

    return 0;
}

// Receive GPS time from ADCS
uint8_t command_adcs_gps_time(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t day;
        uint8_t month;
        uint8_t year;
    }* packet = (struct packet*)data;

    uint8_t valid_time = 0;

    uart_print_pc_hex(data, MSG_LENGTH_ADCS);

    if (packet->year == 0x00 || packet->year >= 38) {
        valid_time = 2;
    } else if (obc_flags.gps_time_sync_state == 1 && boot_flags.deployment_flag >= 5) {
        valid_time = 1;
        struct_tm gps_time;
        gps_time.tm_year = packet->year + 100;
        gps_time.tm_mon = packet->month - 1;
        gps_time.tm_mday = packet->day;
        gps_time.tm_hour = packet->hour;
        gps_time.tm_min = packet->minute;
        gps_time.tm_sec = packet->second + obc_flags.leap_seconds;

        time_t unix_time = mktime(&gps_time);

        SetTimeSec(unix_time + 1);
        current_time = time(0);
        reset_pic_update_clock(current_time);
        // Change memory location based on the new date
        flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag, addr_flags.flash_telemetry.current, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, true, &addr_flags.flash_telemetry);
        fprintf(PC, "\r\nNew time: %04ld/%02d/%02d %02d:%02d:%02d", gps_time.tm_year + 1900,
            (char)gps_time.tm_mon + 1,
            gps_time.tm_mday,
            gps_time.tm_hour,
            gps_time.tm_min,
            gps_time.tm_sec);
    }
    return valid_time;
}

// ADCS copy high sampling data
uint8_t command_adcs_hs_copy(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t slots_30m;
    }* packet = (struct packet*)data;

    uint32_t source_address = (1831UL + 6.23 * (uint32_t)packet->slots_30m) * MEMORY_SECTOR_SIZE;
    uint32_t destination_address = FLASH_ADCS_HS_START;
    const uint32_t n_packets = 6.23 * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

    fprintf(PC, "\r\n");
    fprintf(PC, "Copy ADCS HS data to Main PIC\r\n");
    fprintf(PC, "Source address      = 0x%8lX\r\n", source_address);
    fprintf(PC, "Destination address = 0x%8lX\r\n", destination_address);
    fprintf(PC, "Size                = %lu\r\n", n_packets);

    struct xmodem_command {
        uint8_t origin;  // C0
        uint8_t command; // D1
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    } xmodem_command;
    xmodem_command.origin = 0xC0;
    xmodem_command.command = 0xD1;
    xmodem_command.destination_address = destination_address;
    xmodem_command.source = 3;      // ADCS
    xmodem_command.destination = 0; // 0: COM Shared FM
    xmodem_command.source_address = source_address;
    xmodem_command.n_packets = n_packets;

    uart_print_pc_hex((uint8_t*)&xmodem_command, sizeof(xmodem_command));
    vschedule(current_time + 1, (uint8_t*)&xmodem_command);
    return 0;
}

// ADCS copy GPS data
uint8_t command_adcs_gps_copy(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t slots_1h;
    }* packet = (struct packet*)data;

    uint32_t source_address = (1987UL + 5UL * (uint32_t)packet->slots_1h) * MEMORY_SECTOR_SIZE;
    uint32_t destination_address = FLASH_ADCS_GPS_START;
    const uint32_t n_packets = 5UL * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

    fprintf(PC, "\r\n");
    fprintf(PC, "Copy GPS data to Main PIC\r\n");
    fprintf(PC, "1h slots            = %d\r\n", packet->slots_1h);
    fprintf(PC, "Source address      = 0x%8lX\r\n", source_address);
    fprintf(PC, "Destination address = 0x%8lX\r\n", destination_address);
    fprintf(PC, "Size                = %lu\r\n", n_packets);

    struct xmodem_command {
        uint8_t origin;  // C0
        uint8_t command; // D1
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    } xmodem_command;
    xmodem_command.origin = 0xC0;
    xmodem_command.command = 0xD1;
    xmodem_command.destination_address = destination_address;
    xmodem_command.source = 3;      // ADCS
    xmodem_command.destination = 0; // 0: COM Shared FM
    xmodem_command.source_address = source_address;
    xmodem_command.n_packets = n_packets;

    uart_print_pc_hex((uint8_t*)&xmodem_command, sizeof(xmodem_command));
    vschedule(current_time + 1, (uint8_t*)&xmodem_command);
    return 0;
}

// Copy ADCS data to COM shared FM
uint8_t command_copy_adcs_data_to_uhf(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t day_of_the_year;
        uint8_t slot;
    }* packet = (struct packet*)data;

    const uint8_t sectors_per_day = 5;
    uint32_t source_address = (1UL + (uint32_t)packet->day_of_the_year * (uint32_t)sectors_per_day) * MEMORY_SECTOR_SIZE;
    uint32_t destination_address = FLASH_ADCS_TELEMETRY_START + (uint32_t)packet->slot * (uint32_t)sectors_per_day * MEMORY_SECTOR_SIZE;
    uint32_t n_packets = 5UL * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

    fprintf(PC, "\r\n");
    fprintf(PC, "Copy ADCS data to Main PIC\r\n");
    fprintf(PC, "Day of the year     = %lu\r\n", packet->day_of_the_year + 1);
    fprintf(PC, "Source address      = 0x%8lX\r\n", source_address);
    fprintf(PC, "Destination address = 0x%8lX\r\n", destination_address);
    fprintf(PC, "Size                = %lu\r\n", n_packets);

    struct xmodem_command {
        uint8_t origin;  // C0
        uint8_t command; // D1
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    } xmodem_command;
    xmodem_command.origin = 0xC0;
    xmodem_command.command = 0xD1;
    xmodem_command.destination_address = destination_address;
    xmodem_command.source = 3;      // ADCS
    xmodem_command.destination = 0; // 0: COM Shared FM
    xmodem_command.source_address = source_address;
    xmodem_command.n_packets = n_packets;

    uart_print_pc_hex((uint8_t*)&xmodem_command, sizeof(xmodem_command));
    vschedule(current_time + 1, (uint8_t*)&xmodem_command);

    return 0;
}

// ============ COM Commands ============

// Message from UHF
uint8_t command_uhf_message(uint8_t* data)
{
    enum { length = 22 };
    struct packet {
        uint8_t origin;
        uint8_t gs_message[length];
    }* packet = (struct packet*)data;

    static uint8_t last_upload[length] = { 0 }; // Stores the last uploaded command
    static time_t last_upload_t = 0;

    fprintf(PC, "Uplink: ");
    uart_print_pc_hex(packet->gs_message, length);
    uint8_t valid = uplink_valid(packet->gs_message);
    uint8_t different = memcmp(last_upload, packet->gs_message, length) || (last_upload_t <= (current_time - 4 * 60)); // Commands are different within 4 min window
    memcpy(last_upload, packet->gs_message, length);
    last_upload_t = current_time;
    if (valid && different) {
        fprintf(PC, " Valid uplink");
        uint8_t cmd[BUFF_LENGTH] = { MSG_COMM };
        uint8_t copy_size = packet->gs_message[2] != 0xCC ? 9 : length; // Don't pass CRC onwards for short commands
        memcpy(cmd + 1, packet->gs_message + 3, copy_size);
        vschedule(current_time, cmd);
    } else {
        fprintf(PC, " Invalid (%d) or identical (%d) uplink.", valid, different);
    }
    return !(valid && different);
}

// Change CW mode flags
uint8_t command_change_cw_mode_flags(uint8_t* data)
{
    obc_flags.cw_mode = data[2]; // data[2] is the new flag status
    save_state(data[1]);         // data[1] is the current command ID
    return obc_flags.cw_mode;
}

// Ask ADCS if satellite is over Japan
uint8_t over_japan_check()
{
    uint8_t command[] = { 0xFC };
    stm32_raw_command(command, sizeof(command), false); // Send request to ADCS

    uint8_t response[2] = { 0 };
    uart_download_packet(&uart_port_ADCS, response, sizeof(response), 100000); // Try to get a response

    if (response[1] == 0xA5) {
        return ((int8_t)response[0] > 0); // Elevation > 0 degrees
    } else {
        return false;
    }
}

// Decides if it is OK to send CW or not based on flags and ADCS
uint8_t ok_to_send_cw()
{
    switch (obc_flags.cw_mode) {
    case 0: return false;
    case 1: return over_japan_check();
    case 2: return true;
    default: return false;
    }
}

// COM CW request
uint8_t command_com_cw(uint8_t* data)
{
    enum { com_to_main_size = 25 };

    static uint8_t current_cw = 0;
    telemetry.com_rssi = *(uint16_t*)&data[2];
    telemetry_time.com_time = current_time;

    // Generate CW reply
    uint8_t cmd[24] = { 0 };
    cmd[0] = 0x0B;
    cmd[1] = 0x50;
    memcpy(cmd + 2, cw[current_cw], sizeof(cw[0]));
    cmd[sizeof(cw[0]) + 2] = ok_to_send_cw();
    cmd[23] = 0x0C;
    for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], COMM);
    }
    current_cw++;
    if (current_cw >= sizeof(cw) / sizeof(cw[0]))
        current_cw = 0;
    fprintf(PC, "COM: ");
    uart_print_pc_hex(data, com_to_main_size);
    return 0;
}

// COM Access request
uint8_t command_com_access_request(uint8_t* data)
{
    struct packet {
        uint8_t origin;  // 0xC0
        uint8_t command; // 0x59
        uint8_t time;    // in minutes
    }* packet = (struct packet*)data;

    time_t disable_time = current_time + (60 * (time_t)packet->time);
    scheduled_command_clear_specified_command(0xC0, 0x58);
    schedule(current_time, { 0xC0, 0x58, 0x01 });
    schedule(disable_time, { 0xC0, 0x58, 0x00 });

    uint8_t reply[8] = { 0 };
    send_com_ack(reply);

    return packet->time;
}

// Change COM access
uint8_t command_com_access_change(uint8_t* data)
{
    struct packet {
        uint8_t origin;  // 0xC0
        uint8_t command; // 0x58
        uint8_t state;   // 0: OBC side; 1: COM side
    }* packet = (struct packet*)data;

    output_bit(MUX_SEL_COM_SHARED_FM, packet->state);
    memory_busy = packet->state;

    return packet->state;
}

// ============ OBC/PCIB Commands ============

// Schedule any command
uint8_t command_schedule_anything(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        time_t schedule_time;
        uint8_t schedule_command[12];
    }* packet = (struct packet*)data;

    vschedule(packet->schedule_time, packet->schedule_command);

    return 0;
}

uint8_t command_pcib_telemetry(uint8_t* data)
{
    fprintf(PC, "RELAY: ");
    response_rx = 1; // Received a reply

    uart_print_pc_hex(data, MSG_LENGTH_PCIB);

    telemetry_time.pcib_time = current_time;
    memcpy(telemetry.pcib_message, data + 2, sizeof(telemetry.pcib_message));

    return 0;
}

// Set the clock to a given value (UNIX time, seconds after Jan 1st 1970)
uint8_t command_set_clock(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        time_t current_time;
        int8_t leap_seconds;
        uint8_t gps_time_sync_state;
    }* packet = (struct packet*)data;
    obc_flags.leap_seconds = packet->leap_seconds;
    obc_flags.gps_time_sync_state = packet->gps_time_sync_state;

    uint8_t time_updated = 0;

    if (packet->current_time < T0) {
        time_updated = 1;
        SetTimeSec(T0); // Set clock to as early as possible (Jan 1st 2000, 00:00:00);
        // mai_400_update_clock(T0);
        reset_pic_update_clock(T0);
    } else {
        SetTimeSec(packet->current_time + obc_flags.leap_seconds);
        reset_pic_update_clock(packet->current_time + obc_flags.leap_seconds);
    }
    current_time = time(0);

    // Change memory location based on the new date
    flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag, addr_flags.flash_telemetry.current, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, true, &addr_flags.flash_telemetry);

    struct_tm* local_time = localtime(&current_time);

    fprintf(PC, "New time: %04ld/%02d/%02d %02d:%02d:%02d(0x%08lX) ",
        local_time->tm_year + 1900,
        (uint8_t)local_time->tm_mon + 1,
        local_time->tm_mday,
        local_time->tm_hour,
        local_time->tm_min,
        local_time->tm_sec,
        current_time);

    fprintf(PC, "Leap: %d GPS sync: 0x%02X", obc_flags.leap_seconds, obc_flags.gps_time_sync_state);
    return time_updated;
}

// Display the TRIS status to debug port
uint8_t command_get_tris(uint8_t* data)
{
    fprintf(PC, "\n         fedcba9876543210\r\n");
    fprintf(PC, "tris_a = ");
    print_binary16(get_tris_a());
    fprintf(PC, "tris_b = ");
    print_binary16(get_tris_b());
    fprintf(PC, "tris_c = ");
    print_binary16(get_tris_c());
    fprintf(PC, "tris_d = ");
    print_binary16(get_tris_d());
    fprintf(PC, "tris_e = ");
    print_binary16(get_tris_e());
    fprintf(PC, "tris_f = ");
    print_binary16(get_tris_f());
    fprintf(PC, "tris_g = ");
    print_binary16(get_tris_g());
    return 0xEE;
}

// Prints a memory address relative to satellite time (if ptr = 0x0000, prints satellite time).
uint8_t command_print_memory_address(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint32_t ptr;
    }* packet = (struct packet*)data;
#ifndef PC_SIM
    uint32_t* addr = (uint32_t*)packet->ptr + &current_time;
    fprintf(PC, "*%04lX = %08lX", addr, *addr);
#endif
    return 0;
}

// Sends the antenna deployment command to relay pic
uint8_t command_deploy_antenna(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t time_s;
    }* packet = (struct packet*)data;

    const uint8_t n_tries = 5; // Number of times it try to send PCIB the request

    if (current_time >= T_ANTENNA) { // Never try to deploy ahead of 31m mark
        // Increment deployment flag and save state
        boot_flags.deployment_flag++;
        get_com_shared_fm_access();
        uint8_t* boot_flag_ptr = (uint8_t*)&boot_flags;
        flash_erase_pages(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, BOOT_FLAGS_ADDRESS + sizeof(boot_flags));
        flash_transfer_data_from_ram(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, boot_flag_ptr, sizeof(boot_flags));

        uint8_t time = 30; // in seconds
        if (packet->time_s) {
            time = packet->time_s;
        }
        uint8_t cmd[36] = { 0x0B, 0xDA };
        cmd[2] = time;
        checksum_obc(cmd, sizeof(cmd));
        for (uint8_t j = 0; j < n_tries; j++) {
            for (uint8_t i = 0; i < sizeof(cmd); i++) {
                fputc(cmd[i], PCIB);
            }
            delay_ms(100);
        }
        return 0;
    } else {
        return 1;
    }
}

// Clear completely the main memory.
uint8_t command_clear_state(uint8_t* data)
{
    get_com_shared_fm_access();
    flash_erase(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, ERASE_SECTOR);
    fprintf(PC, "Waiting 10s for reset...\r\n");
    delay_ms(10000);
    reset_cpu();
    return 0;
}

// Save state to memory.
uint8_t command_save_state(uint8_t* data)
{
    get_com_shared_fm_access();
    save_state(data[1]); // data[1] is the current command id
    return 0;
}

// Dump state.
uint8_t command_dump_memory(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint32_t start;
        uint32_t size;
        uint8_t source;
    }* packet = (struct packet*)data;
    fprintf(PC, "\r\nHex dump start");
    switch (packet->source) {
    case 0:
        get_com_shared_fm_access();
        flash_dump(&spi_port_COM_FM, packet->start, packet->start + packet->size);
        break;
    case 1:
        flash_dump(&spi_port_MAIN_FM, packet->start, packet->start + packet->size);
        break;
    case 2:
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_dump(&spi_port_MISSION_FM, packet->start, packet->start + packet->size);
        output_high(MUX_SEL_MSN_SHARED_FM);
        break;
    default:
        get_com_shared_fm_access();
        flash_dump(&spi_port_COM_FM, packet->start, packet->start + packet->size);
        break;
    }
    fprintf(PC, "\r\nHex dump end\r\n");
    return packet->source;
}

// Copy data between flash memories
uint8_t command_copy_memory_sector(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_sector;
        uint16_t source_sector;
        uint16_t n_sectors;
        uint8_t destination_port; // 0:COM, 1:MAIN, 2:MISSION
        uint8_t origin_port;      // 0:COM, 1:MAIN, 2:MISSION
    }* packet = (struct packet*)data;

    uint8_t origin = packet->origin_port;
    uint8_t dest = packet->destination_port;

    uint32_t to_address_ = (uint32_t)packet->destination_sector * MEMORY_SECTOR_SIZE;
    uint32_t from_address_ = (uint32_t)packet->source_sector * MEMORY_SECTOR_SIZE;
    uint32_t size = (uint32_t)packet->n_sectors * MEMORY_SECTOR_SIZE;

    fprintf(PC, "memcpy orig=%d,dest=%d,to_addr=%lX,from_addr=%lX,size=%lX", origin, dest, to_address_, from_address_, size);

    get_com_shared_fm_access();

    // Erase the pages before copying
    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            flash_erase(&spi_port_COM_FM, to_address_ + i, ERASE_SECTOR);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            flash_erase(&spi_port_MAIN_FM, to_address_ + i, ERASE_SECTOR);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address_ + i, ERASE_SECTOR);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    if (origin == 0x00 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_COM_FM, to_address_, size);

    } else if (origin == 0x00 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);

    } else if (origin == 0x00 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x01 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_COM_FM, to_address_, size);

    } else if (origin == 0x01 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);

    } else if (origin == 0x01 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x00) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_COM_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x01) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else {
        return 1;
    }

    return 0;
}

// Copy data between flash memories
uint8_t command_copy_memory_page(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_page;
        uint16_t source_page;
        uint16_t n_pages;
        uint8_t destination_port; // 0:COM, 1:MAIN, 2:MISSION
        uint8_t origin_port;      // 0:COM, 1:MAIN, 2:MISSION
    }* packet = (struct packet*)data;

    uint8_t origin = packet->origin_port;
    uint8_t dest = packet->destination_port;

    uint32_t to_address_ = (uint32_t)packet->destination_page * MEMORY_PAGE_SIZE;
    uint32_t from_address_ = (uint32_t)packet->source_page * MEMORY_PAGE_SIZE;
    uint32_t size = (uint32_t)packet->n_pages * MEMORY_PAGE_SIZE;

    fprintf(PC, "memcpy orig=%d,dest=%d,to_addr=%lX,from_addr=%lX,size=%lX", origin, dest, to_address_, from_address_, size);

    get_com_shared_fm_access();

    // Erase the pages before copying
    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            flash_erase(&spi_port_COM_FM, to_address_ + i, ERASE_PAGE);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            flash_erase(&spi_port_MAIN_FM, to_address_ + i, ERASE_PAGE);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address_ + i, ERASE_PAGE);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    if (origin == 0x00 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_COM_FM, to_address_, size);

    } else if (origin == 0x00 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);

    } else if (origin == 0x00 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x01 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_COM_FM, to_address_, size);

    } else if (origin == 0x01 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);

    } else if (origin == 0x01 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x00) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_COM_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x01) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_MAIN_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address_, &spi_port_MISSION_FM, to_address_, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else {
        return 1;
    }

    return 0;
}

// Erase data from flash memory
uint8_t command_erase_memory_page(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_page;
        uint16_t n_pages;
        uint8_t destination_port; // 0:COM, 1:MAIN, 2:MISSION
    }* packet = (struct packet*)data;

    uint8_t dest = packet->destination_port;

    uint32_t to_address = (uint32_t)packet->destination_page * MEMORY_PAGE_SIZE;
    uint32_t size = (uint32_t)packet->n_pages * MEMORY_PAGE_SIZE;

    get_com_shared_fm_access();

    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            flash_erase(&spi_port_COM_FM, to_address + i, ERASE_PAGE);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            flash_erase(&spi_port_MAIN_FM, to_address + i, ERASE_PAGE);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += MEMORY_PAGE_SIZE) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address + i, ERASE_PAGE);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    return 0;
}

// Erase data from flash memory
uint8_t command_erase_memory_sector(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_sector;
        uint16_t n_sectors;
        uint8_t destination_port; // 0:COM, 1:MAIN, 2:MISSION
    }* packet = (struct packet*)data;

    uint8_t dest = packet->destination_port;

    uint32_t to_address = (uint32_t)packet->destination_sector * MEMORY_SECTOR_SIZE;
    uint32_t size = (uint32_t)packet->n_sectors * MEMORY_SECTOR_SIZE;

    get_com_shared_fm_access();

    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            flash_erase(&spi_port_COM_FM, to_address + i, ERASE_SECTOR);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            flash_erase(&spi_port_MAIN_FM, to_address + i, ERASE_SECTOR);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += MEMORY_SECTOR_SIZE) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address + i, ERASE_SECTOR);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    return 0;
}

// Send data through xmodem protocol
uint8_t command_xmodem_send(uint8_t* data)
{
    const uint8_t max_tries = 5;
    uint8_t current_try = 0;
    struct packet {
        uint8_t origin;  // C0
        uint8_t command; // D0
        uint32_t source_address;
        uint32_t n_packets;
        uint8_t source;
        uint8_t destination;
        uint32_t destination_address;
    }* packet = (struct packet*)data;
    int8_t error = 0;
    fprintf(PC, "Waiting for xmodem transfer...");

    uart_fn* destination_uart = NULL;

    while (current_try < max_tries) {
        switch (packet->destination) {
        case 0: destination_uart = &uart_port_PC; break;   // For PC
        case 1: destination_uart = &uart_port_PCIB; break; // For PCIB
        case 2: destination_uart = &uart_port_PCIB; break; // For Rpi
        default: destination_uart = &uart_port_PC; break;
        }

        if (packet->destination == 2) {                                                     // Rpi
            uint8_t message_pcib[MSG_LENGTH_PCIB] = { 0x0B, 0xCD };                         // Array with request for direct copy relay -> rpi = 0xCD
            checksum_obc(message_pcib, MSG_LENGTH_PCIB);                                    // Add checksum
            uart_send_packet_repeat(&uart_port_PCIB, message_pcib, MSG_LENGTH_PCIB, 5, 10); // Send request 5 times, 100ms spaced
            delay_ms(1000);
            fprintf(PCIB, "\nrx,%05lu\n", packet->destination_address);
        }

        if (packet->destination == 1) { // PCIB
            struct req {
                uint8_t origin;
                uint8_t command;
                uint32_t address;
                uint8_t source;
                uint8_t padding[MSG_LENGTH_PCIB - 9];
                uint8_t checksum;
                uint8_t footer;
            } req;
            req.origin = MSG_OBC;
            req.command = 0xD1; // X-modem receive;
            req.address = packet->destination_address;
            req.source = 1; // OBC
            checksum_obc((uint8_t*)&req, sizeof(req));
            uart_send_packet_repeat(&uart_port_PCIB, (uint8_t*)&req, sizeof(req), 5, 10);
        }

        switch (packet->source) {
        case 0:
            get_com_shared_fm_access();
            error = xmodem_send(destination_uart, &spi_port_COM_FM, packet->source_address, packet->n_packets);
            break;
        case 1:
            error = xmodem_send(destination_uart, &spi_port_MAIN_FM, packet->source_address, packet->n_packets);
            break;
        case 2:
            output_low(MUX_SEL_MSN_SHARED_FM);
            error = xmodem_send(destination_uart, &spi_port_MISSION_FM, packet->source_address, packet->n_packets);
            output_high(MUX_SEL_MSN_SHARED_FM);
            break;
        default:
            get_com_shared_fm_access();
            error = xmodem_send(destination_uart, &spi_port_COM_FM, packet->source_address, packet->n_packets);
            break;
        }
        current_try++;
        delay_ms(1000);
        if (error != -1)
            break;
    }
    return error;
}

// Receive data through xmodem protocol
uint8_t command_xmodem_receive(uint8_t* data)
{
    const uint8_t max_tries = 5;
    uint8_t current_try = 0;
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    }* packet = (struct packet*)data;
    int8_t total_packets = 0;
    fprintf(PC, "Receiving xmodem data...");

    uart_fn* source_uart = NULL;

    switch (packet->source) {
    case 0: source_uart = &uart_port_PC; break;
    case 1: source_uart = &uart_port_PCIB; break;
    case 3: source_uart = &uart_port_ADCS; break;
    default: source_uart = &uart_port_PC; break;
    }

    while (current_try < max_tries) {
        if (packet->source == 1) { // PCIB
            struct req_pcib {
                uint8_t origin;
                uint8_t command;
                uint32_t address;
                uint32_t n_packets;
                uint8_t destination;
                uint8_t padding[MSG_LENGTH_PCIB - 13];
                uint8_t checksum;
                uint8_t footer;
            } req_pcib;
            req_pcib.origin = MSG_OBC;
            req_pcib.command = 0xD0; // X-modem send;
            req_pcib.address = packet->source_address;
            req_pcib.n_packets = packet->n_packets;
            req_pcib.destination = 1; // OBC
            checksum_obc((uint8_t*)&req_pcib, sizeof(req_pcib));
            uart_send_packet_repeat(source_uart, (uint8_t*)&req_pcib, sizeof(req_pcib), 1, 10);
            delay_ms(100);
        }
        if (packet->source == 2 || packet->source == 3) {
            delay_ms(5000);
        }

        switch (packet->destination) {
        case 0:
            get_com_shared_fm_access();
            total_packets = xmodem_receive(source_uart, &spi_port_COM_FM, packet->destination_address);
            break;
        case 1:
            total_packets = xmodem_receive(source_uart, &spi_port_MAIN_FM, packet->destination_address);
            break;
        case 2:
            output_low(MUX_SEL_MSN_SHARED_FM);
            total_packets = xmodem_receive(source_uart, &spi_port_MISSION_FM, packet->destination_address);
            output_high(MUX_SEL_MSN_SHARED_FM);
            break;
        default:
            get_com_shared_fm_access();
            total_packets = xmodem_receive(source_uart, &spi_port_COM_FM, packet->destination_address);
            break;
        }
        current_try++;
        delay_ms(1000);
        if (total_packets != -1)
            break;
    }
    return total_packets;
}

uint8_t command_clear_all_schedule_commands(uint8_t* data)
{
    scheduled_command_clear_all();
    return 0;
}

uint8_t command_boot_flag_set(uint8_t* data)
{
    get_com_shared_fm_access();
    uint8_t value = data[2];
    boot_flags.deployment_flag = value;
    uint8_t* boot_flag_ptr = (uint8_t*)&boot_flags;
    flash_erase_pages(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, BOOT_FLAGS_ADDRESS + sizeof(boot_flags));
    flash_transfer_data_from_ram(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, boot_flag_ptr, sizeof(boot_flags));
    scheduled_command_clear_all();
    return value;
}

// Reset log command
uint8_t command_reset_log(uint8_t* data)
{
    fprintf(PC, "Main PIC reset detected.");
    return boot_flags.deployment_flag;
}

// Telemetry keeping function
uint8_t command_save_telemetry(uint8_t* data)
{
    uart_mux = 1;
    if (!memory_busy) {
        // Write the obc timestamp to telemetry and relative collection times
        telemetry.obc_time = current_time;
        telemetry.reset_time = current_time - telemetry_time.reset_time > 255 ? 255 : current_time - telemetry_time.reset_time;
        telemetry.fab_time = current_time - telemetry_time.fab_time > 255 ? 255 : current_time - telemetry_time.fab_time;
        telemetry.pcib_time = current_time - telemetry_time.pcib_time > 255 ? 255 : current_time - telemetry_time.pcib_time;
        telemetry.adcs_time = current_time - telemetry_time.adcs_time > 255 ? 255 : current_time - telemetry_time.adcs_time;
        telemetry.com_time = current_time - telemetry_time.com_time > 255 ? 255 : current_time - telemetry_time.com_time;

        // Save telemetry to flash
        uint8_t* telemetry_data = (uint8_t*)&telemetry;
        flash_cycle_write(&spi_port_COM_FM, telemetry_data, &addr_flags.flash_telemetry);
        fprintf(PC, "Saving telemetry data: ");
        fprintf(PC, "Addr: 0x%08lX ", addr_flags.flash_telemetry.current - sizeof(telemetry));

        build_cw();             // Prepare CW strings
        initialize_telemetry(); // Reset for next iteration

        // Save satellite log to flash
        log_flush();

        // Save addresses to flash
        struct addr {
            uint32_t flash_log_current;
            uint32_t flash_telemetry_current;
        } addr;

        uint8_t* addr_flag_ptr = (uint8_t*)&addr;

        addr.flash_log_current = addr_flags.flash_log.current;
        addr.flash_telemetry_current = addr_flags.flash_telemetry.current;

        flash_cycle_write(&spi_port_COM_FM, addr_flag_ptr, &addr_flags.flash_addr);
        return 0;
    } else {
        fprintf(PC, "Skipping saving telemetry data.");
        return 1;
    }
}

// Change OCP's default state
uint8_t command_ocp_state(uint8_t* data)
{
    struct packet {
        uint8_t origin;     // 0xC0
        uint8_t command;    // 0xAE
        uint8_t on_off;     // 0: off; 1: on
        uint8_t ocp_number; // 0: adcs; 1: pcib
    }* packet = (struct packet*)data;

    enum { ocp_adcs = 0,
        ocp_pcib = 1 };

    switch (packet->ocp_number) {
    case ocp_adcs:
        output_bit(OCP_EN_ADCS, packet->on_off);
        obc_flags.adcs_on_off = packet->on_off;
        return obc_flags.adcs_on_off;
        break;
    case ocp_pcib:
        output_bit(OCP_EN_PCIB, packet->on_off);
        obc_flags.pcib_on_off = packet->on_off;
        return obc_flags.pcib_on_off;
        break;
    }

    return -1;
}

// void boot_commands_clear_nth(uint8_t n)
uint8_t command_boot_cmd_clear_nth(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t n;
    }* packet = (struct packet*)data;
    boot_commands_clear_nth(packet->n);
    boot_commands_write();
    return 0;
}

// void boot_commands_clear_all()
uint8_t command_boot_cmd_clear_all(uint8_t* data)
{
    boot_commands_clear_all();
    boot_commands_write();
    return 0;
}

// Add a boot command
uint8_t command_boot_cmd_add(uint8_t* data)
{
    enum { length = MSG_LENGTH_COMM - 13 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        time_t delay_from_boot;
        uint8_t new_boot_cmd[length];
    }* packet = (struct packet*)data;

    boot_command bc;
    bc.time = packet->delay_from_boot;
    memcpy(bc.command, packet->new_boot_cmd, length);
    boot_commands_add(bc);
    boot_commands_write();

    return 0;
}

// ============ Reset Commands ============

// Reset pic telemetry data
uint8_t command_reset_telemetry(uint8_t* data)
{
    response_rx = 1; // Received a reply
    struct packet {
        uint8_t message[MSG_LENGTH_RST - 10];
        uint8_t padding[9]; // empty part
        uint8_t footer;
    }* packet = (struct packet*)data;

    fprintf(PC, "RESET: ");
    uart_print_pc_hex(data, MSG_LENGTH_RST);

    if (rst_clock_update) {
        rst_clock_updated = 1;
        rst_clock_update = 0;
        struct_tm rst_time;
        rst_time.tm_year = (unsigned long)packet->message[2] + 100;
        rst_time.tm_mon = (unsigned long)packet->message[3] - 1;
        rst_time.tm_mday = (unsigned long)packet->message[4];
        rst_time.tm_hour = (unsigned long)packet->message[5];
        rst_time.tm_min = (unsigned long)packet->message[6];
        rst_time.tm_sec = (unsigned long)packet->message[7] + 1;
        SetTime(&rst_time);
        time_t new_time = mktime(&rst_time);
        // mai_400_update_clock(new_time);
        current_time = new_time;
        previous_time = new_time;
        reset_time = new_time;
        // Read stored commands from memory
        get_com_shared_fm_access();
        uint8_t* cmd_ptr = (uint8_t*)scheduled_commands;
        flash_transfer_data_to_ram(
            &spi_port_COM_FM,
            SCHEDULED_CMD_ADDRESS,
            cmd_ptr,
            sizeof(scheduled_commands));
        // Remove scheduled commads that are scheduled to run in the past
        for(uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
            if(scheduled_commands[i].time < current_time + 1800L) {
                scheduled_commands[i].time = TIME_T_MAX;
                fprintf(PC, "\r\nWarning: a scheduled command (%02X %02X) in flash memory was scheduled to run in a past date/time and was marked as complete.\r\n", scheduled_commands[i].command[0], scheduled_commands[i].command[1]);
            }
        }
        // Read and schedule boot commands
        boot_commands_schedule();
        // Change memory location based on the new date
        flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag, addr_flags.flash_telemetry.current, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, true, &addr_flags.flash_telemetry);

        struct_tm* local_time = localtime(&current_time);
        fprintf(PC, " New time: %04ld/%02d/%02d %02d:%02d:%02d (0x%08lX)",
            local_time->tm_year + 1900,
            (uint8_t)local_time->tm_mon + 1,
            local_time->tm_mday,
            local_time->tm_hour,
            local_time->tm_min,
            local_time->tm_sec,
            current_time);
    }

    telemetry_time.reset_time = current_time;
    memcpy(telemetry.reset_message, packet->message + 2, sizeof(telemetry.reset_message));
    return rst_clock_updated;
}

// Warn that 24-hour reset is about to happen
uint8_t command_reset_warning(uint8_t* data)
{
    uint8_t i;

    // uint8_t adcs_cmd[] = { 0x22 };
    // mai_400_command(adcs_cmd, sizeof(adcs_cmd));

    save_state(data[1]); // data[1] is the current command id

    // Reply:
    const uint8_t cmd[36] = {
        0xB0, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB1
    };
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }
    uart_clean(RST);

    fprintf(PC, "Waiting for 24h reset...\r\n");
    delay_ms(60000);
    delay_ms(60000);
    fprintf(PC, "No 24h reset happened, doing soft reset instead.\r\n");
    reset_cpu();

    return 0;
}

// Reset pic acknowledge of time change
uint8_t command_time_change_ack(uint8_t* data)
{
    fprintf(PC, "Reset PIC time change ACK.");
    return 0;
}

// Passes a command from ground-station to reset pic
uint8_t command_send_data_to_reset(uint8_t* data)
{
    uint8_t i;
    struct packet {
        uint8_t command;     // C0: comm pic message
        uint8_t reset_cmd;   // F5: reset message
        uint8_t data[8];     // data to be passed to reset pic
        uint8_t silent_mode; // if equals to 1, do not print debug message
    }* packet = (struct packet*)data;
    uint8_t cmd[36] = { 0 };
    cmd[0] = 0xB0;
    for (i = 0; i < 8; i++) {
        cmd[i + 1] = packet->data[i];
    }
    cmd[35] = 0xB1;
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }
    if (packet->silent_mode != 1)
        uart_print_pc_hex(cmd, sizeof(cmd));
    return 0;
}

// ============ EPS Commands ============

uint8_t command_eps_telemetry(uint8_t* data)
{
    enum { cmd_len = 7 };
    response_rx = 1; // Received a reply
    fprintf(PC, "EPS: ");

    uint16_t voltage_hex = make16(data[46], data[47]);
    uint16_t current_hex = make16(data[50], data[51]);
    uint16_t temperature_hex = make16(data[52], data[53]);

    float voltage = voltage_hex * 3.3 * 3 / 4096;
    float current = (3835. * current_hex * 3.3 / 4096) - 6396.3;
    float temperature = 75 - temperature_hex * 3.256 * 30 / 4096;
    fprintf(PC, "V=%f, C=%f, T=%f | ", voltage, current, temperature);

    uint16_t heater_ref_temperature = make16(data[56], data[57]);
    uint16_t heater_ref_voltage = make16(data[58], data[59]);

    if ((heater_ref_temperature != obc_flags.heater_ref_temperature) || (heater_ref_voltage != obc_flags.heater_ref_voltage)) {
        // E0 66 TH TL VH VL
        fprintf(PC, " New T=0x%04lX, V=0x%04lX | ", obc_flags.heater_ref_temperature, obc_flags.heater_ref_voltage);
        uint8_t cmd_temp[cmd_len];
        cmd_temp[0] = 0xC0;                                      // COM_MSG
        cmd_temp[1] = 0x6C;                                      // OBC_CMD
        cmd_temp[2] = 0x66;                                      // EPS_CMD
        cmd_temp[3] = (obc_flags.heater_ref_temperature >> 8);   // TEMP_HIGH
        cmd_temp[4] = (obc_flags.heater_ref_temperature & 0xFF); // TEMP_LOW
        cmd_temp[5] = (obc_flags.heater_ref_voltage >> 8);       // V_HIGH
        cmd_temp[6] = (obc_flags.heater_ref_voltage & 0xFF);     // V_LOW
        vschedule(current_time + 5, cmd_temp);
    }

    uart_print_pc_hex(data, MSG_LENGTH_FAB);
    telemetry_time.fab_time = current_time;
    memcpy(telemetry.fab_message, data + 2, sizeof(telemetry.fab_message));
    return 0;
}

// Passes a command from ground-station to eps pic
uint8_t command_send_data_to_eps(uint8_t* data)
{
    uint8_t i;
    enum { data_size = 5 };
    struct packet {
        uint8_t command;         // C0: com pic message
        uint8_t eps_cmd;         // 6C: eps message
        uint8_t data[data_size]; // data to be passed to reset pic
    }* packet = (struct packet*)data;

    uint8_t cmd[data_size + 1] = { 0 };
    cmd[0] = 0xE0;
    for (i = 0; i < data_size; i++) {
        cmd[i + 1] = packet->data[i];
    }
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], FAB);
    }

    fprintf(PC, "EPS cmd: ");
    uart_print_pc_hex(cmd, sizeof(cmd));

    return 0;
}

uint8_t command_eps_set_heater_ref(uint8_t* data)
{
    struct packet {
        uint8_t command;   // C0: com pic message
        uint8_t eps_cmd;   // 6D: heater ref settings
        uint16_t temp_ref; // temperature reference
        uint16_t v_ref;    // voltage reference
    }* packet = (struct packet*)data;

    obc_flags.heater_ref_temperature = packet->temp_ref;
    obc_flags.heater_ref_voltage = packet->v_ref;
    save_state(packet->eps_cmd);

    fprintf(PC, "New T=0x%04lX, V=0x%04lX", obc_flags.heater_ref_temperature, obc_flags.heater_ref_voltage);

    return 0;
}

uint8_t command_obc_kill_on(uint8_t* data)
{
    uint8_t kill_sw_status = 0;
    if (data[2] == 0x55 && data[4] == 0x55 && data[6] == 0x55 && data[8] == 0x55 && data[3] == 0xAA && data[5] == 0xAA && data[7] == 0xAA && data[9] == 0xAA) {
        kill_sw_status = 1;
        output_high(PIN_A4);
    }
    return kill_sw_status;
}

uint8_t command_obc_kill_off(uint8_t* data)
{
    output_low(PIN_A4);
    return 0;
}

uint8_t command_raw_pcib(uint8_t* data)
{
    enum { cmd_size = 8 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t pcib_command[cmd_size];
    }* packet = (struct packet*)data;

    uint8_t pcib_cmd[MSG_LENGTH_PCIB] = { 0 };
    pcib_cmd[0] = 0x0B;
    memcpy(pcib_cmd + 1, packet->pcib_command, cmd_size);
    checksum_obc(pcib_cmd, MSG_LENGTH_PCIB);
    uart_send_packet_repeat(&uart_port_PCIB, pcib_cmd, MSG_LENGTH_PCIB, 5, 10);
    return packet->pcib_command[0];
}

// Used to stop the simulation
uint8_t command_debug(uint8_t* data)
{
#ifdef PC_SIM
    fprintf(PC, "Exiting simulation");
    continue_program = 0;
#else
    for (uint32_t i = 0; i < 65536; i++) {
        fprintf(PC, "%02X:%02X:%d\r\n", (uint8_t)(i >> 8), (uint8_t)(i & 0xFF), (int8_t)gyro_to_cw((uint8_t)(i >> 8), (uint8_t)(i & 0xFF)));
    }
#endif // !PC_SIM
    return 0;
}

// =====================================================

// Function that executes a command, by looking up on the table of available commands.
void command_execute(uint8_t* data, uint8_t origin, uint8_t log_enabled)
{
    uint8_t error = 0;

    if (origin == data[0] || origin == MSG_WILDCARD) {

        // COM reply
        if (data[0] == 0xC0) {
            if (data[1] == 0x42) {
                if (data[4] != 0x00) { // 0x00 is a request for memory dump
                    send_com_ack(data + 2);
                } else {
#ifndef PC_SIM
                    uint32_t* ptr = *(uint32_t*)&data[5];
                    send_com_ack(ptr + &current_time);
#endif
                }
            }
        }

        // Satellite log
        log_entry log;
        log.origin = data[0];
        log.command = data[1];

        struct_tm* local_time = localtime(&current_time);

        // Execute command
        if (log_enabled)
            fprintf(PC, "%04ld/%02d/%02d %02d:%02d:%02d | %02d | %02X %02X | ", local_time->tm_year + 1900,
                (uint8_t)local_time->tm_mon + 1,
                local_time->tm_mday,
                local_time->tm_hour,
                local_time->tm_min,
                local_time->tm_sec,
                scheduled_command_count(),
                data[0],
                data[1]);

        uint16_t command = make16(data[0], data[1]);

        // The table of available commands.
        switch (command) {
        case 0xABAA: error = command_pcib_telemetry(data); break;
        case 0xAD90: error = command_adcs_telemetry(data); break;
        case 0xADDA: error = command_adcs_gps_time(data); break;
        case 0xC000: error = command_print_memory_address(data); break;
        case 0xC001: error = command_set_clock(data); break;
        case 0xC00A: error = command_copy_memory_page(data); break;
        case 0xC00B: error = command_copy_memory_sector(data); break;
        case 0xC00C: error = command_erase_memory_page(data); break;
        case 0xC00D: error = command_erase_memory_sector(data); break;
        // - 0xC010: reserved for COM
        // - 0xC011: reserved for COM
        // - 0xC013: reserved for COM
        // - 0xC014: reserved for COM
        // - 0xC015: reserved for COM
        // - 0xC016: reserved for COM
        // - 0xC017: reserved for COM
        // - 0xC020: reserved for COM
        // - 0xC021: reserved for COM
        // - 0xC022: reserved for COM
        // - 0xC023: reserved for COM
        // - 0xC024: reserved for COM
        case 0xC025: error = command_boot_cmd_clear_nth(data); break;
        case 0xC026: error = command_boot_cmd_clear_all(data); break;
        // - 0xC027: reserved for COM
        case 0xC028: error = command_boot_cmd_add(data); break;
        // - 0xC030: reserved for COM
        // - 0xC035: reserved for COM
        // - 0xC040: reserved for COM
        // - 0xC041: reserved for COM
        case 0xC042: error = command_uhf_message(data); break;
        // - 0xC044: reserved for COM
        case 0xC050: error = command_com_cw(data); break;
        case 0xC055: error = command_save_state(data); break;
        case 0xC058: error = command_com_access_change(data); break;
        case 0xC059: error = command_com_access_request(data); break;
        case 0xC060: error = command_obc_kill_on(data); break;
        case 0xC061: error = command_obc_kill_off(data); break;
        case 0xC06C: error = command_send_data_to_eps(data); break;
        case 0xC06D: error = command_eps_set_heater_ref(data); break;
        case 0xC070: error = command_stm32_raw_8_16(data); break;
        case 0xC071: error = command_stm32_raw_uhf32(data); break;
        case 0xC072: error = command_stm32_raw_uhf32(data); break;
        case 0xC090: error = command_request_reset(data); break;
        case 0xC091: error = command_request_eps(data); break;
        case 0xC092: error = command_request_pcib(data); break;
        case 0xC093: error = command_request_adcs(data); break;
        case 0xC0A1: error = command_stm32_raw_uhf32_tle(data); break;
        case 0xC0A2: error = command_stm32_raw_uhf32_tle(data); break;
        case 0xC0A6: error = command_adcs_default_mode(data); break;
        case 0xC0A8: error = command_adcs_gps_copy(data); break;
        case 0xC0A9: error = command_adcs_hs_copy(data); break;
        case 0xC0AD: error = command_adcs_mode(data); break;
        case 0xC0AE: error = command_ocp_state(data); break;
        case 0xC0AF: error = command_adcs_raw(data); break;
        case 0xC0CC: error = command_raw_pcib(data); break;
        case 0xC0CD: error = command_change_cw_mode_flags(data); break;
        case 0xC0C5: error = command_clear_state(data); break;
        case 0xC0D0: error = command_xmodem_send(data); break;
        case 0xC0D1: error = command_xmodem_receive(data); break;
        case 0xC0DA: error = command_deploy_antenna(data); break;
        case 0xC0DB: error = command_debug(data); break;
        case 0xC0DD: error = command_dump_memory(data); break;
        case 0xC0DF: error = command_get_tris(data); break;
        case 0xC0E4: error = command_copy_adcs_data_to_uhf(data); break;
        case 0xC0F5: error = command_send_data_to_reset(data); break;
        case 0xC0F6: error = command_schedule_anything(data); break;
        case 0xC0F7: error = command_schedule_mode(data); break;
        case 0xC0F8: error = command_save_telemetry(data); break;
        case 0xC0F9: error = command_clear_all_schedule_commands(data); break;
        // - 0xC0FA: reserved for COM
        // - 0xC0FB: reserved for COM
        case 0xC0FC: error = command_print_flags(data); break;
        case 0xC0FE: error = command_boot_flag_set(data); break;
        case 0xE033: error = command_eps_telemetry(data); break;
        case 0xB0A0: error = command_reset_telemetry(data); break;
        case 0xB0A2: error = command_reset_warning(data); break;
        case 0xB070: error = command_time_change_ack(data); break;
        case 0xDBFF: error = command_reset_log(data); break;
        default: error = command_get_tris(data); break;
        }

        if (log_enabled) {
            fprintf(PC, " (Ret: %02X)", error);
        }

        // Satellite log
        log.time = current_time;
        log.return_value = error;

        if (log_enabled && command != 0xB0A0 && command != 0xE033 && command != 0xABAA && command != 0xAD90 && command != 0xC050 && command != 0xC0F8)
            log_add(log);

        if (log_enabled)
            fprintf(PC, "\r\n");
    }
}

#endif /* INTERPRETER_H */
