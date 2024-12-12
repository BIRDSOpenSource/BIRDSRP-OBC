#ifndef INTERPRETER_H
#define INTERPRETER_H

#include <definitions.h>
#include <scheduler.h>
#include <crc16.h>
#include <libuart_fn.h>
#include <flash_memory.h>
#include <stdint.h>
#include <xmodem.h>
#include <log_control.h>
#include <boot_command.h>
#include <math.h>

// Interpreter: The procedures here are concerned with interpreting
// received commands and executing the appropriate commands.

// Definition of commands. Should follow the prototype: "uint8_t command_name(uint8_t *data)"
// Return value = 0 indicates that the command was successful
// Return value > 0 indicates that there was an error

// ============ Helper functions ============

// Get access to shared FM
void get_com_shared_fm_access()
{
    if (memory_busy) {
        scheduled_command_clear_specified_command(0xC0, 0x58); // Disable scheduled command to regain access to memory in the future
        output_low(MUX_SEL_COM_SHARED_FM);                     // Regain access to memory now
        memory_busy = 0;                                       // Now memory is free
    }
}

uint8_t mux_lock_unlock(uint8_t mux_state, time_t time)
{
    if (time > 1800L) { // Maximum time that the mux can be reserved is 30 minutes
        fprintf(PC, "Warning: mux reservation time too long!");
        time = 1800L;
    }
    if (mux_state) {
        scheduled_command_clear_specified_command(0xC0, 0x02); // Disable scheduled command to regain access to memory in the future
        schedule(current_time + time, { 0xC0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 }); // Release the lock in the future
    }
    mux_lock = mux_state;
    return mux_state;
}

uint8_t command_mux_lock_unlock(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t mux_state; // 0: unlock, 1: lock
        time_t time;       // in seconds
    }* packet = (struct packet*)data;

    uart_print_pc_hex(data, 7);
    return mux_lock_unlock(packet->mux_state, packet->time);
}

uint8_t command_mux_sel_sfm(uint8_t* data)
{
    fprintf(PC, "Changed MUX to position %d", data[2]);
    return mux_sel(data[2]);
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
    float gyro = ((int16_t)make16(msb, lsb)) / 131.;
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

// Conversion for CW beacon in the range 0-15
uint8_t get_adcs_mode_index(uint8_t adcs_mode) {
    switch (adcs_mode) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
            return adcs_mode;  // Direct mapping for 0 to 4
        case 10:
        case 11:
        case 12:
        case 13:
            return adcs_mode - 5;  // Adjust for 10 to 13
        case 20:
        case 21:
            return adcs_mode - 11; // Adjust for 20 to 21
        case 25:
        case 26:
        case 27:
        case 28:
        case 29:
            return adcs_mode - 14; // Adjust for 25 to 28
        default:
            return 0; // Return 0 if the mode is out of range or not mapped
    }
}

// Helper function for data serialization
void set_bits(uint8_t *data, uint32_t value, uint16_t *bit_offset, uint8_t bits) {
    while (bits > 0) {
        uint8_t byte_offset = *bit_offset / 8;
        uint8_t bit_in_byte_offset = *bit_offset % 8;
        uint8_t bits_to_write = (bits > 8 - bit_in_byte_offset) ? 8 - bit_in_byte_offset : bits;

        data[byte_offset] |= ((value & ((1U << bits_to_write) - 1U)) << bit_in_byte_offset);

        value >>= bits_to_write;
        bits -= bits_to_write;
        *bit_offset += bits_to_write;
    }
}

// Helper function for data deserialization
uint32_t get_bits(uint8_t *data, uint16_t *bit_offset, uint8_t bits) {
    uint32_t value = 0;
    uint8_t value_offset = 0;

    while (bits > 0) {
        uint8_t byte_offset = *bit_offset / 8;
        uint8_t bit_in_byte_offset = *bit_offset % 8;
        uint8_t bits_to_read = (bits > 8 - bit_in_byte_offset) ? 8 - bit_in_byte_offset : bits;

        value |= ((uint32_t)(data[byte_offset] >> bit_in_byte_offset) & ((1U << bits_to_read) - 1U)) << value_offset;

        bits -= bits_to_read;
        *bit_offset += bits_to_read;
        value_offset += bits_to_read;
    }

    return value;
}

void deserialize_cw()
{
    uint16_t bit_offset = 0;

    // Page 0 Deserialization
    uint8_t battery_voltage = get_bits(cw[0], &bit_offset, 8);                 // Battery Voltage (RST RAW voltage)
    uint8_t battery_current = get_bits(cw[0], &bit_offset, 8);                 // Battery Current
    uint8_t battery_temperature = get_bits(cw[0], &bit_offset, 8);             // Battery Temperature
    uint8_t cpld_temperature = get_bits(cw[0], &bit_offset, 8);                // CPLD Temperature
    uint8_t solar_cell_sap_z_plus_y_undepl = get_bits(cw[0], &bit_offset, 1);  // Solar Cell SAP +Z (+Y undepl.)
    uint8_t solar_cell_y_plus = get_bits(cw[0], &bit_offset, 1);               // Solar Cell +Y
    uint8_t solar_cell_x_plus = get_bits(cw[0], &bit_offset, 1);               // Solar Cell +X
    uint8_t solar_cell_y_minus = get_bits(cw[0], &bit_offset, 1);              // Solar Cell -Y
    uint8_t solar_cell_sap_z_minus_y_undepl = get_bits(cw[0], &bit_offset, 1); // Solar Cell SAP +Z (-Y undepl.)
    uint8_t solar_cell_z_minus = get_bits(cw[0], &bit_offset, 1);              // Solar Cell -Z
    uint8_t scheduled_commands = get_bits(cw[0], &bit_offset, 2);              // Scheduled commands in memory
    uint8_t battery_heater_flag = get_bits(cw[0], &bit_offset, 1);             // Battery Heater Flag
    uint8_t kill_switch_main_pic = get_bits(cw[0], &bit_offset, 1);            // Kill Switch Main PIC
    uint8_t kill_switch_eps = get_bits(cw[0], &bit_offset, 1);                 // Kill Switch EPS
    uint8_t adcs_mode = get_bits(cw[0], &bit_offset, 4);                       // ADCS mode
    uint8_t format_identifier_0 = get_bits(cw[0], &bit_offset, 1);             // Format identifier

    fprintf(PC, "Page 0 Deserialized Data:\r\n");
    fprintf(PC, "Battery Voltage (RST RAW voltage): %u\r\n", battery_voltage);
    fprintf(PC, "Battery Current: %u\r\n", battery_current);
    fprintf(PC, "Battery Temperature: %u\r\n", battery_temperature);
    fprintf(PC, "CPLD Temperature: %u\r\n", cpld_temperature);
    fprintf(PC, "Solar Cell SAP +Z (+Y undepl.): %u\r\n", solar_cell_sap_z_plus_y_undepl);
    fprintf(PC, "Solar Cell +Y: %u\r\n", solar_cell_y_plus);
    fprintf(PC, "Solar Cell +X: %u\r\n", solar_cell_x_plus);
    fprintf(PC, "Solar Cell -Y: %u\r\n", solar_cell_y_minus);
    fprintf(PC, "Solar Cell SAP +Z (-Y undepl.): %u\r\n", solar_cell_sap_z_minus_y_undepl);
    fprintf(PC, "Solar Cell -Z: %u\r\n", solar_cell_z_minus);
    fprintf(PC, "Scheduled commands in memory: %u\r\n", scheduled_commands);
    fprintf(PC, "Battery Heater Flag: %u\r\n", battery_heater_flag);
    fprintf(PC, "Kill Switch Main PIC: %u\r\n", kill_switch_main_pic);
    fprintf(PC, "Kill Switch EPS: %u\r\n", kill_switch_eps);
    fprintf(PC, "ADCS mode: %u\r\n", adcs_mode);
    fprintf(PC, "Format identifier: %u\r\n", format_identifier_0);

    bit_offset = 0; // Reset bit offset for page 1

    // Page 1 Deserialization
    uint8_t gyro_x = get_bits(cw[1], &bit_offset, 8);                   // Gyro X axis
    uint8_t gyro_y_minus = get_bits(cw[1], &bit_offset, 8);             // Gyro -Y axis
    uint8_t gyro_z_minus = get_bits(cw[1], &bit_offset, 8);             // Gyro -Z axis
    uint8_t magnetometer_x = get_bits(cw[1], &bit_offset, 5);           // Magnetometer X
    uint8_t magnetometer_y = get_bits(cw[1], &bit_offset, 5);           // Magnetometer Y
    uint8_t magnetometer_z = get_bits(cw[1], &bit_offset, 5);           // Magnetometer Z
    uint8_t subsystems_communicating = get_bits(cw[1], &bit_offset, 3); // Subsystems communicating
    uint8_t time_after_reset = get_bits(cw[1], &bit_offset, 5);         // Time after last reset (hours)
    uint8_t format_identifier_1 = get_bits(cw[1], &bit_offset, 1);      // Format identifier

    fprintf(PC, "Page 1 Deserialized Data:\r\n");
    fprintf(PC, "Gyro X axis: %u\r\n", gyro_x);
    fprintf(PC, "Gyro -Y axis: %u\r\n", gyro_y_minus);
    fprintf(PC, "Gyro -Z axis: %u\r\n", gyro_z_minus);
    fprintf(PC, "Magnetometer X: %u\r\n", magnetometer_x);
    fprintf(PC, "Magnetometer Y: %u\r\n", magnetometer_y);
    fprintf(PC, "Magnetometer Z: %u\r\n", magnetometer_z);
    fprintf(PC, "Subsystems communicating: %u\r\n", subsystems_communicating);
    fprintf(PC, "Time after last reset (hours): %u\r\n", time_after_reset);
    fprintf(PC, "Format identifier: %u\r\n", format_identifier_1);
}

// Helper funtion to initialize the cw beacon array.
void build_cw()
{
    uint8_t sc = scheduled_command_count();

    time_t time_after_reset_ = (current_time - reset_time) / 3600;
    uint8_t time_after_reset = time_after_reset_ > 24 ? 0x1F : time_after_reset_;

    uint8_t subsystems_communicating = 0;
    time_t* t = (time_t*)&telemetry_time;
    for (uint8_t i = 0; i < sizeof(telemetry_time) / sizeof(time_t); i++) {
        subsystems_communicating += (t[i] > 0);
    }
    subsystems_communicating -= (telemetry_time.com_time > 0); // Remove COM pic from the report

    memset(&cw, 0, sizeof(cw)); // Erase old data.

    uint16_t bit_offset = 0;

    // Page 0
    set_bits(cw[0], make16(telemetry.reset_message[6], telemetry.reset_message[7]) >> 4, &bit_offset, 8);  // Battery Voltage (RST RAW voltage)
    set_bits(cw[0], make16(telemetry.fab_message[52], telemetry.fab_message[43]) >> 4, &bit_offset, 8);    // Battery Current
    set_bits(cw[0], make16(telemetry.fab_message[54], telemetry.fab_message[55]) >> 4, &bit_offset, 8);    // Battery Temperature
    set_bits(cw[0], make16(telemetry.fab_message[10], telemetry.fab_message[11]) >> 4, &bit_offset, 8);    // CPLD Temperature
    set_bits(cw[0], make16(telemetry.fab_message[30], telemetry.fab_message[31]) > 0x2F0, &bit_offset, 1); // Solar Cell SAP +Z (+Y undepl.)
    set_bits(cw[0], make16(telemetry.fab_message[32], telemetry.fab_message[33]) > 0x2F0, &bit_offset, 1); // Solar Cell +Y
    set_bits(cw[0], make16(telemetry.fab_message[34], telemetry.fab_message[35]) > 0x2F0, &bit_offset, 1); // Solar Cell +X
    set_bits(cw[0], make16(telemetry.fab_message[36], telemetry.fab_message[37]) > 0x2F0, &bit_offset, 1); // Solar Cell -Y
    set_bits(cw[0], make16(telemetry.fab_message[38], telemetry.fab_message[39]) > 0x2F0, &bit_offset, 1); // Solar Cell SAP +Z (-Y undepl.)
    set_bits(cw[0], make16(telemetry.fab_message[40], telemetry.fab_message[41]) > 0x090, &bit_offset, 1); // Solar Cell -Z
    set_bits(cw[0], sc > 3 ? 3 : sc, &bit_offset, 2);                                                      // Scheduled commands in memory
    set_bits(cw[0], telemetry.fab_message[56] > 0, &bit_offset, 1);                                        // Battery Heater Flag
    set_bits(cw[0], (telemetry.fab_message[57] & 0x01) > 0, &bit_offset, 1);                               // Kill Switch Main PIC
    set_bits(cw[0], (telemetry.fab_message[57] & 0x10) > 0, &bit_offset, 1);                               // Kill Switch EPS
    set_bits(cw[0], get_adcs_mode_index(telemetry.adcs_message[0]), &bit_offset, 4);                       // ADCS mode
    set_bits(cw[0], 0, &bit_offset, 1);                                                                    // Format identifier

    bit_offset = 0;

    // Page 1
    set_bits(cw[1], gyro_to_cw(telemetry.adcs_message[5], telemetry.adcs_message[6]), &bit_offset, 8);   // Gyro X axis
    set_bits(cw[1], gyro_to_cw(telemetry.adcs_message[7], telemetry.adcs_message[8]), &bit_offset, 8);   // Gyro -Y axis
    set_bits(cw[1], gyro_to_cw(telemetry.adcs_message[9], telemetry.adcs_message[10]), &bit_offset, 8);  // Gyro -Z axis
    set_bits(cw[1], make16(telemetry.pcib_message[4], telemetry.pcib_message[5]) >> 11, &bit_offset, 5); // Magnetometer X
    set_bits(cw[1], make16(telemetry.pcib_message[6], telemetry.pcib_message[7]) >> 11, &bit_offset, 5); // Magnetometer Y
    set_bits(cw[1], make16(telemetry.pcib_message[8], telemetry.pcib_message[9]) >> 11, &bit_offset, 5); // Magnetometer Z
    set_bits(cw[1], subsystems_communicating, &bit_offset, 3);                                           // Subsystems communicating
    set_bits(cw[1], time_after_reset, &bit_offset, 5);                                                   // Time after last reset (hours)
    set_bits(cw[1], 1, &bit_offset, 1);                                                                  // Format identifier

    fprintf(PC, "CW: 0x");
    uart_print_pc_hex_short(cw[0], sizeof(cw[0]));
    fprintf(PC, " 0x");
    uart_print_pc_hex_short(cw[1], sizeof(cw[1]));
    fputc(' ', PC);
}

void save_flags()
{
    uint8_t* boot_flag_ptr = (uint8_t*)&boot_flags;
    flash_erase_pages(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, BOOT_FLAGS_ADDRESS + sizeof(boot_flags));
    flash_transfer_data_from_ram(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, boot_flag_ptr, sizeof(boot_flags));
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

void send_mcp_command(uint8_t mcp_command, uint16_t data, uint8_t silent)
{
    if (mux_sel(mux_mcpic) != mux_mcpic) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return;
    }

    uint8_t cmd[MSG_LENGTH_MCPIC] = { 0 };

    struct mcp_packet {
        uint8_t origin;
        uint8_t command;
        uint16_t data;
        uint8_t checksum;
        uint8_t footer;
    }* mcp_packet = (struct mcp_packet*)cmd;

    mcp_packet->origin = MSG_OBC;
    mcp_packet->command = mcp_command;
    mcp_packet->data = data;

    checksum_obc(cmd, sizeof(cmd));

    for (uint8_t i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], MCPIC);
    }

    if (!silent) {
        fprintf(PC, "MCPIC cmd: ");
        uart_print_pc_hex(cmd, sizeof(cmd));
    }
}

// ============ Commands for Telemetry request ============

// Request for reset telemetry
uint8_t command_request_reset(uint8_t* data)
{
    uart_clean(RST);
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

// request for adcs telemetry
uint8_t command_request_adcs(uint8_t* data)
{
    uart_clean(ADCS);

    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    uint8_t adcs_command[MSG_LENGTH_ADCS] = { 0x0B };
    adcs_command[1] = 0xAB;
    *(time_t*)&adcs_command[2] = current_time;
    checksum_obc(adcs_command, sizeof(adcs_command));
    for (uint8_t i = 0; i < sizeof(adcs_command); i++) {
        fputc(adcs_command[i], ADCS);
    }
    return 0;
}

// Request for Mission Control PIC Status
uint8_t command_request_mission_control_PIC_status(uint8_t* data)
{
    uart_clean(MCPIC);

    if (mux_sel(mux_mcpic) != mux_mcpic) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    send_mcp_command(0x10, 0, true);
    return 0;
}

// Request for pcib telemetry
uint8_t command_request_pcib(uint8_t* data)
{
    uart_clean(PCIB);

    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

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

    checksum_obc(cmd, sizeof(request));
    for (uint8_t i = 0; i < sizeof(request); i++) {
        fputc(cmd[i], PCIB);
    }
    return 0;
}

// Request for tmcr1 telemetry
uint8_t command_request_tmcr1(uint8_t* data)
{
    uart_clean(TMCR1);

    if (mux_sel(mux_tmcr1) != mux_tmcr1) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    struct request {
        uint8_t origin;
        uint8_t cmd;
        time_t obc_time;
        uint8_t data[MSG_LENGTH_TMCR1 - 8];
        uint8_t checksum;
        uint8_t footer;
    } request;

    uint8_t* cmd = (uint8_t*)&request;
    memset(cmd, 0, sizeof(request));

    request.origin = 0x0B;
    request.cmd = 0xAA;
    request.obc_time = current_time;

    checksum_obc(cmd, sizeof(request));
    for (uint8_t i = 0; i < sizeof(request); i++) {
        fputc(cmd[i], TMCR1);
    }
    return 0;
}

// Request for tmcr2 telemetry
uint8_t command_request_tmcr2(uint8_t* data)
{
    uart_clean(TMCR2);

    if (mux_sel(mux_tmcr2) != mux_tmcr2) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    struct request {
        uint8_t origin;
        uint8_t cmd;
        time_t obc_time;
        uint8_t data[MSG_LENGTH_TMCR2 - 8];
        uint8_t checksum;
        uint8_t footer;
    } request;

    uint8_t* cmd = (uint8_t*)&request;
    memset(cmd, 0, sizeof(request));

    request.origin = 0x0B;
    request.cmd = 0xAA;
    request.obc_time = current_time;

    checksum_obc(cmd, sizeof(request));
    for (uint8_t i = 0; i < sizeof(request); i++) {
        fputc(cmd[i], TMCR2);
    }
    return 0;
}

// ============ ADCS Commands ============

enum adcs_modes { // updated on 2023/05/01
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

// Change ADCS mode internally and externally (helper function)
void change_adcs_mode(uint8_t mode, uint8_t permanent)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
    }

    uint8_t adcs_command[3] = { 0x01 };
    adcs_mode = mode;
    adcs_command[1] = mode;
    adcs_command[2] = permanent;
    stm32_raw_command(adcs_command, sizeof(adcs_command), 0);
}

// Change ADCS mode internally and externally
uint8_t command_adcs_mode(uint8_t* data)
{
    // Mux is changed in the helper function change_adcs_mode()
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t mode;
        uint8_t permanent; // when true, change mode permanently
    }* packet = (struct packet*)data;
    change_adcs_mode(packet->mode, packet->permanent);
    return packet->mode;
}

// Change ADCS default mode
uint8_t command_adcs_default_mode(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
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
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
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
uint8_t command_adcs_comm_test(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    enum {
        adcs_cmd_size = 16,
        repetitions = 100,
        delay = 2100
    };

    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t adcs_command[adcs_cmd_size];
    }* packet = (struct packet*)data;

    fprintf(PC, "Sending command %02X %d times to ADCS.\r\n", packet->adcs_command[0], repetitions);

    uint8_t msg[MSG_LENGTH_ADCS] = { 0 };
    msg[0] = 0x0B;
    memcpy(msg + 1, packet->adcs_command, adcs_cmd_size);
    msg[MSG_LENGTH_ADCS - 1] = 0x0C;

    for (uint8_t j = 0; j < repetitions; j++) {
        uart_print_pc_hex(msg, sizeof(msg));
        fprintf(PC, " (attempt %d)\r\n", j);
        for (uint8_t i = 0; i < sizeof(msg); i++) {
            fputc(msg[i], ADCS);
        }
        delay_ms(delay);
    }

    return 0;
}

// Send a raw command to ADCS
uint8_t command_adcs_raw(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    enum { adcs_cmd_size = 16 };

    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t adcs_command[adcs_cmd_size];
    }* packet = (struct packet*)data;

    uint8_t msg[MSG_LENGTH_ADCS] = { 0 };
    msg[0] = 0x0B;
    memcpy(msg + 1, packet->adcs_command, adcs_cmd_size);
    checksum_obc(msg, sizeof(msg));

    uart_print_pc_hex(msg, sizeof(msg));

    for (uint8_t i = 0; i < sizeof(msg); i++) {
        fputc(msg[i], ADCS);
    }

    return 0;
}

uint8_t command_stm32_raw_8_16(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
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
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
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
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

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
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    response_rx = 1; // Received a reply

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
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
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
        struct_tm gps_time;
        gps_time.tm_year = packet->year + 100;
        gps_time.tm_mon = packet->month - 1;
        gps_time.tm_mday = packet->day;
        gps_time.tm_hour = packet->hour;
        gps_time.tm_min = packet->minute;
        gps_time.tm_sec = packet->second + obc_flags.leap_seconds;

        time_t unix_time = mktime(&gps_time);

        time_t delta = unix_time - current_time;
        if (delta < 0)
            delta = -delta;
        if (delta > 30 * 60) { // Do not update time if the difference to the current time is larger than 30 minutes
            valid_time = 3;
        } else {
            valid_time = 1;
            SetTimeSec(unix_time);
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
    }
    return valid_time;
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
    uint8_t new_length = packet->gs_message[2] != 0xCC ? 14 : length; // Don't pass CRC onwards for short commands
    uart_print_pc_hex(packet->gs_message, new_length);
    uint8_t valid = uplink_valid(packet->gs_message);
    uint8_t different = memcmp(last_upload, packet->gs_message, new_length) || (last_upload_t <= (current_time - 4 * 60)); // Commands are different within 4 min window
    memcpy(last_upload, packet->gs_message, new_length);
    last_upload_t = current_time;
    if (valid && different) {
        fprintf(PC, " Valid uplink");
        uint8_t cmd[BUFF_LENGTH] = { MSG_COMM };
        uint8_t copy_size = packet->gs_message[2] != 0xCC ? 9 : new_length; // Don't pass CRC onwards for short commands
        memcpy(cmd + 1, packet->gs_message + 3, copy_size);
        vschedule(current_time, cmd);
    } else {
        fprintf(PC, " Invalid (%d) or identical (%d) uplink.", !valid, !different);
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
    uart_download_packet(&uart_port_MSN, response, sizeof(response), 100000); // Try to get a response

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
        uint8_t origin;       // C0
        uint8_t command;      // F6
        time_t schedule_time; // Time_T 4 bytes
        uint8_t schedule_command[12];
    }* packet = (struct packet*)data;

    vschedule(packet->schedule_time, packet->schedule_command);

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
        uint8_t gps_time_sync_state; // Not set here, kept for backward compatibility
        uint8_t skip_telemetry_config;
    }* packet = (struct packet*)data;
    obc_flags.leap_seconds = packet->leap_seconds;
    // obc_flags.gps_time_sync_state = packet->gps_time_sync_state;

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

    if(!packet->skip_telemetry_config) { // Useful for clock manipulation without disturbing telemetry collection
        // Change memory location based on the new date
        flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag, addr_flags.flash_telemetry.current, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, true, &addr_flags.flash_telemetry);
    }

    struct_tm* local_time = localtime(&current_time);

    fprintf(PC, " New time: %04ld/%02d/%02d %02d:%02d:%02d(0x%08lX) ",
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
    fprintf(PC, "\r\n         fedcba9876543210");
    fprintf(PC, "\r\ntris_a = ");
    print_binary16(get_tris_a());
    fprintf(PC, "\r\ntris_b = ");
    print_binary16(get_tris_b());
    fprintf(PC, "\r\ntris_c = ");
    print_binary16(get_tris_c());
    fprintf(PC, "\r\ntris_d = ");
    print_binary16(get_tris_d());
    fprintf(PC, "\r\ntris_e = ");
    print_binary16(get_tris_e());
    fprintf(PC, "\r\ntris_f = ");
    print_binary16(get_tris_f());
    fprintf(PC, "\r\ntris_g = ");
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

// Sets the value of a memory address relative to satellite time
uint8_t command_set_obc_variable(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t offset;
        uint8_t size;
        uint8_t data[sizeof(uint32_t)];
        uint8_t save_state;
    }* packet = (struct packet*)data;

    uint8_t* dst = (uint8_t*)&current_time + packet->offset;
    uint8_t* src = packet->data;

    fprintf(PC, "Changing variable at 0x%04X, size %d to 0x%08LX", dst, packet->size, *(uint32_t*)&packet->data[0]);
    for (uint8_t i = 0; i < packet->size; i++) {
        *(dst + i) = *(src + i);
    }

    if (packet->save_state == 1) {
        save_state(data[1]); // data[1] is the current command id
    } else if (packet->save_state == 2) {
        save_flags();
        scheduled_command_clear_all();
    }

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

    if (current_time >= T_ANTENNA) { // Never try to deploy ahead of 31m mark
        // Increment deployment flag and save state
        boot_flags.deployment_flag++;
        get_com_shared_fm_access();
        save_flags();

        uint8_t time = 30;    // in seconds
        if (packet->time_s) { // if specified time is not zero, then use specified value
            time = packet->time_s;
        }
        fprintf(PC, "Deploying UHF Antenna for %ds... ", time);
        output_high(DIO_BURNER_ANTENNA);
        delay_ms((uint32_t)time * 1000);
        output_low(DIO_BURNER_ANTENNA);
        fprintf(PC, "Done!\r\n");
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

// Helper function to copy data from flash memory to flash memory
uint8_t copy(
    uint8_t origin, // 0:COM, 1:MAIN, 2:MISSION
    uint8_t dest,   // 0:COM, 1:MAIN, 2:MISSION
    uint32_t to_page_or_sector_or_address,
    uint32_t from_page_or_sector_or_address,
    uint32_t size, // in page or sectors or sectors
    uint8_t mode   // mode: 0 = pages, 1 = sectors, 2 = addressed by byte, size as pages
)
{
    get_com_shared_fm_access();

    uint8_t erase_mode;
    uint32_t increment;

    if (mode == 0 || mode == 2) {
        erase_mode = ERASE_PAGE;
        increment = MEMORY_PAGE_SIZE;
    } else {
        erase_mode = ERASE_SECTOR;
        increment = MEMORY_SECTOR_SIZE;
    }

    uint32_t to_address = 0;
    uint32_t from_address = 0;
    if (mode == 2) {
        to_address = to_page_or_sector_or_address;
        from_address = from_page_or_sector_or_address;
    } else {
        to_address = to_page_or_sector_or_address * increment;
        from_address = from_page_or_sector_or_address * increment;
    }

    size *= increment;

    fprintf(PC, "memcpy orig=%d,dest=%d,to_addr=%lX,from_addr=%lX,size=%lX,mode=%d", origin, dest, to_address, from_address, size, mode);

    // Erase the pages before copying
    if (dest == 0x00) {
        for (uint32_t i = 0; i < size; i += increment) {
            flash_erase(&spi_port_COM_FM, to_address + i, erase_mode);
        }
    } else if (dest == 0x01) {
        for (uint32_t i = 0; i < size; i += increment) {
            flash_erase(&spi_port_MAIN_FM, to_address + i, erase_mode);
        }
    } else if (dest == 0x02) {
        for (uint32_t i = 0; i < size; i += increment) {
            output_low(MUX_SEL_MSN_SHARED_FM);
            flash_erase(&spi_port_MISSION_FM, to_address + i, erase_mode);
            output_high(MUX_SEL_MSN_SHARED_FM);
        }
    }

    if (origin == 0x00 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address, &spi_port_COM_FM, to_address, size);

    } else if (origin == 0x00 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address, &spi_port_MAIN_FM, to_address, size);

    } else if (origin == 0x00 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_COM_FM, from_address, &spi_port_MISSION_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x01 && dest == 0x00) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address, &spi_port_COM_FM, to_address, size);

    } else if (origin == 0x01 && dest == 0x01) {
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address, &spi_port_MAIN_FM, to_address, size);

    } else if (origin == 0x01 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MAIN_FM, from_address, &spi_port_MISSION_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x00) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address, &spi_port_COM_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x01) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address, &spi_port_MAIN_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else if (origin == 0x02 && dest == 0x02) {
        output_low(MUX_SEL_MSN_SHARED_FM);
        flash_transfer_data_to_flash(&spi_port_MISSION_FM, from_address, &spi_port_MISSION_FM, to_address, size);
        output_high(MUX_SEL_MSN_SHARED_FM);

    } else {
        return 1;
    }

    return 0;
}

// Copy data sectors between flash memories
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

    return copy(packet->origin_port, packet->destination_port, packet->destination_sector, packet->source_sector, packet->n_sectors, 1); // 1 -> sector copy
}

// Copy data pages between flash memories
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

    return copy(packet->origin_port, packet->destination_port, packet->destination_page, packet->source_page, packet->n_pages, 0); // 0 -> page copy
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

uint8_t xmodem_send_fn(
    uint32_t source_address,
    uint32_t n_packets,
    uint8_t source,
    uint8_t destination,
    uint32_t destination_address)
{
    fprintf(PC, "Waiting for xmodem transfer...");
    uart_fn* destination_uart = NULL;

    const uint8_t max_tries = 5;
    uint8_t current_try = 0;
    int8_t error = 0;

    while (current_try < max_tries) {
        switch (destination) {
        case 0:
            destination_uart = &uart_port_PC;
            break; // For PC
        case 1:
            if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
                if (verbose) {
                    fprintf(PC, "MUX change failed!");
                }
                return 1;
            }
            destination_uart = &uart_port_MSN;
            break; // For PCIB
        case 2:
            if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
                if (verbose) {
                    fprintf(PC, "MUX change failed!");
                }
                return 1;
            }
            destination_uart = &uart_port_MSN;
            break; // For Rpi
        default: destination_uart = &uart_port_PC; break;
        }

        if (destination == 2) {                                                            // Rpi
            uint8_t message_pcib[MSG_LENGTH_PCIB] = { 0x0B, 0xCD };                        // Array with request for direct copy relay -> rpi = 0xCD
            checksum_obc(message_pcib, MSG_LENGTH_PCIB);                                   // Add checksum
            uart_send_packet_repeat(&uart_port_MSN, message_pcib, MSG_LENGTH_PCIB, 5, 10); // Send request 5 times, 100ms spaced
            delay_ms(1000);
            fprintf(PCIB, "\nrx,%05lu\n", destination_address);
        }

        if (destination == 1) { // PCIB
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
            req.address = destination_address;
            req.source = 1; // OBC
            checksum_obc((uint8_t*)&req, sizeof(req));
            uart_send_packet_repeat(&uart_port_MSN, (uint8_t*)&req, sizeof(req), 5, 10);
        }

        switch (source) {
        case 0:
            get_com_shared_fm_access();
            error = xmodem_send(destination_uart, &spi_port_COM_FM, source_address, n_packets);
            break;
        case 1:
            error = xmodem_send(destination_uart, &spi_port_MAIN_FM, source_address, n_packets);
            break;
        case 2:
            output_low(MUX_SEL_MSN_SHARED_FM);
            error = xmodem_send(destination_uart, &spi_port_MISSION_FM, source_address, n_packets);
            output_high(MUX_SEL_MSN_SHARED_FM);
            break;
        default:
            get_com_shared_fm_access();
            error = xmodem_send(destination_uart, &spi_port_COM_FM, source_address, n_packets);
            break;
        }
        current_try++;
        delay_ms(1000);
        if (error != -1)
            break;
    }
    return error;
}

// Send data through xmodem protocol
uint8_t command_xmodem_send(uint8_t* data)
{
    struct packet {
        uint8_t origin;  // C0
        uint8_t command; // D0
        uint32_t source_address;
        uint32_t n_packets;
        uint8_t source;
        uint8_t destination;
        uint32_t destination_address;
    }* packet = (struct packet*)data;

    return xmodem_send_fn(packet->source_address, packet->n_packets, packet->source, packet->destination, packet->destination_address);
}

// Short version of XMODEM send command, similar to flash copy command
uint8_t command_xmodem_send_sector(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_sector;
        uint16_t source_sector;
        uint16_t n_sectors;
        uint8_t destination;
        uint8_t source;
    }* packet = (struct packet*)data;

    uint32_t destination_address = packet->destination_sector * MEMORY_SECTOR_SIZE;
    uint32_t source_address = packet->source_sector * MEMORY_SECTOR_SIZE;
    uint32_t n_packets = packet->n_sectors * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

    return xmodem_send_fn(source_address, n_packets, packet->source, packet->destination, destination_address);
}

uint8_t xmodem_receive_fn(
    uint32_t destination_address,
    uint8_t source,
    uint8_t destination,
    uint32_t source_address,
    uint32_t n_packets)
{
    int8_t total_packets = 0;
    fprintf(PC, "Receiving xmodem data...");

    uart_fn* source_uart = NULL;
    const uint8_t max_tries = 5;

    uint8_t current_try = 0;

    switch (source) {
    case 0: source_uart = &uart_port_PC; break;
    case 1:
        if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
            if (verbose) {
                fprintf(PC, "MUX change failed!");
            }
            return 1;
        }
        source_uart = &uart_port_MSN;
        break;
    // case 2: reserved (thermal mission)
    case 3:
        if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
            if (verbose) {
                fprintf(PC, "MUX change failed!");
            }
            return 1;
        }
        source_uart = &uart_port_MSN;
        break;
    case 4:
        if (mux_sel(mux_tmcr1) != mux_tmcr1) { // If MUX did not change
            if (verbose) {
                fprintf(PC, "MUX change failed!");
            }
            return 1;
        }
        source_uart = &uart_port_MSN;
        break;
    case 5:
        if (mux_sel(mux_tmcr2) != mux_tmcr2) { // If MUX did not change
            if (verbose) {
                fprintf(PC, "MUX change failed!");
            }
            return 1;
        }
        source_uart = &uart_port_MSN;
        break;
    default: source_uart = &uart_port_PC; break;
    }

    while (current_try < max_tries) {
        if (source == 1 || source == 4 || source == 5) { // PCIB, TMCR1 or TMCR2
            struct req_pcib {
                uint8_t origin;
                uint8_t command;
                uint32_t address;
                uint32_t n_packets;
                uint8_t destination;
                uint8_t padding[MSG_LENGTH_PCIB - 13]; // the lenght is the same for the 3 subsystems
                uint8_t checksum;
                uint8_t footer;
            } req_pcib;
            req_pcib.origin = MSG_OBC;
            req_pcib.command = 0xD0; // X-modem send;
            req_pcib.address = source_address;
            req_pcib.n_packets = n_packets;
            req_pcib.destination = 1; // OBC
            checksum_obc((uint8_t*)&req_pcib, sizeof(req_pcib));
            uart_send_packet_repeat(source_uart, (uint8_t*)&req_pcib, sizeof(req_pcib), 1, 10);
            delay_ms(100);
        } else if (source == 2 || source == 3) { // Thermal or ADCS
            struct req_adcs {
                uint8_t origin;
                uint8_t command;
                uint32_t address;
                uint32_t n_packets;
                uint8_t destination;
                uint8_t padding[MSG_LENGTH_ADCS - 13];
                uint8_t checksum;
                uint8_t footer;
            } req_adcs;
            req_adcs.origin = MSG_OBC;
            req_adcs.command = 0xD0; // X-modem send;
            req_adcs.address = source_address;
            req_adcs.n_packets = n_packets;
            req_adcs.destination = 1; // OBC
            checksum_obc((uint8_t*)&req_adcs, sizeof(req_adcs));
            uart_send_packet_repeat(source_uart, (uint8_t*)&req_adcs, sizeof(req_adcs), 1, 10);
            delay_ms(100);
        }
        if (source == 2 || source == 3) {
            delay_ms(5000);
        }

        switch (destination) {
        case 0:
            get_com_shared_fm_access();
            total_packets = xmodem_receive(source_uart, &spi_port_COM_FM, destination_address);
            break;
        case 1:
            total_packets = xmodem_receive(source_uart, &spi_port_MAIN_FM, destination_address);
            break;
        case 2:
            output_low(MUX_SEL_MSN_SHARED_FM);
            total_packets = xmodem_receive(source_uart, &spi_port_MISSION_FM, destination_address);
            output_high(MUX_SEL_MSN_SHARED_FM);
            break;
        default:
            get_com_shared_fm_access();
            total_packets = xmodem_receive(source_uart, &spi_port_COM_FM, destination_address);
            break;
        }
        current_try++;
        delay_ms(1000);
        if (total_packets != -1)
            break;
    }
    return total_packets;
}

// Receive data through xmodem protocol
uint8_t command_xmodem_receive(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint32_t destination_address;
        uint8_t source;
        uint8_t destination;
        uint32_t source_address;
        uint32_t n_packets;
    }* packet = (struct packet*)data;
    return (xmodem_receive_fn(packet->destination_address, packet->source, packet->destination, packet->source_address, packet->n_packets));
}

// Short version of XMODEM receive command, similar to flash copy command
uint8_t command_xmodem_receive_sector(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_sector;
        uint16_t source_sector;
        uint16_t n_sectors;
        uint8_t destination;
        uint8_t source;
    }* packet = (struct packet*)data;

    uint32_t destination_address = packet->destination_sector * MEMORY_SECTOR_SIZE;
    uint32_t source_address = packet->source_sector * MEMORY_SECTOR_SIZE;
    uint32_t n_packets = packet->n_sectors * MEMORY_SECTOR_SIZE / XMODEM_DLENGTH;

    fprintf(PC, "memcpy orig=%d,dest=%d,to_addr=%lX,from_addr=%lX,size=%lX\r\n", packet->source, packet->destination, destination_address, source_address, n_packets);

    return xmodem_receive_fn(destination_address, packet->source, packet->destination, source_address, n_packets);
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
    save_flags();
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
    if (!memory_busy) {
        // Write the obc timestamp to telemetry and relative collection times
        telemetry.obc_time = current_time;
        telemetry.reset_time = current_time - telemetry_time.reset_time > 255 ? 255 : current_time - telemetry_time.reset_time;
        telemetry.fab_time = current_time - telemetry_time.fab_time > 255 ? 255 : current_time - telemetry_time.fab_time;
        telemetry.msn_time = current_time - telemetry_time.msn_time > 255 ? 255 : current_time - telemetry_time.msn_time;
        telemetry.pcib_time = current_time - telemetry_time.pcib_time > 255 ? 255 : current_time - telemetry_time.pcib_time;
        telemetry.adcs_time = current_time - telemetry_time.adcs_time > 255 ? 255 : current_time - telemetry_time.adcs_time;
        telemetry.com_time = current_time - telemetry_time.com_time > 255 ? 255 : current_time - telemetry_time.com_time;

        // Save telemetry to flash
        uint8_t* telemetry_data = (uint8_t*)&telemetry;
        flash_cycle_write(&spi_port_COM_FM, telemetry_data, &addr_flags.flash_telemetry);
        fprintf(PC, "Saving telemetry data: ");
        fprintf(PC, "Addr: 0x%08lX ", addr_flags.flash_telemetry.current - sizeof(telemetry));

        build_cw(); // Prepare CW strings
        if (verbose)
            deserialize_cw();   // For debug only
        initialize_telemetry(); // Reset for next iteration

        // Save satellite log to flash
        log_flush();

        // Save addresses to flash
        address_rotation addr;

        uint8_t* addr_flag_ptr = (uint8_t*)&addr;

        addr.flash_log_current = addr_flags.flash_log.current;
        addr.flash_telemetry_current = addr_flags.flash_telemetry.current;
        addr.flash_sel_zes_current = addr_flags.flash_sel_zes.current;
        addr.flash_sel_ref_current = addr_flags.flash_sel_ref.current;

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
        uint8_t ocp_number; // 0: adcs; 1: MCP; 2: pcib
    }* packet = (struct packet*)data;

    enum {
        ocp_adcs = 0,
        ocp_MCP = 1,
        ocp_relay = 2
    };

    switch (packet->ocp_number) {
    case ocp_adcs:
        output_bit(OCP_EN_ADCS, packet->on_off);
        obc_flags.adcs_on_off = packet->on_off;
        save_state(data[1]); // data[1] is the current command id
        return obc_flags.adcs_on_off;
    case ocp_MCP:
        output_bit(OCP_EN_MCP, packet->on_off);
        obc_flags.MCP_on_off = packet->on_off;
        save_state(data[1]); // data[1] is the current command id
        return obc_flags.MCP_on_off;
    case ocp_relay:
        output_bit(OCP_EN_RELAY, packet->on_off);
        obc_flags.relay_on_off = packet->on_off;
        save_state(data[1]); // data[1] is the current command id
        return obc_flags.relay_on_off;
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

    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    uint8_t relay_warning[MSG_LENGTH_PCIB] = { 0 };
    relay_warning[0] = MSG_OBC;
    relay_warning[1] = 0xFF; // Reset warning
    checksum_obc(relay_warning, sizeof(relay_warning));
    uart_send_packet_repeat(&uart_port_MSN, relay_warning, sizeof(relay_warning), 5, 100000);

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

    uint16_t voltage_hex = make16(data[50], data[51]);
    uint16_t current_hex = make16(data[54], data[55]);
    uint16_t temperature_hex = make16(data[56], data[57]);

    float voltage = voltage_hex * 3.3 * 3 / 4096;
    float current = (3952.5 * current_hex * 3.28 / 4096) - 6479.5;
    float temperature = 75 - temperature_hex * 3.256 * 30 / 4096;
    fprintf(PC, "V=%f, C=%f, T=%f | ", voltage, current, temperature);

    uint16_t heater_ref_temperature = make16(data[60], data[61]);
    uint16_t heater_ref_voltage = make16(data[62], data[63]);

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

typedef enum {
    mcp_cmd_telemetry_req = 0x10,
    mcp_cmd_opera_on = 0x20,
    mcp_cmd_opera_off = 0x2F,
    mcp_cmd_zes_reset = 0x30,
    mcp_cmd_ref_status = 0x35,
    mcp_cmd_tmcr1_on = 0x40,
    mcp_cmd_tmcr1_off = 0x4F,
    mcp_cmd_tmcr2_on = 0x50,
    mcp_cmd_tmcr2_off = 0x5F
} mcp_command;

// Turn on/off OCPs in MCP
uint8_t command_mcp(uint8_t* data)
{
    struct packet {
        uint8_t origin;  // C0
        uint8_t command; // 2A for MCP OCP control
        uint8_t mcp_command;
        uint16_t data;
    }* packet = (struct packet*)data;

    send_mcp_command(packet->mcp_command, packet->data, false);

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
        output_high(OBC_KILL_DIO);
    }
    return kill_sw_status;
}

uint8_t command_obc_kill_off(uint8_t* data)
{
    output_low(OBC_KILL_DIO);
    return 0;
}

// ADCS RAW command part a
uint8_t command_adcs_raw_part_a(uint8_t* data)
{
    enum { raw_a_size = 16 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part_a[raw_a_size];
        uint8_t crc0;
        uint8_t crc1;
    }* packet = (struct packet*)data;

    memcpy(adcs_raw_part_a, packet->part_a, raw_a_size);
    adcs_raw_part_a_crc0 = packet->crc0;

    uart_print_pc_hex(adcs_raw_part_a, sizeof(adcs_raw_part_a));
    fprintf(PC, "Received adcs raw command part a.");

    return 0;
}

// ADCS RAW command part b
uint8_t command_adcs_raw_part_b(uint8_t* data)
{
    enum { raw_b_size = 16 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part_b[raw_b_size];
        uint8_t crc0;
        uint8_t crc1;
    }* packet = (struct packet*)data;

    memcpy(adcs_raw_part_b, packet->part_b, raw_b_size);
    adcs_raw_part_b_crc0 = packet->crc0;

    uart_print_pc_hex(adcs_raw_part_b, sizeof(adcs_raw_part_b));
    fprintf(PC, "Received adcs raw command part b.");

    return 0;
}

// Helper function to calculate ADCS checksum
void checksum_adcs(uint8_t* data, int size)
{
    uint16_t result = 0;
    for (int i = 0; i < (size - 2); i++) {
        result += data[i];
    }
    unsigned char* ptr = (unsigned char*)&result;
    data[size - 1] = ptr[1];
    data[size - 2] = ptr[0];
}

// Send MAI-400 command (max 40-bytes)
void mai_400_command(uint8_t* data, int length)
{
    uint8_t adcs_command[MSG_LENGTH_ADCS] = { 0x0B };
    memcpy(adcs_command + 1, data, length);
    checksum_adcs(adcs_command + 1, 40);
    checksum_obc(adcs_command, sizeof(adcs_command));
    for (int i = 0; i < sizeof(adcs_command); i++) {
        fputc(adcs_command[i], ADCS);
    }
}

// ADCS RAW command part c
uint8_t command_adcs_raw_part_c(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    enum { raw_c_size = 6,
        padding_size = 8 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part_c[raw_c_size];
        uint8_t padding[padding_size];
        uint8_t crc0_part_a;
        uint8_t crc0_part_b;
        uint8_t crc0;
        uint8_t crc1;
    }* packet = (struct packet*)data;

    if (adcs_raw_part_a_crc0 != packet->crc0_part_a) {
        uart_print_pc_hex(packet->part_c, raw_c_size);
        fprintf(PC, "Received adcs raw part c. Part a pairing error.");
    } else if (adcs_raw_part_b_crc0 != packet->crc0_part_b) {
        uart_print_pc_hex(packet->part_c, raw_c_size);
        fprintf(PC, "Received adcs raw part c. Part b pairing error.");
    } else {
        uint8_t adcs_cmd[40];
        memcpy(adcs_cmd, adcs_raw_part_a, sizeof(adcs_raw_part_a));
        memcpy(adcs_cmd + sizeof(adcs_raw_part_a), adcs_raw_part_b, sizeof(adcs_raw_part_b));
        memcpy(adcs_cmd + sizeof(adcs_raw_part_a) + sizeof(adcs_raw_part_b), packet->part_c, raw_c_size);
        mai_400_command(adcs_cmd, sizeof(adcs_cmd));
        uart_print_pc_hex(adcs_cmd, sizeof(adcs_cmd));
        fprintf(PC, "Received adcs raw part c. Pairing ok.");
    }
    memset(&adcs_raw_part_a, 0, sizeof(adcs_raw_part_a)); // Erase old data.
    memset(&adcs_raw_part_b, 0, sizeof(adcs_raw_part_b)); // Erase old data.
    return 0;
}

uint8_t command_to_deploy_SAP(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t time_s;
    }* packet = (struct packet*)data;

    if (current_time >= T_ANTENNA) { // Never try to deploy ahead of 31m mark
        uint8_t time = 30;           // in seconds
        if (packet->time_s) {        // if specified time is not zero, then use specified value
            time = packet->time_s;
        }
        fprintf(PC, "Deploying SAP for %ds... ", time);
        output_high(DIO_BURNER_SAP);
        delay_ms((uint32_t)time * 1000);
        output_low(DIO_BURNER_SAP);
        fprintf(PC, "Done!\r\n");
        return 0;
    } else {
        return 1;
    }
}

uint8_t command_to_deploy_SMA(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t time_s;
    }* packet = (struct packet*)data;

    if (current_time >= T_ANTENNA) { // Never try to deploy ahead of 31m mark
        uint8_t time = 60;           // in seconds
        if (packet->time_s) {        // if specified time is not zero, then use specified value
            time = packet->time_s;
        }
        fprintf(PC, "Deploying SMA for %ds... ", time);
        output_high(DIO_BURNER_SMA);
        delay_ms((uint32_t)time * 1000);
        output_low(DIO_BURNER_SMA);
        fprintf(PC, "Done!\r\n");
        return 0;
    } else {
        return 1;
    }
    return 0;
}

// General command for ZES SEL mission (sub-function 1)
uint8_t command_sel1_zes(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t repetitions;
        uint32_t delay_between_repetitions;
    }* packet = (struct packet*)data;

    schedule(current_time, { 0xC0, 0x0F });      // Check UART
    schedule(current_time + 15, { 0xC0, 0x19 }); // Data initialization

    struct new_packet {
        uint8_t origin;
        uint8_t command;
        uint8_t repetitions;
        uint32_t delay_between_repetitions;
    } new_packet;

    new_packet.origin = MSG_COMM;
    new_packet.command = 0x12;
    new_packet.repetitions = packet->repetitions;                             // Number of times to do the copy command to main bus
    new_packet.delay_between_repetitions = packet->delay_between_repetitions; // time between copies

    vschedule(current_time + 30, (uint8_t*)&new_packet); // Data copy
    return 0;
}

// General command for REF SEL mission (sub-function 2)
uint8_t command_sel2_ref(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t repetitions;
        uint32_t delay_between_repetitions;
    }* packet = (struct packet*)data;

    schedule(current_time, { 0xC0, 0x1F });      // Check UART
    schedule(current_time + 15, { 0xC0, 0x80 }); // Data initialization

    struct new_packet {
        uint8_t origin;
        uint8_t command;
        uint8_t repetitions;
        uint32_t delay_between_repetitions;
    } new_packet;

    new_packet.origin = MSG_COMM;
    new_packet.command = 0x81;
    new_packet.repetitions = packet->repetitions;                             // Number of times to do the copy command to main bus
    new_packet.delay_between_repetitions = packet->delay_between_repetitions; // time between copies

    vschedule(current_time + 30, (uint8_t*)&new_packet); // Data copy
    return 0;
}

// Send 0x3A to ZES
// Send CMD 1 of SEL 1 ZES (0x3A) to ZES_RX, and ZES_TX will send back hex number 5A (0x5A) as acknowledgement.
uint8_t command_sel1_a(uint8_t* data)
{
    if (mux_sel(mux_sel_zes) != mux_sel_zes) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    uint8_t timeout = 0;

    const uint8_t n_tries = 3; // Number of total tries for the procedure

    for (uint8_t attempt = 0; attempt < n_tries; attempt++) {
        fprintf(PC, "\r\nAttempt %d\r\n", attempt);
        fputc(0x3A, SEL_ZES);
        fprintf(PC, "Ready to receive acknowledgement from SEL... \r\n");
        while (++timeout < 5) {
            fprintf(PC, "Waiting for acknowledgement... \r\n");
            if (kbhit(SEL_ZES)) {
                uint8_t r = fgetc(SEL_ZES);
                if (r == 0x5A) {
                    fprintf(PC, "Ack = %X | \r\n", r);
                    return r;
                }
            }
            delay_ms(1000);
        }
        timeout = 0;
    }

    fprintf(PC, "Failed to get an ack after %d tries.\r\n", n_tries);
    return 0xFF;
}

// Send reset function (Initialization of SEL Board)
uint8_t command_sel1_b(uint8_t* data)
{
    send_mcp_command(mcp_cmd_zes_reset, 0, 0);
    return 0;
}

// Send 0x2A to ZES
// Send CMD 3 of SEL 1 ZES (0x2A) to ZES_RX, and ZES_TX will send back hex number A2 (0xA2) as data initialization.
uint8_t command_sel1_c(uint8_t* data)
{
    if (mux_sel(mux_sel_zes) != mux_sel_zes) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    uint8_t timeout = 0;

    const uint8_t n_tries = 3; // Number of total tries for the procedure

    for (uint8_t attempt = 0; attempt < n_tries; attempt++) {
        fprintf(PC, "\r\n\r\nAttempt %d\r\n", attempt);
        fputc(0x2A, SEL_ZES); // Prepare Data from ZES board

        fprintf(PC, "Ready to receive acknowledgement from SEL... \r\n");
        while (++timeout < 5) {
            fprintf(PC, "Waiting for acknowledgement... \r\n");
            if (kbhit(SEL_ZES)) {
                uint8_t r = fgetc(SEL_ZES);
                if (r == 0xA2) {
                    fprintf(PC, "Ack = %X | \r\n", r);
                    return r;
                }
            }
            delay_ms(1000);
        }
        timeout = 0;
    }

    fprintf(PC, "Failed to get an ack after %d tries.\r\n", n_tries);
    return 0xFF;
}

// Send 0x2E to ZES (revised)
// Send CMD 4 of (0x2E) to ZES_RX, and ZES_TX will send back a first data stream.
uint8_t command_sel1_d(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t repetitions;
        uint32_t delay_between_repetitions;
    }* packet = (struct packet*)data;

    if (mux_sel(mux_sel_zes) != mux_sel_zes) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    uint8_t sel1_d_buffer[128];

    for (uint8_t i = 0; i < packet->repetitions; i++) {
        fprintf(PC, "Part no. %d: ", i);
        fputc(0x2E, SEL_ZES);

        uart_download_packet(&uart_port_MSN, sel1_d_buffer, sizeof(sel1_d_buffer), 100000);

        if (sel1_d_buffer[0] == 0x2A && sel1_d_buffer[127] == 0x89) {
            for (uint8_t k = 0; k < 128; k++) {
                fprintf(PC, "%X ", sel1_d_buffer[k]);
            }
            get_com_shared_fm_access();
            fprintf(PC, "\r\nSaving SEL ZES data: ");
            fprintf(PC, "Addr: 0x%08lX ", addr_flags.flash_sel_zes.current);
            flash_cycle_write(&spi_port_COM_FM, sel1_d_buffer, &addr_flags.flash_sel_zes);
        } else {
            fprintf(PC, "Failed to get data from SEL.\r\n");
        }
        delay_ms(packet->delay_between_repetitions);
    }

    return 0;
}

// Send 0x26 to REF
// Send CMD 1 of SEL 1 REF (0x26) to REF_RX, and REF_TX will send back hex number A9 (0xA9) as acknowledgement.
uint8_t command_sel2_a(uint8_t* data)
{
    if (mux_sel(mux_sel_ref) != mux_sel_ref) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    uint8_t timeout = 0;

    const uint8_t n_tries = 3; // Number of total tries for the procedure

    for (uint8_t attempt = 0; attempt < n_tries; attempt++) {
        fprintf(PC, "\r\nAttempt %d\r\n", attempt);
        fputc(0x26, SEL_REF);
        delay_ms(1000); // TODO test with and without this delay and see if it is needed

        fprintf(PC, "Ready to receive acknowledgement from SEL... \r\n");
        while (++timeout < 5) {
            fprintf(PC, "Waiting for acknowledgement... \r\n");
            if (kbhit(SEL_REF)) {
                uint8_t r = fgetc(SEL_REF);
                if (r == 0xA9) {
                    fprintf(PC, "Ack = %X | \r\n", r);
                    return r;
                }
            }
            delay_ms(1000);
        }
        timeout = 0;
    }

    fprintf(PC, "Failed to get an ack after %d tries.\r\n", n_tries);
    return 0xFF;
}

// Fifteen minutes status check
uint8_t command_sel2_b(uint8_t* data)
{
    send_mcp_command(mcp_cmd_ref_status, 0, 0);
    return 0;
}

// Send 0x5E to REF
// Send CMD 3 of SEL 1 REF (0x5E) to REF_RX, and REF_TX will send back hex number A7 (0xA7) as data initialization.
uint8_t command_sel2_c(uint8_t* data)
{
    if (mux_sel(mux_sel_ref) != mux_sel_ref) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    uint8_t timeout = 0;

    const uint8_t n_tries = 3; // Number of total tries for the procedure

    for (uint8_t attempt = 0; attempt < n_tries; attempt++) {
        fprintf(PC, "\r\nAttempt %d\r\n", attempt);
        fputc(0x5E, SEL_REF);
        delay_ms(700);

        fprintf(PC, "Ready to receive acknowledgement from SEL... \r\n");
        while (++timeout < 5) {
            fprintf(PC, "Waiting for acknowledgement... \r\n");
            if (kbhit(SEL_REF)) {
                uint8_t r = fgetc(SEL_REF);
                if (r == 0xA7) {
                    fprintf(PC, "Ack = %X | \r\n", r);
                    return r;
                }
            }
            delay_ms(1000);
        }
        timeout = 0;
    }

    fprintf(PC, "Failed to get an ack after %d tries.\r\n", n_tries);
    return 0xFF;
}

// Send 0x7E to REF
// Send CMD 4 of (0x7E) to REF_RX, and REF_TX will send back a first data stream.
uint8_t command_sel2_d(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t repetitions;
        uint32_t delay_between_repetitions;
    }* packet = (struct packet*)data;

    if (mux_sel(mux_sel_ref) != mux_sel_ref) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    uint8_t sel2_d_buffer[128];

    for (uint8_t i = 0; i < packet->repetitions; i++) {
        fprintf(PC, "Part no. %d: ", i);
        // Sending command to the SEL_REF
        fputc(0x7E, SEL_REF);

        uart_download_packet(&uart_port_MSN, sel2_d_buffer, sizeof(sel2_d_buffer), 100000);

        if (sel2_d_buffer[0] == 0x2A && sel2_d_buffer[127] == 0x89) {
            for (uint8_t k = 0; k < 128; k++) {
                fprintf(PC, "%X ", sel2_d_buffer[k]);
            }
            get_com_shared_fm_access();
            fprintf(PC, "\r\nSaving SEL REF data: ");
            fprintf(PC, "Addr: 0x%08lX ", addr_flags.flash_sel_ref.current);
            flash_cycle_write(&spi_port_COM_FM, sel2_d_buffer, &addr_flags.flash_sel_ref);
        } else {
            fprintf(PC, "Failed to get data from SEL.\r\n");
        }
        delay_ms(packet->delay_between_repetitions);
    }

    return 0;
}

uint8_t command_iMTQ_Dipole_Actuation(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    fprintf(PC, "iMTQ Dipole Activation.......\r\n");
    const unsigned char cmd[44] = { 0x0B, 0x06, 0x1D, 0x06, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0x10, 0x27, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x41, 0x00, 0x0C };
    for (int i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

uint8_t command_iMTQ_No_Opeartion(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    fprintf(PC, "iMTQ No Opeartion.......\r\n");
    const unsigned char cmd[44] = { 0x0B, 0x06, 0x1D, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x41, 0x00, 0x0C };
    for (int i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

uint8_t command_3v3_Enable(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    fprintf(PC, "RW 3v3 Enable.......\r\n");
    const unsigned char cmd[44] = { 0x0B, 0x06, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x41, 0x00, 0x0C };
    for (int i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

uint8_t command_RW_Motor_Enable(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    fprintf(PC, "RW Motor Enable.......\r\n");
    const unsigned char cmd[44] = { 0x0B, 0x06, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x41, 0x00, 0x0C };
    for (int i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

uint8_t command_RW_Enable(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    fprintf(PC, "RW Enable......\r\n");
    const unsigned char cmd[44] = { 0x0B, 0x06, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x41, 0x00, 0x0C };
    for (int i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

uint8_t command_RW_Speed(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    fprintf(PC, "RW Speed.......\r\n");
    const unsigned char cmd[44] = { 0x0B, 0x06, 0x3A, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x41, 0x00, 0x0C };
    for (int i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

uint8_t command_RW_Reset(uint8_t* data)
{
    if (mux_sel(mux_adcs) != mux_adcs) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    fprintf(PC, "RW Reset.......\r\n");
    const unsigned char cmd[44] = { 0x0B, 0x06, 0x3A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x41, 0x00, 0x0C };
    for (int i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], ADCS);
    }

    return 0;
}

// Receive Mission Control PIC Status
uint8_t command_mission_control_PIC_status(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint16_t status;
    }* packet = (struct packet*)data;

    response_rx = 1; // Received a reply

    uint16_t status = packet->status;

    // Print each bit with its corresponding shorthand and value
    fprintf(PC, "MCP: ");
    fprintf(PC, "opera_5v5_dc_dc = %u, ", (uint8_t)((status & (1 << 0)) ? 1 : 0));
    fprintf(PC, "opera_ocp1 = %u, ", (uint8_t)((status & (1 << 1)) ? 1 : 0));
    fprintf(PC, "opera_ocp2 = %u, ", (uint8_t)((status & (1 << 2)) ? 1 : 0));
    fprintf(PC, "sel_com_pr = %u, ", (uint8_t)((status & (1 << 3)) ? 1 : 0));
    fprintf(PC, "sel_com_res = %u, ", (uint8_t)((status & (1 << 4)) ? 1 : 0));
    fprintf(PC, "sel_com_pg = %u, ", (uint8_t)((status & (1 << 5)) ? 1 : 0));
    fprintf(PC, "sel_com_flag = %u, ", (uint8_t)((status & (1 << 6)) ? 1 : 0));
    fprintf(PC, "tmcr1_ocp = %u, ", (uint8_t)((status & (1 << 7)) ? 1 : 0));
    fprintf(PC, "tmcr2_ocp = %u, ", (uint8_t)((status & (1 << 8)) ? 1 : 0));
    fprintf(PC, "sel_status = %u | ", (uint8_t)((status & (1 << 9)) ? 1 : 0));

    uart_print_pc_hex(data, MSG_LENGTH_MCPIC);

    telemetry_time.msn_time = current_time;
    memcpy(telemetry.msn_message, data + 2, sizeof(telemetry.msn_message));

    return 0;
}

// Used to stop the simulation
uint8_t command_debug(uint8_t* data)
{
#ifdef PC_SIM
    fprintf(PC, "Exiting simulation");
    continue_program = 0;
#else
    verbose = !verbose;
#endif // !PC_SIM
    return 0;
}

uint8_t command_opera_full(uint8_t* data)
{
    enum { packet_size = GA_part_a_size + GA_part_b_size - sizeof(time_t) - sizeof(uint16_t) + 2 };

    if (mux_sel(mux_opera) != mux_opera) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    uart_print_pc_hex(data, BUFF_LENGTH);

    uint8_t opera_cmd[packet_size] = { 0 };

    opera_cmd[0] = 0x2A;
    memcpy(opera_cmd + 1, data + 2, packet_size - 2);
    opera_cmd[packet_size - 1] = 0x80;

    uart_send_packet(&uart_port_MSN, opera_cmd, sizeof(opera_cmd));
    uint8_t ack[1] = { 0 };
    uart_download_packet(&uart_port_MSN, ack, sizeof(ack), 100000); // Try to get the ACK
    memset(&opera_GA_part_a, 0, sizeof(opera_GA_part_a));           // Erase old data.
    return ack[0];
}

uint8_t command_opera_GA_part_a(uint8_t* data)
{
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part_a_parameters[GA_part_a_size];
        // No padding is required
        uint8_t crc0;
        uint8_t crc1;
    }* packet = (struct packet*)data;

    memcpy(opera_GA_part_a, packet->part_a_parameters, GA_part_a_size);

    fprintf(PC, "Received OPERA GA part a: ");
    uart_print_pc_hex(opera_GA_part_a, sizeof(opera_GA_part_a));

    return 0;
}

uint8_t command_opera_GA_part_b(uint8_t* data)
{
    enum { packet_size = GA_part_a_size + GA_part_b_size - sizeof(time_t) - sizeof(uint16_t) + 2 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t part_b_parameters[GA_part_b_size];
    }* packet = (struct packet*)data;

    uint8_t opera_cmd[packet_size] = { 0xC0, 0xFF };
    uint8_t opera_off[packet_size] = { 0xC0, 0xFF, 0xFF };

    time_t* opera_turn_on_time = (time_t*)opera_GA_part_a;                            // The first part in the command is time information
    uint16_t* opera_mission_duration = (uint16_t*)(opera_GA_part_a + sizeof(time_t)); // The second part in the command is the mission duration

    uint8_t scheduled = !!*opera_turn_on_time;

    if (!*opera_turn_on_time) {
        *opera_turn_on_time = current_time;
    }

    memcpy(opera_cmd + 2, opera_GA_part_a + sizeof(time_t) + sizeof(uint16_t), sizeof(opera_GA_part_a) - sizeof(time_t));
    memcpy(opera_cmd + 2 + sizeof(opera_GA_part_a) - sizeof(time_t) - sizeof(uint16_t), packet->part_b_parameters, GA_part_b_size);

    fprintf(PC, "Received OPERA part b: ");
    uart_print_pc_hex(opera_cmd + 2, sizeof(opera_cmd) - sizeof(time_t) - 2);

    fprintf(PC, " Scheduling commands...");

    schedule(*opera_turn_on_time, { 0xC0, 0x2A, mcp_cmd_opera_on, 0x00, 0x00 });
    vschedule(*opera_turn_on_time + obc_flags.opera_boot_duration, opera_cmd);
    vschedule(*opera_turn_on_time + obc_flags.opera_boot_duration + *opera_mission_duration, opera_off);
    schedule(*opera_turn_on_time + obc_flags.opera_boot_duration + *opera_mission_duration + 40, { 0xC0, 0x2A, mcp_cmd_opera_off, 0x00, 0x00 });

    return scheduled;
}

uint8_t command_opera_GA_single(uint8_t* data)
{
    enum {
        length = 16 - sizeof(time_t) - sizeof(uint16_t),
        packet_size = GA_part_a_size + GA_part_b_size - sizeof(time_t) - sizeof(uint16_t) + 2
    };

    struct packet {
        uint8_t origin;
        uint8_t command;
        time_t opera_turn_on_time;
        uint16_t opera_mission_duration;
        uint8_t opera_command[length];
    }* packet = (struct packet*)data;

    uint8_t opera_cmd[packet_size] = { 0xC0, 0xFF };
    uint8_t opera_off[packet_size] = { 0xC0, 0xFF, 0xFF };

    uint8_t scheduled = !!packet->opera_turn_on_time;

    if (!packet->opera_turn_on_time) {
        packet->opera_turn_on_time = current_time;
    }

    memcpy(opera_cmd + 2, packet->opera_command, length);

    fprintf(PC, "Received OPERA command: ");
    uart_print_pc_hex(opera_cmd, sizeof(opera_cmd));

    fprintf(PC, " Scheduling commands...");

    schedule(packet->opera_turn_on_time, { 0xC0, 0x2A, mcp_cmd_opera_on, 0x00, 0x00 });
    vschedule(packet->opera_turn_on_time + obc_flags.opera_boot_duration, opera_cmd);
    vschedule(packet->opera_turn_on_time + obc_flags.opera_boot_duration + packet->opera_mission_duration, opera_off);
    schedule(packet->opera_turn_on_time + obc_flags.opera_boot_duration + packet->opera_mission_duration + 40, { 0xC0, 0x2A, mcp_cmd_opera_off, 0x00, 0x00 });

    return scheduled;
}

uint32_t swap32(uint32_t k)
{
    return ((k << 24) | ((k & 0x0000FF00) << 8) | ((k & 0x00FF0000) >> 8) | (k >> 24));
}

// Ask OPERA to copy data to mission shared FM, then copy data to COM shared FM
uint8_t command_copy_opera_to_com(uint8_t* data)
{
    enum {
        packet_lenght = 10,
        packet_size = GA_part_a_size + GA_part_b_size - sizeof(time_t) - sizeof(uint16_t) + 2
    };

    struct packet {
        uint8_t origin;  // 0xC0
        uint8_t command; // 0xEB
        uint8_t month;
        uint8_t day;
        uint8_t mission_number;
        uint32_t address;
        uint8_t n_pages;
        uint8_t state;
    }* packet = (struct packet*)data;

    uint8_t cpacket[MSG_LENGTH_OPERA] = { 0 };

    struct copy_packet {
        uint8_t origin;  // 0x2A
        uint8_t command; // 0x2C
        uint8_t month;
        uint8_t day;
        uint8_t mission_number;
        uint32_t address;
        uint8_t n_pages;
    }* copy_packet = (struct copy_packet*)cpacket;

    uint8_t ack[1] = { 0 };
    time_t scheduled_time = current_time;
    fprintf(PC, "OPERA copy command: ");
    uint8_t state = packet->state;

    time_t opera_copy_delay = 17L * packet->n_pages;

    enum state_machine {
        state_turn_on = 0,
        state_copy_opera_mission = 1,
        state_turn_off_opera = 2,
        state_copy_mission_com = 3
    };

    uint8_t opera_off[packet_size] = { 0xC0, 0xFF, 0xFF };

    switch (state) {
    case state_turn_on:
        fprintf(PC, "Turning on OPERA...");
        send_mcp_command(mcp_cmd_opera_on, 0, true);
        scheduled_time += obc_flags.opera_boot_duration;
        packet->state = state_copy_opera_mission;
        vschedule(scheduled_time, data);
        return state;
    case state_copy_opera_mission:
        fprintf(PC, "Copying data from OPERA to Mission SFM...");
        memcpy(cpacket, data, packet_lenght);
        copy_packet->origin = 0x2A;
        copy_packet->command = 0x2C;
        copy_packet->address = swap32(packet->address); // Convert from little endian to big endian
        cpacket[sizeof(cpacket) - 1] = 0x80;            // Footer
        if (mux_sel(mux_opera) == mux_opera) {          // Try to change mission shared FM mux position to OPERA
            mux_lock_unlock(true, opera_copy_delay);    // Get exclusive access for some time
        } else {
            fprintf(PC, "Error: mux did not change!");
            ack[0] = 0xDE;
        }
        delay_ms(100);
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_OPERA, 1, 100); // Send 1 times with 100ms delay
        uart_download_packet(&uart_port_MSN, ack, sizeof(ack), 100000);             // Try to get a response
        scheduled_time += opera_copy_delay;                                         // OPERA copy time
        packet->state = state_turn_off_opera;
        vschedule(scheduled_time, data);
        return ack[0];
    case state_turn_off_opera:
        fprintf(PC, "Sending OPERA turn off command...\r\n");
        vschedule(current_time, opera_off);
        scheduled_time += 40;
        packet->state = state_copy_mission_com;
        vschedule(scheduled_time, data);
        return state;
    case state_copy_mission_com:
        fprintf(PC, "Turning off OPERA and copying data from Mission SFM to COM SFM...");
        send_mcp_command(mcp_cmd_opera_off, 0, true);
        copy(2, 0, packet->address, packet->address, packet->n_pages, 2); // Copy from mission shared FM (2) to com shared FM (0), in byte address mode (2)
        return state;
    default:
        fprintf(PC, "Invalid state!");
        return state;
    }
    return 0;
}

// Ask subsystem to copy data to mission shared FM, then copy data to COM shared FM
uint8_t command_copy_mission_to_com(uint8_t* data)
{
    enum {
        adcs_copy_delay = 54,  // seconds for one sector copy
        pcib_copy_delay = 6,   // seconds for one sector copy
        tmcr1_copy_delay = 54, // seconds for one sector copy
        tmcr2_copy_delay = 54, // seconds for one sector copy
    };

    struct packet {
        uint8_t origin;  // 0xC0
        uint8_t command; // 0xEA
        uint16_t destination_sector;
        uint16_t source_sector;
        uint16_t size;       // in sectors
        uint8_t data_source; // 0: OBC
    }* packet = (struct packet*)data;

    enum {
        dsource_obc = 0,
        dsource_adcs = 1,
        dsource_relay = 2,
        dsource_tmcr_n = 3,
        dsource_tmcr_b4 = 4
        // OPERA has a separate command
    };

    uint8_t cpacket[MAX_LENGTH] = { 0 };

    struct copy_packet {
        uint8_t origin;
        uint8_t command;
        uint16_t destination_sector;
        uint16_t source_sector;
        uint16_t size;
        uint8_t data_destination;
        uint8_t data_source;
    }* copy_packet = (struct copy_packet*)cpacket;

    switch (packet->data_source) {
    case dsource_obc:
        copy(2, 0, packet->destination_sector, packet->source_sector, packet->size, 1); // Source and destination: 0=COM, 1=MAIN, 2=MISSION ; mode = 1 -> sector copy; get access to memory
        break;
    case dsource_adcs:
        fprintf(PC, "Waiting for ADCS copy...");
        if (mux_sel(mux_adcs) == mux_adcs) {                                 // Try to change mission shared FM mux position to ADCS
            mux_lock_unlock(true, ((time_t)adcs_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        delay_ms(100);
        copy_packet->origin = MSG_OBC;
        copy_packet->command = 0xCB; // Copy sectors command on ADCS pic
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_ADCS);
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_ADCS, 5, 100); // Send 5 times with 100ms delay
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)adcs_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    case dsource_relay:
        fprintf(PC, "Waiting for RELAY copy...");
        if (mux_sel(mux_pcib) == mux_pcib) {                                 // Try to change mission shared FM mux position to PCIB
            mux_lock_unlock(true, ((time_t)pcib_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        delay_ms(100);
        copy_packet->origin = MSG_OBC;
        copy_packet->command = 0x0B; // Copy sectors command on RELAY PIC
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_PCIB);
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_PCIB, 1, 10); // Send 1 time
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)pcib_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    case dsource_tmcr_b4:
        fprintf(PC, "Waiting for TMCR B4 copy...");
        if (mux_sel(mux_tmcr1) == mux_tmcr1) {                                 // Try to change mission shared FM mux position to TMCR1
            mux_lock_unlock(true, ((time_t)tmcr1_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        delay_ms(100);
        copy_packet->origin = MSG_OBC;
        copy_packet->command = 0x0B; // Copy sectors command on RELAY PIC
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_TMCR1);
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_TMCR1, 1, 10); // Send 1 time
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)tmcr1_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    case dsource_tmcr_n:
        fprintf(PC, "Waiting for TMCR N copy...");
        if (mux_sel(mux_tmcr2) == mux_tmcr2) {                                 // Try to change mission shared FM mux position to TMCR2
            mux_lock_unlock(true, ((time_t)tmcr2_copy_delay) * packet->size); // Get exclusive access for some time
        } else {
            return 2; // Mux did not change
        }
        delay_ms(100);
        copy_packet->origin = MSG_OBC;
        copy_packet->command = 0x0B; // Copy sectors command on RELAY PIC
        copy_packet->destination_sector = packet->destination_sector;
        copy_packet->source_sector = packet->source_sector;
        copy_packet->size = packet->size;
        copy_packet->data_destination = 1; // 0:LOCAL, 1:SHARED
        copy_packet->data_source = 0;      // 0:LOCAL, 1:SHARED
        checksum_obc(cpacket, MSG_LENGTH_TMCR2);
        uart_send_packet_repeat(&uart_port_MSN, cpacket, MSG_LENGTH_TMCR2, 1, 10); // Send 1 time
        packet->data_source = 0;
        packet->source_sector = packet->destination_sector;
        vschedule(current_time + (time_t)tmcr2_copy_delay * (time_t)packet->size, (uint8_t*)packet);
        break;
    default:
        return 1;
    }
    return 0;
}

// Resetting satellite (all powera lines) any time
uint8_t command_reset_all_power_lines(uint8_t* data)
{
    uint8_t i;

    save_state(data[1]); // data[1] is the current command id

    // Reply:
    const uint8_t cmd[36] = {
        0xB0, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB1
    };
    for (i = 0; i < sizeof(cmd); i++) {
        fputc(cmd[i], RST);
    }

    fprintf(PC, "Waiting for resetting all power lines...\r\n");
    delay_ms(30000);
    // delay_ms(60000);

    return 0;
}

uint8_t command_rpi_end_of_transmission(uint8_t* data)
{
    scheduled_command_clear_specified_command(0xC0, 0x5E); // Remove all S-band turn off commands
    scheduled_command_clear_specified_command(0xC0, 0xBE); // Remove all RPi turn off commands
    schedule(current_time, { 0xC0, 0x5E, 0x00 });          // Turn off S-band TX now
    schedule(current_time + 1, { 0xC0, 0xBE, 0x02 });      // Rpi off (soft) after 1s
    schedule(current_time + 20, { 0xC0, 0xBE, 0x00 });     // Rpi off (hard) after 20s
    return 0;
}

// Enable / disable S-band TX
uint8_t command_enable_disable_sband(uint8_t* data)
{
    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t enable_disable; // 0 = disabled, 1 = enabled
        uint8_t padding[MSG_LENGTH_PCIB - 3];
    } pcib_pkg;

    uint8_t* ptr = (uint8_t*)&pcib_pkg;
    memset(ptr, 0, MSG_LENGTH_PCIB);
    memcpy(ptr, data, 3);
    pcib_pkg.origin = MSG_OBC;
    checksum_obc(ptr, MSG_LENGTH_PCIB);

    fprintf(PC, "TX ");
    if (pcib_pkg.enable_disable == 0x00) {
        fprintf(PC, "OFF | ");
    } else if (pcib_pkg.enable_disable == 0x01) {
        fprintf(PC, "ON | ");
    }
    uart_print_pc_hex(ptr, MSG_LENGTH_PCIB);

    sband_tx_status = pcib_pkg.enable_disable;
    uart_send_packet_repeat(&uart_port_MSN, ptr, MSG_LENGTH_PCIB, 5, 10); // PCIB -> MSN for structures

    return (pcib_pkg.enable_disable & 0x01);
}

// Enable / disable raspberry pi
uint8_t command_enable_disable_rpi(uint8_t* data)
{
    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t enable_disable; // 0 = disable, 1 = enable, 2 = warn
        uint8_t padding[MSG_LENGTH_PCIB - 3];
    } pcib_pkg;

    uint8_t* ptr = (uint8_t*)&pcib_pkg;
    memset(ptr, 0, MSG_LENGTH_PCIB);
    memcpy(ptr, data, 3);
    pcib_pkg.origin = MSG_OBC;
    checksum_obc(ptr, MSG_LENGTH_PCIB);

    if (pcib_pkg.enable_disable == 0x00) {
        fprintf(PC, "RPI OFF | ");
        rpi_status = pcib_pkg.enable_disable;
    } else if (pcib_pkg.enable_disable == 0x01) {
        fprintf(PC, "RPI ON | ");
        rpi_status = pcib_pkg.enable_disable;
    } else if (pcib_pkg.enable_disable == 0x02) {
        fprintf(PC, "RPI SOFT | ");
        rpi_status = pcib_pkg.enable_disable;
    }

    uart_print_pc_hex(ptr, MSG_LENGTH_PCIB);

    uart_send_packet_repeat(&uart_port_MSN, ptr, MSG_LENGTH_PCIB, 5, 10);

    return pcib_pkg.enable_disable;
}

uint8_t command_rpi_clean_up(uint8_t* data)
{
    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    const uint8_t packet_erase_time = 60; // in seconds
    const uint8_t rpi_turn_on_time = 60;  // in seconds

    fprintf(PC, "\r\nRPi command: clean_up\r\n");

    schedule(current_time + 2, { 0xC0, 0xBE, 0x01 });                                         // Turn on Rpi
    schedule(current_time + rpi_turn_on_time, { 0xC0, 0xCC, 0xC0 });                          // Clean up command
    schedule(current_time + rpi_turn_on_time + packet_erase_time, { 0xC0, 0xBE, 0x02 });      // Turn off Rpi (soft)
    schedule(current_time + rpi_turn_on_time + packet_erase_time + 20, { 0xC0, 0xBE, 0x00 }); // Turn off Rpi (hard)

    return 0;
}

uint8_t command_rpi_clean_logfile(uint8_t* data)
{
    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    const uint8_t packet_erase_time = 60; // in seconds
    const uint8_t rpi_turn_on_time = 60;  // in seconds

    fprintf(PC, "\r\nRPi command: clean_logfile\r\n");

    schedule(current_time + 2, { 0xC0, 0xBE, 0x01 });                                         // Turn on Rpi
    schedule(current_time + rpi_turn_on_time, { 0xC0, 0xCC, 0xC1 });                          // Clean logfile command
    schedule(current_time + rpi_turn_on_time + packet_erase_time, { 0xC0, 0xBE, 0x02 });      // Turn off Rpi (soft)
    schedule(current_time + rpi_turn_on_time + packet_erase_time + 20, { 0xC0, 0xBE, 0x00 }); // Turn off Rpi (hard)

    return 0;
}

uint8_t command_pcib_telemetry(uint8_t* data)
{
    uint8_t i = 1;

    enum {
        length = MSG_LENGTH_PCIB,
        magnetometer_size = 6
    };

    struct packet {
        uint8_t origin;
        uint8_t cmd;
        uint16_t sband_temperature;
        uint8_t sband_tx_status;
        uint8_t rpi_status;
        uint8_t magnetometer[magnetometer_size];
        uint8_t fill[length - 8 - magnetometer_size];
        uint8_t checksum;
        uint8_t footer;
    }* packet = (struct packet*)data;

    response_rx = 1; // Received a reply

    // Pruint8_t for debug:
    fprintf(PC, "RELAY: Ts=%lX, Stx=%d, Rp=%d | ", packet->sband_temperature, packet->sband_tx_status, packet->rpi_status);

    uart_print_pc_hex(data, MSG_LENGTH_PCIB);

    // PCIB telemetry -> OBC telemetry:
    struct pcib_telemetry {
        uint16_t sband_temperature;
        uint8_t sband_tx_status;
        uint8_t rpi_status;
        uint8_t magnetometer[magnetometer_size];
    } pcib_telemetry;

    pcib_telemetry.sband_temperature = packet->sband_temperature;
    pcib_telemetry.sband_tx_status = packet->sband_tx_status;
    pcib_telemetry.rpi_status = packet->rpi_status;
    memcpy(pcib_telemetry.magnetometer, packet->magnetometer, magnetometer_size);

    telemetry_time.pcib_time = current_time;
    memcpy(telemetry.pcib_message, &pcib_telemetry, sizeof(telemetry.pcib_message));

    // Monitor and fix on/off status:
    if (sband_tx_status != packet->sband_tx_status) {
        fprintf(PC, " Stx -> %d", sband_tx_status);
        struct packet_s_tx {
            uint8_t origin;         // 0xC0
            uint8_t command;        // 0x5E
            uint8_t enable_disable; // 0 = disabled, 1 = enabled
            uint8_t padding[MSG_LENGTH_PCIB - 3];
        } packet_s_tx;
        packet_s_tx.origin = 0xC0;
        packet_s_tx.command = 0x5E;
        packet_s_tx.enable_disable = sband_tx_status;
        memset(packet_s_tx.padding, 0, sizeof(packet_s_tx.padding));
        vschedule(current_time + i++, (uint8_t*)&packet_s_tx);
    }
    if (rpi_status != packet->rpi_status) {
        fprintf(PC, " Rpi -> %d", rpi_status);
        struct packet_rpi {
            uint8_t origin;
            uint8_t command;
            uint8_t enable_disable; // 0 = disable, 1 = enable, 2 = warn
            uint8_t padding[MSG_LENGTH_PCIB - 3];
        } packet_rpi;
        packet_rpi.origin = 0xC0;
        packet_rpi.command = 0xBE;
        packet_rpi.enable_disable = rpi_status;
        memset(packet_rpi.padding, 0, sizeof(packet_rpi.padding));
        vschedule(current_time + i++, (uint8_t*)&packet_rpi);
    }

    return 0;
}

uint8_t command_tmcr1_telemetry(uint8_t* data)
{
    enum { length = MSG_LENGTH_TMCR1 };
    struct packet {
        uint8_t origin;
        uint8_t cmd;
        uint8_t fill[length - 4];
        uint8_t checksum;
        uint8_t footer;
    }* packet = (struct packet*)data;

    (void)packet; // TODO remove

    response_rx = 1; // Received a reply

    fprintf(PC, "TMCR1: ");

    uart_print_pc_hex(data, MSG_LENGTH_TMCR1);

    // TODO write TMCR1 data to telemetry

    return 0;
}

uint8_t command_tmcr2_telemetry(uint8_t* data)
{
    enum { length = MSG_LENGTH_TMCR2 };
    struct packet {
        uint8_t origin;
        uint8_t cmd;
        uint8_t fill[length - 4];
        uint8_t checksum;
        uint8_t footer;
    }* packet = (struct packet*)data;

    (void)packet; // TODO remove

    response_rx = 1; // Received a reply

    fprintf(PC, "TMCR2: ");

    uart_print_pc_hex(data, MSG_LENGTH_TMCR2);

    // TODO write TMCR2 data to telemetry

    return 0;
}

uint8_t command_raw_pcib(uint8_t* data)
{
    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    enum { cmd_size = 8 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t pcib_command[cmd_size];
    }* packet = (struct packet*)data;

    if (packet->pcib_command[0] == 0xBE) {
        rpi_status = packet->pcib_command[1];
    } else if (packet->pcib_command[0] == 0x5E) {
        sband_tx_status = packet->pcib_command[1];
    }

    uint8_t pcib_cmd[MSG_LENGTH_PCIB] = { 0 };
    pcib_cmd[0] = 0x0B;
    memcpy(pcib_cmd + 1, packet->pcib_command, cmd_size);
    checksum_obc(pcib_cmd, MSG_LENGTH_PCIB);
    uart_send_packet_repeat(&uart_port_MSN, pcib_cmd, MSG_LENGTH_PCIB, 5, 10);
    return packet->pcib_command[0];
}

uint8_t command_raw_tmcr(uint8_t* data)
{
    enum { cmd_size = 8 };
    struct packet {
        uint8_t origin;
        uint8_t command;
        uint8_t tmcr_command[cmd_size];
    }* packet = (struct packet*)data;

    if(packet->command == 0xCE) {
        if (mux_sel(mux_tmcr1) != mux_tmcr1) { // If MUX did not change
            if (verbose) {
                fprintf(PC, "MUX change failed!");
            }
            return 1;
        }
    } else if(packet->command == 0xCF) {
        if (mux_sel(mux_tmcr2) != mux_tmcr2) { // If MUX did not change
            if (verbose) {
                fprintf(PC, "MUX change failed!");
            }
            return 1;
        }
    } else {
        fprintf(PC, "Unexpected command ID!");
        return -1;
    }

    uint8_t tmcr_cmd[MSG_LENGTH_TMCR1] = { 0 };
    tmcr_cmd[0] = 0x0B;
    memcpy(tmcr_cmd + 1, packet->tmcr_command, cmd_size);
    checksum_obc(tmcr_cmd, MSG_LENGTH_TMCR1);
    uart_send_packet_repeat(&uart_port_MSN, tmcr_cmd, MSG_LENGTH_TMCR1, 5, 10);
    return packet->tmcr_command[0];
}

// Revert s-band configuration command
uint8_t command_sband_defaults(uint8_t* data)
{
    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }
    fprintf(PC, "Reverting s-band configuration... ");
    uint8_t pcib_pkg[MSG_LENGTH_PCIB] = { 0 };
    memset(pcib_pkg, 0, MSG_LENGTH_PCIB);
    memcpy(pcib_pkg, data, 10);
    pcib_pkg[0] = MSG_OBC;
    checksum_obc(pcib_pkg, MSG_LENGTH_PCIB);

    uart_send_packet_repeat(&uart_port_MSN, pcib_pkg, MSG_LENGTH_PCIB, 5, 10);

    return 0;
}

uint8_t command_sband_downlink(uint8_t* data)
{
    enum { param_size = 12 };

    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    struct packet {
        uint8_t origin;  // 0xC0
        uint8_t command; // 0xED
        time_t scheduled_time;
        uint8_t mission_parameters[param_size];
        uint8_t padding[MSG_LENGTH_PCIB - 18];
    }* packet = (struct packet*)data;

    const uint8_t rpi_turn_on_time = 60;  // in seconds
    const uint8_t sband_warm_up_time = 1; // in seconds

    uint16_t bit_offset = 0;

    uint16_t downlink_timeout = get_bits(packet->mission_parameters, &bit_offset, 10);
    uint8_t adcs_mode = get_bits(packet->mission_parameters, &bit_offset, 5);

    downlink_timeout = downlink_timeout ? downlink_timeout : 300; // in seconds; if zero, 300 seconds

    // If not zero, convert and print the scheduled time
    if(packet->scheduled_time) {
        fprintf(PC, "\r\n");

        struct tm* local_time = localtime(&packet->scheduled_time);
        fprintf(PC, "Scheduled Time: %04ld/%02d/%02d %02d:%02d:%02d (0x%08lX)\r\n",
            local_time->tm_year + 1900,
            local_time->tm_mon + 1,
            local_time->tm_mday,
            local_time->tm_hour,
            local_time->tm_min,
            local_time->tm_sec,
            (uint32_t)packet->scheduled_time); // 32 bits

        // Print the deserialized values for verification
        fprintf(PC, "Downlink Timeout: %lu\r\n", downlink_timeout); // 16 bits
        fprintf(PC, "ADCS Mode: %u\r\n", adcs_mode);                // 8 bits
    } else {
        fprintf(PC, "Starting s-band downlink routine.\r\n");
    }

    if (packet->scheduled_time != 0) {
        time_t scheduled_time = packet->scheduled_time - rpi_turn_on_time - sband_warm_up_time;
        packet->scheduled_time = 0;
        vschedule(scheduled_time, data); // Scheduling itself with time set as zero
        if (adcs_mode) {
            struct adcs_mode_st {
                uint8_t origin;
                uint8_t command;
                uint8_t mode;
                uint8_t permanent;
            } adcs_mode_st;
            adcs_mode_st.origin = MSG_COMM;
            adcs_mode_st.command = 0xAD;
            adcs_mode_st.mode = adcs_mode;
            adcs_mode_st.permanent = false;
            vschedule(scheduled_time - 3 * 60 * 60, (uint8_t*)&adcs_mode_st); // ADCS nadir camera 3 hours before
            adcs_mode_st.mode = obc_flags.adcs_initial_value;
            vschedule(scheduled_time + 10 * 60, (uint8_t*)&adcs_mode_st); // ADCS back to default mode after 10m
        }
        fprintf(PC, "Scheduling s-band DL command.");
        return 0;
    }

    packet->command = 0xEE;

    schedule(current_time + 2, { 0xC0, 0xBE, 0x01 });                                                             // Turn on Rpi
    schedule(current_time + rpi_turn_on_time, { 0xC0, 0x5E, 0x01 });                                              // Turn on S-band TX
    vschedule(current_time + rpi_turn_on_time + sband_warm_up_time, data);                                        // Downlink command
    schedule(current_time + rpi_turn_on_time + sband_warm_up_time + downlink_timeout, { 0xC0, 0x5E, 0x00 });      // Turn off S-band TX
    schedule(current_time + rpi_turn_on_time + sband_warm_up_time + downlink_timeout + 1, { 0xC0, 0xBE, 0x02 });  // Turn off Rpi (soft)
    schedule(current_time + rpi_turn_on_time + sband_warm_up_time + downlink_timeout + 21, { 0xC0, 0xBE, 0x00 }); // Turn off Rpi (hard)

    return 0;
}

// End to end test command
uint8_t command_sband_end_to_end(uint8_t* data)
{
    enum { param_size = 12 };

    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    struct packet {
        uint8_t origin;  // 0xC0
        uint8_t command; // 0xEE
        time_t scheduled_time;
        uint8_t mission_parameters[param_size];
        uint8_t padding[MSG_LENGTH_PCIB - 18];
    }* packet = (struct packet*)data;

    uint16_t bit_offset = 0;

    uint16_t downlink_timeout = get_bits(packet->mission_parameters, &bit_offset, 10);
    uint8_t adcs_mode = get_bits(packet->mission_parameters, &bit_offset, 5);
    uint16_t file_number = get_bits(packet->mission_parameters, &bit_offset, 16);
    uint16_t first_packet = get_bits(packet->mission_parameters, &bit_offset, 16);
    uint16_t end_packet = get_bits(packet->mission_parameters, &bit_offset, 16);
    uint8_t number_of_repetitions = get_bits(packet->mission_parameters, &bit_offset, 5);
    uint8_t bitrate = get_bits(packet->mission_parameters, &bit_offset, 3);
    uint8_t power = get_bits(packet->mission_parameters, &bit_offset, 1);
    uint8_t delay_before_tx = get_bits(packet->mission_parameters, &bit_offset, 6);
    uint8_t image_seq_start = get_bits(packet->mission_parameters, &bit_offset, 6);
    uint8_t image_seq_end = get_bits(packet->mission_parameters, &bit_offset, 6);
    uint8_t type = get_bits(packet->mission_parameters, &bit_offset, 3);
    uint8_t camera_selection = get_bits(packet->mission_parameters, &bit_offset, 3);

    downlink_timeout = downlink_timeout ? downlink_timeout : 300; // in seconds; if zero, 300 seconds

    fprintf(PC, "\r\n");

    // Print the deserialized values for verification
    fprintf(PC, "Downlink Timeout: %lu\r\n", downlink_timeout);          // 16 bits
    fprintf(PC, "ADCS Mode: %u\r\n", adcs_mode);                         // 8 bits
    fprintf(PC, "File Number: %lu\r\n", file_number);                    // 16 bits
    fprintf(PC, "First Packet: %lu\r\n", first_packet);                  // 16 bits
    fprintf(PC, "End Packet: %lu\r\n", end_packet);                      // 16 bits
    fprintf(PC, "Number of Repetitions: %u\r\n", number_of_repetitions); // 8 bits

    fprintf(PC, "Bitrate: ");
    switch (bitrate) {
    case 0: fprintf(PC, "10kbps\r\n"); break;
    case 1: fprintf(PC, "20kbps\r\n"); break;
    case 2: fprintf(PC, "25kbps\r\n"); break;
    case 3: fprintf(PC, "50kbps\r\n"); break;
    case 4: fprintf(PC, "64kbps\r\n"); break;
    case 5: fprintf(PC, "100kbps\r\n"); break;
    case 6: fprintf(PC, "250kbps\r\n"); break;
    case 7: fprintf(PC, "500kbps\r\n"); break;
    }

    fprintf(PC, "Power: %u\r\n", power);                      // 8 bits
    fprintf(PC, "Delay Before Tx: %u\r\n", delay_before_tx);  // 8 bits
    fprintf(PC, "Image seq. start: %u\r\n", image_seq_start); // 8 bits
    fprintf(PC, "Image seq. end: %u\r\n", image_seq_end);     // 8 bits
    fprintf(PC, "Type: %u\r\n", type);                        // 8 bits
    fprintf(PC, "Camera Selection: %u%u%u\r\n",
        (camera_selection >> 2) & 0x1,
        (camera_selection >> 1) & 0x1,
        camera_selection & 0x1); // 3 bits

    struct sband_command {
        uint8_t origin;
        uint8_t command;
        uint8_t mission_parameters[param_size];
        uint8_t padding[MSG_LENGTH_PCIB - 14];
    } pcib_pkg;

    uint8_t* ptr = (uint8_t*)&pcib_pkg;
    memset(ptr, 0, MSG_LENGTH_PCIB);
    pcib_pkg.origin = MSG_OBC;
    pcib_pkg.command = 0xEE;
    memcpy(pcib_pkg.mission_parameters, packet->mission_parameters, param_size);
    checksum_obc(ptr, MSG_LENGTH_PCIB);
    uart_print_pc_hex(ptr, MSG_LENGTH_PCIB);

    mux_lock_unlock(true, downlink_timeout);
    uart_send_packet_repeat(&uart_port_MSN, ptr, MSG_LENGTH_PCIB, 5, 10);

    return 0;
}

// S-band test command for anechoic chamber
uint8_t command_sband_test(uint8_t* data)
{
    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    struct cband_test {
        uint8_t origin;
        uint8_t command;
        uint8_t bitrate;
        uint8_t low_high_power;
        uint8_t padding[MSG_LENGTH_PCIB - 4];
    } pcib_pkg;

    uint8_t* ptr = (uint8_t*)&pcib_pkg;
    memset(ptr, 0, MSG_LENGTH_PCIB);
    memcpy(ptr, data, 10);
    pcib_pkg.origin = MSG_OBC;
    checksum_obc(ptr, MSG_LENGTH_PCIB);
    uart_print_pc_hex(ptr, MSG_LENGTH_PCIB);

    fprintf(PC, " S-band test command: ");

    switch (pcib_pkg.bitrate) {
    case 0: fprintf(PC, "10kbps, "); break;
    case 1: fprintf(PC, "20kbps, "); break;
    case 2: fprintf(PC, "25kbps, "); break;
    case 3: fprintf(PC, "50kbps, "); break;
    case 4: fprintf(PC, "64kbps, "); break;
    case 5: fprintf(PC, "100kbps, "); break;
    case 6: fprintf(PC, "250kbps, "); break;
    case 7: fprintf(PC, "500kbps, "); break;
    default: fprintf(PC, "invalid bitrate."); return 1;
    }

    switch (pcib_pkg.low_high_power) {
    case 0: fprintf(PC, "low power."); break;
    case 1: fprintf(PC, "high power."); break;
    default: fprintf(PC, "invalid power."); return 2;
    }

    uart_send_packet_repeat(&uart_port_MSN, ptr, MSG_LENGTH_PCIB, 5, 10);

    return 0;
}

// Data downlink from S-band through Relay PIC
uint8_t command_relay_data_downlink_from_flash(uint8_t* data)
{
    enum { data_size = 16 };
    if (mux_sel(mux_pcib) != mux_pcib) { // If MUX did not change
        if (verbose) {
            fprintf(PC, "MUX change failed!");
        }
        return 1;
    }

    uint8_t cmd[MSG_LENGTH_PCIB] = {0};

    struct relay_command {
        uint8_t origin;
        uint8_t command;
        uint8_t data[data_size];
    }* relay_command = (struct relay_command*)cmd;

    relay_command->origin = MSG_OBC;
    relay_command->command = data[1];
    memcpy(relay_command->data, data + 2, data_size);
    checksum_obc(cmd, MSG_LENGTH_PCIB);
    uart_print_pc_hex(cmd, MSG_LENGTH_PCIB);

    uint8_t duration = data[6];

    uint8_t mux_status = 0;

    if(relay_command->command == 0x36) {
        mux_status = mux_lock_unlock(true, ((time_t)duration) * 60);
    }

    uart_send_packet_repeat(&uart_port_MSN, cmd, MSG_LENGTH_PCIB, 5, 10);

    return mux_status;
}

// =====================================================

typedef struct mission {
    time_t mission_time;                            // time when the command is sent to the mission (Unix time)
    uint16_t mission_duration;                      // how long does it take to execute the mission in seconds; 0 = do not turn off
    uint8_t adcs_mode;                              // adcs mode during mission
    uint16_t adcs_maneuver_duration;                // how early to change the ADCS before sending the mission command (in seconds); 0 = never
    uint16_t mission_on_time;                       // how early to turn on the mission before sending the mission command (in seconds)
    uint8_t turn_on_value;                          // active high will turn on the subsystem
    uint16_t pin_number;                            // pin number to turn on/off the subsystem; 0 = do not change
    uart_fn* command_port;                          // UART port to send the command
    uart_fn* on_off_command_port;                   // UART port to send on/off commands
    uint8_t mux_position_command;                   // mux position when sending mission commands
    uint8_t mux_position_on_off;                    // mux position when sending on/off commands
    uint8_t command[MSG_LENGTH_PCIB];               // the command to be sent to the subsystem; checksum and footer is calculated automatically
    uint8_t command_size;                           // the size of the command above; 0 = do not send
    uint8_t turn_on_command[MSG_LENGTH_PCIB];       // the command to be sent to the subsystem; checksum and footer is calculated automatically
    uint8_t turn_on_command_size;                   // the size of the command above; 0 = do not send
    uint8_t turn_off_soft_command[MSG_LENGTH_PCIB]; // the command to be sent to the subsystem; checksum and footer is calculated automatically
    uint8_t turn_off_soft_command_size;             // the size of the command above; 0 = do not send
    uint8_t turn_off_hard_command[MSG_LENGTH_PCIB]; // the command to be sent to the subsystem; checksum and footer is calculated automatically
    uint8_t turn_off_hard_command_size;             // the size of the command above; 0 = do not send
    uint16_t turn_off_duration;                     // time it takes between sending the turn off warning and mission power off (in seconds); 0 = never turn off
    uint8_t mux_lock;                               // if true, the mux position is locked for mux_lock_duration seconds
    uint16_t mux_lock_duration;                     // duration to lock the mux seconds
} mission;

mission camera = {
    0,                        // mission_time
    65,                       // mission_duration
    adcs_mode_horizon_camera, // adcs_mode
    3 * 60 * 60,              // adcs_maneuver_duration (3 hours)
    60,                       // mission_on_time
    true,                     // turn_on_value
    0,                        // pin_number
    &uart_port_MSN,           // command_port
    &uart_port_MSN,           // on_off_port
    mux_pcib,                 // mux_position_command
    mux_pcib,                 // mux_position_on_off
    { 0 },                    // command
    MSG_LENGTH_PCIB,          // command_size
    { MSG_OBC, 0xBE, 0x01 },  // turn_on_command
    MSG_LENGTH_PCIB,          // turn_on_command_size
    { MSG_OBC, 0xBE, 0x02 },  // turn_off_soft_command
    MSG_LENGTH_PCIB,          // turn_off_soft_command_size
    { MSG_OBC, 0xBE, 0x00 },  // turn_off_hard_command
    MSG_LENGTH_PCIB,          // turn_off_hard_command_size
    20,                       // turn_off_duration
    false,                    // mux_lock
    65                        // mux_lock_duration
};

// TODO: Add copy command at the end of the mission

// Controls the mission execution
uint8_t command_generic_mission(mission *m, uint8_t data[])
{
    uint8_t* state = &data[2];        // The state is kept in the command
    uint8_t* adcs_enabled = &data[3]; // ADCS enabled flag is also kept in the command

    enum state_machine {
        state_schedule = 0,
        state_initial_adcs_mode = 1,
        state_turn_on_subsystem = 2,
        state_send_uplink_command = 3,
        state_turn_off_subsystem_soft = 4,
        state_turn_off_subsystem_hard = 5,
        state_end = 6
    };

    // Print the current state
    switch (*state) {
    case state_schedule: fprintf(PC, "State: schedule\n"); break;
    case state_initial_adcs_mode: fprintf(PC, "State: initial adcs mode\n"); break;
    case state_turn_on_subsystem: fprintf(PC, "State: turn on subsystem\n"); break;
    case state_send_uplink_command: fprintf(PC, "State: send uplink command\n"); break;
    case state_turn_off_subsystem_soft: fprintf(PC, "State: turn off subsystem soft\n"); break;
    case state_turn_off_subsystem_hard: fprintf(PC, "State: turn off subsystem hard\n"); break;
    case state_end: fprintf(PC, "State: end\n"); break;
    default: fprintf(PC, "Unknown state\n"); break;
    }

    // Execute the actions for the current state
    switch (*state) {
    case state_schedule:
        break;
    case state_initial_adcs_mode:
        change_adcs_mode(m->adcs_mode, false);
        break;
    case state_turn_on_subsystem:
        if (m->pin_number) {
            output_bit(m->pin_number, m->turn_on_value);
        }
        if (m->turn_on_command_size) {
            checksum_obc(m->turn_on_command, m->turn_on_command_size);
            mux_sel(m->mux_position_on_off);
            uart_send_packet(m->on_off_command_port, m->turn_on_command, m->turn_on_command_size);
            if(m->turn_on_command[1] == 0xBE) { // Workaround to set the monitor byte
                rpi_status = m->turn_on_command[2];
            }
        }
        break;
    case state_send_uplink_command:
        checksum_obc(m->command, m->command_size);
        mux_sel(m->mux_position_command);
        if(m->mux_lock) {
            mux_lock_unlock(true, m->mux_lock_duration);
        }
        uart_send_packet(m->command_port, m->command, m->command_size);
        break;
    case state_turn_off_subsystem_soft:
        if (*adcs_enabled && m->adcs_maneuver_duration) {
            change_adcs_mode(obc_flags.adcs_initial_value, false);
        }
        if (m->turn_off_soft_command_size) {
            checksum_obc(m->turn_off_soft_command, m->turn_off_soft_command_size);
            mux_sel(m->mux_position_on_off);
            uart_send_packet(m->on_off_command_port, m->turn_off_soft_command, m->turn_off_soft_command_size);
            if(m->turn_off_soft_command[1] == 0xBE){ // Workaround to set the monitor byte
                rpi_status = m->turn_off_soft_command[2];
            }
        }
        break;
    case state_turn_off_subsystem_hard:
        if (m->pin_number) {
            output_bit(m->pin_number, !m->turn_on_value);
        }
        if (m->turn_off_hard_command_size) {
            checksum_obc(m->turn_off_hard_command, m->turn_off_hard_command_size);
            mux_sel(m->mux_position_on_off);
            uart_send_packet(m->on_off_command_port, m->turn_off_hard_command, m->turn_off_hard_command_size);
            if(m->turn_off_hard_command[1] == 0xBE){ // Workaround to set the monitor byte
                rpi_status = m->turn_off_hard_command[2];
            }
        }
        break;
    default:
        fprintf(PC, "Invalid state!");
        return *state;
    }

    // Determine the next state
    switch (*state) {
    case state_schedule:
        *adcs_enabled = m->mission_time ? true : false; // Disable the ADCS maneuvers if not a scheduled command
        if(m->mission_time == 0){
            if(m->mission_on_time == 0) {
                m->mission_time = current_time;
            } else {
                m->mission_time = current_time + m->mission_on_time;
            }
        }
        if (*adcs_enabled) {
            *state = state_initial_adcs_mode;
            vschedule(m->mission_time - m->adcs_maneuver_duration, data);
            break;
        }
    case state_initial_adcs_mode:
        if (m->mission_on_time) {
            *state = state_turn_on_subsystem;
            vschedule(m->mission_time - m->mission_on_time, data);
            break;
        }
    case state_turn_on_subsystem:
        *state = state_send_uplink_command;
        vschedule(m->mission_time, data);
        break;
    case state_send_uplink_command:
        if (m->mission_duration) {
            *state = state_turn_off_subsystem_soft;
            vschedule(m->mission_time + m->mission_duration, data);
            break;
        }
    case state_turn_off_subsystem_soft:
        if (m->turn_off_duration) {
            *state = state_turn_off_subsystem_hard;
            vschedule(m->mission_time + m->mission_duration + m->turn_off_duration, data);
            break;
        }
    case state_turn_off_subsystem_hard:
    default:
        *state = state_end;
        break;
    }

    return 0;
}

uint16_t camera_still_duration(uint8_t* data)
{
    enum constants {
        camera_config_length = 11,
        camera_capture_length = 4,
        a = 1,
        b = 60
    };

    struct rpi_camera_still {
        uint8_t origin;
        uint8_t command;
        uint8_t data[camera_config_length + camera_capture_length];
    }* packet = (struct rpi_camera_still*)data;

    uint16_t bitOffset = 0;

    // Camera capture configuration command
    uint32_t shutterSpeed[3];

    for (int i = 0; i < 3; ++i) {
        uint8_t shutterSpeedMantissa = get_bits(packet->data, &bitOffset, 4);
        uint8_t shutterSpeedExponent = get_bits(packet->data, &bitOffset, 3);
        shutterSpeed[i] = (double)shutterSpeedMantissa * pow(10, shutterSpeedExponent) + 0.5; // Using base 10 exponentiation

        bitOffset += 20;
    }

    bitOffset += 7;

    // Camera capture command
    uint8_t cameraSelection[3];
    cameraSelection[0] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[1] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[2] = get_bits(packet->data, &bitOffset, 1);

    bitOffset += 16;
    uint8_t sequentialPictures = get_bits(packet->data, &bitOffset, 6);
    uint8_t intervalBetweenPictures = get_bits(packet->data, &bitOffset, 7);

    // Print human-readable output to PC
    if (verbose) {
        fprintf(PC,
            "Camera capture configuration:\r\n"
            "Camera Selection: %u, %u, %u\r\n"
            "Sequential Pictures: %u\r\n"
            "Interval Between Pictures: %u seconds\r\n"
            "Shutter Speeds: %lu, %lu, %lu (microseconds)\r\n",
            cameraSelection[0],                               // 8-bit value
            cameraSelection[1],                               // 8-bit value
            cameraSelection[2],                               // 8-bit value
            sequentialPictures,                               // 8-bit value
            intervalBetweenPictures,                          // 8-bit value
            shutterSpeed[0], shutterSpeed[1], shutterSpeed[2] // 32-bit values
        );
    }

    uint16_t delay = (uint16_t)sequentialPictures * ((uint16_t)intervalBetweenPictures + (a * cameraSelection[0] * shutterSpeed[0] / 1000000L + a * cameraSelection[1] * shutterSpeed[1] / 1000000L + a * cameraSelection[2] * shutterSpeed[2] / 1000000L)) + b;

    if (verbose) {
        fprintf(PC, "Delay: %lu\r\n", delay);
    }

    return delay;
}

uint16_t camera_image_transform_duration(uint8_t* data)
{
    enum constants {
        camera_image_transform_length = 11,
        a = 20,
        b = 20
    };

    struct rpi_camera_transform {
        uint8_t origin;
        uint8_t command;
        uint8_t data[camera_image_transform_length];
    }* packet = (struct rpi_camera_transform*)data;

    uint16_t bitOffset = 85;

    uint8_t cameraSelection[3];
    cameraSelection[0] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[1] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[2] = get_bits(packet->data, &bitOffset, 1);

    // Print human-readable output to PC
    if (verbose) {
        fprintf(PC,
            "Camera Selection: %u%u%u\r\n",
            cameraSelection[0],
            cameraSelection[1],
            cameraSelection[2]);
    }

    uint16_t delay = (uint16_t)cameraSelection[0] * a + (uint16_t)cameraSelection[1] * a + (uint16_t)cameraSelection[2] * a + b;

    if (verbose) {
        fprintf(PC, "Delay: %lu\r\n", delay);
    }

    return delay;
}

uint16_t camera_video_duration(uint8_t* data)
{
    enum constants {
        camera_video_length = 11,
        a = 20
    };

    struct rpi_camera_video {
        uint8_t origin;
        uint8_t command;
        uint8_t data[camera_video_length];
    }* packet = (struct rpi_camera_video*)data;

    uint16_t bitOffset = 21; // Skip ADCS mode and ADCS maneuver duration

    uint8_t cameraSelection[3];
    cameraSelection[0] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[1] = get_bits(packet->data, &bitOffset, 1);
    cameraSelection[2] = get_bits(packet->data, &bitOffset, 1);

    bitOffset += 16;

    uint8_t timeout[3];

    for (int i = 0; i < 3; ++i) {
        bitOffset += 10;
        timeout[i] = get_bits(packet->data, &bitOffset, 6);
    }

    // Print human-readable output to PC
    if (verbose) {
        fprintf(PC,
            "Video capture command:\r\n"
            "Camera Selection: %u%u%u\r\n"
            "Timeout: %u, %u, %u seconds\r\n",
            cameraSelection[0],
            cameraSelection[1],
            cameraSelection[2],
            timeout[0], timeout[1], timeout[2]);
    }

    uint16_t delay = (uint16_t)cameraSelection[0] * timeout[0] + (uint16_t)cameraSelection[1] * timeout[1] + (uint16_t)cameraSelection[2] * timeout[2] + a;

    if (verbose) {
        fprintf(PC, "Delay: %lu\r\n", delay);
    }

    return delay;
}

uint8_t command_camera_mission(uint8_t* data)
{
    enum constants {
        camera_config_length = 11,
        camera_capture_length = 4,
        camera_image_transform_length = 11,
        camera_video_length = 11,
        xmodem_rpi_to_bus_length = 4,
    };

    enum inner_commands {
        camera_capture_configuration = 0,
        camera_capture = 1,
        image_transformation = 2,
        video_capture = 3,
        xmodem_rpi_to_bus = 4
    };

    uint8_t inner_command = data[2];

    if (inner_command == camera_capture_configuration) {

        struct packet_0 {
            uint8_t origin;
            uint8_t cmd;
            uint8_t inner_command;
            uint8_t data[camera_config_length];
        }* packet_0 = (struct packet_0*)data;

        fprintf(PC, "Camera capture configuration command, saving configuration...");
        memcpy(obc_flags.camera_parameters, packet_0->data, camera_config_length);
        save_state(packet_0->cmd);
        fprintf(PC, " done!");

    } else if (inner_command == camera_capture) {

        struct packet_1 {
            uint8_t origin;
            uint8_t cmd;
            uint8_t inner_command;
            time_t start_time;
            uint8_t data[camera_capture_length];
            uint8_t adcs_mode;
            uint16_t adcs_maneuver_duration;
        }* packet_1 = (struct packet_1*)data;

        fprintf(PC, "Camera capture command, starting state machine...");

        camera.mission_time = packet_1->start_time;
        camera.adcs_maneuver_duration = packet_1->adcs_maneuver_duration;
        camera.mux_lock = false;
        camera.adcs_mode = packet_1->adcs_mode;

        camera.command[0] = MSG_OBC;
        camera.command[1] = 0xCA;
        memcpy(&camera.command[2], obc_flags.camera_parameters, camera_config_length);            // first part of the config comes from the obc flags
        memcpy(&camera.command[2 + camera_config_length], packet_1->data, camera_capture_length); // second part comes from this command
        camera.mission_duration = camera_still_duration(camera.command);
        camera.mux_lock_duration = camera.mission_duration;

        schedule(current_time, { MSG_COMM, 0xCB, 0x00 }); // Start the state machine

    } else if (inner_command == image_transformation) {

        struct packet_2 {
            uint8_t origin;
            uint8_t cmd;
            uint8_t inner_command;
            time_t start_time;
            uint8_t data[camera_image_transform_length];
        }* packet_2 = (struct packet_2*)data;

        fprintf(PC, "Camera image transformation command, starting state machine...");

        camera.mission_time = packet_2->start_time;
        camera.adcs_maneuver_duration = 0;
        camera.mux_lock = false;

        camera.command[0] = MSG_OBC;
        camera.command[1] = 0xCB;
        memcpy(&camera.command[2], packet_2->data, camera_image_transform_length);
        camera.mission_duration = camera_image_transform_duration(camera.command);
        camera.mux_lock_duration = camera.mission_duration;

        schedule(current_time, { MSG_COMM, 0xCB, 0x00 }); // Start the state machine

    } else if (inner_command == video_capture) {

        struct packet_3 {
            uint8_t origin;
            uint8_t cmd;
            uint8_t inner_command;
            time_t start_time;
            uint8_t data[camera_video_length];
        }* packet_3 = (struct packet_3*)data;

        fprintf(PC, "Camera image video command, starting state machine...");

        camera.mission_time = packet_3->start_time;
        camera.adcs_maneuver_duration = ((packet_3->data[0] & 0xE0) >> 5);
        camera.adcs_maneuver_duration |= ((uint16_t)packet_3->data[1] << 3);
        camera.adcs_maneuver_duration |= ((uint16_t)(packet_3->data[2] & 0x1F) << 11);
        camera.mux_lock = false;
        camera.adcs_mode = packet_3->data[0] & 0x1F;

        camera.command[0] = MSG_OBC;
        camera.command[1] = 0xCC;
        memcpy(&camera.command[2], packet_3->data, camera_video_length);
        camera.mission_duration = camera_video_duration(camera.command);
        camera.mux_lock_duration = camera.mission_duration;

        schedule(current_time, { MSG_COMM, 0xCB, 0x00 }); // Start the state machine

    } else if (inner_command == xmodem_rpi_to_bus) {

        struct packet_4 {
            uint8_t origin;
            uint8_t cmd;
            uint8_t inner_command;
            time_t start_time;
            uint16_t timeout;
            uint8_t data[xmodem_rpi_to_bus_length];
        }* packet_4 = (struct packet_4*)data;

        fprintf(PC, "XMODEM copy command RPi to bus, starting state machine...");

        camera.mission_time = packet_4->start_time;
        camera.mission_duration = packet_4->timeout;
        camera.adcs_maneuver_duration = 0;
        camera.mux_lock = true;
        camera.mux_lock_duration = camera.mission_duration;

        camera.command[0] = MSG_OBC;
        camera.command[1] = 0xC2;
        memcpy(&camera.command[2], packet_4->data, xmodem_rpi_to_bus_length);

        schedule(current_time, { MSG_COMM, 0xCB, 0x00 }); // Start the state machine

    } else {
        fprintf(PC, "Inner command not implemented.");
    }

    return inner_command;
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

        // Arranged in Alphanumeric
        case 0x1CAA: error = command_tmcr1_telemetry(data); break;
        case 0x1DAA: error = command_tmcr2_telemetry(data); break;
        case 0xABAA: error = command_pcib_telemetry(data); break;
        case 0xAB02: error = command_mux_lock_unlock(data); break;
        case 0xABF0: error = command_rpi_end_of_transmission(data); break;
        case 0xAD90: error = command_adcs_telemetry(data); break;
        case 0xADDA: error = command_adcs_gps_time(data); break;
        case 0xB0A0: error = command_reset_telemetry(data); break;
        case 0xB0A2: error = command_reset_warning(data); break;
        case 0xC000: error = command_print_memory_address(data); break;
        case 0xC001: error = command_set_clock(data); break;
        case 0xC002: error = command_mux_lock_unlock(data); break;
        case 0xC003: error = command_set_obc_variable(data); break;
        case 0xC008: error = command_xmodem_receive_sector(data); break;
        case 0xC009: error = command_xmodem_send_sector(data); break;
        case 0xC00A: error = command_copy_memory_page(data); break;
        case 0xC00B: error = command_copy_memory_sector(data); break;
        case 0xC00C: error = command_erase_memory_page(data); break;
        case 0xC00D: error = command_erase_memory_sector(data); break;
        case 0xC00F: error = command_sel1_a(data); break;
        // case 0xC010: reserved for COM
        // case 0xC011: reserved for COM
        case 0xC012: error = command_sel1_d(data); break;
        // case 0xC013: reserved for COM
        // case 0xC014: reserved for COM
        // case 0xC015: reserved for COM
        // case 0xC016: reserved for COM
        // case 0xC017: reserved for COM
        case 0xC018: error = command_sel1_b(data); break;
        case 0xC019: error = command_sel1_c(data); break;
        case 0xC01A: error = command_sel1_zes(data); break; // Reassigned from 0xC010
        case 0xC01B: error = command_sel2_ref(data); break; // Reassigned from 0xC010
        case 0xC01F: error = command_sel2_a(data); break;
        // case 0xC020: reserved for COM
        // case 0xC021: reserved for COM
        // case 0xC022: reserved for COM
        // case 0xC023: reserved for COM
        // case 0xC024: reserved for COM
        case 0xC02A: error = command_mcp(data); break;
        case 0xC025: error = command_boot_cmd_clear_nth(data); break;
        case 0xC026: error = command_boot_cmd_clear_all(data); break;
        // case 0xC027: reserved for COM
        case 0xC028: error = command_boot_cmd_add(data); break;
        // case 0xC02B: reserved for COM
        // case 0xC02C: reserved for COM
        // case 0xC02D: reserved for COM
        // case 0xC02E: reserved for COM
        // case 0xC02F: reserved for COM
        case 0xC031: error = command_sel2_b(data); break;
        // case 0xC035: reserved for COM
        case 0xC036: error = command_relay_data_downlink_from_flash(data); break; // From Mission FM
        case 0xC037: error = command_relay_data_downlink_from_flash(data); break; // From Local FM
        case 0xC03A: error = command_adcs_raw_part_a(data); break;
        case 0xC03B: error = command_adcs_raw_part_b(data); break;
        case 0xC03C: error = command_adcs_raw_part_c(data); break;
        // case 0xC040: reserved for COM
        // case 0xC041: reserved for COM
        case 0xC042: error = command_uhf_message(data); break;
        // case 0xC044: reserved for COM
        case 0xC050: error = command_com_cw(data); break;
        case 0xC055: error = command_save_state(data); break;
        case 0xC058: error = command_com_access_change(data); break;
        case 0xC059: error = command_com_access_request(data); break;
        case 0xC05E: error = command_enable_disable_sband(data); break;
        case 0xC060: error = command_obc_kill_on(data); break;
        case 0xC061: error = command_obc_kill_off(data); break;
        case 0xC06C: error = command_send_data_to_eps(data); break;
        case 0xC06D: error = command_eps_set_heater_ref(data); break;
        case 0xC070: error = command_stm32_raw_8_16(data); break;
        case 0xC071: error = command_stm32_raw_uhf32(data); break;
        case 0xC072: error = command_stm32_raw_uhf32(data); break;
        case 0xC080: error = command_sel2_c(data); break;
        case 0xC081: error = command_sel2_d(data); break;
        case 0xC090: error = command_request_reset(data); break;
        case 0xC091: error = command_request_eps(data); break;
        case 0xC092: error = command_request_mission_control_PIC_status(data); break;
        case 0xC093: error = command_request_adcs(data); break;
        case 0xC094: error = command_request_pcib(data); break;
        case 0xC095: error = command_request_tmcr1(data); break;
        case 0xC096: error = command_request_tmcr2(data); break;
        case 0xC0A0: error = command_reset_telemetry(data); break;
        case 0xC0A1: error = command_stm32_raw_uhf32_tle(data); break;
        case 0xC0A2: error = command_stm32_raw_uhf32_tle(data); break;
        case 0xC0A6: error = command_adcs_default_mode(data); break;
        case 0xC0AA: error = command_to_deploy_SAP(data); break;
        case 0xC0AB: error = command_to_deploy_SMA(data); break;
        case 0xC0AC: error = command_adcs_comm_test(data); break;
        case 0xC0AD: error = command_adcs_mode(data); break;
        case 0xC0AE: error = command_ocp_state(data); break;
        case 0xC0AF: error = command_adcs_raw(data); break;
        case 0xC0B7: error = command_reset_all_power_lines(data); break;
        case 0xC0BE: error = command_enable_disable_rpi(data); break;
        case 0xC0C0: error = command_rpi_clean_up(data); break;
        case 0xC0C1: error = command_rpi_clean_logfile(data); break;
        case 0xC0C5: error = command_clear_state(data); break;
        case 0xC0CA: error = command_camera_mission(data); break;
        case 0xC0CB: error = command_generic_mission(&camera, data); break;
        case 0xC0CC: error = command_raw_pcib(data); break;
        case 0xC0CD: error = command_change_cw_mode_flags(data); break;
        case 0xC0CE: error = command_raw_tmcr(data); break;
        case 0xC0CF: error = command_raw_tmcr(data); break;
        case 0xC0D0: error = command_xmodem_send(data); break;
        case 0xC0D1: error = command_xmodem_receive(data); break;
        case 0xC0D2: error = command_iMTQ_Dipole_Actuation(data); break;
        case 0xC0D3: error = command_iMTQ_No_Opeartion(data); break;
        case 0xC0D4: error = command_3v3_Enable(data); break;
        case 0xC0D5: error = command_RW_Motor_Enable(data); break;
        case 0xC0D6: error = command_RW_Enable(data); break;
        case 0xC0D7: error = command_RW_Speed(data); break;
        case 0xC0D8: error = command_RW_Reset(data); break;
        case 0xC0DA: error = command_deploy_antenna(data); break;
        case 0xC0DB: error = command_debug(data); break;
        case 0xC0DD: error = command_dump_memory(data); break;
        case 0xC0DE: error = command_sband_defaults(data); break;
        case 0xC0DF: error = command_get_tris(data); break;
        case 0xC0EA: error = command_copy_mission_to_com(data); break;
        case 0xC0EB: error = command_copy_opera_to_com(data); break;
        case 0xC0ED: error = command_sband_downlink(data); break;
        case 0xC0EE: error = command_sband_end_to_end(data); break;
        case 0xC0EF: error = command_sband_test(data); break;
        case 0xC0F0: error = command_mux_sel_sfm(data); break;
        case 0xC0F5: error = command_send_data_to_reset(data); break;
        case 0xC0F6: error = command_schedule_anything(data); break;
        case 0xC0F7: error = command_schedule_mode(data); break;
        case 0xC0F8: error = command_save_telemetry(data); break;
        case 0xC0F9: error = command_clear_all_schedule_commands(data); break;
        case 0xC0FA: error = command_print_flags(data); break;
        case 0xC0FB: error = command_opera_GA_part_a(data); break;
        case 0xC0FC: error = command_opera_GA_part_b(data); break;
        case 0xC0FD: error = command_opera_GA_single(data); break;
        case 0xC0FE: error = command_boot_flag_set(data); break;
        case 0xC0FF: error = command_opera_full(data); break;
        case 0xCB00: error = command_mission_control_PIC_status(data); break;
        case 0xDBFF: error = command_reset_log(data); break;
        case 0xE033: error = command_eps_telemetry(data); break;

        default: error = command_get_tris(data); break;
        }

        if (log_enabled) {
            fprintf(PC, " (%s R:%02X)", mux_str_list[mux_cpld_position], error);
        }

        // Satellite log
        log.time = current_time;
        log.return_value = error;

        if (
            log_enabled
            && command != 0x1CAA  // TMCR1 PIC
            && command != 0x1DAA  // TMCR2 PIC
            && command != 0xABAA  // RELAY PIC
            && command != 0xB0A0  // Reset PIC
            && command != 0xE033  // EPS PIC
            && command != 0xCB00  // Mission control pic
            && command != 0xAD90  // ADCS PIC
            && command != 0xC050  // COM PIC
            && command != 0xC0F8) // Telemetry record
            log_add(log);

        if (log_enabled)
            fprintf(PC, "\r\n");
    }
}

#endif /* INTERPRETER_H */
