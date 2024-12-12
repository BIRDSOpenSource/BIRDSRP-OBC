#ifndef MEMORY_SETUP_H
#define MEMORY_SETUP_H

#include <definitions.h>
#include <flash_memory.h>
#include <boot_command.h>
#include <interpreter.h>

void initialize_memory()
{
    uint8_t* boot_flag_ptr = (uint8_t*)&boot_flags;
    uint8_t* obc_flag_ptr = (uint8_t*)&obc_flags;

    // Initialize flags
    boot_flags.deployment_flag = 0;
    flash_initialize_flash_ctrl(FLASH_ADDR_START, FLASH_ADDR_END, FLASH_ADDR_START, FLASH_ADDR_DELTA, &addr_flags.flash_addr);
    flash_initialize_flash_ctrl(FLASH_LOG_START, FLASH_LOG_END, FLASH_LOG_START, FLASH_LOG_DELTA, &addr_flags.flash_log);
    flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag, 0xFFFFFFFF, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, false, &addr_flags.flash_telemetry);
    flash_initialize_flash_ctrl(FLASH_SEL_ZES_START, FLASH_SEL_ZES_END, FLASH_SEL_ZES_START, FLASH_SEL_ZES_DELTA, &addr_flags.flash_sel_zes);
    flash_initialize_flash_ctrl(FLASH_SEL_REF_START, FLASH_SEL_REF_END, FLASH_SEL_REF_START, FLASH_SEL_REF_DELTA, &addr_flags.flash_sel_ref);

    // Save boot flags
    flash_erase_pages(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, BOOT_FLAGS_ADDRESS + sizeof(boot_flags));
    flash_transfer_data_from_ram(&spi_port_COM_FM, BOOT_FLAGS_ADDRESS, boot_flag_ptr, sizeof(boot_flags));

    // Save OBC flags
    flash_erase_pages(&spi_port_COM_FM, OBC_FLAGS_ADDRESS, OBC_FLAGS_ADDRESS + sizeof(obc_flags));
    flash_transfer_data_from_ram(&spi_port_COM_FM, OBC_FLAGS_ADDRESS, obc_flag_ptr, sizeof(obc_flags));

    // Initialize boot commands
    boot_commands_clear_all();
    boot_commands_write();

    // Save addresses
    address_rotation addr;

    uint8_t* addr_flag_ptr = (uint8_t*)&addr;
    addr.flash_log_current = addr_flags.flash_log.current;
    addr.flash_telemetry_current = addr_flags.flash_telemetry.current;
    addr.flash_sel_zes_current = addr_flags.flash_sel_zes.current;
    addr.flash_sel_ref_current = addr_flags.flash_sel_ref.current;

    flash_erase_pages(&spi_port_COM_FM, ADDR_FLAGS_ADDRESS, ADDR_FLAGS_ADDRESS + sizeof(addr));
    flash_cycle_write(&spi_port_COM_FM, addr_flag_ptr, &addr_flags.flash_addr);
    // [!] Addresses are also saved in interpreter.h in command_save_telemetry
}

void retrieve_memory()
{
    // Load OBC flags
    uint8_t* obc_flag_ptr = (uint8_t*)&obc_flags;
    flash_transfer_data_to_ram(
        &spi_port_COM_FM,
        OBC_FLAGS_ADDRESS,
        obc_flag_ptr,
        sizeof(obc_flags));

    // Load addresses of addresses
    flash_initialize_flash_ctrl(FLASH_ADDR_START, FLASH_ADDR_END, FLASH_ADDR_START, FLASH_ADDR_DELTA, &addr_flags.flash_addr);
    flash_recover_last_addr(&spi_port_COM_FM, &addr_flags.flash_addr);
    unsigned long long address_address;
    if (addr_flags.flash_addr.current >= addr_flags.flash_addr.start + addr_flags.flash_addr.delta) {
        address_address = addr_flags.flash_addr.current - addr_flags.flash_addr.delta;
    } else {
        address_address = addr_flags.flash_addr.end - addr_flags.flash_addr.delta;
    }

    // Load addresses
    address_rotation addr;

    uint8_t* addr_flag_ptr = (uint8_t*)&addr;
    flash_transfer_data_to_ram(
        &spi_port_COM_FM,
        address_address,
        addr_flag_ptr,
        sizeof(addr));
    flash_initialize_flash_ctrl(FLASH_LOG_START, FLASH_LOG_END, addr.flash_log_current, FLASH_LOG_DELTA, &addr_flags.flash_log);
    flash_initialize_flash_ctrl_from_memory_date_based(FLASH_TELEMETRY_SECTORS_PER_DAY, boot_flags.deployment_flag, addr.flash_telemetry_current, FLASH_TELEMETRY_START, FLASH_TELEMETRY_DELTA, false, &addr_flags.flash_telemetry);
    flash_initialize_flash_ctrl(FLASH_SEL_ZES_START, FLASH_SEL_ZES_END, addr.flash_sel_zes_current, FLASH_SEL_ZES_DELTA, &addr_flags.flash_sel_zes);
    flash_initialize_flash_ctrl(FLASH_SEL_REF_START, FLASH_SEL_REF_END, addr.flash_sel_ref_current, FLASH_SEL_REF_DELTA, &addr_flags.flash_sel_ref);
    boot_commands_read();
}

void initial_deployment_setup()
{
    uint8_t* cmd_ptr = (uint8_t*)scheduled_commands;
    // Antenna deployment schedule
    scheduler_initialize();
    flash_transfer_data_from_ram(&spi_port_COM_FM, SCHEDULED_CMD_ADDRESS, cmd_ptr, sizeof(scheduled_commands));
    current_time = T0 + (time_t)boot_flags.deployment_flag * 24L * 60L * 60L; // Set clock to as early as possible (Jan 1st 2000, 00:00:00), increasing every day by the deployment flag;
    SetTimeSec(current_time);
    previous_time = current_time;

    struct antenna_deployment_command {
        uint8_t origin;
        uint8_t command;
        uint8_t duration;
    } cmd;

    uint8_t factor = boot_flags.deployment_flag > 3 ? 3 : boot_flags.deployment_flag;

    cmd.origin = MSG_COMM;
    cmd.command = 0xDA;
    cmd.duration = 30 + 10 * factor;

    vschedule(T_ANTENNA + (time_t)boot_flags.deployment_flag * 24L * 60L * 60L, (uint8_t*)&cmd); // Antenna deploy command (Jan 1st 2000, 00:30:30), 30s.
}

void nominal_setup()
{
    // Normal boot.
    rst_clock_update = 1;   // Update clock with reset pic
    scheduler_initialize(); // Needed if not loading from FM
    // scheduled commands are loaded after reset time sync instead of here
}

void adcs_setup()
{
    // // Get ADCS memory addresses
    // uint32_t adcs_ptr = get_adcs_pointer();
    // output_low(MUX_SEL_MSN_SHARED_FM);
    // flash_transfer_data_to_ram(
    //     MISSION_FM,
    //     adcs_ptr,
    //     obc_flags.adcs_addr,
    //     sizeof(obc_flags.adcs_addr));
    // output_high(MUX_SEL_MSN_SHARED_FM);
    adcs_mode = obc_flags.adcs_initial_value;
}

void memory_setup()
{
    // Read boot flags from flash memory
    flash_transfer_data_to_ram(
        &spi_port_COM_FM,
        BOOT_FLAGS_ADDRESS,
        (uint8_t*)&boot_flags,
        sizeof(boot_flags));

    if (boot_flags.deployment_flag == 0xFF) { // The memory has no data
        fprintf(PC, "Initializing flash memory...\r\n");
        initialize_memory();
    } else {
        fprintf(PC, "Retrieving addresses from flash memory...\r\n");
        retrieve_memory();
    }

    if (boot_flags.deployment_flag < 5) { // Less than 5 deployment tries
        initial_deployment_setup();
    } else {
        nominal_setup();
    }
    adcs_setup();
}

#endif // !MEMORY_SETUP_H
