#include "device.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "definitions.h"
#include "libuart.h"
#include "libuart_fn.h"

#include "rtcperipheral.h"

// Create buffers (real and pseudo UARTs)
uart_use(PC);
uart_use(COMM);
uart_use(FAB);
uart_use(RST);
uart_use(ADCS);
uart_use(MCPIC);
uart_use(PCIB);
uart_use(TMCR1);
uart_use(TMCR2);

// Create supporting functions (real UARTs only)
uart_declare_fn(PC);
uart_declare_fn(COMM);
uart_declare_fn(FAB);
uart_declare_fn(RST);
uart_declare_fn(MSN);

#include "memory_setup.h"
#include "flash_memory.h"
#include "flash_memory_fn.h"
#include "crc16.h"
#include "scheduler.h"
#include "log_control.h"
#include "boot_command.h"
#include "xmodem.h"
#include "interpreter.h"

#INT_RDA // MSN
void interrupt_rda()
{
    switch (mux_cpld_position) {
    case mux_adcs:
        uart_update(ADCS, message_ADCS); // Receive data
        break;
    case mux_mcpic:
        uart_update(MCPIC, message_MCPIC); // Receive data
        break;
    case mux_pcib:
        uart_update(PCIB, message_PCIB); // Receive data
        break;
    case mux_tmcr1:
        uart_update(TMCR1, message_TMCR1); // Receive data
        break;
    case mux_tmcr2:
        uart_update(TMCR2, message_TMCR2); // Receive data
        break;
    default:
        uart_update(ADCS, message_ADCS); // Receive data
        break;
    }
}

#INT_RDA2 HIGH // COM
void interrupt_rda2()
{
    uart_update(COMM, message_COMM); // Receive data
    if (uart_ready(COMM)) {          // If ready, execute imediately
        set_timer0(1);
    }
}

#INT_RDA3 // FAB
void interrupt_rda3()
{
    uart_update(FAB, message_FAB); // Receive data
}

#INT_RDA4 // RST
void interrupt_rda4()
{
    uart_update(RST, message_RST); // Receive data
}

#INT_TIMER0
void interrupt_timer0()
{
    clock_update = 1;
}

void schedule_new_commands()
{
    static uint8_t buffer[MAX_LENGTH];
    if (uart_ready(COMM)) {
        memcpy(buffer, uart_message(COMM), MSG_LENGTH_COMM);
        command_execute(buffer, MSG_COMM, 1);
        uart_clean(COMM);
    }
    if (uart_ready(MCPIC)) {
        memcpy(buffer, uart_message(MCPIC), MSG_LENGTH_MCPIC);
        command_execute(buffer, MSG_MCPIC, 1);
        uart_clean(MCPIC);
    }
    if (uart_ready(ADCS)) {
        memcpy(buffer, uart_message(ADCS), MSG_LENGTH_ADCS);
        command_execute(buffer, MSG_ADCS, 1);
        uart_clean(ADCS);
    }
    if (uart_ready(PCIB)) {
        memcpy(buffer, uart_message(PCIB), MSG_LENGTH_PCIB);
        command_execute(buffer, MSG_PCIB, 1);
        uart_clean(PCIB);
    }
    if (uart_ready(TMCR1)) {
        memcpy(buffer, uart_message(TMCR1), MSG_LENGTH_TMCR1);
        command_execute(buffer, MSG_TMCR1, 1);
        uart_clean(TMCR1);
    }
    if (uart_ready(TMCR2)) {
        memcpy(buffer, uart_message(TMCR2), MSG_LENGTH_TMCR2);
        command_execute(buffer, MSG_TMCR2, 1);
        uart_clean(TMCR2);
    }
    if (uart_ready(FAB)) {
        memcpy(buffer, uart_message(FAB), MSG_LENGTH_FAB);
        command_execute(buffer, MSG_FAB, 1);
        uart_clean(FAB);
    }
    if (uart_ready(RST)) {
        memcpy(buffer, uart_message(RST), MSG_LENGTH_RST);
        command_execute(buffer, MSG_RST, 1);
        uart_clean(RST);
    }
    if (uart_ready(PC)) {
        memcpy(buffer, uart_message(PC), MSG_LENGTH_PC);
        command_execute(buffer, MSG_WILDCARD, 1);
        uart_clean(PC);
    }
}

void periodic_requests_commands()
{
    const uint8_t period = 120;
    periodic_command_clear_rx_flag(period, 0);                 // Reset
    periodic_command(period, 0, verbose, { MSG_COMM, 0x90 });  // Reset
    periodic_command(period, 3, verbose, { MSG_COMM, 0x90 });  // Reset
    periodic_command(period, 6, verbose, { MSG_COMM, 0x90 });  // Reset
    periodic_command(period, 9, verbose, { MSG_COMM, 0x90 });  // Reset
    periodic_command(period, 12, verbose, { MSG_COMM, 0x90 }); // Reset

    periodic_command_clear_rx_flag(period, 15);                // EPS
    periodic_command(period, 15, verbose, { MSG_COMM, 0x91 }); // EPS
    periodic_command(period, 18, verbose, { MSG_COMM, 0x91 }); // EPS
    periodic_command(period, 21, verbose, { MSG_COMM, 0x91 }); // EPS
    periodic_command(period, 24, verbose, { MSG_COMM, 0x91 }); // EPS
    periodic_command(period, 27, verbose, { MSG_COMM, 0x91 }); // EPS

    periodic_command_clear_rx_flag(period, 30);                // MSN
    periodic_command(period, 30, verbose, { MSG_COMM, 0x92 }); // MSN
    periodic_command(period, 33, verbose, { MSG_COMM, 0x92 }); // MSN
    periodic_command(period, 36, verbose, { MSG_COMM, 0x92 }); // MSN
    periodic_command(period, 39, verbose, { MSG_COMM, 0x92 }); // MSN
    periodic_command(period, 42, verbose, { MSG_COMM, 0x92 }); // MSN

    periodic_command_clear_rx_flag(period, 45);                // ADCS
    periodic_command(period, 45, verbose, { MSG_COMM, 0x93 }); // ADCS
    periodic_command(period, 48, verbose, { MSG_COMM, 0x93 }); // ADCS
    periodic_command(period, 51, verbose, { MSG_COMM, 0x93 }); // ADCS
    periodic_command(period, 54, verbose, { MSG_COMM, 0x93 }); // ADCS
    periodic_command(period, 57, verbose, { MSG_COMM, 0x93 }); // ADCS

    periodic_command_clear_rx_flag(period, 60);                // RELAY
    periodic_command(period, 60, verbose, { MSG_COMM, 0x94 }); // RELAY
    periodic_command(period, 63, verbose, { MSG_COMM, 0x94 }); // RELAY
    periodic_command(period, 66, verbose, { MSG_COMM, 0x94 }); // RELAY
    periodic_command(period, 69, verbose, { MSG_COMM, 0x94 }); // RELAY
    periodic_command(period, 72, verbose, { MSG_COMM, 0x94 }); // RELAY

    periodic_command_clear_rx_flag(period, 75);                // TMCR1
    periodic_command(period, 75, verbose, { MSG_COMM, 0x95 }); // TMCR1
    periodic_command(period, 78, verbose, { MSG_COMM, 0x95 }); // TMCR1
    periodic_command(period, 81, verbose, { MSG_COMM, 0x95 }); // TMCR1
    periodic_command(period, 84, verbose, { MSG_COMM, 0x95 }); // TMCR1
    periodic_command(period, 87, verbose, { MSG_COMM, 0x95 }); // TMCR1

    periodic_command_clear_rx_flag(period, 90);                 // TMCR2
    periodic_command(period, 90, verbose, { MSG_COMM, 0x96 });  // TMCR2
    periodic_command(period, 93, verbose, { MSG_COMM, 0x96 });  // TMCR2
    periodic_command(period, 96, verbose, { MSG_COMM, 0x96 });  // TMCR2
    periodic_command(period, 99, verbose, { MSG_COMM, 0x96 });  // TMCR2
    periodic_command(period, 102, verbose, { MSG_COMM, 0x96 }); // TMCR2
    periodic_command_clear_rx_flag(period, 105);                // TMCR2

    periodic_command(period, 105, true, { MSG_COMM, 0xF8 }); // Telemetry generation

    // Automatic SEL mission:
    const time_t sel_period = 24 * 60 * 60;                                               // SEL period is 24h
    const time_t sel_delta_zes = sel_period - 3 * 60 * 60;                                // Start 3h before midnight
    const time_t sel_delta_ref = sel_delta_zes + 6 * 60;                                  // Start 6m after SEL ZES
    periodic_command(sel_period, sel_delta_zes, 1, { MSG_COMM, 0x1A, 0x15, 0xD0, 0x07 }); // SEL mission ZES
    periodic_command(sel_period, sel_delta_ref, 1, { MSG_COMM, 0x1B, 0x0B, 0xD0, 0x07 }); // SEL mission REF
    const time_t sel_flag_period = 15 * 60;                                               // SEL period for checking flags is 15 minutes
    const time_t sel_flag_delta = sel_flag_period - 5 * 60;                               // SEL delta for checking flags: 5m earlier to avoid conflict with 24h operation
    periodic_command(sel_flag_period, sel_flag_delta, 1, { MSG_COMM, 0x31 });             // SEL mission for checking flags
}

void periodic_tasks()
{
    if (clock_update) {
        disable_interrupts(GLOBAL);
        clock_update = 0;
        current_time = time(0);
        // ctime(&current_time, &current_time_str);
        schedule_new_commands();
        if (current_time != previous_time) {
            scheduled_command_check();
            periodic_requests_commands();
        }
        struct_tm* local_time = localtime(&current_time);
        if (local_time->tm_hour == 23 && local_time->tm_min == 59 && local_time->tm_sec == 59) {
            execute(1, { MSG_COMM, 0x55 }); // Save state
            fprintf(PC, "No 24h reset happened, doing soft reset instead.\r\n");
            reset_cpu();
        }
        previous_time = current_time;
        // clean HW uart:
        uart_clean(PC);
        uart_clean(COMM);
        uart_clean(FAB);
        uart_clean(RST);
        uart_clean(ADCS);
        uart_clean(MCPIC);
        uart_clean(PCIB);
        uart_clean(TMCR1);
        uart_clean(TMCR2);
        enable_interrupts(GLOBAL);
    }
}

int main(int argc, char** argv)
{
    // Ensure OCPs are off as soon as possible
    output_bit(OCP_EN_ADCS, false);  // ADCS
    output_bit(OCP_EN_MCP, false);   // Mission Control PIC
    output_bit(OCP_EN_RELAY, false); // Relay PIC

    // UART configuration
    uart_init(PC);
    uart_init(COMM);
    uart_init(FAB);
    uart_init(RST);
    uart_init(ADCS);
    uart_init(MCPIC);
    uart_init(PCIB);
    uart_init(TMCR1);
    uart_init(TMCR2);

    fprintf(PC, "\r\n  _    ___  ___  ___   _    ___  __     ___       _       _ _ _ _       \r\n");
    fprintf(PC, " | |  | __|| _ || _ \\ / \\  | _ \\|  \\   / __| __ _| |_ ___| | (_) |_ ___ \r\n");
    fprintf(PC, " | |_ | _| ||_|||___// _ \\ |   /||  |  \\__ \\/ _` |  _/ -_) | | |  _/ -_)\r\n");
    fprintf(PC, " |___||___||___||_| /_/ \\_\\|_\\_\\|__/   |___/\\__,_|\\__\\___|_|_|_|\\__\\___|\r\n\r\n");

    fprintf(PC, "Compiled on: "__DATE__
                " "__TIME__
                "\r\n\r\n");

    // Satellite time initialization
    current_time = time(0);
    // In case RTC registers have invalid data, load the initial time
    if (current_time < T0 || current_time > Tn) {
        fprintf(PC, "> Invalid clock, recovering...\r\n");
        current_time = T0;
        SetTimeSec(T0);
    }
    previous_time = current_time;
    // ctime(&current_time, &current_time_str);

    // > RTC & Timers
    setup_rtc(RTC_ENABLE | RTC_CLOCK_SOSC, 0x00); // enables internal RTC
    setup_timer_0(T0_INTERNAL | T0_DIV_32);

    // Telemetry initialization
    initialize_telemetry();

    // 2-way multiplexer of flash memories
    output_low(MUX_SEL_COM_SHARED_FM);  // COM SHARED FM to OBC side
    output_high(MUX_SEL_MSN_SHARED_FM); // Mission SHARED FM

    // 8-way multiplexer of CPLD
    mux_sel(mux_mcpic);

    // Memory
    memory_setup();
    print_flags();

    // Raf_DIO Initial settings
    output_low(DIO_BURNER_ANTENNA); // DIO (Burner Circuit UHF Antenna)
    output_low(DIO_BURNER_SAP);     // DIO (Burner Circuit SAP Deployment)
    output_low(DIO_BURNER_SMA);     // DIO (Burner Circuit SMA Heater)

    // Interrputions
    enable_interrupts(INT_TIMER0);
    enable_interrupts(GLOBAL);
    enable_interrupts(INT_RDA);  // MSN
    enable_interrupts(INT_RDA2); // COMM
    enable_interrupts(INT_RDA3); // FAB
    enable_interrupts(INT_RDA4); // RST

    // OCPs
    output_bit(OCP_EN_ADCS, obc_flags.adcs_on_off);   // ADCS
    output_bit(OCP_EN_MCP, obc_flags.MCP_on_off);     // Mission Control PIC
    output_bit(OCP_EN_RELAY, obc_flags.relay_on_off); // Relay PIC

    execute(1, { MSG_PC, 0xFF }); // Reset log

    // > Main Loop
    while (TRUE) {
        // Periodic tasks:
        periodic_tasks();
        // UART tasks:
        if (kbhit(PC)) {
            uart_update(PC, message_COMM); // Receive the same messages as COMM
        }
    }
    return 0;
}
