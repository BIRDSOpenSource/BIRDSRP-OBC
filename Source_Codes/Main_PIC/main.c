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
uart_use(PC);
uart_use(PCIB);
uart_use(COMM);
uart_use(ADCS);
uart_use(FAB);
uart_use(RST);

#include "memory_setup.h"
#include "flash_memory.h"
#include "flash_memory_fn.h"
#include "crc16.h"
#include "scheduler.h"
#include "log_control.h"
#include "boot_command.h"
#include "xmodem.h"
#include "interpreter.h"

#INT_RDA // PCIB
void interrupt_rda()
{
    uart_update(PCIB, message_PCIB); // Receive data
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
    if (uart_ready(PCIB)) {
        memcpy(buffer, uart_message(PCIB), MSG_LENGTH_PCIB);
        command_execute(buffer, MSG_PCIB, 1);
        uart_clean(PCIB);
    }
    if (uart_ready(ADCS)) {
        memcpy(buffer, uart_message(ADCS), MSG_LENGTH_ADCS);
        command_execute(buffer, MSG_ADCS, 1);
        uart_clean(ADCS);
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
    const uint8_t period = 100;
    periodic_command_clear_rx_flag(period, 0);           // Reset
    periodic_command(period, 0, 0, { MSG_COMM, 0x90 });  // Reset
    periodic_command(period, 4, 0, { MSG_COMM, 0x90 });  // Reset
    periodic_command(period, 8, 0, { MSG_COMM, 0x90 });  // Reset
    periodic_command(period, 12, 0, { MSG_COMM, 0x90 }); // Reset
    periodic_command(period, 16, 0, { MSG_COMM, 0x90 }); // Reset
    periodic_command_clear_rx_flag(period, 20);          // EPS
    periodic_command(period, 20, 0, { MSG_COMM, 0x91 }); // EPS
    periodic_command(period, 24, 0, { MSG_COMM, 0x91 }); // EPS
    periodic_command(period, 28, 0, { MSG_COMM, 0x91 }); // EPS
    periodic_command(period, 32, 0, { MSG_COMM, 0x91 }); // EPS
    periodic_command(period, 36, 0, { MSG_COMM, 0x91 }); // EPS
    periodic_command_clear_rx_flag(period, 40);          // PCIB
    periodic_command(period, 40, 0, { MSG_COMM, 0x92 }); // PCIB
    periodic_command(period, 44, 0, { MSG_COMM, 0x92 }); // PCIB
    periodic_command(period, 48, 0, { MSG_COMM, 0x92 }); // PCIB
    periodic_command(period, 52, 0, { MSG_COMM, 0x92 }); // PCIB
    periodic_command(period, 54, 0, { MSG_COMM, 0x92 }); // PCIB
    periodic_command_clear_rx_flag(period, 60);          // ADCS
    periodic_command(period, 60, 0, { MSG_COMM, 0x93 }); // ADCS
    periodic_command(period, 64, 0, { MSG_COMM, 0x93 }); // ADCS
    periodic_command(period, 68, 0, { MSG_COMM, 0x93 }); // ADCS
    periodic_command(period, 72, 0, { MSG_COMM, 0x93 }); // ADCS
    periodic_command(period, 76, 0, { MSG_COMM, 0x93 }); // ADCS
    periodic_command_clear_rx_flag(period, 80);          // ADCS
    periodic_command(period, 80, 1, { MSG_COMM, 0xF8 }); // Telemetry generation
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
        uart_clean(PCIB);
        uart_clean(COMM);
        uart_clean(FAB);
        uart_clean(RST);
        enable_interrupts(GLOBAL);
    }
}
int main(int argc, char** argv)
{
    // UART configuration
    uart_init(PC);
    uart_init(COMM);
    uart_init(RST);
    uart_init(FAB);
    uart_init(PCIB);
    uart_init(ADCS);

    fprintf(pc, "\r\n");
    fprintf(pc, "____   ________________________________________________________   _________\r\n");
    fprintf(pc, "\\   \\ /   /\\_   _____/\\______   \\__    ___/\\_   _____/\\_   ___ \\ /   _____/\r\n");
    fprintf(pc, " \\   y   /  |    __)_  |       _/ |    |    |    __)_ /    \\  \\/ \\_____  \\ \r\n");
    fprintf(pc, "  \\     /   |        \\ |    |   \\ |    |    |        \\\\     \\____/        \\\r\n");
    fprintf(pc, "   \\___/   /_______  / |____|_  / |____|   /_______  / \\______  /_______  /\r\n");
    fprintf(pc, "                   \\/         \\/                   \\/         \\/        \\/ \r\n\r\n");
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

    // Memory
    output_low(MUX_SEL_COM_SHARED_FM);  // COM SHARED FM to OBC side
    output_high(MUX_SEL_MSN_SHARED_FM); // Mission SHARED FM
    memory_setup();
    print_flags();

    // Interrputions
    enable_interrupts(INT_TIMER0);
    enable_interrupts(GLOBAL);
    enable_interrupts(INT_RDA);  // PCIB
    enable_interrupts(INT_RDA2); // COMM
    enable_interrupts(INT_RDA3); // FAB
    enable_interrupts(INT_RDA4); // RST

    // OCPs
    output_bit(OCP_EN_ADCS, obc_flags.adcs_on_off); // ADCS
    output_bit(OCP_EN_PCIB, obc_flags.pcib_on_off); // PCIB

    execute(1, { MSG_PC, 0xFF }); // Reset log

    // > Main Loop
    while (TRUE) {
        // Periodic tasks:
        periodic_tasks();
        // UART tasks:
        switch (uart_mux) {
        case 0:
            if (kbhit(ADCS)) {
                uart_update(ADCS, message_ADCS);
            }
            break;
        case 1:
            if (kbhit(PC)) {
                uart_update(PC, message_COMM); // Receive the same messages as COMM
            }
            break;
        default:
            break;
        }
    }
    return 0;
}
