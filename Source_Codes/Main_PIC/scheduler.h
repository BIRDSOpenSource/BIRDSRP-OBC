#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <string.h>
#include <definitions.h>

void command_execute(uint8_t* data, uint8_t origin, uint8_t log_enabled);

void scheduler_initialize()
{
    for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
        scheduled_commands[i].time = TIME_T_MAX;
    }
}

int scheduled_command_add(time_t time, uint8_t* command)
{
    scheduled_command cmd;
    cmd.time = time;
    memcpy(cmd.command, command, BUFF_LENGTH);

    for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
        if (scheduled_commands[i].time == TIME_T_MAX) {
            memcpy(&scheduled_commands[i], &cmd, sizeof(scheduled_command));
            return 0;
        }
    }

    return -1; // Error signaling
}

int scheduled_command_count()
{
    int count = 0;
    for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
        if (scheduled_commands[i].time != TIME_T_MAX) {
            count++;
        }
    }
    return count;
}

void scheduled_command_clear_all()
{
    for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
        scheduled_commands[i].time = TIME_T_MAX;
    }
}

void scheduled_command_clear_specified_command(uint8_t source, uint8_t command)
{
    for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
        if (scheduled_commands[i].command[0] == source && scheduled_commands[i].command[1] == command) {
            scheduled_commands[i].time = TIME_T_MAX; // Disable the command from executing again (== reschedule it at infinity).
        }
    }
}

void scheduled_command_check()
{
    // time_t current_time = time(0);
    for (uint8_t i = 0; i < SCHEDULED_COMMANDS_MAX; i++) {
        if (current_time >= scheduled_commands[i].time) {
            command_execute(scheduled_commands[i].command, MSG_COMM, 1); // scheduled commands can only be COMM commands
            scheduled_commands[i].time = TIME_T_MAX;                     // Disable the command from executing again (== reschedule it at infinity).
            i = 0; // Check the whole table again for changes
        }
    }
}

#ifndef PC_SIM
#define ZERO (0, 0)
#else
#define ZERO (0)
#endif

// Schedule a constant
#define schedule(time, ...)               \
    do {                                  \
        uint8_t cmd[] = __VA_ARGS__;      \
        scheduled_command_add(time, cmd); \
    } while ZERO

// Schedule a variable
#define vschedule(time, cmd)              \
    do {                                  \
        scheduled_command_add(time, cmd); \
    } while ZERO

// Execute a command immediately
#define execute(log, ...)                        \
    do {                                         \
        uint8_t cmd[] = __VA_ARGS__;             \
        command_execute(cmd, MSG_WILDCARD, log); \
    } while ZERO

#define periodic_command_clear_rx_flag(period, delta) \
    do {                                              \
        if ((current_time % period == delta)) {       \
            response_rx = 0;                          \
        }                                             \
    } while ZERO

#define periodic_command(period, delta, log, ...) \
    do {                                          \
        if ((current_time % period == delta)) {   \
            if (!response_rx)                     \
                execute(log, __VA_ARGS__);        \
        }                                         \
    } while ZERO

#endif /* SCHEDULER_H */
