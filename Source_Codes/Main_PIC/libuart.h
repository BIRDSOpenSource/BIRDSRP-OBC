#ifndef LIBUART_H
#define LIBUART_H

#include "definitions.h"

typedef struct message_config {
    char identifier; // Message identifier
    char length;     // Message length
    int standard;    // Message follows standard structure (true/false)
} message_config;

message_config mconfig[] = {
    { MSG_COMM, MSG_LENGTH_COMM, MSG_CHECKSUM_COMM },
    { MSG_PCIB, MSG_LENGTH_PCIB, MSG_CHECKSUM_PCIB },
    { MSG_MCPIC, MSG_LENGTH_MCPIC, MSG_CHECKSUM_MCPIC },
    { MSG_RST, MSG_LENGTH_RST, MSG_CHECKSUM_RST },
    { MSG_FAB, MSG_LENGTH_FAB, MSG_CHECKSUM_FAB },
    { MSG_ADCS, MSG_LENGTH_ADCS, MSG_CHECKSUM_ADCS },
    { MSG_TMCR1, MSG_LENGTH_TMCR1, MSG_CHECKSUM_TMCR1 },
    { MSG_TMCR2, MSG_LENGTH_TMCR2, MSG_CHECKSUM_TMCR2 }
};

message_config* message_COMM = &mconfig[0];
message_config* message_PCIB = &mconfig[1];
message_config* message_MCPIC = &mconfig[2];
message_config* message_RST = &mconfig[3];
message_config* message_FAB = &mconfig[4];
message_config* message_ADCS = &mconfig[5];
message_config* message_TMCR1 = &mconfig[6];
message_config* message_TMCR2 = &mconfig[7];

typedef uint8_t (*bytes_available_fn)();
typedef uint8_t (*get_char_fn)();
typedef void (*put_char_fn)(uint8_t data);

typedef struct uart_fn {
    bytes_available_fn* bytes_available;
    get_char_fn* get_char;
    put_char_fn* put_char;
} uart_fn;

#define uart_declare_fn(uart_stream)                   \
    inline uint8_t bytes_available_##uart_stream()     \
    {                                                  \
        return kbhit(uart_stream);                     \
    }                                                  \
    inline uint8_t get_character_##uart_stream()       \
    {                                                  \
        return fgetc(uart_stream);                     \
    }                                                  \
    inline void put_character_##uart_stream(uint8_t c) \
    {                                                  \
        fputc(c, uart_stream);                         \
    }                                                  \
    uart_fn uart_port_##uart_stream = { bytes_available_##uart_stream, &get_character_##uart_stream, &put_character_##uart_stream }

#define uart_use(uart_stream) \
    uart uart_##uart_stream;  \
    char buffer_##uart_stream[MSG_LENGTH_##uart_stream]

#define uart_init(uart_stream)                          \
    uart_##uart_stream.data = &buffer_##uart_stream;    \
    uart_##uart_stream.size = MSG_LENGTH_##uart_stream; \
    uart_reset(&uart_##uart_stream)
#define uart_update(uart_stream, message_config) \
    uart_process(&uart_##uart_stream, message_config, fgetc(uart_stream));
#define uart_update_all(uart_stream) \
    uart_process_all(&uart_##uart_stream, fgetc(uart_stream));

#define uart_ready(uart_stream) uart_##uart_stream.state == ready
#define uart_message(uart_stream) uart_##uart_stream.data
#define uart_clean(uart_stream) uart_reset(&uart_##uart_stream)

enum state_en {
    waiting,
    receiving,
    ready
};

typedef struct uart {
    char* data;
    int size;
    char* position;
    char* end;
    state_en state;
    int standard;
} uart;

inline void uart_reset(uart* buffer)
{
    buffer->position = buffer->data;
    buffer->end = buffer->data;
    buffer->state = waiting;
}

inline void check_string(uart* buffer)
{
    unsigned int checksum = 0;
    if (!buffer->standard) {
        buffer->state = ready;
    } else {
        for (unsigned int* ptr = (buffer->data + 1); ptr < (buffer->end - 1); ptr++) {
            checksum ^= *ptr;
        }
        if ((*(buffer->position - 1) == checksum) && (*buffer->position == buffer->data[0] + 1)) {
            buffer->state = ready;
        } else {
            buffer->state = waiting;
        }
    }
    buffer->position = buffer->data;
    buffer->end = buffer->data;
}

inline void uart_process_all(uart* buffer, char rx)
{
    switch (buffer->state) {
    case ready:
    case waiting:
        for (int i = 0; i < sizeof(mconfig) / sizeof(message_config); i++) {
            if (mconfig[i].identifier == rx) { // Found a message unique identifier
                buffer->end += (mconfig[i].length - 1);
                buffer->standard = mconfig[i].standard;
                *(buffer->position) = rx;
                buffer->position++;
                buffer->state = receiving;
                break;
            }
        }
        break;
    case receiving:
        *(buffer->position) = rx;
        if (buffer->position >= buffer->end) {
            check_string(buffer);
            break;
        }
        buffer->position++;
        break;
    default:
        break;
    }
    if (buffer->position >= buffer->data + buffer->size) {
        uart_reset(buffer);
    }
}

inline void uart_process(uart* buffer, message_config* cfg, char rx)
{
    switch (buffer->state) {
    case ready:
    case waiting:
        if (cfg->identifier == rx) { // Found a message unique identifier
            buffer->end += (cfg->length - 1);
            buffer->standard = cfg->standard;
            *(buffer->position) = rx;
            buffer->position++;
            buffer->state = receiving;
            break;
        }
        break;
    case receiving:
        *(buffer->position) = rx;
        if (buffer->position >= buffer->end) {
            check_string(buffer);
            break;
        }
        buffer->position++;
        break;
    default:
        break;
    }
    if (buffer->position >= buffer->data + buffer->size) {
        uart_reset(buffer);
    }
}

#endif // !LIBUART_H
