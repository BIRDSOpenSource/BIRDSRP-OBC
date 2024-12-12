#ifndef LIBUART_FN_H
#define LIBUART_FN_H

#include "definitions.h"

void uart_print_pc_hex(uint8_t* data, int size)
{
    for (int i = 0; i < size - 1; i++) {
        fprintf(PC, "%02X ", data[i]);
    }
    fprintf(PC, "%02X", data[size - 1]);
}

void uart_print_pc_hex_short(uint8_t* data, int size)
{
    for (int i = 0; i < size; i++) {
        fprintf(PC, "%02X", data[i]);
    }
}

void uart_download_packet(uart_fn* port, uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    uint8_t n = 0;
    for (uint32_t i = 0; i < timeout; i++) {
        if (port->bytes_available()) {
            buffer[n++] = port->get_char();
            if (n >= size)
                break;
        }
    }
}

// Send packet macro; buffer is of type uint8_t*;
void uart_send_packet(uart_fn* port, uint8_t* buffer, uint32_t size)
{
    for (uint8_t* i = buffer; i < size + buffer; i++) {
        port->put_char(*i);
    }
}

void uart_send_packet_repeat(uart_fn* port, uint8_t* buffer, uint32_t size, uint8_t repetitions, uint32_t delay)
{
    for(uint8_t i=0; i<repetitions; i++){
        uart_send_packet(port, buffer, size);
        delay_ms(delay);
    }
}

#endif // !LIBUART_FN_H
