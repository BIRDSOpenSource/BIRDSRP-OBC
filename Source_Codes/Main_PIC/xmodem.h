#ifndef XMODEM_H
#define XMODEM_H

// Based on:
// https://web.mit.edu/6.115/www/amulet/xmodem.htm
// https://pythonhosted.org/xmodem/xmodem.html#data-flow-example-including-error-recovery

#include <definitions.h>
#include <flash_memory.h>
#include <libuart_fn.h>

#define XMODEM_SOH 0x01                   // Start of Header
#define XMODEM_EOT 0x04                   // End of Transmission
#define XMODEM_ACK 0x06                   // Acknowledge
#define XMODEM_NAK 0x15                   // Not Acknowledge
#define XMODEM_DLENGTH 128                // Data length
#define XMODEM_PLENGTH XMODEM_DLENGTH + 4 // Packet length

typedef struct xmodem_packet {
    uint8_t header;
    uint8_t packet_no;
    uint8_t packet_no_;
    uint8_t packet_data[XMODEM_DLENGTH];
    uint8_t checksum;
} xmodem_packet;

uint8_t xmodem_calc_chksum(uint8_t* ptr, uint8_t count)
{
    uint8_t result = 0;
    for (uint8_t* i = ptr; i < ptr + count; i++) {
        result += *i;
    }
    return result;
}

void xmodem_create_packet(spi_fn* spi_port, uint32_t fm_address, uint32_t packet_no, uint8_t* packet_data)
{
    xmodem_packet* packet = (xmodem_packet*)packet_data;
    packet->header = XMODEM_SOH;
    packet->packet_no = packet_no & 0xFF;
    packet->packet_no_ = 0xFF - packet->packet_no;
    flash_transfer_data_to_ram(spi_port, fm_address, packet->packet_data, XMODEM_DLENGTH);
    packet->checksum = xmodem_calc_chksum(packet->packet_data, XMODEM_DLENGTH);
}

int8_t xmodem_send(uart_fn* uart_stream, spi_fn* spi_port, uint32_t fm_address, uint16_t n_packets)
{
    const uint32_t timeout = 5000000;
    uint16_t current_packet = 1;
    uint8_t packet[XMODEM_PLENGTH];
    for (uint32_t t = 0; t < timeout; t++) {
        if (uart_stream->bytes_available()) {
            uint8_t command = uart_stream->get_char();
            t = 0;
            switch (command) {
            case XMODEM_ACK:
                current_packet++;
                fm_address += XMODEM_DLENGTH;
                if (current_packet > n_packets) {
                    uart_stream->put_char(XMODEM_EOT);
                    return 0;
                }
            case XMODEM_NAK:
                xmodem_create_packet(spi_port, fm_address, current_packet, packet);
                uart_send_packet(uart_stream, packet, XMODEM_PLENGTH);
                break;
            default:
                break;
            }
        }
    }
    return -1;
}

uint8_t xmodem_validate(uint8_t* buffer, uint16_t packet_no)
{
    xmodem_packet* packet = (xmodem_packet*)buffer;
    uint8_t correct_header = (packet->header == XMODEM_SOH);
    uint8_t correct_no = (packet->packet_no == (packet_no & 0xFF));
    uint8_t correct_no_ = (packet->packet_no_ == 0xFF - (packet_no & 0xFF));
    uint8_t correct_checksum = (packet->checksum == xmodem_calc_chksum(packet->packet_data, XMODEM_DLENGTH));
    return correct_header && correct_no && correct_no_ && correct_checksum;
}

int16_t xmodem_receive(uart_fn* uart_stream, spi_fn* spi_port, uint32_t fm_address)
{
    const uint32_t uart_timeout = 1000000;
    uint16_t max_nak = 2;
    uint16_t packet_no = 1;
    uint16_t nak_count = 0;
    uint8_t buffer[XMODEM_PLENGTH] = { 0 };
    xmodem_packet* packet = (xmodem_packet*)buffer;
    uart_stream->put_char(XMODEM_NAK); // Request 1st packet
    while (nak_count < max_nak) {
        uart_download_packet(uart_stream, buffer, XMODEM_PLENGTH, uart_timeout);
        if (buffer[0] == XMODEM_EOT) {
            uart_stream->put_char(XMODEM_ACK);
            return packet_no;
        }
        if (xmodem_validate(buffer, packet_no)) {
            // uart_send_packet(uart_stream, packet->packet_data, XMODEM_DLENGTH); // for debug
            uint32_t remaining_addressess = (MEMORY_SECTOR_SIZE - (fm_address % MEMORY_SECTOR_SIZE)) % MEMORY_SECTOR_SIZE;
            if (remaining_addressess < XMODEM_DLENGTH) {
                flash_erase(spi_port, fm_address + remaining_addressess, ERASE_SECTOR);
            }
            flash_transfer_data_from_ram(spi_port, fm_address, packet->packet_data, XMODEM_DLENGTH);
            packet_no++;
            fm_address += XMODEM_DLENGTH;
            nak_count = 0;
            max_nak = 64; // Increase max number of nak once communication is established
            uart_stream->put_char(XMODEM_ACK);
        } else {
            uart_stream->put_char(XMODEM_NAK);
            nak_count++;
        }
    }
    return -1;
}

#endif /* XMODEM_H */
