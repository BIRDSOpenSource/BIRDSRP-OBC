#ifndef FLASH_MEMORY_H
#define FLASH_MEMORY_H

#include <definitions.h>

// Driver for the MT25QL01GBBB flash memory

// A limitation of CCS compiler is that port names can not be passed as parameters to functions.
// Therefore, C macros and function pointers were used to achieve generic procedures.
// All functions start with the function_ prefix to separate from other names used in the program.

// Pin definitions should start with CS_PIN_ and end with the correspondent spi port name.
// This ensures that the define macros below will know the corresponding pins.
#define CS_PIN_MAIN_FM PIN_E2    // MAIN_FM
#define CS_PIN_COM_FM PIN_B3     // COM_FM
#define CS_PIN_MISSION_FM PIN_A2 // MISSION_FM

// These are constant values used for the flash memory operation.
#define READ_ID 0x9F
#define READ_STATUS_REG 0x05
#define READ_DATA_BYTES 0x13
#define ENABLE_WRITE 0x06
#define WRITE_PAGE 0x12
#define ERASE_SECTOR 0xDC    // 64 KB
#define ERASE_SUBSECTOR 0x5C // 32 KB
#define ERASE_PAGE 0x21      // 04 KB
#define DIE_ERASE 0xC4

// Memory size (available addresses)
#define MEMORY_N_SECTORS 2048
#define MEMORY_SIZE 128 * 1024 * 1024
#define MEMORY_SECTOR_SIZE (64 * 1024)
#define MEMORY_SUBSECTOR_SIZE (32 * 1024)
#define MEMORY_PAGE_SIZE (4 * 1024)

typedef void (*spi_send_fn)(uint8_t data);
typedef uint8_t (*spi_receive_fn)();

typedef struct spi_fn {
    spi_send_fn* spi_send;
    spi_receive_fn* spi_receive;
    uint16_t cs_pin;
} spi_fn;

#define spi_declaration(spi_port)                 \
    inline void spi_send_##spi_port(uint8_t data) \
    {                                             \
        spi_xfer(spi_port, data);                 \
    }                                             \
    inline uint8_t spi_receive_##spi_port()       \
    {                                             \
        return spi_xfer(spi_port);                \
    }                                             \
    spi_fn spi_port_##spi_port = { &spi_send_##spi_port, &spi_receive_##spi_port, CS_PIN_##spi_port }

spi_declaration(COM_FM);     // Will create a spi_port_COM_FM structure with function pointers
spi_declaration(MAIN_FM);    // Will create a spi_port_MAIN_FM structure with function pointers
spi_declaration(MISSION_FM); // Will create a spi_port_MISSION_FM structure with function pointers

// Write a byte (data) to an address (page_address) on a given flash memory (spi_port).
void flash_write(spi_fn* spi_functions, uint32_t page_address, uint8_t data)
{
    /* Byte extraction */
    uint8_t* address = (uint8_t*)&page_address;
    /* address[0] : 0x 00 00 00 _ _ */
    /* address[1] : 0x 00 00 _ _ 00 */
    /* address[2] : 0x 00 _ _ 00 00 */
    /* address[3] : 0x _ _ 00 00 00 */
    output_low(spi_functions->cs_pin);
    spi_functions->spi_send(ENABLE_WRITE);
    output_high(spi_functions->cs_pin);
    output_low(spi_functions->cs_pin);
    spi_functions->spi_send(WRITE_PAGE);
    spi_functions->spi_send(address[3]);
    spi_functions->spi_send(address[2]);
    spi_functions->spi_send(address[1]);
    spi_functions->spi_send(address[0]);
    spi_functions->spi_send(data);
    output_high(spi_functions->cs_pin);
    delay_us(20);
}

// Read a byte (data) from an address (page_address) on a given flash memory (spi_port).
uint8_t flash_read(spi_fn* spi_functions, uint32_t page_address)
{
    /* Byte extraction */
    unsigned int8* address = (uint8_t*)&page_address;
    /* address[0] : 0x 00 00 00 _ _ */
    /* address[1] : 0x 00 00 _ _ 00 */
    /* address[2] : 0x 00 _ _ 00 00 */
    /* address[3] : 0x _ _ 00 00 00 */
    output_low(spi_functions->cs_pin);
    spi_functions->spi_send(READ_DATA_BYTES);
    spi_functions->spi_send(address[3]);
    spi_functions->spi_send(address[2]);
    spi_functions->spi_send(address[1]);
    spi_functions->spi_send(address[0]);
    uint8_t data = spi_functions->spi_receive();
    output_high(spi_functions->cs_pin);
    return data;
}

// Erase a sector (64KB), subsector (32KB) or page (4KB) starting in a given address (page address).
// Any address within the subsector is valid for entry.
// erase_command can be ERASE_SECTOR, ERASE_SUBSECTOR or ERASE_PAGE
void flash_erase(spi_fn* spi_functions, uint32_t page_address, uint8_t erase_command)
{
    /* Byte extraction */
    unsigned int8* address = (unsigned int8*)&page_address;
    /* address[0] : 0x 00 00 00 _ _ */
    /* address[1] : 0x 00 00 _ _ 00 */
    /* address[2] : 0x 00 _ _ 00 00 */
    /* address[3] : 0x _ _ 00 00 00 */
    output_low(spi_functions->cs_pin);
    spi_functions->spi_send(ENABLE_WRITE);
    output_high(spi_functions->cs_pin);
    output_low(spi_functions->cs_pin);
    delay_us(2);
    spi_functions->spi_send(erase_command);
    spi_functions->spi_send(address[3]);
    spi_functions->spi_send(address[2]);
    spi_functions->spi_send(address[1]);
    spi_functions->spi_send(address[0]);
    delay_us(2);
    output_high(spi_functions->cs_pin);
    delay_ms(750);
}

#endif // !FLASH_MEMORY_H
