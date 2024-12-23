#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

// CRC calculation function
uint16_t mk_crc(uint8_t* data, uint8_t size)
{
    uint32_t crcreg = 0xffff;
    uint32_t calc = 0x8408;
    for (uint32_t k = 0; k < size; k++) {
        uint8_t cal_data = data[k];
        for (uint32_t i = 0; i < 8; i++) {
            uint8_t w = (crcreg ^ cal_data) & 0x0001;
            crcreg = crcreg >> 1;
            if (w == 1) {
                crcreg = crcreg ^ calc;
            }
            cal_data = cal_data >> 1;
        }
    }
    crcreg = crcreg ^ 0xffff;
    return crcreg;
}

#endif /* CRC16_H */
