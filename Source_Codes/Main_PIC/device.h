#ifndef DEVICE_H
#define DEVICE_H

// For the real case ----------------------------------------------------------------------
#include <18F67J94.h>

#FUSES NOWDT NOBROWNOUT SOSC_DIG
#use delay(crystal = 16Mhz, clock = 16Mhz) // For crystal

#device HIGH_INTS = TRUE

#pin_select TX1 = PIN_D6
#pin_select RX1 = PIN_D7
#use rs232(baud = 115200, parity = N, UART1, bits = 8, stream = PCIB, ERRORS) // PCIB

#pin_select TX2 = PIN_D3
#pin_select RX2 = PIN_D2
#use rs232(baud = 9600, parity = N, UART2, bits = 8, stream = COMM, ERRORS) // COM PIC

#pin_select TX3 = PIN_E5
#pin_select RX3 = PIN_E4
#use rs232(baud = 9600, parity = N, UART3, bits = 8, stream = FAB, ERRORS) // FAB PIC

#pin_select TX4 = PIN_E3
#pin_select RX4 = PIN_F2
#use rs232(baud = 19200, parity = N, UART4, bits = 8, stream = RST, ERRORS) // RESET PIC

#use rs232(baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8, stream = PC, ERRORS)            // DEBUG
#use rs232(baud = 9600, parity = N, xmit = PIN_D5, rcv = PIN_F6, bits = 8, stream = ADCS, ERRORS)          // ADCS

#use spi(MASTER, CLK = PIN_E1, DI = PIN_E0, DO = PIN_E6, BAUD = 1000000, BITS = 8, STREAM = MAIN_FM, MODE = 0)    // MAIN flash memory port
#use spi(MASTER, CLK = PIN_B2, DI = PIN_B5, DO = PIN_B4, BAUD = 1000000, BITS = 8, STREAM = COM_FM, MODE = 0)     // COM flash memory port
#use spi(MASTER, CLK = PIN_A3, DI = PIN_A0, DO = PIN_A1, BAUD = 1000000, BITS = 8, STREAM = MISSION_FM, MODE = 0) // MISSION flash memory port

#define MUX_SEL_COM_SHARED_FM PIN_C5 // Low = Main PIC; High = COM PIC
#define MUX_SEL_MSN_SHARED_FM PIN_A5 // Low = Main PIC; High = Mission
#define OCP_EN_ADCS PIN_G4      // ADCS board enable
#define OCP_EN_PCIB PIN_F5      // PCIB enable

#endif // !DEVICE_H
