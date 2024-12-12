#ifndef DEVICE_H
#define DEVICE_H

// For the real case ----------------------------------------------------------------------
#include <18F67J94.h>

#FUSES NOWDT NOBROWNOUT SOSC_DIG
#use delay(crystal = 16Mhz, clock = 16Mhz) // For crystal

#device HIGH_INTS = TRUE

#pin_select TX1 = PIN_G2
#pin_select RX1 = PIN_G3
#use rs232(baud = 115200, parity = N, UART1, bits = 8, stream = MSN, ERRORS) // UART to Mission

#pin_select TX2 = PIN_D3
#pin_select RX2 = PIN_D2
#use rs232(baud = 9600, parity = N, UART2, bits = 8, stream = COMM, ERRORS) // COM PIC

#pin_select TX3 = PIN_E5
#pin_select RX3 = PIN_E4
#use rs232(baud = 9600, parity = N, UART3, bits = 8, stream = FAB, ERRORS) // FAB PIC is EPS1 PIC

#pin_select TX4 = PIN_E3
#pin_select RX4 = PIN_F2
#use rs232(baud = 19200, parity = N, UART4, bits = 8, stream = RST, ERRORS) // RESET PIC

#use rs232(baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8, stream = PC, ERRORS)     // DEBUG
#use rs232(baud = 9600, parity = N, xmit = PIN_G0, rcv = PIN_G1, bits = 8, stream = MCP_SW, ERRORS) // MAIN to Mission Control PIC

#use spi(MASTER, CLK = PIN_E1, DI = PIN_E0, DO = PIN_E6, BAUD = 1000000, BITS = 8, STREAM = MAIN_FM, MODE = 0)    // MAIN flash memory port
#use spi(MASTER, CLK = PIN_B2, DI = PIN_B5, DO = PIN_B4, BAUD = 1000000, BITS = 8, STREAM = COM_FM, MODE = 0)     // COM flash memory port
#use spi(MASTER, CLK = PIN_A3, DI = PIN_A0, DO = PIN_A1, BAUD = 1000000, BITS = 8, STREAM = MISSION_FM, MODE = 0) // MISSION flash memory port

#define SEL_ZES MSN
#define TMCR1 MSN
#define ADCS MSN
#define TMCR2 MSN
#define OPERA MSN
#define MCPIC MSN
#define PCIB MSN
#define SEL_REF MSN

#define MUX_SEL_COM_SHARED_FM PIN_C5 // Low = Main PIC; High = COM PIC
#define MUX_SEL_MSN_SHARED_FM PIN_A5 // Low = Main PIC; High = Mission
#define OCP_EN_ADCS PIN_F7           // ADCS board enable
#define OCP_EN_MCP PIN_D6            // DIO (MainPIC_to_MCPIC) changed to initialy being high.
#define OCP_EN_RELAY PIN_G0          // DIO (On_off_MainPIC_to_Relay)

#define DIO_BURNER_ANTENNA PIN_G4 // DIO (Burner Circuit UHF Antenna)
#define DIO_BURNER_SAP PIN_C2     // DIO (Burner Circuit SAP Deployment)
#define DIO_BURNER_SMA PIN_F5     // DIO (Burner Circuit SMA Heater)

#define MUX_CPLD_SEL_0 PIN_D7
#define MUX_CPLD_SEL_1 PIN_D5
#define MUX_CPLD_SEL_2 PIN_F6

#define OBC_KILL_DIO PIN_A4

#use timer(timer = 1, tick = 1ms, bits = 16, noisr)

#endif // !DEVICE_H
