/* This file contains everything related to compic functions and compic settings
   ,compic comuniction ports
*/

//_______________________________________________________________________________________________________________
//#use rs232(baud=9600, parity=N, xmit=PIN_B7, rcv=PIN_B6, bits=8, stream=PORT1)    //PC reading port
//_______________________________________________________________________________________________________________
#pin_select TX1 = PIN_C6
#pin_select RX1 = PIN_C7
#use rs232(uart1, baud = 115200, parity = N, bits = 8, stream = TR_CP, errors) //UART________TRX

#pin_select TX2 = PIN_G1
#pin_select RX2 = PIN_G2
#use rs232(uart2, baud = 9600, parity = N, bits = 8, stream = MP_CP, errors) //UART________MAIN-PIC

#pin_select TX4 = PIN_E3
#pin_select RX4 = PIN_E4
#use rs232(uart4, baud = 19200, parity = N, bits = 8, stream = RP_CP, errors) //UART________RESET-PIC
//_______________________________________________________________________________________________________________
#use spi(MASTER, CLK = PIN_E1, DI = PIN_E6, DO = PIN_E0, BAUD = 400000, BITS = 8, STREAM = CFM, MODE = 0) //FLASH_M_______COM
#use spi(MASTER, CLK = PIN_B2, DI = PIN_B5, DO = PIN_B4, BAUD = 400000, BITS = 8, STREAM = SFM, MODE = 0) //FLASH_M_______SHARED
//_______________________________________________________________________________________________________________

unsigned int16 rssi_value;
unsigned int32 data_size;

#include <compic_fun.c> //_1_2_
#include <com_flash.c>  //_1_2_
#include <packet_codesri.c>
#include <cwsri.c> //_1
#include <compic_reset.c>
#include <cmd_fun.c>

//_________________Tranceiver Control__________________________________________
void tx_on(); //__Data Transmission Enable__
void rx_on(); //__Commamd Receiving Enable__
void cw_on(); //__CW Transmission Enable__

unsigned int16 trx_temp_read(); //__Tranceiver Temperature Read__
unsigned int16 trx_rssi_read(); //__Received Power Level__
                                //_____________________________________________________________________________

//_________________Input Command Buffer________________________________________
void rx_bfr_zero(); //Make receive buffer Zero__
                    //_____________________________________________________________________________

//_________________Flash memory functions______________________________________
void cf_write_enable();
void sf_write_enable();

void cf_sector_erase(unsigned int32 sector_address);
void sf_sector_erase(unsigned int32 sector_address);

void cf_byte_write(unsigned int32 byte_address, int8 data);
void sf_byte_write(unsigned int32 byte_address, int8 data);

int8 cf_byte_read(unsigned int32 address);
int8 sf_byte_read(unsigned int32 address);

void flash_copy(unsigned int32 start_address, unsigned int32 data_length);
//_____________________________________________________________________________
