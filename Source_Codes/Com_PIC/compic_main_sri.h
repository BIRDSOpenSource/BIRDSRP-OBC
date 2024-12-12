/* This file contains everything related to compic functions and compic settings
   ,compic comuniction ports
*/

//_______________________________________________________________________________________________________________
//#use rs232(baud=9600, parity=N, xmit=PIN_B7, rcv=PIN_B6, bits=8, stream=PORT1)    //PC reading port
//_______________________________________________________________________________________________________________
#use rs232(uart1, baud = 115200, parity = N, bits = 8, stream = TR_CP, errors) //UART________TRX

#use rs232(baud = 4800, parity = N, xmit = PIN_B3, rcv = PIN_B4, bits = 8, stream = RP_CP, errors) //UART________RESET-PIC
#use rs232(baud = 9600, parity = N, xmit = PIN_B1, rcv = PIN_B2, bits = 8, stream = MP_CP, errors) //UART________MAIN-PIC
// #use rs232(baud = 38400, parity = N, xmit = PIN_A2, bits = 8, stream = CBAND, errors)           //UART________CBAND
//_______________________________________________________________________________________________________________
#use spi(MASTER, CLK = PIN_D4, DI = PIN_D6, DO = PIN_D7, BAUD = 400000, BITS = 8, STREAM = CFM, MODE = 0) //FLASH_M_______COM
#use spi(MASTER, CLK = PIN_C3, DI = PIN_C4, DO = PIN_C5, BAUD = 400000, BITS = 8, STREAM = SFM, MODE = 0) //FLASH_M_______SHARED
//_______________________________________________________________________________________________________________

unsigned int16 rssi_value;
unsigned int32 data_size;

#include <compic_fun.c> //_1_2_
#include <com_flash.c>  //_1_2_
#include <packet_codesri.c>
#include <cwsri.c> //_1
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
