// Flash memory commands
#define READ_DATA_BYTES 0x13 //0x03 for 3byte mode
#define ENABLE_WRITE 0x06
#define WRITE_BYTE 0x12   //0x02 for 3byte mode
#define ERASE_SECTOR 0xDC //0xD8 for 3byte mode

//______________________Write Enable Codes____________________________________________________
void cf_write_enable()
{
    LATD5 = 0;
    delay_us(50);
    spi_xfer(CFM, ENABLE_WRITE); //Send 0x06
    LATD5 = 1;
    return;
}

void sf_write_enable()
{
    LATC2 = 0;
    delay_us(50);
    spi_xfer(SFM, ENABLE_WRITE); //Send 0x06
    LATC2 = 1;
    return;
}
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------

//_____________________Sector Erase Functions___________________________________________________
void cf_sector_erase(unsigned int32 sector_address)
{
    unsigned int8 address[4];

    address[0] = (unsigned int8)((sector_address >> 24) & 0xFF); // 0x _ _ 00 00 00
    address[1] = (unsigned int8)((sector_address >> 16) & 0xFF); // 0x 00 _ _ 00 00
    address[2] = (unsigned int8)((sector_address >> 8) & 0xFF);  // 0x 00 00 _ _ 00
    address[3] = (unsigned int8)((sector_address)&0xFF);         // 0x 00 00 00 _ _

    cf_write_enable();

    LATD5 = 0; //lower the CS PIN
    delay_us(2);
    ///////////////////////////////////////////////////////////////////
    spi_xfer(CFM, ERASE_SECTOR); //SECTOR ERASE COMAND   (0xDC)
    spi_xfer(CFM, address[0]);
    spi_xfer(CFM, address[1]);
    spi_xfer(CFM, address[2]);
    spi_xfer(CFM, address[3]);
    //////////////////////////////////////////////////////////////////
    delay_us(2);
    LATD5 = 1; //take CS PIN higher back
    delay_ms(3000);
    return;
}

void sf_sector_erase(unsigned int32 sector_address)
{
    unsigned int8 address[4];

    address[0] = (unsigned int8)((sector_address >> 24) & 0xFF); // 0x _ _ 00 00 00
    address[1] = (unsigned int8)((sector_address >> 16) & 0xFF); // 0x 00 _ _ 00 00
    address[2] = (unsigned int8)((sector_address >> 8) & 0xFF);  // 0x 00 00 _ _ 00
    address[3] = (unsigned int8)((sector_address)&0xFF);         // 0x 00 00 00 _ _

    sf_write_enable();

    LATC2 = 0; //lower the CS PIN
    delay_us(2);
    ///////////////////////////////////////////////////////////////////
    spi_xfer(SFM, ERASE_SECTOR); //SECTOR ERASE COMAND   (0xDC)
    spi_xfer(SFM, address[0]);
    spi_xfer(SFM, address[1]);
    spi_xfer(SFM, address[2]);
    spi_xfer(SFM, address[3]);
    //////////////////////////////////////////////////////////////////
    delay_us(2);
    LATC2 = 1; //take CS PIN higher back
    delay_ms(3000);
    return;
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

//________________________BYTE WRITE Function____________________________________________________
void cf_byte_write(unsigned int32 byte_address, int8 data)
{
    unsigned int8 address[4];

    //Byte extraction
    address[0] = (unsigned int8)((byte_address >> 24) & 0xFF); // 0x _ _ 00 00 00
    address[1] = (unsigned int8)((byte_address >> 16) & 0xFF); // 0x 00 _ _ 00 00
    address[2] = (unsigned int8)((byte_address >> 8) & 0xFF);  // 0x 00 00 _ _ 00
    address[3] = (unsigned int8)((byte_address)&0xFF);         // 0x 00 00 00 _ _

    cf_write_enable();

    LATD5 = 0; //lower the CS PIN
    delay_us(2);

    ////////////////////////////////////////////////////////////////
    spi_xfer(CFM, WRITE_BYTE); //Byte WRITE COMAND  (0x12)
    spi_xfer(CFM, address[0]);
    spi_xfer(CFM, address[1]);
    spi_xfer(CFM, address[2]);
    spi_xfer(CFM, address[3]);

    spi_xfer(CFM, data);
    ////////////////////////////////////////////////////////////////

    LATD5 = 1; //take CS PIN higher back

    return;
}

void sf_byte_write(unsigned int32 byte_address, int8 data)
{
    unsigned int8 address[4];

    //Byte extraction
    address[0] = (unsigned int8)((byte_address >> 24) & 0xFF); // 0x _ _ 00 00 00
    address[1] = (unsigned int8)((byte_address >> 16) & 0xFF); // 0x 00 _ _ 00 00
    address[2] = (unsigned int8)((byte_address >> 8) & 0xFF);  // 0x 00 00 _ _ 00
    address[3] = (unsigned int8)((byte_address)&0xFF);         // 0x 00 00 00 _ _

    sf_write_enable();

    LATC2 = 0; //lower the CS PIN
    delay_us(2);

    ////////////////////////////////////////////////////////////////
    spi_xfer(SFM, WRITE_BYTE); //PAGE WRITE COMAND  (0x12)
    spi_xfer(SFM, address[0]);
    spi_xfer(SFM, address[1]);
    spi_xfer(SFM, address[2]);
    spi_xfer(SFM, address[3]);

    spi_xfer(SFM, data);
    ////////////////////////////////////////////////////////////////

    LATC2 = 1; //take CS PIN higher back

    return;
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

//________________________BYTE read Function____________________________________________________
unsigned int8 cf_byte_read(unsigned int32 address)
{

    unsigned int8 address_[4];
    //Byte extraction
    address_[0] = (unsigned int8)((address >> 24) & 0xFF); // 0x _ _ 00 00 00
    address_[1] = (unsigned int8)((address >> 16) & 0xFF); // 0x 00 _ _ 00 00
    address_[2] = (unsigned int8)((address >> 8) & 0xFF);  // 0x 00 00 _ _ 00
    address_[3] = (unsigned int8)((address)&0xFF);         // 0x 00 00 00 _ _

    LATD5 = 0; //lower the CS PIN
    delay_us(2);
    //////////////////////////////////////////////////////////////////
    int8 data;
    spi_xfer(CFM, READ_DATA_BYTES); //READ DATA COMAND   (0x13)
    spi_xfer(CFM, address_[0]);
    spi_xfer(CFM, address_[1]);
    spi_xfer(CFM, address_[2]);
    spi_xfer(CFM, address_[3]);

    data = spi_xfer(CFM);
    //////////////////////////////////////////////////////////////////

    LATD5 = 1; //take CS PIN higher back
    return data;
}

unsigned int8 sf_byte_read(unsigned int32 address)
{

    unsigned int8 address_[4];
    //Byte extraction
    address_[0] = (unsigned int8)((address >> 24) & 0xFF); // 0x _ _ 00 00 00
    address_[1] = (unsigned int8)((address >> 16) & 0xFF); // 0x 00 _ _ 00 00
    address_[2] = (unsigned int8)((address >> 8) & 0xFF);  // 0x 00 00 _ _ 00
    address_[3] = (unsigned int8)((address)&0xFF);         // 0x 00 00 00 _ _

    LATC2 = 0; //lower the CS PIN
    delay_us(2);
    //////////////////////////////////////////////////////////////////
    int8 data;
    spi_xfer(SFM, READ_DATA_BYTES); //READ DATA COMAND   (0x13)
    spi_xfer(SFM, address_[0]);
    spi_xfer(SFM, address_[1]);
    spi_xfer(SFM, address_[2]);
    spi_xfer(SFM, address_[3]);

    data = spi_xfer(SFM);
    //////////////////////////////////////////////////////////////////

    LATC2 = 1; //take CS PIN higher back
    return data;
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

void flash_copy(unsigned int32 start_address, unsigned int32 data_length)
{

    cf_sector_erase(start_address);

    for (int32 inc = 0; inc < data_length; inc++) {
        cf_byte_write(start_address + inc, sf_byte_read(start_address + inc));
    }
    return;
}

/*
int8 READ_CHIP_ID()
{
   RC7 = 0;           //lower the CS PIN
 
   ////////////////////////////////////////////////////////////////
   int8 chip_id;
   spi_xfer(SPIPORT,READ_ID);    //READ ID COMAND   (0x9F)
   chip_id = spi_xfer(SPIPORT);
   ////////////////////////////////////////////////////////////////
 
   RC7 = 1;         //take CS PIN higher back
   return chip_id;
}


int8 READ_STATUS_REGISTER()
{
 RC7 = 0;           //lower the CS PIN
 
 /////////////////////////////////////////////////////////////////
 int8 status_reg;
 spi_xfer(SPIPORT,READ_STATUS_REG); //READ STATUS REGISTER COMAND  (0x05)
 status_reg = spi_xfer(SPIPORT);
 /////////////////////////////////////////////////////////////////
 
 RC7 = 1;         //take CS PIN higher back
 return status_reg;
}



*/
