#define NRZI3_DATA_SIZE 101

unsigned int8 sat_id;
int crc = 0; // 0 when CRC is wrong, 1 when CRC is correct.
int8 mbi = 0;
unsigned int32 num = 0;

// CRC calculation function
int16 mk_crc(unsigned int8* data, unsigned int8 size)
{
    unsigned int32 crcreg = 0xffff;
    unsigned int32 calc = 0x8408;
    unsigned int8 w;
    int32 i;
    int32 k;
    unsigned int8 cal_data[NRZI3_DATA_SIZE];

    for (k = 0; k < size; k++) {
        cal_data[k] = data[k];

        for (i = 0; i < 8; i++) {
            w = (crcreg ^ cal_data[k]) & 0x0001;
            crcreg = crcreg >> 1;

            if (w == 1) {
                crcreg = crcreg ^ calc;
            }

            cal_data[k] = cal_data[k] >> 1;
        }
    }

    crcreg = crcreg ^ 0xffff;
    return crcreg;
}
//___________________________________________________________________________<<

//___________________________________________________________________________>>
//_________________Receive data packets________________________________________
#define CMD_LEGACY_LENGTH 14
#define CMD_LENGTH 22
unsigned int8 cmd_pckt[CMD_LENGTH] = { 0 }; // Makes all elements zero

unsigned int ib = 0; //input buffer increment

//______________________This function make rx buffer zero______________________________<<
void rx_bfr_zero()
{
    for (int p = 0; p < CMD_LENGTH; p++) {
        cmd_pckt[p] = 0x00;
    }
    ib = 0;
}
//_____________________________________________________________________________________<<

//_____________________________________________________________________________________<<
//_________________Receive data dump buffer______________________________________________
//_________________This function makes the temporary buffer zero_________________________

#define BUFFER_DELTA 10
#define BUFFER_LENGTH (CMD_LENGTH + BUFFER_DELTA)
unsigned int8 buffer[BUFFER_LENGTH];
void buffer_zero()
{
    for (int p = 0; p < BUFFER_LENGTH; p++) {
        buffer[p] = 0x00;
    }
}

//_____________________________________________________________________________________<<

//_____________________________________________________________________________________>>
//_________________Acknowledgement Success packet________________________________________

#define ACK_LENGTH 32
unsigned int8 ack_pckt[ACK_LENGTH];

void send_success_ack()
{
    //    Ground stationCall sign---------------[ 0-----5 ]
    ack_pckt[0] = 0x4A; // J
    ack_pckt[1] = 0x47; // G
    ack_pckt[2] = 0x36; // 6
    ack_pckt[3] = 0x59; // Y
    ack_pckt[4] = 0x42; // B
    ack_pckt[5] = 0x57; // W

    //    Dest SSID-----------------------------[ 6 ]
    ack_pckt[6] = 0x30; //char 0

    //    Satellite Call Sign-------------------[ 7-----12 ]
    ack_pckt[7] = 'P';
    ack_pckt[8] = 'K';
    ack_pckt[9] = '-';
    ack_pckt[10] = 'S';
    ack_pckt[11] = 'A';
    ack_pckt[12] = 'T';

    //    Source SSID---------------------------[ 13 ]
    ack_pckt[13] = 0x30; //char 0

    //    Control-------------------------------[ 14 ]
    ack_pckt[14] = 0x3E; //

    //    PID-----------------------------------[ 15 ]
    ack_pckt[15] = 0xF0; //

    //    Header--------------------------------[ 16 ]
    ack_pckt[16] = 0xAA; //

    //    SAT Header----------------------------[ 17]
    ack_pckt[17] = 0x03; //

    //    Pckt sequence number higher byte------[ 18 ]
    ack_pckt[18] = cmd_pckt[1]; //

    //    Pckt sequence number lower byte-------[ 19 ]
    ack_pckt[19] = cmd_pckt[2]; //

    //    CMD back------------------------------[ 20 - 28 ]
    ack_pckt[20] = cmd_pckt[3];
    ack_pckt[21] = cmd_pckt[4];
    ack_pckt[22] = cmd_pckt[5];
    ack_pckt[23] = cmd_pckt[6];
    ack_pckt[24] = cmd_pckt[7];
    ack_pckt[25] = cmd_pckt[8];
    ack_pckt[26] = cmd_pckt[9];
    ack_pckt[27] = cmd_pckt[10];
    ack_pckt[28] = cmd_pckt[11];

    //    Footer--------------------------------[ 29 ]
    ack_pckt[29] = 0xAA; //

    //    CRC Calculation-----------------------[ 30 - 31 ]
    int16 outdata = mk_crc(ack_pckt, 30);

    ack_pckt[30] = outdata;
    ack_pckt[31] = outdata >> 8;

    //    Send pckt------------------------------
    output_high(PIN_D3);
    delay_ms(2000);
    for (int ih = 0; ih < ACK_LENGTH; ih++) //buffer print
    {
        fputc(ack_pckt[ih], TR_CP);
    }
    delay_ms(500);
    output_low(PIN_D3);
}
//___________________________________________________________________________<<

//_____________________________________________________________________________________>>
//_________________Acknoledgement Success packet_________________________________________

void send_not_success_ack()
{
    //    Ground stationCall sign---------------[ 0-----5 ]
    ack_pckt[0] = 0x4a; // J
    ack_pckt[1] = 0x47; // G
    ack_pckt[2] = 0x36; // 6
    ack_pckt[3] = 0x59; // Y
    ack_pckt[4] = 0x42; // B
    ack_pckt[5] = 0x57; // W

    //    Dest SSID-----------------------------[ 6 ]
    ack_pckt[6] = 0x30; //char 0

    //    Satellite Call Sign-------------------[ 7-----12 ]
    ack_pckt[7] = 'P';
    ack_pckt[8] = 'K';
    ack_pckt[9] = '-';
    ack_pckt[10] = 'S';
    ack_pckt[11] = 'A';
    ack_pckt[12] = 'T';

    //    Source SSID---------------------------[ 13 ]
    ack_pckt[13] = 0x30; //char 0

    //    Control-------------------------------[ 14 ]
    ack_pckt[14] = 0x3E; //

    //    PID-----------------------------------[ 15 ]
    ack_pckt[15] = 0xF0; //

    //    Header--------------------------------[ 16 ]
    ack_pckt[16] = 0xAA; //

    //    SAT Header----------------------------[ 17]
    ack_pckt[17] = 0x03; //

    //    Pckt sequence number higher byte------[ 18 ]
    ack_pckt[18] = cmd_pckt[1]; //

    //    Pckt sequence number lower byte-------[ 19 ]
    ack_pckt[19] = cmd_pckt[2]; //

    //    CMD back------------------------------[ 20 - 28 ]
    ack_pckt[20] = 0x69;
    ack_pckt[21] = 0x69;
    ack_pckt[22] = 0x69;
    ack_pckt[23] = 0x69;
    ack_pckt[24] = 0x69;
    ack_pckt[25] = 0x69;
    ack_pckt[26] = 0x69;
    ack_pckt[27] = 0x69;
    ack_pckt[28] = 0x69;

    //    Footer--------------------------------[ 29 ]
    ack_pckt[29] = 0xAA; //

    //    CRC Calculation-----------------------[ 30 - 31 ]
    int16 outdata = mk_crc(ack_pckt, 30);

    ack_pckt[30] = outdata;
    ack_pckt[31] = outdata >> 8;

    //    Send pckt------------------------------
    output_high(PIN_D3);
    delay_ms(2000);
    for (int ih = 0; ih < ACK_LENGTH; ih++) //buffer print
    {
        fputc(ack_pckt[ih], TR_CP);
    }
    delay_ms(500);
    output_low(PIN_D3);
}
//_____________________________________________________________________________________<<

//_____________________________________________________________________________________>>
//_________________Output data packets___________________________________________________

#define DATA_PCKT_LENGTH 105
unsigned int8 data_pckt[DATA_PCKT_LENGTH];

void send_data_packets(unsigned int32 start_address, unsigned int32 data_size, unsigned int32 packets, int1 x, unsigned int8 skip = 1)
{
    unsigned int32 num_of_pckt;

    if (packets == 0) {
        num_of_pckt = (data_size / 81) + 1;
    }

    else if (data_size == 0) {
        num_of_pckt = packets;
    }

    for (unsigned int32 pckt_seq_num = 1; pckt_seq_num <= num_of_pckt; pckt_seq_num++) {
        //=================Ground stationCall sign part=============================
        //========================= 0 --- 5 ========================================
        data_pckt[0] = 0x4a;  // J
        data_pckt[1] = 0x47;  // G
        data_pckt[2] = 0x36;  // 6
        data_pckt[3] = 0x59;  // Y
        data_pckt[4] = 0x42;  // B
        data_pckt[5] = 0x57;  // W
                              //=========================Dest SSID========================================
                              //=========================   6     ========================================
        data_pckt[6] = 0x30;  //char 0
                              //===================Satellite Call Sign====================================
                              //========================= 7 --- 12  ======================================
        data_pckt[7] = 'P';
        data_pckt[8] = 'K';
        data_pckt[9] = '-';
        data_pckt[10] = 'S';
        data_pckt[11] = 'A';
        data_pckt[12] = 'T';
                              //========================Source SSID=======================================
                              //=========================   13   =========================================
        data_pckt[13] = 0x30; //char 0
                              //==========================Control=========================================
                              //=========================   14   =========================================
        data_pckt[14] = 0x3E; //
                              //============================PID===========================================
                              //=========================   15   =========================================
        data_pckt[15] = 0xF0; //
                              //=====================Packet_specified_data================================
                              //========================= 16 ---- 17 =====================================
        data_pckt[16] = 0xFF;
        data_pckt[17] = 0xF0;
        data_pckt[18] = 0xFf;
        data_pckt[19] = pckt_seq_num >> 16;
        data_pckt[20] = pckt_seq_num >> 8;
        data_pckt[21] = pckt_seq_num;
        //===========================Pay_Load=======================================
        //========================= 22 ---- 102 ====================================
        if (x == 1) {
            for (int32 ele_num = 22; ele_num <= 102; ele_num++) {
                data_pckt[ele_num] = cf_byte_read(start_address + 81 * (pckt_seq_num - 1) + ele_num - 22);
            }
        }

        if (x == 0) {
            for (int32 ele_num = 22; ele_num <= 102; ele_num++) {
                data_pckt[ele_num] = sf_byte_read(start_address + 81 * (pckt_seq_num - 1) + ele_num - 22);
            }
        }

        //===========================CRC Value======================================
        //======================== 103 ---- 104 ====================================
        int16 outdata = mk_crc(data_pckt, 103);

        data_pckt[103] = outdata;
        data_pckt[104] = outdata >> 8;

        //__________________________________________________________________________

        for (int8 ia = 0; ia <= 104; ia++) {
            fputc(data_pckt[ia], tr_cp);
        }

        if (pckt_seq_num % 2 == 0)
            pckt_seq_num += 2 * skip;

        delay_ms(220);
    }
}

//___________________________________________________________________________<<

void pckt_set_send(unsigned int32 initial_address, unsigned int8 packet_set_size, unsigned int8 packet_set_num, int1 fm_select)
{
    unsigned int32 next_size = packet_set_size * (packet_set_num - 1);

    unsigned int32 start_address = initial_address + 81 * next_size;

    for (unsigned int32 pckt_seq_num = 1; pckt_seq_num <= packet_set_size; pckt_seq_num++) {
        //=================Ground stationCall sign part=============================
        //========================= 0 --- 5 ========================================
        data_pckt[0] = 0x4a;  // J
        data_pckt[1] = 0x47;  // G
        data_pckt[2] = 0x36;  // 6
        data_pckt[3] = 0x59;  // Y
        data_pckt[4] = 0x42;  // B
        data_pckt[5] = 0x57;  // W
                              //=========================Dest SSID========================================
                              //=========================   6     ========================================
        data_pckt[6] = 0x30;  //char 0
                              //===================Satellite Call Sign====================================
                              //========================= 7 --- 12  ======================================
        data_pckt[7] = 'P';
        data_pckt[8] = 'K';
        data_pckt[9] = '-';
        data_pckt[10] = 'S';
        data_pckt[11] = 'A';
        data_pckt[12] = 'T';
                              //========================Source SSID=======================================
                              //=========================   13   =========================================
        data_pckt[13] = 0x30; //char 0
                              //==========================Control=========================================
                              //=========================   14   =========================================
        data_pckt[14] = 0x3E; //
                              //============================PID===========================================
                              //=========================   15   =========================================
        data_pckt[15] = 0xF0; //
                              //=====================Packet_specified_data================================
                              //========================= 16 ---- 17 =====================================
        data_pckt[16] = 0xFF;
        data_pckt[17] = 0xF0;
        data_pckt[18] = 0xFF;
        data_pckt[19] = pckt_seq_num >> 16;
        data_pckt[20] = pckt_seq_num >> 8;
        data_pckt[21] = pckt_seq_num;
        //===========================Pay_Load=======================================
        //========================= 22 ---- 102 ====================================
        if (fm_select == 1) //com flash
        {
            for (int32 ele_num = 22; ele_num <= 102; ele_num++) {
                data_pckt[ele_num] = cf_byte_read(start_address + 81 * (pckt_seq_num - 1) + ele_num - 22);
            }
        }

        else if (fm_select == 0) //shared flash
        {
            for (int32 ele_num = 22; ele_num <= 102; ele_num++) {
                data_pckt[ele_num] = sf_byte_read(start_address + 81 * (pckt_seq_num - 1) + ele_num - 22);
            }
        }

        //===========================CRC Value======================================
        //======================== 103 ---- 104 ====================================
        int16 outdata = mk_crc(data_pckt, 103);

        data_pckt[103] = outdata;
        data_pckt[104] = outdata >> 8;

        //__________________________________________________________________________

        for (int8 ia = 0; ia <= 104; ia++) {
            fputc(data_pckt[ia], tr_cp);
        }
        delay_ms(200);
    }
}

//_____________________________________________________________________________________>>
//_________________Output a packet_______________________________________________________

void send_a_packet(int1 x)
{
    unsigned int32 start_address = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);
    unsigned int32 pckt = make32(cmd_pckt[8], cmd_pckt[9], cmd_pckt[10], cmd_pckt[11]);

    //=================Ground stationCall sign part=============================
    //========================= 0 --- 5 ========================================
    data_pckt[0] = 0x4A;  // J
    data_pckt[1] = 0x47;  // G
    data_pckt[2] = 0x36;  // 6
    data_pckt[3] = 0x59;  // Y
    data_pckt[4] = 0x42;  // B
    data_pckt[5] = 0x57;  // W
                          //=========================Dest SSID========================================
                          //=========================   6     ========================================
    data_pckt[6] = 0x30;  //char 0
                          //===================Satellite Call Sign====================================
                          //========================= 7 --- 12  ======================================
    data_pckt[7] = 'P';
    data_pckt[8] = 'K';
    data_pckt[9] = '-';
    data_pckt[10] = 'S';
    data_pckt[11] = 'A';
    data_pckt[12] = 'T';
                          //========================Source SSID=======================================
                          //=========================   13   =========================================
    data_pckt[13] = 0x30; //char 0
                          //==========================Control=========================================
                          //=========================   14   =========================================
    data_pckt[14] = 0x3E; //
                          //============================PID===========================================
                          //=========================   15   =========================================
    data_pckt[15] = 0xF0; //
                          //=====================Packet_specified_data================================
                          //========================= 16 ---- 17 =====================================
    data_pckt[16] = 0xFF;
    data_pckt[17] = 0xF0;
    data_pckt[18] = 0xFf;
    data_pckt[19] = pckt >> 16;
    data_pckt[20] = pckt >> 8;
    data_pckt[21] = pckt;
    //===========================Pay_Load=======================================
    //========================= 22 ---- 102 ====================================
    if (x == 1) {
        for (int32 ele_num = 22; ele_num <= 102; ele_num++) {
            data_pckt[ele_num] = cf_byte_read(start_address + 81 * (pckt - 1) + ele_num - 22);
        }
    }

    if (x == 0) {
        for (int32 ele_num = 22; ele_num <= 102; ele_num++) {
            data_pckt[ele_num] = sf_byte_read(start_address + 81 * (pckt - 1) + ele_num - 22);
        }
    }

    //===========================CRC Value======================================
    //======================== 103 ---- 104 ====================================
    int16 outdata = mk_crc(data_pckt, 103);

    data_pckt[103] = outdata;
    data_pckt[104] = outdata >> 8;

    //__________________________________________________________________________

    for (int8 ia = 0; ia <= 104; ia++) {
        fputc(data_pckt[ia], tr_cp);
    }
    delay_ms(200);
}
//___________________________________________________________________________<<

//_____________________________________________________________________________________<<
//_________________________This function corrects the receiving buffer___________________

// unsigned int8 sat_id;
// int crc = 0; // 0 when CRC is wrong, 1 when CRC is correct.
void buffer_correction()
{
    unsigned int16 cr, pk; // these are the crc check variables
    for (int i = 0; i < BUFFER_DELTA; i++) {
        if (buffer[i] == 0x42) {
            for (int ih = 0; ih < CMD_LENGTH; ih++) {
                cmd_pckt[ih] = buffer[ih + i];
            }
            next_in = 0;
            buffer_zero();
            break;
        }
    }
    sat_id = cmd_pckt[1];
    if (cmd_pckt[2] == 0xCC) { // Extended KITSUNE format (22 bytes)
        cr = mk_crc(cmd_pckt, CMD_LENGTH - 2);
        pk = make16(cmd_pckt[CMD_LENGTH - 1], cmd_pckt[CMD_LENGTH - 2]);
    } else { // Herritage BIRDS format (14 bytes)
        cr = mk_crc(cmd_pckt, CMD_LEGACY_LENGTH - 2);
        pk = make16(cmd_pckt[CMD_LEGACY_LENGTH - 1], cmd_pckt[CMD_LEGACY_LENGTH - 2]);
    }
    crc = (cr == pk);
}
//___________________________________________________________________________<<

//_____________________________________________________________________________________<<
//_________________CMD to obc____________________________________________________________
#define COM_TO_MAIN_LENGTH (CMD_LENGTH + 3) // Adding header, checksum and footer fields
unsigned int8 com_to_main[COM_TO_MAIN_LENGTH];

#define MAIN_TO_COM_LENGTH 24
unsigned int8 main_to_com[MAIN_TO_COM_LENGTH] = { 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0 };

void wait_for_main_pic_response()
{
    for (; num < 200000; num++) {
        if (kbhit(mp_cp)) {
            main_to_com[mbi] = fgetc(mp_cp);
            mbi++;
        }
    }
    mbi = 0;
    num = 0;
}

void make_main_pic_buffr_zero()
{
    for (int p = 0; p < MAIN_TO_COM_LENGTH; p++) {
        main_to_com[p] = 0x00;
    }
}

void initialize_com_to_main()
{
    com_to_main[0] = 0xC0; // header
    for (int i = 1; i < COM_TO_MAIN_LENGTH - 2; i++) {
        com_to_main[i] = 0; // data
    }
    com_to_main[COM_TO_MAIN_LENGTH - 2] = 0;    // checksum
    com_to_main[COM_TO_MAIN_LENGTH - 1] = 0xC1; // footer
}

void add_checksum_com_to_main()
{
    char checksum = 0;
    for (int i = 1; i < COM_TO_MAIN_LENGTH - 2; i++) {
        checksum ^= com_to_main[i];
    }
    com_to_main[COM_TO_MAIN_LENGTH - 2] = checksum;
}

void communicate_with_main_pic(int8 p, unsigned int8 time)
{
    make_main_pic_buffr_zero();
    initialize_com_to_main();

    if (p == 0) { // for com to main commands
        for (int i = 0; i < CMD_LENGTH; i++) {
            com_to_main[i + 1] = cmd_pckt[i];
        }
    } else if (p == 1) { // cw ask
        com_to_main[1] = 0x50;
        *(int16*)&com_to_main[2] = rssi_value;
    } else if (p == 2) { // get access
        com_to_main[1] = 0x59;
        com_to_main[2] = time;
    }

    add_checksum_com_to_main();

    for (int8 ia = 0; ia < COM_TO_MAIN_LENGTH; ia++) {
        fputc(com_to_main[ia], MP_CP);
    }
}
//___________________________________________________________________________<<

//___________________________________________________________________________<<

//____________________This function create packet of rssi and temp of trx________________
void send_com_data()
{
    //    Ground stationCall sign---------------[ 0-----5 ]
    ack_pckt[0] = 0x4a; // J
    ack_pckt[1] = 0x47; // G
    ack_pckt[2] = 0x36; // 6
    ack_pckt[3] = 0x59; // Y
    ack_pckt[4] = 0x42; // B
    ack_pckt[5] = 0x57; // W

    //    Dest SSID-----------------------------[ 6 ]
    ack_pckt[6] = 0x30; //char 0

    //    Satellite Call Sign-------------------[ 7-----12 ]
    ack_pckt[7] = 'P';
    ack_pckt[8] = 'K';
    ack_pckt[9] = '-';
    ack_pckt[10] = 'S';
    ack_pckt[11] = 'A';
    ack_pckt[12] = 'T';

    //    Source SSID---------------------------[ 13 ]
    ack_pckt[13] = 0x30; //char 0

    //    Control-------------------------------[ 14 ]
    ack_pckt[14] = 0x3E; //

    //    PID-----------------------------------[ 15 ]
    ack_pckt[15] = 0xF0; //

    //    Header--------------------------------[ 16 ]
    ack_pckt[16] = 0xAA; //

    //    SAT Header----------------------------[ 17]
    ack_pckt[17] = 0x03; //

    //    Pckt sequence number higher byte------[ 18 ]
    ack_pckt[18] = cmd_pckt[1]; //

    //    Pckt sequence number lower byte-------[ 19 ]
    ack_pckt[19] = cmd_pckt[2]; //

    //    CMD back------------------------------[ 20 - 28 ]
    int16 tempr = trx_temp_read();

    unsigned int32 rssi64 = 0;

    for (int lk = 0; lk < 100; lk++) {
        rssi64 = rssi64 + trx_rssi_read();
        delay_us(100);
    }

    int16 rssi = rssi64 / 100;

    ack_pckt[20] = cmd_pckt[3];
    ack_pckt[21] = 0xDD;
    ack_pckt[22] = rssi >> 8;
    ack_pckt[23] = rssi;
    ack_pckt[24] = 0xDD;
    ack_pckt[25] = 0xDD;
    ack_pckt[26] = tempr >> 8;
    ack_pckt[27] = tempr;
    ack_pckt[28] = 0xDD;

    //    Footer--------------------------------[ 29 ]
    ack_pckt[29] = 0xAA; //

    //    CRC Calculation-----------------------[ 30 - 31 ]
    int16 outdata = mk_crc(ack_pckt, 30);

    ack_pckt[30] = outdata;
    ack_pckt[31] = outdata >> 8;

    //    Send pckt------------------------------
    output_high(PIN_D3);
    delay_ms(2000);
    for (int ih = 0; ih < ACK_LENGTH; ih++) //buffer print
    {
        fputc(ack_pckt[ih], tr_cp);
    }
    delay_ms(500);
    output_low(PIN_D3);
}

unsigned int16 rssi_read()
{
    unsigned int32 rssi = 0;

    for (int lk = 0; lk < 20; lk++) {
        rssi = rssi + trx_rssi_read();
        delay_us(10);
    }

    unsigned int16 rssi1 = rssi / 20; //averaging for better value

    return rssi1;
}
