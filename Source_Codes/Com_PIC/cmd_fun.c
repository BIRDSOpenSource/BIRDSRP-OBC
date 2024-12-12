//from here the mission________________________________________________________

//_______________________Packet_download_______________________________________
void packet_download()
{
    unsigned int32 start_adress = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);
    unsigned int32 pckt_num = make32(cmd_pckt[8], cmd_pckt[9], cmd_pckt[10], cmd_pckt[11]);

    tx_on();
    delay_ms(1000);
    if (cmd_pckt[3] == 0x10) {
        send_data_packets(start_adress, 0, pckt_num, 1);
        rx_bfr_zero();
    }

    if (cmd_pckt[3] == 0x11) {
        send_data_packets(start_adress, 0, pckt_num, 0);
        rx_bfr_zero();
    }
    delay_ms(500);
    rx_on();
}

//____________________Lenth_Download___________________________________________
void length_download()
{
    unsigned int32 start_adress = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);
    unsigned int32 length = make32(cmd_pckt[8], cmd_pckt[9], cmd_pckt[10], cmd_pckt[11]);

    tx_on();
    delay_ms(1000);

    if (cmd_pckt[3] == 0x13) {
        send_data_packets(start_adress, length, 0, 1);
        rx_bfr_zero();
    }

    if (cmd_pckt[3] == 0x14) {
        send_data_packets(start_adress, length, 0, 0);
        rx_bfr_zero();
    }

    delay_ms(500);
    rx_on();
}

//____________________One_packt_download_______________________________________

void one_pckt_download()
{
    tx_on();
    delay_ms(1000);

    if (cmd_pckt[3] == 0x15) {
        send_a_packet(1);
        rx_bfr_zero();
    }

    if (cmd_pckt[3] == 0x16) {
        send_a_packet(0);
        rx_bfr_zero();
    }

    delay_ms(500);
    rx_on();
}

//__________________________SET of packet download________________________________________
void set_of_packets_download()
{
    unsigned int32 start_adress = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);

    tx_on();
    delay_ms(1000);
    if (cmd_pckt[3] == 0x40) {
        pckt_set_send(start_adress, cmd_pckt[11], cmd_pckt[10], 1);
        rx_bfr_zero();
    }

    else if (cmd_pckt[3] == 0x41) {
        pckt_set_send(start_adress, cmd_pckt[11], cmd_pckt[10], 0);
        rx_bfr_zero();
    }

    delay_ms(500);
    rx_on();
}

//set of acess downpppppppppppppppppppppppppppppppppppppppppppppppppppppppppppp
void set_of_packets_download_with_access()
{
    unsigned int32 start_adress = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);
    for (int try = 0; try < 5; try ++) {
        communicate_with_main_pic(2, cmd_pckt[8]); //getting accees
        wait_for_main_pic_response();
        if (main_to_com[23] == 0x0C)
            break;
    }

    tx_on();
    delay_ms(1000);

    pckt_set_send(start_adress, cmd_pckt[11], cmd_pckt[10], 0);
    rx_bfr_zero();

    delay_ms(500);
    rx_on();
}

//________________________FM_data_copy_________________________________________
void fm_copy()
{
    send_success_ack();
    unsigned int32 start_adress = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);
    unsigned int32 length = make32(cmd_pckt[8], cmd_pckt[9], cmd_pckt[10], cmd_pckt[11]);

    flash_copy(start_adress, length);

    send_success_ack();
    rx_bfr_zero();

    rx_on();
}

//________________________Copy With Access_________________________________________________
void fm_copy_with_access()
{
    unsigned int32 start_adress = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);
    unsigned int16 packet = make16(cmd_pckt[10], cmd_pckt[11]); //as 256byte packets

    for (int try = 0; try < 5; try ++) {
        communicate_with_main_pic(2, cmd_pckt[8]); //getting accees
        wait_for_main_pic_response();
        if (main_to_com[23] == 0x0C)
            break;
    }

    if (main_to_com[12] == 0x66) {
        send_success_ack(); // success of access
        delay_ms(1000);
        flash_copy(start_adress, packet * 256);
        send_success_ack(); // success of copieng
    }

    else
        send_not_success_ack();

    rx_bfr_zero();
    rx_on();
}

//________________________Copy With Access_________________________________________________
void fm_copy_and_data_down_from_cfm()
{
    unsigned int32 start_adress = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);
    unsigned int16 packet = make16(cmd_pckt[10], cmd_pckt[11]); //as 81byte packets

    for (int try = 0; try < 5; try ++) {
        communicate_with_main_pic(2, cmd_pckt[8]); //getting accees for 1 min
        wait_for_main_pic_response();
        if (main_to_com[23] == 0x0C)
            break;
    }

    if (main_to_com[12] == 0x66) {
        flash_copy(start_adress, packet * 81);

        tx_on();
        delay_ms(1000);
        send_data_packets(start_adress, 0, packet, 1);
        delay_ms(500);
        rx_on();
    }

    else {
        tx_on();
        delay_ms(1000);
        send_not_success_ack();
        delay_ms(500);
        rx_on();
    }

    rx_bfr_zero();
}

//999999999999999999999999999999999999999999999999999999999999999999999999999999999999999
//________________________Copy With Access_________________________________________________
void data_down_from_sfm_with_access()
{
    unsigned int32 start_adress = make32(cmd_pckt[4], cmd_pckt[5], cmd_pckt[6], cmd_pckt[7]);
    unsigned int16 packet = make16(cmd_pckt[10], cmd_pckt[11]); //as 81byte packets
    unsigned int8 skip = cmd_pckt[9];

    for (int try = 0; try < 5; try ++) {
        communicate_with_main_pic(2, cmd_pckt[8]); //getting accees
        wait_for_main_pic_response();
        if (main_to_com[23] == 0x0C)
            break;
    }

    delay_ms(2000);

    if (main_to_com[12] == 0x66) {

        tx_on();
        delay_ms(1000);
        send_data_packets(start_adress, 0, packet, 0, skip);
        delay_ms(500);
        rx_on();
    }

    rx_bfr_zero();
}

void sync()
{
    unsigned int8 delay = cmd_pckt[11];

    if (delay <= 120) {
        send_success_ack();
        for (int i = 0; i < delay; i++) {
            delay_ms(5000);
            restart_wdt(); //***********************//}
        }
    }

    else {
        send_not_success_ack();
    }

    rx_bfr_zero();
}

// ___________________CMD to main PIC___________________________________________________
void cmd_to_mpic()
{
    for (int try = 0; try < 5; try ++) {
        communicate_with_main_pic(0, 0);
        wait_for_main_pic_response();
        if (main_to_com[23] == 0x0C)
            break;
    }

    if (main_to_com[12] == 0x66) {
        memcpy(cmd_pckt, main_to_com + 2, 9); // echo from main pic, usually the same as before, except for command 0x00, when time is returned instead
        send_success_ack();
    }

    if (main_to_com[12] == 0x00)
        send_not_success_ack();

    rx_bfr_zero();
}

//_________________________GET_FM_ACCESS_________________________________________________
void get_sfm_access_given_time()
{
    if ((cmd_pckt[11] == 0x01) || (cmd_pckt[11] == 0x05) || (cmd_pckt[11] == 0x0a)) // main pic comands
    {

        for (int try = 0; try < 5; try ++) {
            communicate_with_main_pic(2, cmd_pckt[11]);
            wait_for_main_pic_response();
            if (main_to_com[23] == 0x0C)
                break;
        }

        if (main_to_com[12] == 0x66)
            send_success_ack();

        if (main_to_com[12] == 0x00)
            send_not_success_ack();

        rx_bfr_zero();
    }
}

//__________________________SPECIAL______________________________________________________
void delay_for_data_downlink()
{
    send_success_ack();

    for (int i = 0; i < 30; i++) {
        delay_ms(20000);
        restart_wdt(); //***********************//}
    }

    rx_bfr_zero();
}

//__________________force CW send________________________________________________________
void cw_send_bgsc()
{
    cw_on();

    if (cmd_pckt[4]) {
        LATD3 = 1;
        delay_ms(30000);
        LATD3 = 0;
    } else {
        for (int try = 0; try < 5; try++) {
            communicate_with_main_pic(1, 0); // Asking CW Data.
            wait_for_main_pic_response();
            if (main_to_com[23] == 0x0C)
                break;
        }
        cw_pckt();
        delay_ms(500);
    }

    rx_on();
    rx_bfr_zero();
}

//__________________CW update mission____________________________________________________
void cw_update_mission()
{
    cf_sector_erase(0x00000000);

    cf_byte_write(0x00000000, cmd_pckt[10]); //should be ff

    cf_byte_write(0x00000001, cmd_pckt[4]);
    cf_byte_write(0x00000002, cmd_pckt[5]);
    cf_byte_write(0x00000003, cmd_pckt[6]);
    cf_byte_write(0x00000004, cmd_pckt[7]);
    cf_byte_write(0x00000005, cmd_pckt[8]);
    cf_byte_write(0x00000006, cmd_pckt[9]);

    send_success_ack();
    rx_bfr_zero();
}
//__________________Enable/disable CW____________________________________________________
void enable_cw()
{
    cw_enabled = true;
    send_success_ack();
    rx_bfr_zero();
}

void disable_cw()
{
    cw_enabled = false;
    send_success_ack();
    rx_bfr_zero();
}

//_______________________________SEND temp rssi__________________________________________
void send_com_temp_rssi_to_gs()
{
    send_com_data();
    rx_bfr_zero();
}

//_______________________________CW_send_________________________________________________
void send_cw()
{
    cw_on();
    cw_pckt();
    rx_on();
    rx_bfr_zero();
}
