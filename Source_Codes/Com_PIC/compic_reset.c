// compic_reset.c: functions related to reset pic

#define COM_TO_RESET_LENGTH 36
unsigned int8 com_to_reset[COM_TO_RESET_LENGTH] = { 0 };

// This function send data a array to reset pic______________________________________________________________________
void send_cmd_to_reset_pic()
{
    for (int k = 0; k < sizeof(com_to_reset); k++) {
        fputc(com_to_reset[k], RP_CP);
    }
}

void make_reset_pic_buffr_zero()
{
    for (int p = 0; p < COM_TO_RESET_LENGTH; p++) {
        com_to_reset[p] = 0x00;
    }
}

void send_heartbeat_to_reset()
{
    make_reset_pic_buffr_zero();
    com_to_reset[0] = 0xC0;
    com_to_reset[COM_TO_RESET_LENGTH - 1] = 0xC1;
    com_to_reset[1] = 0xDE;
    com_to_reset[2] = 0xAD;
    send_cmd_to_reset_pic();
}
