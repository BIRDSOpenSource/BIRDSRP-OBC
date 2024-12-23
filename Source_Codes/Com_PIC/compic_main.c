#include <18F67J94.h>
#device adc = 16
#fuses WDT, NOBROWNOUT
#use delay(crystal = 16mhz)

#ifndef EM
#warning "Building for FM"
#define SPACECRAFT_ID 0x55
#else
#warning "Building for EM"
#define SPACECRAFT_ID 0x56
#endif // !EM

int8 next_in = 0;
int cw_enabled = true; // Current state of CW

#include <compic_main_sri.h> // Com main header file.

#INT_RDA
void serial_isr()
{
    buffer[next_in] = fgetc(TR_CP);
    next_in = (next_in + 1) % BUFFER_LENGTH; // Protects from buffer overflow.
}

void main(void)
{
    settings(); // Initial settings of COM PIC goes here.
    setup_wdt(WDT_128S);
    restart_wdt();
    delay_ms(5000L); // Just a delay before starting.
    rx_on();

    // // Uncomment block below for continuous CW carrier transmission
    // cw_on();
    // LATD3 = 1;
    // while(true){
    //     restart_wdt();
    // }

    while (true) {
        //______________(1)______CW_RELATED_TASKS__________________________________________<<
        rssi_value = rssi_read(); // Reading satellite noise level.
        for (int try = 0; try < 5; try ++) {
            communicate_with_main_pic(1, 0); // Asking CW Data.
            wait_for_main_pic_response();
            if (main_to_com[23] == 0x0C)
                break;
        }
        // const int8 ok_to_send_cw_index = 8; // After header and CW: 2 (header,cmd) + 6 (cw length)
        // int8 ok_to_send_cw = main_to_com[ok_to_send_cw_index];
        // if(ok_to_send_cw && cw_enabled){
            // send_cw();
        // }
        send_cw();
        send_heartbeat_to_reset();

        restart_wdt(); //***********************//
        rx_on();

        //______________(3)____RECEIVING_MODE_START_HERE___________________________________<<
        for (int16 i = 0; i < 60; i++) {
            buffer_correction();
            if ((sat_id != SPACECRAFT_ID) && (cmd_pckt[2] == 0x90)) // Be polite and keep quiet when other satellites are talking.
                delay_for_data_downlink();
            if (crc && ((sat_id == SPACECRAFT_ID) || (sat_id == 0x70))) // Checking crc, 49 is CURTIS satellite ID, 70 is the common ID for all satellites.
            {
                if (cmd_pckt[2] == 0x77)
                    sync(); // 38 Sync or delay
                switch (cmd_pckt[3]) {
                case 0x10: packet_download(); break;
                case 0x11: packet_download(); break;
                case 0x13: length_download(); break;
                case 0x14: length_download(); break;
                case 0x15: one_pckt_download(); break;
                case 0x16: one_pckt_download(); break;
                case 0x17: fm_copy(); break;                        // Without getting access.
                case 0x20: cw_update_mission(); break;
                case 0x21: cw_send_bgsc(); break;
                case 0x22: fm_copy_with_access(); break;
                case 0x23: enable_cw(); break;
                case 0x24: disable_cw(); break;
                case 0x27: fm_copy_and_data_down_from_cfm(); break; // 25 00 00 00 00    00 00 packet size
                case 0x30: send_com_temp_rssi_to_gs(); break;       // 30 00 00 00 00    00 00 00 00
                case 0x35: data_down_from_sfm_with_access(); break;
                case 0x40: set_of_packets_download(); break;
                case 0x41: set_of_packets_download(); break;        // 40 00 00 00 00    00 00 setsize setnumber
                case 0x42: get_sfm_access_given_time(); break;
                case 0x44: set_of_packets_download_with_access(); break;
                case 0xFA: reset_reset_pic(); break;
                case 0xFB: dc_dc_conv_main_com(); break;
                default: cmd_to_mpic(); break; // Previously A0.
                }
            }
            next_in = 0;
            rx_bfr_zero();
            delay_ms(2000);
            restart_wdt();
        }
    }
}
