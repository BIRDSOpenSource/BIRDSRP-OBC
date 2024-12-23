// reset_pic_start_pic.c: functions related to start pic

// This function send data a array to start pic______________________________________________________________________
// there are NOT tries used for in this
void send_cmd_to_start_pic()
{
    for (int k = 0; k < sizeof(reset_to_start_array); k++) {
        fputc(reset_to_start_array[k], StartPic);
    }
}

// Report health to start pic (I am alive!)
void report_health_to_start()
{
    if (start_to_reset_array[1] == 0xAA) {
        print_line();
        printf_debug("Send health report to Start pic >> ");
        print_line();

        reset_to_start_array[0] = 0xB0;
        reset_to_start_array[35] = 0xB1;
        reset_to_start_array[1] = 0xAA;

        send_cmd_to_start_pic();
        make_data_array_zero(reset_to_start_array, sizeof(reset_to_start_array));

        spi = 0;
    }
}

