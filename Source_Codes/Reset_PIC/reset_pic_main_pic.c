// reset_pic_main_pic.c: functions related to main pic

// This function send data a array to reset pic and wait for ACK______________________________________________________________________
// there are NOT trys used for in this
void send_cmd_to_main_pic_and_wait_for_ack(int not, char wait_time)
{
    mpi = 0;

    for (int i = 0; i < not ; i++) {
        fprintf(Debug, "Sending data array to MainPIC try - %02d\n\r", i + 1);
        for (int k = 0; k <= 35; k++) {
            fputc(reset_to_main_array[k], MainPIC);
        }
        mpi = 0;
        delay_ms(wait_time);
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[35] == 0xB1))
            break;
    }
}

// Resonding 90 sec frequent communication command_____________________________________________________________________________________A0
void respond_to_mainpic_90sec_cmd() // MP HF
{
    if (main_to_reset_array[1] == 0xA0) {
        int i;
        print_line();
        fprintf(Debug, "\n\r");
        fprintf(Debug, "90Sec Comunication Data Array From MainPIC >> ");
        for (i = 0; i <= 35; i++) {
            fprintf(Debug, "%X ", main_to_reset_array[i]);
        }
        fprintf(Debug, "\n\r");
        print_line();
        print_line();

        make_data_array_zero(reset_to_main_array, 36);

        _raw_power_adc_val = measure_raw_voltage();
        _3v3_1_current_adc_val = measure_3v3_1_current();     //float _3v3_1_current = 0 ;
        _5v0_2_current_adc_val = measure_5v0_2_current();     //float _5v0_2_current = 0 ;
        _5v0_current_adc_val = measure_5v0_current();         //float _5V0_current = 0 ;
        _12v0_current_adc_val = measure_12v0_current();       //float _12V0_current = 0 ;
        _unreg_1_current_adc_val = measure_unreg_1_current(); //float _UNREG_1_current = 0 ;
        _unreg_2_current_adc_val = measure_unreg_2_current(); //float _UNREG_2_current = 0 ;
        _unreg_3_current_adc_val = measure_unreg_3_current(); //float _UNREG_3_current = 0 ;
        
        reset_to_main_array[0] = 0xB0;

        reset_to_main_array[1] = 0xA0;
        reset_to_main_array[2] = year;
        reset_to_main_array[3] = month;
        reset_to_main_array[4] = day;
        reset_to_main_array[5] = hour;
        reset_to_main_array[6] = minute;
        reset_to_main_array[7] = second;
        reset_to_main_array[8] = (unsigned int8)((_raw_power_adc_val >> 8) & 0xFF);
        reset_to_main_array[9] = (unsigned int8)((_raw_power_adc_val)&0xFF);
        reset_to_main_array[10] = (unsigned int8)((_3v3_1_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[11] = (unsigned int8)((_3v3_1_current_adc_val)&0xFF);
        reset_to_main_array[12] = (unsigned int8)((_5v0_2_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[13] = (unsigned int8)((_5v0_2_current_adc_val)&0xFF);
        reset_to_main_array[14] = (unsigned int8)((_5v0_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[15] = (unsigned int8)((_5v0_current_adc_val)&0xFF);
        reset_to_main_array[16] = (unsigned int8)((_unreg_1_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[17] = (unsigned int8)((_unreg_1_current_adc_val)&0xFF);
        reset_to_main_array[18] = (unsigned int8)((_unreg_2_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[19] = (unsigned int8)((_unreg_2_current_adc_val)&0xFF);
        reset_to_main_array[20] = (unsigned int8)((_unreg_3_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[21] = (unsigned int8)((_unreg_3_current_adc_val)&0xFF);
        reset_to_main_array[22] = (unsigned int8)((_12v0_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[23] = (unsigned int8)((_12v0_current_adc_val)&0xFF);
        reset_to_main_array[24] = powerline_status;
        reset_to_main_array[25] = mainpic_status;

        reset_to_main_array[35] = 0xB1;

        for (i = 0; i < 36; i++) {
            fputc(reset_to_main_array[i], MainPIC);
        }
        // fprintf(Debug, "Sending data array to MainPIC >>  \n\r",);
        // for (i = 0; i < 36; i++) {
        //    fprintf(Debug, "%X ",  reset_to_main_array[i]);
        // }
        // fprintf(Debug, "\n\r");
        make_data_array_zero(main_to_reset_array, 36);
        mpi = 0;
        main_pic_counter = 0;
    }
}

void check_main_pic_health()
{

    if (main_pic_counter >= 1800) // wait 1800/60 = 30 min before reseting main pic // Normal Mode
    {
        fprintf(Debug, "Main PIC hang up Reset \n\r");
        main_pic_counter = 0;
        main_pic_rst_counter++;

        mainpic_power(0);
        delay_ms(5000); // Delay
        mainpic_power(1);
        //delay_ms(500);
    }
}

// Send warning to main pic before 24Hour reset___________________________________________________________________________________________________________________________A2
void warn_to_main_pic_before_24h_reset()
{
    if ((hour == 23) && (minute == 59) && (0 == second)) {
        reset_to_main_array[0] = 0xB0;
        reset_to_main_array[35] = 0xB1;
        reset_to_main_array[1] = 0xA2;

        print_line();
       fprintf(Debug, "\n\r");
        send_cmd_to_main_pic_and_wait_for_ack(20, 200);

        // if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[35] == 0xB1) && (main_to_reset_array[1] == 0xA2)) {
        //     fprintf(Debug, "24 hour reset warning successfully sent to MainPIC \n\r");
        // }

        // else {
        //     fprintf(Debug, "24 hour reset warning was not successfull to MainPIC \n\r");
        // }
        // print_line();

        delay_ms(1000);

        make_data_array_zero(main_to_reset_array, 36);
        mpi = 0;
    }
}

// 24 hour system reset ///___________________________________________________________________________________________________________________________
void system_reset_24h()
{
    if ((hour == 0) && (minute == 0) && (second == 0)) {
        //_________________________________________________________________________________
        print_line();
       fprintf(Debug, "\n\r");

        mainpic_power(0);
        compic_power(0);
        _3v3_1_line(0);
        _5v0_2_line(0); //ADCS power line update:: agreed on turning off ADCS during 24hr reset @27.03.2024 meeting
        //_5v0_line(0);   // SEL power line
        _12v0_line(0);
        _unreg_1_line(0);
        _unreg_2_line(0);
        _unreg_3_line(0);

        for (int i = 0; i < 5; i++) {
            delay_ms(1000);
            fprintf(Debug, "Waiting to turn on system after 24 hour reset - %02d seconds\n\r", i + 1);
        }
        mainpic_power(1);
        compic_power(1);
        _3v3_1_line(1);
        _5v0_2_line(1);
        //_5v0_line(1);
        _12v0_line(1); ////2024.08.05 requested by Dr. victor //2024.01.27 agreed on with Dr. Victor
        _unreg_1_line(1);
        _unreg_2_line(1);
        _unreg_3_line(1);

        fprintf(Debug, "24 hour system reset was done\n\r");
        

        print_line();
        //_________________________________________________________________________________
    }
}

// updating RTC time using main pic command___________________________________0x70
void update_rtc_by_main_pic_cmd()
{
    int i;
    if (main_to_reset_array[1] == 0x70) //MP HF
    {
       fprintf(Debug, "\n\r");
        fprintf(Debug, "CMD for Updating Time Received  >> ");
        // for (i = 0; i <= 35; i++) {
        //     fprintf(Debug, "%X ", main_to_reset_array[i]);
        // }
        // print_line();

        // Acknowledging to the comand
        make_data_array_zero(reset_to_main_array, 36);
        reset_to_main_array[0] = 0xB0;
        reset_to_main_array[1] = 0x70;
        reset_to_main_array[35] = 0xB1;

        for (i = 0; i < 36; i++) {
            fputc(reset_to_main_array[i], MainPIC);
        }

        //Updating the RTC
        year = main_to_reset_array[2];
        month = main_to_reset_array[3];
        day = main_to_reset_array[4];
        hour = main_to_reset_array[5];
        minute = main_to_reset_array[6];
        second = main_to_reset_array[7];

        make_data_array_zero(main_to_reset_array, 36);
        mpi = 0;

        //!      //Printing New RTC value
        fprintf(Debug, "Updated New Time >> ");
        fprintf(Debug, "%u-", year);
        fprintf(Debug, "%u-", month);
        fprintf(Debug, "%u__", day);
        fprintf(Debug, "%u:", hour);
        fprintf(Debug, "%u:", minute);
        fprintf(Debug, "%u\n\r", second);
    }
}

void system_reset_by_cmd()
{
        fprintf(Debug, "\n\rsystem_reset_by_cmd: CMD for Resetting the satellite  >> ");
        make_data_array_zero(main_to_reset_array, 36);
        mpi = 0;
        mainpic_power(0);
        compic_power(0);
        _3v3_1_line(0);
        _5v0_2_line(0); //ADCS power line
        _5v0_line(0);   // SEL power line
        _12v0_line(0);
        _unreg_1_line(0);
        _unreg_2_line(0);
        _unreg_3_line(0);

        for (int i = 0; i < 5; i++) 
        {
            delay_ms(1000);
         }
        mainpic_power(1);
        compic_power(1);
         _3v3_1_line(1);
        _unreg_1_line(1);
        // _unreg_3_line(1);
        fprintf(Debug, "\n\rsystem_reset_by_cmd: MainPIC, COMPIC, 3V3#1 , UNREG#1 are ON >> \n\r");

       //_________________________________________________________________________________
}

//only for debugging
void houseKeepingDataDebug()
{

        _raw_power_adc_val = measure_raw_voltage();
        _3v3_1_current_adc_val = measure_3v3_1_current();     //float _3v3_1_current = 0 ;
        _5v0_2_current_adc_val = measure_5v0_2_current();     //float _5v0_2_current = 0 ;
        _5v0_current_adc_val = measure_5v0_current();         //float _5V0_current = 0 ;
        _12v0_current_adc_val = measure_12v0_current();       //float _12V0_current = 0 ;
        _unreg_1_current_adc_val = measure_unreg_1_current(); //float _UNREG_1_current = 0 ;
        _unreg_2_current_adc_val = measure_unreg_2_current(); //float _UNREG_2_current = 0 ;
        _unreg_3_current_adc_val = measure_unreg_3_current(); //float _UNREG_3_current = 0 ;
        
        reset_to_main_array[0] = 0xB0;

        reset_to_main_array[1] = 0xA0;
        reset_to_main_array[2] = year;
        reset_to_main_array[3] = month;
        reset_to_main_array[4] = day;
        reset_to_main_array[5] = hour;
        reset_to_main_array[6] = minute;
        reset_to_main_array[7] = second;
        reset_to_main_array[8] = (unsigned int8)((_raw_power_adc_val >> 8) & 0xFF);
        reset_to_main_array[9] = (unsigned int8)((_raw_power_adc_val)&0xFF);
        reset_to_main_array[10] = (unsigned int8)((_3v3_1_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[11] = (unsigned int8)((_3v3_1_current_adc_val)&0xFF);
        reset_to_main_array[12] = (unsigned int8)((_5v0_2_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[13] = (unsigned int8)((_5v0_2_current_adc_val)&0xFF);
        reset_to_main_array[14] = (unsigned int8)((_5v0_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[15] = (unsigned int8)((_5v0_current_adc_val)&0xFF);
        reset_to_main_array[16] = (unsigned int8)((_unreg_1_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[17] = (unsigned int8)((_unreg_1_current_adc_val)&0xFF);
        reset_to_main_array[18] = (unsigned int8)((_unreg_2_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[19] = (unsigned int8)((_unreg_2_current_adc_val)&0xFF);
        reset_to_main_array[20] = (unsigned int8)((_unreg_3_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[21] = (unsigned int8)((_unreg_3_current_adc_val)&0xFF);
        reset_to_main_array[22] = (unsigned int8)((_12v0_current_adc_val >> 8) & 0xFF);
        reset_to_main_array[23] = (unsigned int8)((_12v0_current_adc_val)&0xFF);
        reset_to_main_array[24] = powerline_status;
        reset_to_main_array[25] = mainpic_status;

        reset_to_main_array[35] = 0xB1;

        //fprintf(Debug, "\n\r",);
        fprintf(Debug, "20%02d/", year);
        fprintf(Debug, "%02d/", month);
        fprintf(Debug, "%02d ", day);
        fprintf(Debug, "%02d:", hour);
        fprintf(Debug, "%02d:", minute);
        fprintf(Debug, "%02d | ", second);



        unsigned int16 rawVoltage_hex = make16(reset_to_main_array[8], reset_to_main_array[9]);

        float Raw_voltage = rawVoltage_hex * 3.3 * 3 / 4096;
        fprintf(Debug, "Raw Voltage=%f | ", Raw_voltage);

        for (int i = 0; i < 36; i++) {
           fprintf(Debug, "%X ",  reset_to_main_array[i]);
        }
        fprintf(Debug, "\n\r");

}