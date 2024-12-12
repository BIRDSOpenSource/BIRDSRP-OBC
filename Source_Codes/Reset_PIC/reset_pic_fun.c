// reset_pic_fun.c: uart configuration, rtc functions

// UART PORT definition
#use rs232(uart1, baud = 19200, parity = N, bits = 8, stream = MainPIC, errors) // Hardware UART line between ResetPIC and MainPIC
//#use rs232(baud=19200, parity=N, xmit=PIN_B1, rcv=PIN_B0, bits=8, stream = ComPIC, errors)        // Attached interupt software UART line between ResetPIC and ComPIC
#use rs232(baud = 57600, parity = N, xmit = PIN_B7, bits = 8, stream = debug, errors) // Debuging software UART line     // B7, B6
int mpi = 0;

void make_data_array_zero(unsigned int8 array[], int array_size)
{
    for (int i = 0; i < array_size; i++) {
        array[i] = 0;
    }
}

// Reset external watch dog timer///___________________________________________________________________________________________________________________________
// void restart_ext_wdt()
// {
//    output_high(PIN_B2);
//    delay_us(100);
//    output_low(PIN_B2);
//    return;
// }

//!// UART Related fucntions____________________________________________________________________________________________________________________________
void print_line()
{
    // fputc(0x20, debug);
    // fputc(0x0A, debug);
    // fputc(0x0D, debug);
    return;
}

// void print_main_to_reset_array()
// {

//    if( (main_to_reset_array[0] == 0xB0) && (main_to_reset_array[35] == 0xB1) )
//    {
//       fprintf(Debug,"Data array from MainPIC >> ");

//       for(int i = 0; i<=35; i++)
//       {
//          fprintf(Debug,"%X ",main_to_reset_array[i]);
//       }
//       print_line();

//       make_data_array_zero( main_to_reset_array, 36);
//    }
//    return;
// }

// RTC Related fucntions___________________________________________________________________________________________________________________________
unsigned int8 second = 1;
unsigned int8 minute = 0;
unsigned int8 hour = 0;
unsigned int8 day = 1;
unsigned int8 month = 1;
unsigned int8 year = 21; // 20/08/21,   23:59:00

int previous_second;

void print_rtc()
{
    if (previous_second != second) {
        // fprintf(Debug, "Time ");
        // fprintf(Debug, "%02d", day);
        // fprintf(Debug, "/%02d", month);
        // fprintf(Debug, "/%02d", year);
        // fprintf(Debug, "--%02d", hour);
        // fprintf(Debug, ":%02d", minute);
        // fprintf(Debug, ":%02d", second); // 20-01-01__05:20:22

        // fprintf(Debug, " %Ld-", main_pic_rst_counter);
        // fprintf(Debug, "%03Ld,", main_pic_counter);
        //fprintf(Debug," %Ld-", com_pic_rst_counter);
        //fprintf(Debug,"%03Ld\n\r", com_pic_counter);

        // fprintf(Debug, "%Lx ,", measure_raw_voltage());
        // fprintf(Debug, "%Lx ,", measure_3v3_1_current());
        // fprintf(Debug, "%Lx ,", measure_5v0_2_current());
        // fprintf(Debug, "%Lx ,", measure_5v0_current());
        // fprintf(Debug, "%Lx ,", measure_unreg_1_current());
        // fprintf(Debug, "%Lx ,", measure_unreg_2_current());
        // fprintf(Debug, "%Lx ,", measure_unreg_3_current());
        // fprintf(Debug, "%Lx ,", measure_12v0_current());
        // fprintf(Debug, "%Lx \n\r", powerline_status);
    }
    previous_second = second;
}

void update_rtc()
{
    if (second < 59) // updating seconds
    {
        second++;
    }

    else {
        second = 0;
        minute++;
    }

    if (minute == 60) // updating minutes
    {
        minute = 0;
        hour++;
    }

    if (hour == 24) // updating day
    {
        hour = 0;
        day++;
    }

    if ((day == 31) && (month == 4 || month == 6 || month == 9 || month == 11)) // 30 days months
    {
        day = 1;
        month++;
    }

    if ((day == 32) && (month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10)) // 31 days months
    {
        day = 1;
        month++;
    }

    if ((day == 29) && (month == 2)) // february
    {
        day = 1;
        month++;
    }

    if ((day == 32) && (month == 12)) //december
    {
        day = 1;
        month = 1;
        year++;
    }
}
