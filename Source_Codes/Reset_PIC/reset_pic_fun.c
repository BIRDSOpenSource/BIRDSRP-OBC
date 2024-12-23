// reset_pic_fun.c: uart configuration, rtc functions

// UART PORT definition
#pin_select TX3 = PIN_E1
#pin_select RX3 = PIN_E0
#use rs232(uart3, baud = 19200, parity = N, bits = 8, stream = MainPIC, errors) // Hardware UART line between ResetPIC and MainPIC

#pin_select TX1 = PIN_C6
#pin_select RX1 = PIN_C7
#use rs232(uart1, baud = 19200, parity = N, bits = 8, stream = StartPIC, errors) // Hardware UART line between ResetPIC and StartPIC

#pin_select TX2 = PIN_G1
#pin_select RX2 = PIN_G0
#use rs232(uart2, baud = 19200, parity = N, bits = 8, stream = ComPIC, errors) // Hardware UART line between ResetPIC and ComPIC

int mpi = 0;
int cpi = 0;
int spi = 0;

// Use this to enable debug:
// #use rs232(baud = 57600, parity = N, xmit = PIN_B7, bits = 8, stream = debug, errors) // Debuging software UART line     // B7, B6
// #define printf_debug(...) fprintf(Debug, __VA_ARGS__)
// Use this instead do disable debug:
#define printf_debug(...)

void make_data_array_zero(unsigned int8 array[], int array_size)
{
    for (int i = 0; i < array_size; i++) {
        array[i] = 0;
    }
}

// Reset external watch dog timer///___________________________________________________________________________________________________________________________
// void restart_ext_wdt()
// {
//    output_high(PIN_F2);
//    delay_us(100);
//    output_low(PIN_F2);
//    return;
// }

//!// UART Related fucntions____________________________________________________________________________________________________________________________
void print_line()
{
    printf_debug("\r\n");
    return;
}

// void print_main_to_reset_array()
// {

//    if( (main_to_reset_array[0] == 0xB0) && (main_to_reset_array[35] == 0xB1) )
//    {
//       printf_debug("Data array from MainPIC >> ");

//       for(int i = 0; i<=35; i++)
//       {
//          printf_debug("%X ",main_to_reset_array[i]);
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
        printf_debug("Time ");
        printf_debug("%02d", day);
        printf_debug("/%02d", month);
        printf_debug("/%02d", year);
        printf_debug("--%02d", hour);
        printf_debug(":%02d", minute);
        printf_debug(":%02d", second); // 20-01-01__05:20:22

        printf_debug(", %Ld-", main_pic_rst_counter);
        printf_debug("%03Ld", main_pic_counter);
        printf_debug(", %Ld-", com_pic_rst_counter);
        printf_debug("%03Ld", com_pic_counter);

        printf_debug(", %Lx", measure_raw_voltage());
        printf_debug(", %Lx", measure_3v3_1_current());
        printf_debug(", %Lx", measure_3v3_2_current());
        printf_debug(", %Lx", measure_5v0_current());
        printf_debug(", %Lx", measure_unreg_1_current());
        printf_debug(", %Lx", measure_unreg_2_current());
        printf_debug(", %Lx", measure_unreg_3_current());
        printf_debug(", %Lx", measure_12v0_current());
        printf_debug(", %Lx\n\r", powerline_status);
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

    if ((day == 32) && (month == 12)) // december
    {
        day = 1;
        month = 1;
        year++;
    }
}
