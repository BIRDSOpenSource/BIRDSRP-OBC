#include <16F1789.h>
#fuses NOWDT, MCLR
#device ADC = 12
#use delay(crystal = 16Mhz, clock = 16MHz)
#include <reset_pic_reg.h>
unsigned int16 main_pic_counter = 0;
// unsigned int16 com_pic_counter  = 0;

// unsigned int16 com_pic_rst_counter  = 0;
unsigned int16 main_pic_rst_counter = 0;
char powerline_status = 0;
unsigned int8 mainpic_status = 0;
unsigned int8 uhf_on_flag = 0;
unsigned int8 rp_mlc = 0;

// UART Data buffers
unsigned int8 main_to_reset_array[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned int8 reset_to_main_array[36] = { 0xB0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xB1 };

#include <adc_power_fun.c>
#include <reset_pic_fun.c>
#include <reset_pic_main_pic.c>


#PRIORITY INT_TIMER1, INT_RDA // setting interupt priority

unsigned int8 main_loop_finished = 1; // flag to check if the main loop finished

#INT_TIMER1
void timer1_isr() // This timer manages ResetPIC RTC and WDT
{
    set_timer1(0x8000);
    if (second == 0) {                 // Every minute
        if (main_loop_finished == 1) { // Main loop is finishing correctly
            main_loop_finished = 0;    // Test if main loop will finish in next cycle
        } else {
            output_low(PIN_B2); // ensure WDT is not being fed
            while (true) { };   // wait for external WDT reset
        }
    }
    update_rtc();
    output_high(PIN_B2); // WDT disable (clear)
    main_pic_counter++;
    // com_pic_counter ++;
    output_low(PIN_B2);
}

#INT_RDA
void serial_isr() // MAIN PIC uart interupt loop
{
    if (kbhit(MainPIC)) {
        main_to_reset_array[mpi] = fgetc(MainPIC);
        mpi++;
    }
}

void settings()
{
    fprintf(Debug, "Booting the reset PIC\n\r");
    enable_interrupts(INT_TIMER1); // enable timer 1 interupt
    enable_interrupts(INT_RDA);    // enable uart interupt
    enable_interrupts(GLOBAL);     // start interupt processing

    setup_timer_1(T1_EXTERNAL | T1_DIV_BY_1); // setting timer 1 divider
    T1OSCEN = 1;                              // enabling timer1
    set_timer1(32768);                        // setting timer 1 value

    setup_adc(ADC_CLOCK_INTERNAL);
    setup_adc_ports(sAN0 | sAN1 | sAN2 | sAN3 | sAN4 | sAN5 | sAN6 | sAN7); // setting all analog ports

    mainpic_power(1);
    compic_power(1);
    _3v3_1_line(1);
    _5v0_2_line(1);
    _5v0_line(0);
    _12v0_line(1);//2024.08.04
    _unreg_1_line(0);
    _unreg_2_line(0);
    _unreg_3_line(1);
}

void main()
{
    settings();

    while (true) {
        print_rtc(); //

        // To protect the main PIC UART increment overflow
        if (rp_mlc >= 50) // 50*100ms = 5 seconds
        {
            mpi = 0;
            rp_mlc = 0;
            houseKeepingDataDebug();//only for debugging
        }
        rp_mlc++;

        // 30 Mins UHF ON CMD____________________________________________________________//
        if (uhf_on_flag == 0) {
            if (minute >= 31) {
                _unreg_1_line(1); // turn on UHF power
                _unreg_2_line(1); // turn on S-band power
                uhf_on_flag = 1;
            }
        }

        // From MainPIC CMD________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[35] == 0xB1)) {
            update_rtc_by_main_pic_cmd();   // 0x70
            respond_to_mainpic_90sec_cmd(); // 0xA0
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xE1)) {
            _3v3_1_line(1); // turn on 3V3#1 line
            fprintf(Debug, "3V3#1 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xE0)) {
            _3v3_1_line(0); // turn off 3V3#1 line
            fprintf(Debug, "3V3#1 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xE2)) {
            _5v0_2_line(1); // turn on 5V0#2 line
            fprintf(Debug, "5V0#2 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xE3)) {
            _5v0_2_line(0); // turn off 5V0#2 line
            fprintf(Debug, "5V0#2 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xA5)) {
            _5v0_line(1); // turn on 5V line
            fprintf(Debug, "5V0#0 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xA6)) {
            _5v0_line(0); // turn off 5V line
            fprintf(Debug, "5V0#0 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB1)) {
            _unreg_1_line(1); // turn on Unreg#1 line
            fprintf(Debug, "Unreg#1 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB0)) {
            _unreg_1_line(0); // turn off Unreg#1 line
            fprintf(Debug, "Unreg#1 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB2)) {
            _unreg_2_line(1); // turn on Unreg#2 line
            fprintf(Debug, "Unreg#2 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB3)) {
            _unreg_2_line(0); // turn off Unreg#2 line
            fprintf(Debug, "Unreg#2 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB4)) {
            _unreg_3_line(1); // turn on Unreg#3 line
            fprintf(Debug, "Unreg#3 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB5)) {
            _unreg_3_line(0); // turn off Unreg#3 line
            fprintf(Debug, "Unreg#3 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xC1)) {
            _12v0_line(1); // turn on 12V line
            fprintf(Debug, "12V line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xC0)) {
            _12v0_line(0); // turn off 12V line
            fprintf(Debug, "12V line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xFA)) {
            while (true) { }; // simulate main loop crash
        }
        //2024.01.20 NEW CMD: Reset satellite by CMD__________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xA4) && (main_to_reset_array[35] == 0xB1)) {
            fprintf(Debug, "System reset command recieved. \n\r");
            system_reset_by_cmd();
        }
       
        // To MainPIC CMD__________________________________________________________________________
        warn_to_main_pic_before_24h_reset(); // 0xA2
        system_reset_24h();                  // 0xA3
        //_________________________________________________________________________________________

        check_main_pic_health(); // MPMF, MPC
        // check_com_pic_health()                           ;                       // CPC

        delay_ms(100);
        main_loop_finished = 1;
    }
    

}
