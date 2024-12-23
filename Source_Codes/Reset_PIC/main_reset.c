#include <18F67J94.h>
#fuses NOWDT
#device ADC = 12
#use delay(crystal = 16Mhz, clock = 16MHz)

unsigned int16 main_pic_counter = 0;
unsigned int16 main_pic_rst_counter = 0;
unsigned int16 com_pic_counter = 0;
unsigned int16 com_pic_rst_counter = 0;
char powerline_status = 0;
unsigned int8 mainpic_status = 0;
unsigned int8 uhf_on_flag = 0;
unsigned int8 rp_mlc = 0;

// UART Data buffers
unsigned int8 com_to_reset_array[36] = { 0 };
unsigned int8 main_to_reset_array[36] = { 0 };
unsigned int8 reset_to_main_array[36] = { 0 };
unsigned int8 reset_to_start_array[36] = { 0 };
unsigned int8 start_to_reset_array[36] = { 0 };

#include <adc_power_fun.c>
#include <reset_pic_fun.c>
#include <reset_pic_main_pic.c>
#include <reset_pic_com_pic.c>
#include <reset_pic_start_pic.c>

#PRIORITY INT_TIMER1, INT_RDA3 // setting interupt priority

unsigned int8 main_loop_finished = 1; // flag to check if the main loop finished

#INT_TIMER1
void timer1_isr() // This timer manages ResetPIC RTC and WDT
{
    set_timer1(0x8000);
    if (second == 0) {                 // Every minute
        if (main_loop_finished == 1) { // Main loop is finishing correctly
            main_loop_finished = 0;    // Test if main loop will finish in next cycle
        } else {
            output_low(PIN_F2); // ensure WDT is not being fed
            while (true) { };   // wait for external WDT reset
        }
    }
    update_rtc();
    output_high(PIN_F2); // WDT disable (clear)
    main_pic_counter++;
    com_pic_counter++;
    output_low(PIN_F2);
}

#INT_RDA3
void serial_isr() // MAIN PIC uart interupt loop
{
    if (kbhit(MainPIC)) {
        main_to_reset_array[mpi] = fgetc(MainPIC);
        mpi++;
    }
}

#INT_RDA2
void serial_isr_2() // COM PIC uart interupt loop
{
    if (kbhit(ComPIC)) {
        com_to_reset_array[cpi] = fgetc(ComPIC);
        cpi++;
    }
}

#INT_RDA
void serial_isr_3() // Start PIC uart interupt loop
{
    if (kbhit(StartPIC)) {
        start_to_reset_array[spi] = fgetc(StartPIC);
        spi++;
    }
}

void settings()
{
    printf_debug("Booting the reset PIC\r\n");
    printf_debug("Compiled on "__DATE__
                 " "__TIME__
                 "\r\n");

    enable_interrupts(INT_TIMER1); // enable timer 1 interrupt
    enable_interrupts(INT_RDA);    // enable uart1 (start) interrupt
    enable_interrupts(INT_RDA2);   // enable uart2 (com) interrupt
    enable_interrupts(INT_RDA3);   // enable uart3 (main) interrupt
    enable_interrupts(GLOBAL);     // start interrupt processing

    // Timer 1: starts from 32768, overflows on (65535+1), external clock 32.768 kHz, period 1s
    setup_timer_1(T1_EXTERNAL | T1_DIV_BY_1 | T1_ENABLE_SOSC); // setting timer 1 divider
    set_timer1(32768);                                         // setting timer 1 value

    setup_adc(ADC_CLOCK_INTERNAL);
    setup_adc_ports(sAN0 | sAN1 | sAN2 | sAN3 | sAN4 | sAN5 | sAN6 | sAN9); // setting analog ports in use

    // !! Default on/off settings should also be changed in 24h reset function !!
    mainpic_compic_power(1);
    _3v3_1_line(1);
    _3v3_2_line(1);
    _5v0_line(1);
    _12v0_line(1);
    _unreg_1_line(0);
    _unreg_2_line(0);
    _unreg_3_line(1);
}

void main()
{
    settings();

    while (true) {
        print_rtc();

        // Make 24h reset happen earlier for debug purposes in first days
        // if(day <= 5) {
        //     if(hour == 0 && minute == 2){
        //         hour = 23;
        //         minute = 58;
        //         second = 59;
        //     }
        // }

        // To protect the main, com and start PIC UART increment overflow
        if (rp_mlc >= 50) // 50*100ms = 5 seconds
        {
            mpi = 0;
            cpi = 0;
            spi = 0;
            rp_mlc = 0;
        }
        rp_mlc++;

        // 31 Mins UHF ON CMD____________________________________________________________//
        if (uhf_on_flag == 0) {
            if (minute == 31) {
                _unreg_1_line(1); // turn on UHF power
                _unreg_2_line(1); // turn on Cband power
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
            printf_debug("3V3#1 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xE0)) {
            _3v3_1_line(0); // turn off 3V3#1 line
            printf_debug("3V3#1 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xE2)) {
            _3v3_2_line(1); // turn on 3V3#2 line
            printf_debug("3V3#2 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xE3)) {
            _3v3_2_line(0); // turn off 3V3#2 line
            printf_debug("3V3#2 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xA5)) {
            _5v0_line(1); // turn on 5V line
            printf_debug("5V line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xA6)) {
            _5v0_line(0); // turn off 5V line
            printf_debug("5V line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB1)) {
            _unreg_1_line(1); // turn on Unreg#1 line
            printf_debug("Unreg#1 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB0)) {
            _unreg_1_line(0); // turn off Unreg#1 line
            printf_debug("Unreg#1 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB2)) {
            _unreg_2_line(1); // turn on Unreg#2 line
            printf_debug("Unreg#2 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB3)) {
            _unreg_2_line(0); // turn off Unreg#2 line
            printf_debug("Unreg#2 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB4)) {
            _unreg_3_line(1); // turn on Unreg#3 line
            printf_debug("Unreg#3 line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xB5)) {
            _unreg_3_line(0); // turn off Unreg#3 line
            printf_debug("Unreg#3 line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________

        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xC1)) {
            _12v0_line(1); // turn on 12V line
            printf_debug("12V line turned ON \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xC0)) {
            _12v0_line(0); // turn off 12V line
            printf_debug("12V line turned OFF \n\r");
            make_data_array_zero(main_to_reset_array, 36);
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xFA)) {
            while (true) { }; // simulate main loop crash
        }
        //_________________________________________________________________________________________
        if ((main_to_reset_array[0] == 0xB0) && (main_to_reset_array[1] == 0xFB)) {
            printf_debug("Reset Main/Com PIC through DC/DC converter switch.\r\n");
            mainpic_compic_power(0);
            for (int i = 0; i < 5; i++) {
                delay_ms(1000);
                printf_debug("Waiting to turn on Main/Com DC/DC converter - %02d seconds\n\r", i + 1);
            }
            mainpic_compic_power(1);
            make_data_array_zero(main_to_reset_array, 36);
        }

        // From ComPIC CMD_________________________________________________________________________
        if ((com_to_reset_array[0] == 0xC0) && (com_to_reset_array[35] == 0xC1)) {
            compic_periodic_cmd();
        }
        //_________________________________________________________________________________________
        if ((com_to_reset_array[0] == 0xC0) && (com_to_reset_array[1] == 0xFB)) {
            printf_debug("Reset Main/Com PIC through DC/DC converter switch.\r\n");
            mainpic_compic_power(0);
            for (int i = 0; i < 5; i++) {
                delay_ms(1000);
                printf_debug("Waiting to turn on Main/Com DC/DC converter - %02d seconds\n\r", i + 1);
            }
            mainpic_compic_power(1);
            make_data_array_zero(com_to_reset_array, 36);
        }

        // From StartPIC CMD_________________________________________________________________________
        if ((start_to_reset_array[0] == 0x50) && (start_to_reset_array[35] == 0x51)) {
            report_health_to_start();
            make_data_array_zero(start_to_reset_array, sizeof(start_to_reset_array));
        }

        // To MainPIC CMD__________________________________________________________________________
        warn_to_main_pic_before_24h_reset();  // 0xA2
        system_reset_24h();                   // 0xA3
        //_________________________________________________________________________________________

        check_main_pic_health(); // MPMF, MPC
        check_com_pic_health();  // CPC

        delay_ms(100);
        main_loop_finished = 1;
    }
}
