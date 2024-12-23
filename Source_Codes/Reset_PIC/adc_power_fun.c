// adc_power_fun.c: power lines on/off functions, A/D converter measurement functions

///////////////////////////////////////////////////////////////////////////////
/////////////////////// Power Line Related Function ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

// D1 = 3V3V#1 buck enable
// D4 = 3V3#1 OCP enable

// D2 = 3V3#2 buck enable
// D5 = 3V3#2 OCP enable

// D3 = 5V0 buck enable
// D6 = 5V0 OCP enble

// B2 = 12V0 buck enable
// B1 = 12V0 OCP enble

// D7 = UNREG-1 enable
// D0 = UNREG-2 enable
// B0 = UNREG-3 enable

// F6 = Compic power enable
// F5 = Mainpic power enable

// F2 = WDT reset pin
// C3 = Raw power monitor enable

void _3v3_1_line(int1 i)
{
    if (i == 1) {
        output_high(PIN_D1);
        delay_ms(5);
        output_high(PIN_D4);
        bit_set(powerline_status, 7);
    } else {
        output_low(PIN_D4);
        delay_ms(5);
        output_low(PIN_D1);
        bit_clear(powerline_status, 7);
    }
    return;
}

void _3v3_2_line(int1 i)
{
    if (i == 1) {
        output_high(PIN_D2);
        delay_ms(5);
        output_high(PIN_D5);
        bit_set(powerline_status, 6);
    } else {
        output_low(PIN_D5);
        delay_ms(5);
        output_low(PIN_D2);
        bit_clear(powerline_status, 6);
    }
    return;
}

void _5v0_line(int1 i)
{
    if (i == 1) {
        output_high(PIN_D3);
        delay_ms(5);
        output_high(PIN_D6);
        bit_set(powerline_status, 5);
    } else {
        output_low(PIN_D6);
        delay_ms(5);
        output_low(PIN_D3);
        bit_clear(powerline_status, 5);
    }
    return;
}

void _unreg_1_line(int1 i)
{
    if (i == 1) {
        output_high(PIN_D7);
        bit_set(powerline_status, 4);
    }

    else {
        output_low(PIN_D7);
        bit_clear(powerline_status, 4);
    }
    return;
}

void _unreg_2_line(int1 i)
{
    if (i == 1) {
        output_high(PIN_D0);
        bit_set(powerline_status, 3);
    }

    else {
        output_low(PIN_D0);
        bit_clear(powerline_status, 3);
    }
    return;
}

void _unreg_3_line(int1 i)
{
    if (i == 1) {
        output_high(PIN_B0);
        bit_set(powerline_status, 2);
    }

    else {
        output_low(PIN_B0);
        bit_clear(powerline_status, 2);
    }
    return;
}

void _12v0_line(int1 i)
{
    if (i == 1) {
        output_high(PIN_B2);
        delay_ms(5);
        output_high(PIN_B1);
        bit_set(powerline_status, 1);
    } else {
        output_low(PIN_B1);
        delay_ms(5);
        output_low(PIN_B2);
        bit_clear(powerline_status, 1);
    }
    return;
}

void compic_power(int1 i)
{
    if (i == 1) {
        output_high(PIN_F6);
        bit_set(powerline_status, 0);
    }

    else {
        output_low(PIN_F6);
        bit_clear(powerline_status, 0);
    }
    return;
}

void mainpic_power(int1 i)
{
    if (i == 1) {
        output_high(PIN_F5);
        mainpic_status = 1;
    }

    else {
        output_low(PIN_F5);
        mainpic_status = 0;
    }
    return;
}

void mainpic_compic_power(int1 i)
{
    if (i == 1) {
        output_high(PIN_C4);
        delay_ms(5);
        output_high(PIN_F5);
        delay_ms(5);
        output_high(PIN_F6);
        mainpic_status = 1;
        bit_set(powerline_status, 0);
    }

    else {
        output_low(PIN_F5);
        delay_ms(5);
        output_low(PIN_F6);
        delay_ms(5);
        output_low(PIN_C4);
        mainpic_status = 0;
        bit_clear(powerline_status, 0);
    }
    return;
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////// ADC Related Function //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

unsigned int16 _raw_power_adc_val = 0;       //float _raw_voltage = 0 ;
unsigned int16 _3v3_1_current_adc_val = 0;   //float _3v3#1_current = 0 ;
unsigned int16 _3v3_2_current_adc_val = 0;   //float _3v3#2_current = 0 ;
unsigned int16 _5v0_current_adc_val = 0;     //float _5v0_current = 0 ;
unsigned int16 _12v0_current_adc_val = 0;    //float _12v0_current = 0 ;
unsigned int16 _unreg_1_current_adc_val = 0; //float _unreg_1_current = 0 ;
unsigned int16 _unreg_2_current_adc_val = 0; //float _unreg_2_current = 0 ;
unsigned int16 _unreg_3_current_adc_val = 0; //float _unreg_3_current = 0 ;

unsigned int16 measure_raw_voltage()
{
    output_high(PIN_C3); // raw voltage monitor on
    delay_us(100);

    set_adc_channel(2);
    delay_us(20);
    return read_adc();
}

unsigned int16 measure_3v3_1_current()
{
    set_adc_channel(1);
    delay_us(20);
    unsigned int16 adc = read_adc();
    return adc;
}

unsigned int16 measure_3v3_2_current()
{
    set_adc_channel(0);
    delay_us(20);
    return read_adc();
}

unsigned int16 measure_5v0_current()
{
    set_adc_channel(4);
    delay_us(20);
    return read_adc();
}

unsigned int16 measure_unreg_1_current()
{
    set_adc_channel(6);
    delay_us(20);
    return read_adc();
}

unsigned int16 measure_unreg_2_current()
{
    set_adc_channel(9);
    delay_us(20);
    return read_adc();
}

unsigned int16 measure_unreg_3_current()
{
    set_adc_channel(3);
    delay_us(20);
    return read_adc();
}

unsigned int16 measure_12v0_current()
{
    set_adc_channel(5);
    delay_us(20);
    return read_adc();
}
