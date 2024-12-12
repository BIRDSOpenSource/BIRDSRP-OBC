// This is the function for initial settings for compic

void settings()
{
    TRISD5 = 0;
    LATD5 = 1; //CFM CS pin_output
    TRISC2 = 0;
    LATC2 = 1; //SFM CS pin_output

    TRISD1 = 0;
    LATD1 = 0; //TX on Pin_output
    TRISD2 = 0;
    LATD2 = 0; //RX on Pin_output
    TRISD3 = 0;
    LATD3 = 0; //CW ky pin_output

    setup_adc(ADC_CLOCK_INTERNAL);
    setup_adc_ports(san4 | san5); //chanal 4-5 is enable for ADC

    enable_interrupts(global);
    enable_interrupts(INT_RDA);
}

//_____________________TRX Controlling_________________________________________
void tx_on()
{
    LATD1 = 1; // TX on pin (1)
    delay_ms(100);
    LATD2 = 0; // 0
    delay_ms(100);
    LATD3 = 0; // 0
    delay_ms(100);
    return;
}

void cw_on()
{
    LATD1 = 0; // 0
    delay_ms(100);
    LATD2 = 1; // CW on pin (1)
    delay_ms(100);
    LATD3 = 0; // 0
    delay_ms(100);
    return;
}

void rx_on()
{
    LATD1 = 0; // 0
    delay_ms(100);
    LATD2 = 0; // 0
    delay_ms(100);
    LATD3 = 0; // 0
    delay_ms(100);

    return;
}
//---------------------------------------------------------------------

//_________________Received signal strength indication_________________________
unsigned int16 trx_rssi_read()
{
    set_adc_channel(4);
    delay_us(10);
    return (read_adc());
}

//_________________TEMP funtion________________________________________________
unsigned int16 trx_temp_read()
{
    set_adc_channel(5);
    delay_us(10);
    return (read_adc());
}
