// This is the function for initial settings for compic

void settings()
{
    output_high(PIN_E2);//CFM CS pin_output
    output_high(PIN_B3);//SFM CS pin_output

    output_low(PIN_D3);//TX on Pin_output
    output_low(PIN_D2);//RX on Pin_output
    output_low(PIN_D1);//CW ky pin_output

    setup_adc(ADC_CLOCK_INTERNAL);
    setup_adc_ports(san4 | san6); //channel 4 and 6 is enabled for ADC

    enable_interrupts(global);
    enable_interrupts(INT_RDA);
}

//_____________________TRX Controlling_________________________________________
void tx_on()
{
    output_high(PIN_D3);// TX on pin (1)
    delay_ms(100);
    output_low(PIN_D2);// 0
    delay_ms(100);
    output_low(PIN_D1);// 0
    delay_ms(100);
    return;
}

void cw_on()
{
    output_low(PIN_D3);// 0
    delay_ms(100);
    output_high(PIN_D2);// CW on pin (1)
    delay_ms(100);
    output_low(PIN_D1);// 0
    delay_ms(100);
    return;
}

void rx_on()
{
    output_low(PIN_D3);// 0
    delay_ms(100);
    output_low(PIN_D2);// 0
    delay_ms(100);
    output_low(PIN_D1);// 0
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
    set_adc_channel(6);
    delay_us(10);
    return (read_adc());
}
