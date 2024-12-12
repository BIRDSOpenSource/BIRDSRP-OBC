//___character_space_function___///////////////////////////////////
void ch_space()
{
    LATD3 = 0;
    delay_ms(180);
    return;
}

//___Word_space_function___////////////////////////////////////////
void word_space()
{
    LATD3 = 0;
    delay_ms(420);
    return;
}

//___dit_function___///////////////////////////////////////////////
void dit()
{
    LATD3 = 1;
    delay_ms(60);
    LATD3 = 0;
    delay_ms(60);
    return;
}

//___dit_function___///////////////////////////////////////////////
void dah()
{
    LATD3 = 1;
    delay_ms(180);
    LATD3 = 0;
    delay_ms(60);
    return;
}

///////////////////////___character-A___//////////////////////////
void cw_a()
{
    dit();
    dah();
    ch_space();
}

///////////////////////___character-B___//////////////////////////
void cw_b()
{
    dah();
    dit();
    dit();
    dit();
    ch_space();
}

///////////////////////___character-C___//////////////////////////
void cw_c()
{
    dah();
    dit();
    dah();
    dit();
    ch_space();
}

///////////////////////___character-D___//////////////////////////
void cw_d()
{
    dah();
    dit();
    dit();
    ch_space();
}

///////////////////////___character-E___//////////////////////////
void cw_e()
{
    dit();
    ch_space();
}

///////////////////////___character-F___//////////////////////////
void cw_f()
{
    dit();
    dit();
    dah();
    dit();
    ch_space();
}

///////////////////////___character-G___//////////////////////////
void cw_g()
{
    dah();
    dah();
    dit();
    ch_space();
}

///////////////////////___character-H___//////////////////////////
void cw_h()
{
    dit();
    dit();
    dit();
    dit();
    ch_space();
}

///////////////////////___character-I___//////////////////////////
void cw_i()
{
    dit();
    dit();
    ch_space();
}

///////////////////////___character-J//////////////////////////
void cw_j()
{
    dit();
    dah();
    dah();
    dah();
    ch_space();
}

///////////////////////___character-K___//////////////////////////
void cw_k()
{
    dah();
    dit();
    dah();
    ch_space();
}

///////////////////////___character-L___//////////////////////////
void cw_l()
{
    dit();
    dah();
    dit();
    dit();
    ch_space();
}

///////////////////////___character-M___//////////////////////////
void cw_m()
{
    dah();
    dah();
    ch_space();
}

///////////////////////___character-N___//////////////////////////
void cw_n()
{
    dah();
    dit();
    ch_space();
}

///////////////////////___character-O___//////////////////////////
void cw_o()
{
    dah();
    dah();
    dah();
    ch_space();
}

///////////////////////___character-P___//////////////////////////
void cw_p()
{
    dit();
    dah();
    dah();
    dit();
    ch_space();
}

///////////////////////___character-Q___//////////////////////////
void cw_q()
{
    dah();
    dah();
    dit();
    dah();
    ch_space();
}

///////////////////////___character-R___//////////////////////////
void cw_r()
{
    dit();
    dah();
    dit();
    ch_space();
}

///////////////////////___character-S___//////////////////////////
void cw_s()
{
    dit();
    dit();
    dit();
    ch_space();
}

///////////////////////___character-T___//////////////////////////
void cw_t()
{
    dah();
    ch_space();
}

///////////////////////___character-U___//////////////////////////
void cw_u()
{
    dit();
    dit();
    dah();
    ch_space();
}

///////////////////////___character-V___//////////////////////////
void cw_v()
{
    dit();
    dit();
    dit();
    dah();
    ch_space();
}

///////////////////////___character-W___//////////////////////////
void cw_w()
{
    dit();
    dah();
    dah();
    ch_space();
}

///////////////////////___character-X___//////////////////////////
void cw_x()
{
    dah();
    dit();
    dit();
    dah();
    ch_space();
}

///////////////////////___character-Y___//////////////////////////
void cw_y()
{
    dah();
    dit();
    dah();
    dah();
    ch_space();
}

///////////////////////___character-Z___//////////////////////////
void cw_z()
{
    dah();
    dah();
    dit();
    dit();
    ch_space();
}

///////////////////////___Number-1___/////////////////////////////
void cw_1()
{
    dit();
    dah();
    dah();
    dah();
    dah();
    ch_space();
}

///////////////////////___Number-2___/////////////////////////////
void cw_2()
{
    dit();
    dit();
    dah();
    dah();
    dah();
    ch_space();
}

///////////////////////___Number-3___/////////////////////////////
void cw_3()
{
    dit();
    dit();
    dit();
    dah();
    dah();
    ch_space();
}

///////////////////////___Number-4___/////////////////////////////
void cw_4()
{
    dit();
    dit();
    dit();
    dit();
    dah();
    ch_space();
}

///////////////////////___Number-5___/////////////////////////////
void cw_5()
{
    dit();
    dit();
    dit();
    dit();
    dit();
    ch_space();
}

///////////////////////___Number-6___/////////////////////////////
void cw_6()
{
    dah();
    dit();
    dit();
    dit();
    dit();
    ch_space();
}

///////////////////////___Number-7___/////////////////////////////
void cw_7()
{
    dah();
    dah();
    dit();
    dit();
    dit();
    ch_space();
}

///////////////////////___Number-8___/////////////////////////////
void cw_8()
{
    dah();
    dah();
    dah();
    dit();
    dit();
    ch_space();
}

///////////////////////___Number-9___/////////////////////////////
void cw_9()
{
    dah();
    dah();
    dah();
    dah();
    dit();
    ch_space();
}

///////////////////////___Number-9___/////////////////////////////
void cw_0()
{
    dah();
    dah();
    dah();
    dah();
    dah();
    ch_space();
}

void cw_leopard()
{
    cw_l();
    cw_e();
    cw_o();
    cw_p();
    cw_a();
    cw_r();
    cw_d();
    return;
}

void call_sign()
{
    cw_k();
    cw_y();
    cw_u();
    cw_t();
    cw_e();
    cw_c();
    cw_h();
    return;
}

void cw_letter(unsigned int8 cwl)
{
    //Hexa decimal values
    if (cwl == 0x00 || cwl == 0x30)
        cw_0();
    else if (cwl == 0x01 || cwl == 0x31)
        cw_1();
    else if (cwl == 0x02 || cwl == 0x32)
        cw_2();
    else if (cwl == 0x03 || cwl == 0x33)
        cw_3();
    else if (cwl == 0x04 || cwl == 0x34)
        cw_4();
    else if (cwl == 0x05 || cwl == 0x35)
        cw_5();
    else if (cwl == 0x06 || cwl == 0x36)
        cw_6();
    else if (cwl == 0x07 || cwl == 0x37)
        cw_7();
    else if (cwl == 0x08 || cwl == 0x38)
        cw_8();
    else if (cwl == 0x09 || cwl == 0x39)
        cw_9();
    else if (cwl == 0x0A || cwl == 0x61)
        cw_a();
    else if (cwl == 0x0B || cwl == 0x62)
        cw_b();
    else if (cwl == 0x0C || cwl == 0x63)
        cw_c();
    else if (cwl == 0x0D || cwl == 0x64)
        cw_d();
    else if (cwl == 0x0E || cwl == 0x65)
        cw_e();
    else if (cwl == 0x0F || cwl == 0x66)
        cw_f();
    //-------------------------

    //normal ascii
    else if (cwl == 0x67)
        cw_g();
    else if (cwl == 0x68)
        cw_h();
    else if (cwl == 0x69)
        cw_i();
    else if (cwl == 0x6a)
        cw_j();
    else if (cwl == 0x6b)
        cw_k();
    else if (cwl == 0x6c)
        cw_l();
    else if (cwl == 0x6d)
        cw_m();
    else if (cwl == 0x6e)
        cw_n();
    else if (cwl == 0x6f)
        cw_o();
    else if (cwl == 0x70)
        cw_p();
    else if (cwl == 0x71)
        cw_q();
    else if (cwl == 0x72)
        cw_r();
    else if (cwl == 0x73)
        cw_s();
    else if (cwl == 0x74)
        cw_t();
    else if (cwl == 0x75)
        cw_u();
    else if (cwl == 0x76)
        cw_v();
    else if (cwl == 0x77)
        cw_w();
    else if (cwl == 0x78)
        cw_x();
    else if (cwl == 0x79)
        cw_y();
    else if (cwl == 0x7a)
        cw_z();

    return;
}

//                                     [BIRDS3]__[CALL SIGN]__[CW_MSN]__
void cw_pckt()
{
    unsigned int8 hk[14] = {0};
    delay_ms(1000);
    call_sign();

    delay_ms(500); // KITSUNE part the CW

    cw_leopard();
    delay_ms(500);

    // cw_msn();
    // delay_ms(500);

    if (main_to_com[1] == 0x50) {
        hk[0] = (main_to_com[2] & 0xF0) >> 4;
        hk[1] = (main_to_com[2]) & 0x0F;

        hk[2] = (main_to_com[3] & 0xF0) >> 4;
        hk[3] = (main_to_com[3]) & 0x0F;

        hk[4] = (main_to_com[4] & 0xF0) >> 4;
        hk[5] = (main_to_com[4]) & 0x0F;

        hk[6] = (main_to_com[5] & 0xF0) >> 4;
        hk[7] = (main_to_com[5]) & 0x0F;

        hk[8] = (main_to_com[6] & 0xF0) >> 4;
        hk[9] = (main_to_com[6]) & 0x0F;

        hk[10] = (main_to_com[7] & 0xF0) >> 4;
        hk[11] = (main_to_com[7]) & 0x0F;
    }

    cw_letter(hk[0]);
    cw_letter(hk[1]);

    cw_letter(hk[2]);
    cw_letter(hk[3]);

    cw_letter(hk[4]);
    cw_letter(hk[5]);

    cw_letter(hk[6]);
    cw_letter(hk[7]);

    cw_letter(hk[8]);
    cw_letter(hk[9]);

    cw_letter(hk[10]);
    cw_letter(hk[11]);

    return;
}

//_________________________________________________________________________________________________
