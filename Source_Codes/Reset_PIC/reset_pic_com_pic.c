void compic_periodic_cmd()
{
    if (com_to_reset_array[1] == 0xDE && com_to_reset_array[2] == 0xAD) {
        int i;
        print_line();
        printf_debug("Periodic Comunication Data Array From ComPIC >> ");
        for (i = 0; i < sizeof(com_to_reset_array); i++) {
            printf_debug("%X ", com_to_reset_array[i]);
        }
        print_line();
        print_line();

        make_data_array_zero(com_to_reset_array, sizeof(com_to_reset_array));

        cpi = 0;
        com_pic_counter = 0;
    }
}

void check_com_pic_health()
{

    if (com_pic_counter >= 1800) // wait 1800/60 = 30 min before reseting com pic // Normal Mode
    {
        printf_debug("Com PIC hang up Reset \n\r");
        com_pic_counter = 0;
        com_pic_rst_counter++;

        compic_power(0);
        delay_ms(5000); // Delay
        compic_power(1);
        // delay_ms(500);
    }
}
