// this section handle the RTC related functions_______________________________

// this function set the RTC time______________________________________________
void Set_RTC( char y, char mo, char d, char h, char mi, char s )
{
   // Validate inputs
   if (mo < 1 || mo > 12 || d < 1 || d > 31 || h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59)
         return;
   year   = y  ;
   month  = mo ;
   day    = d  ;
   hour   = h  ;
   minute = mi ;
   second = s  ;
}

int previous_second ;

// this function will update the time when timer 1 interupt happens every 1 second
void RTC_Function()
{
   static const int daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
   bool isLeapYear = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
   
   second = (second + 1) % 60;                                                   // updating seconds
   if (second == 0) {
      minute = (minute + 1) % 60;                                                // updating minutes
      if (minute == 0) {
            hour = (hour + 1) % 24;                                              // updating hours
            if (hour == 0) {
               day++;                                                            // updating days
               int maxDays = daysInMonth[month] + (isLeapYear && month == 2);
               if (day > maxDays) {
                  day = 1;
                  month++;                                                       // updating months
                  if (month > 12) {
                        month = 1;
                        year++;                                                  // updating years
                  }
               }
            }
      }
   }
}

void PRINT_POWER_LINE_STATUS()
{
   fprintf(PC,"M=%d,", (POWER_LINE_STATUS>>7) & 0x01 ); 
   fprintf(PC,"C=%d,", (POWER_LINE_STATUS>>6) & 0x01 );
   fprintf(PC,"3=%d,", (POWER_LINE_STATUS>>5) & 0x01 );
   fprintf(PC,"3=%d,", (POWER_LINE_STATUS>>4) & 0x01 );
   fprintf(PC,"5=%d,", (POWER_LINE_STATUS>>3) & 0x01 );
   fprintf(PC,"U=%d,", (POWER_LINE_STATUS>>2) & 0x01 );
   fprintf(PC,"U=%d" , (POWER_LINE_STATUS>>1) & 0x01 );
   printline();
}

// this function will print the Reset pic time in every 1 second
void Print_RTC()
{
   if(previous_second != second )
   {
      fprintf(PC,"RP ");
      fprintf(PC,"%02d", year);
      fprintf(PC,"/%02d", month);
      fprintf(PC,"/%02d", day);
      fprintf(PC," %02d", hour);
      fprintf(PC,":%02d", minute);
      fprintf(PC,":%02d ", second);                                         // 20-01-01__05:20:22
      
      fprintf(PC,"%d>",NUMOF_MPIC_RST);
      fprintf(PC,"%03Ld", MPIC_TIME_COUNTER);
      fprintf(PC," %d>",NUMOF_CPIC_RST);
      fprintf(PC,"%03Ld ", CPIC_TIME_COUNTER);
      
      PRINT_POWER_LINE_STATUS();
   } 
   previous_second = second ;
}
