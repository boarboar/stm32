/* S280116 - HW ref. A181115
 *  
 * Example code: Time library with RTC clock(STM32F103C8T6) without Time library (https://github.com/PaulStoffregen/Time)
 * 
 *    
 *  Tested on Aruino IDE 1.6.5 + Arduino_STM32. A181115 in "maple-mini" mode.
 */ 
 

#include <RTClock.h> 
#include <SPI.h>
#include <Adafruit_GFX_AS.h>    // Core graphics library, with extra fonts.
#include <Adafruit_ILI9341_STM.h> // STM32 DMA Hardware-specific library

#define cs   PA3
#define dc   PA1
#define rst  PA2

Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(cs, dc, rst);       // Invoke custom library

#define LED PC13

typedef struct TimeElements
{ 
  uint8_t Second; 
  uint8_t Minute; 
  uint8_t Hour; 
  uint8_t Wday;   // Day of week, sunday is day 1
  uint8_t Day;
  uint8_t Month; 
  uint8_t Year;   // Offset from 1970; 
} TimeElements ; 


uint32_t      dateTime_t;
TimeElements  dateTime;

const String  compTime = __TIME__;
const String  compDate = __DATE__;

RTClock rt (RTCSEL_LSE);  // Initialise RTC with LSE

void setup()  
{
  pinMode(LED, OUTPUT);  

  // Some delay waiting serial ready
  digitalWrite(LED, HIGH);
  delay(2000);
  digitalWrite(LED, LOW);

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  //tft.setTextColor(ILI9341_BLACK);  
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  //tft.drawString("Starting...",20,20,4);
  
  /*
  Serial.print("Compilation Date/Time: ");
  Serial.print(compTime);
  Serial.print(" ");
  Serial.println(compDate);
  
  Serial.print("RTC Counter (unix time): ");
  Serial.println(rt.getTime());
  */

  tft.println("Compilation Date/Time: ");
  tft.print(compTime);
  tft.print(" ");
  tft.println(compDate);
  
  RTCdisplay(); // Display RTC date/time

  // Convert compilation time strings to "unix" format
  dateTime.Second = compTime.substring(6,8).toInt();
  dateTime.Minute = compTime.substring(3,5).toInt();
  dateTime.Hour   = compTime.substring(0,2).toInt();
  dateTime.Day    = compDate.substring(4,6).toInt();
  switch (compDate[0]) 
  {
      case 'J': dateTime.Month = compDate[1] == 'a' ? 1 : compDate[2] == 'n' ? 6 : 7; break;
      case 'F': dateTime.Month = 2; break;
      case 'A': dateTime.Month = compDate[2] == 'r' ? 4 : 8; break;
      case 'M': dateTime.Month = compDate[2] == 'r' ? 3 : 5; break;
      case 'S': dateTime.Month = 9; break;
      case 'O': dateTime.Month = 10; break;
      case 'N': dateTime.Month = 11; break;
      case 'D': dateTime.Month = 12; break;
  };
  dateTime.Year = compDate.substring(7,11).toInt() - 1970;
  dateTime_t = makeTime(dateTime);    // Store compilation time in "unix" format

  delay(5000);
  
  // Set RTC at compilation time if needed
  if (dateTime_t > (uint32_t)rt.getTime())      // Compilation time > RTC time?
  // Yes. Set RCT time from compilation time
  {
    /*
    Serial.println();
    Serial.println("RTC Set to compilation Date/Time");
    rt.setTime(dateTime_t);
    */

    
    tft.println("RTC Set to compilation Date/Time");
    rt.setTime(dateTime_t);
    
    /*
    Serial.print("New RTC Date/Time: ");
    Serial.print(dateTime.Hour);
    Serial.print(":");
    Serial.print(dateTime.Minute);
    Serial.print(":");
    Serial.print(dateTime.Second);
    Serial.print(" ");
    Serial.print(dateTime.Day);
    Serial.print("-");
    Serial.print(dateTime.Month);
    Serial.print("-");
    Serial.println(dateTime.Year + 1970);
    Serial.println();
    */
  }
  else
  {
    tft.println("RTC OK");
  }
}

void loop()
{  
  RTCdisplay();  
  digitalWrite(LED,!digitalRead(LED));
  delay(1000);
}

void RTCdisplay()
{
  // Display RCT time
  /*
  Serial.print("RTC Date/Time:    ");
  breakTime(rt.getTime(), dateTime);
  Serial.print(dateTime.Hour);
  printDigits(dateTime.Minute);
  printDigits(dateTime.Second);
  Serial.print(" ");
  Serial.print(dateTime.Day);
  Serial.print("-");
  Serial.print(dateTime.Month);
  Serial.print("-");
  Serial.println(dateTime.Year+1970);
  */

  //tft.println("RTC Date/Time:    ");

  tft.fillRect(20, 200, 200, 32, ILI9341_BLACK); 
  char buf[24];
  char *bptr=buf;
  
  breakTime(rt.getTime(), dateTime);
  printDigits(bptr, dateTime.Hour, 2);
  bptr+=2;
  *bptr++=':';
  printDigits(bptr, dateTime.Minute, 2);
  bptr+=2;
  *bptr++=':';
  printDigits(bptr, dateTime.Second, 2);
  
  
  bptr=buf;
  tft.drawString(buf, 20, 200, 4);
  printDigits(bptr, dateTime.Day, 2);
  bptr+=2;
  *bptr++='/';
  printDigits(bptr, dateTime.Month, 2);
  bptr+=2;
  *bptr++='/';
  printDigits(bptr, dateTime.Year, 4);
  tft.drawString(buf, 20, 220, 2);
  /*
  tft.print(dateTime.Hour);
  printDigits(dateTime.Minute);
  printDigits(dateTime.Second);
  tft.print(" ");
  tft.print(dateTime.Day);
  tft.print("-");
  tft.print(dateTime.Month);
  tft.print("-");
  tft.println(dateTime.Year+1970);
  */
}

void printDigits(char *buf, uint16_t t, uint16_t ndig) 
{
  char tmp[8];
  int8_t shift;
  uint8_t i;
  itoa(t, tmp);
  shift=ndig-strlen(tmp);
  if(shift<0) return;
  
  for(i=0; i<shift; i++) buf[i]='0';
  strcpy(buf+i, tmp);
}

void printDigits(int digits)
{
  // Print two digits preceding colon and leading 0
  /*
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
  */
  tft.print(":");
  if(digits < 10)
    tft.print('0');
  tft.print(digits);
}

/*============================================================================*/  
/* Functions to convert to and from system time (From https://github.com/PaulStoffregen/Time */

// Leap year calulator expects year argument as years offset from 1970
#define LEAP_YEAR(Y)  ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)

static  const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0
 
void breakTime(uint32_t timeInput, struct TimeElements &tm){
// Break the given time_t into time components
// This is a more compact version of the C library localtime function
// Note that year is offset from 1970 !!!

  uint8_t year;
  uint8_t month, monthLength;
  uint32_t time;
  unsigned long days;

  time = (uint32_t)timeInput;
  tm.Second = time % 60;
  time /= 60; // Now it is minutes
  tm.Minute = time % 60;
  time /= 60; // Now it is hours
  tm.Hour = time % 24;
  time /= 24; // Now it is days
  tm.Wday = ((time + 4) % 7) + 1;  // Sunday is day 1 
  
  year = 0;  
  days = 0;
  while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
    year++;
  }
  tm.Year = year; // Year is offset from 1970 
  
  days -= LEAP_YEAR(year) ? 366 : 365;
  time  -= days; // Now it is days in this year, starting at 0
  
  days=0;
  month=0;
  monthLength=0;
  for (month=0; month<12; month++) {
    if (month==1) { // February
      if (LEAP_YEAR(year)) {
        monthLength=29;
      } else {
        monthLength=28;
      }
    } else {
      monthLength = monthDays[month];
    }
    
    if (time >= monthLength) {
      time -= monthLength;
    } else {
        break;
    }
  }
  tm.Month = month + 1;  // Jan is month 1  
  tm.Day = time + 1;     // Day of month
}

uint32_t makeTime(struct TimeElements &tm){   
// Assemble time elements into "unix" format 
// Note year argument is offset from 1970
  
  int i;
  uint32_t seconds;

  // Seconds from 1970 till 1 jan 00:00:00 of the given year
  seconds= tm.Year*(SECS_PER_DAY * 365);
  for (i = 0; i < tm.Year; i++) {
    if (LEAP_YEAR(i)) {
      seconds +=  SECS_PER_DAY;   // Add extra days for leap years
    }
  }
  
  // Add days for this year, months start from 1
  for (i = 1; i < tm.Month; i++) {
    if ( (i == 2) && LEAP_YEAR(tm.Year)) { 
      seconds += SECS_PER_DAY * 29;
    } else {
      seconds += SECS_PER_DAY * monthDays[i-1];  // MonthDay array starts from 0
    }
  }
  seconds+= (tm.Day-1) * SECS_PER_DAY;
  seconds+= tm.Hour * SECS_PER_HOUR;
  seconds+= tm.Minute * SECS_PER_MIN;
  seconds+= tm.Second;
  return (uint32_t)seconds; 
}


/* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}  

/* itoa:  convert n to characters in s */
 void itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  


/* itoa:  convert n to characters in s */
 void ltoa(int32_t n, char s[])
 {
     int32_t i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  

