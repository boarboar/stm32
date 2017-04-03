#include <Arduino.h>
#include <SoftwareSerial.h>
#include "comm_mgr.h"

void itoa(int n, char s[]);
void ltoa(int32_t n, char s[]);

inline void itoa_cat(int n, char s[]) { itoa(n, s+strlen(s)); }
inline void ltoa_cat(int n, char s[]) { ltoa(n, s+strlen(s)); }

SoftwareSerial swSer(12, 13, false, 256);
 
void CommManager::Init(uint32_t comm_speed) {
  bytes = 0;
  buf[bytes] = 0;
  swSer.begin(comm_speed);
  while (swSer.available() > 0)  swSer.read();
  Serial.println("\nSoftware serial started");
}

// read serial data into buffer
bool CommManager::Command(char *cmd, int16_t timeout)
{  
  strcpy(buf, cmd);
  strcat(buf, "%");
  itoa_cat(CRC(), buf);
  Serial.print(">");
  Serial.println(buf);
  while (swSer.available() > 0)  swSer.read();
  swSer.println(buf);
  long t=millis();
  boolean res=false;
  bytes=0;   
  buf[bytes]=0; 
  while (!res && millis()<t+timeout && bytes<CM_BUF_SIZE)
  {
    while(!res && swSer.available()) 
    {
      buf[bytes] = swSer.read();
      if (buf[bytes] == 10 || buf[bytes] == 13)
      {
        if (bytes > 0) { 
          buf[bytes]=0;        
        } 
        res=true; 
     }
      else
        bytes++;
    }    
    if(!res) yield();
  }
  
  if(bytes>=CM_BUF_SIZE) { 
    Serial.println("OVERFLOW");
    bytes=0; //overflow, probably caused hang up at start...    
    buf[bytes]=0; 
    return false;     
  }

  if(!res) {
    Serial.println("TIMEOUT");
    return false;
  }

  
  Serial.print("<");
  Serial.println(buf);
  
  uint8_t crc=CRC();

  if(buf[pos]=='%') {
    pos++;
    while(isspace(buf[pos])) pos++;
    uint8_t mcrc=(uint8_t)ReadInt();    
    if(crc!=mcrc) {
      Serial.println("CRC FAIL");
      return false;    
    } 
  }
  pos=0;

  Serial.print("OK (");
  Serial.print(millis()-t);
  Serial.println(" ms) ");
  
  return true;
}



int16_t CommManager::ReadInt() {
  int16_t val=0;
  boolean sign=false;
  while(isspace(buf[pos])) pos++;
  if(buf[pos]=='+') pos++;
  else if(buf[pos]=='-') { 
    sign=true; 
    pos++;
  }
  while (isdigit(buf[pos]))
  {
    val *= 10;
    val += buf[pos] - '0';
    pos++;
  }
  if(sign) val=-val;
  return val;
}

uint8_t CommManager::CRC()
{
  uint8_t crc=0, i;
  pos=0;
  while(buf[pos] && buf[pos]!='%')
    {      
      crc = crc ^ buf[pos];
      for (i=0; i<8; i++) {
        if (crc & 1) {
            crc = (crc >> 1) ^0x8c;
        }
        else {
            crc = (crc >> 1);
        }
      }
      pos++;
    }
  return crc;
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




