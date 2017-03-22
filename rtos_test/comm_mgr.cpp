#include <MapleFreeRTOS821.h>
#include "log.h"
#include "comm_mgr.h"
#include "base64.h"

extern ComLogger xLogger;
 
void CommManager::Init(uint32_t comm_speed) {
  bytes = 0;
  buf[bytes] = 0;
  Serial3.begin(comm_speed); 
  while (Serial3.available()) Serial3.read();  // eat garbage
}

// read serial data into buffer
boolean CommManager::ReadSerialCommand()
{
  while (Serial3.available() && bytes < CM_BUF_SIZE)
  {
    buf[bytes] = Serial3.read();
    if (buf[bytes] == 10 || buf[bytes] == 13)
    {
      if (bytes > 0) { 
        buf[bytes]=0; 
        /*
        Serial3.print("CMDOK[ ");
        Serial3.print(buf);
        Serial3.print(" ]");

        if(buf[0]=='U') {
          Serial3.println("trying to decode...");
          unsigned char bufx[BUF_SIZE];
          int sz=base64_decode(buf+1, bufx);
          bufx[sz]=0;          
          Serial3.println((char *)bufx);
          Serial3.println("trying to encode...");
          uint16_t data[4]={1,9,1,7};
          sz=base64_encode((unsigned char const*) data, (char *)bufx, sizeof(uint16_t)*4);
          bufx[sz]=0;          
          Serial3.println((char *)bufx);          
        }
        */
        return true; 
      } 
      return false; // skip 10 or 13 left         
    }
    bytes++;
  }
  if(bytes>=CM_BUF_SIZE) { 
    bytes=0; //overflow, probably caused hang up at start...    
    buf[bytes]=0; 
    //return false;     
  }
  return false;
}

// process command
// cmd:= setcmd | getcmd
// getcmd := verb | reg
// setcmd := verb | reg | val
// val := int | list
// list := '[' int, ... ']'
boolean CommManager::ProcessCommand()
{
  pos=0;
  verb=buf[0];
  switch(verb) {
    case 'G': // get cmd 
      strcpy(rsp, "G ");
      break;
    case 'S': // get cmd 
      strcpy(rsp, "S ");
      break;
    default:  
      strcpy(buf, "BAD CMD");
      Serial3.println("R -1");
      return false;
  }
  pos=1;
  reg=ReadInt();
  if(reg==0) {
      strcpy(buf, "BAD REG");
      Serial3.println("R -2");
      return false;
  }
  strcat(rsp, " ");
  itoa(reg, bufn);
  strcat(rsp, bufn);

  // validate reg - TODO

  if(verb=='G') {
    // do get - TODO
    Serial3.println("R 0,777");
    return true;
  }

  while(isspace(buf[pos])) pos++;     

  if(!buf[pos]) {
    Serial3.println("R -3");
    return false;
  }
 
  if(buf[pos]=='[') {
    // array    
    vcnt=0;
    pos++;
    while(1) { 
        while(isspace(buf[pos])) pos++;     
        if(!buf[pos]) {
          Serial3.println("R -4");
          return false;
        }
        if(buf[pos]==']') break;        
        val[vcnt]=ReadInt(); // add to array
        vcnt++;
        if(vcnt>CM_NVAL) {
          // too many vals
          Serial3.println("R -5");
          return false;
        }        
        while(isspace(buf[pos])) pos++;     
        if(buf[pos]==',') pos++;
    }     
    strcat(rsp, " [@");
    itoa(vcnt, bufn);
    strcat(rsp, bufn);    
  } else {
    vcnt=1;
    val[0]=ReadInt();
    strcat(rsp, " ");
    itoa(val[0], bufn);
    strcat(rsp, bufn);    
  }

  // do set 
  
  Serial3.println("R 0");
        
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

const char *CommManager::GetBuffer() {
  return buf;
}

const char *CommManager::GetDbgBuffer() {
  return rsp;
}

void CommManager::Consume() {
  bytes=0;
  buf[bytes]=0;
}

/*
void CommManager::Consume(char *pcBuf, uint16_t uLen) {
  if(pcBuf) {
    uint16_t len=bytes;
    if(uLen<bytes) len=uLen;
    strncpy(pcBuf, buf, len);
    bytes=0;
    buf[bytes]=0;
  }
}
*/

/*

void CommManager::Respond(const char *rsp) {
  Serial3.println(rsp);
}
*/

/*

boolean CommManager::Match(const char *cmd) {
  uint8_t savepos=pos;
  while(pos<bytes && *cmd && buf[pos]==*cmd) {
    pos++;
    cmd++;
  }
  if(!*cmd) return true;
  else {
    pos=savepos;
    return false;
  }
  //return *cmd==0;
}

int16_t CommManager::ReadInt() {
  int16_t i=0;
  boolean sign=false;
  while(isspace(buf[pos])) pos++;
  if(buf[pos]=='+') pos++;
  else if(buf[pos]=='-') { 
    sign=true; 
    pos++;
  }
  while (isdigit(buf[pos]))
  {
    i *= 10;
    i += buf[pos] - '0';
    pos++;
  }
  //*val=sign ? -i : i;
  //return true;
  if(sign) i=-i;
  return i;
}

char CommManager::ReadChar() {
  if(pos>=bytes) return 0;
  char c = buf[pos];
  pos++;
  return c;
}

*/



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

