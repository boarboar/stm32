#include <Arduino.h>

#include "comm_mgr.h"
#include "base64.h"
 
void CommManager::Init(uint32_t comm_speed) {
  bytes = 0;
  buf[bytes] = 0;
  Serial3.begin(comm_speed); 
  while (Serial3.available()) Serial3.read();  // eat garbage
}

// read serial data into buffer
boolean CommManager::ReadSerialCommand()
{
  while (Serial3.available() && bytes < BUF_SIZE)
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
  if(bytes>=BUF_SIZE) { 
    bytes=0; //overflow, probably caused hang up at start...    
    buf[bytes]=0; 
    //return false;     
  }
  return false;
}

// read serial data into buffer
boolean CommManager::ProcessCommand()
{
  strcpy(buf, "UNKN");
  Serial3.println(buf);
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
void CommManager::Consume() {
  bytes=0;
  buf[bytes]=0;
}

void CommManager::Respond(const char *rsp) {
  Serial3.println(rsp);
}
*/

/*
const char *CommManager::GetBuffer() {
  return buf;
}

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

