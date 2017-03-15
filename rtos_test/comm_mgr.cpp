#include <Arduino.h>

#include "comm_mgr.h"

void CommManager::Init(uint32_t comm_speed) {
  bytes = 0;
  buf[bytes] = 0;
  Serial3.begin(comm_speed); 
  while (Serial3.available()) Serial3.read();  // eat garbage
}

// read serial data into buffer. execute command
boolean CommManager::ReadSerialCommand()
{
  while (Serial3.available() && bytes < BUF_SIZE)
  {
    buf[bytes] = Serial3.read();
    if (buf[bytes] == 10 || buf[bytes] == 13)
    {
      if (bytes > 0) { 
        buf[bytes]=0; 
        Serial3.print("CMDOK[ ");
        Serial3.print(buf);
        Serial3.print(" ]");
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

void CommManager::ReadBuffer(char *pcBuf, uint16_t uLen) {
  if(pcBuf) {
    uint16_t len=bytes;
    if(uLen<bytes) len=uLen;
    strncpy(pcBuf, buf, len);
    bytes=0;
  }
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

//boolean CommandReader::ReadInt(int16_t *val) {
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

