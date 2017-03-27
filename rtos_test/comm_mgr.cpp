#include <MapleFreeRTOS821.h>
#include "log.h"
#include "comm_mgr.h"
#include "base64.h"


// TODO allow , as a separator

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
// getcmd := verb [,] reg
// setcmd := verb [,] reg [,] val
// val := int | list
// list := '[' int [,] ... ']'
boolean CommManager::ProcessCommand()
{
  pos=0;
  verb=buf[0];
  switch(verb) {
    case 'g': 
      verb='G';
    case 'G': // get cmd 
      strcpy(rsp, "G");
      break;
    case 's':  
      verb='S'; 
    case 'S': // get cmd 
      strcpy(rsp, "S");
      break;
    default:  
      strcpy(buf, "BAD CMD");
      Serial3.println("R -1");
      return false;
  }

  while(isspace(buf[pos]) || buf[pos]==',' ) pos++;     
  
  pos=1;
  reg=ReadInt();
  if(reg==0) {
      strcpy(buf, "BAD REG");
      Serial3.println("R -2");
      return false;
  }
  strcat(rsp, " ");
  //itoa(reg, bufn);
  //strcat(rsp, bufn);
  itoa_cat(reg, rsp);

  // validate reg - TODO

  if(verb=='G') {
    // do get - TODO
    // switch ...
    Serial3.println("R 0,777");
    return true;
  }

  while(isspace(buf[pos]) || buf[pos]==',' ) pos++;     

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
    //itoa(vcnt, rsp+strlen(rsp));    
    itoa_cat(vcnt, rsp);    
  } else {
    vcnt=1;
    val[0]=ReadInt();
    strcat(rsp, " ");
    //itoa(val[0], rsp+strlen(rsp));     
    itoa_cat(val[0], rsp);
  }

  // do set 
  // switch ...
  
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

void CommManager::Complete() {
  bytes=0;
  buf[bytes]=0;
  verb=reg=vcnt=0;
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




