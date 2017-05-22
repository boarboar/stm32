#include <MapleFreeRTOS821.h>
#include "log.h"
#include "mpu.h"
#include "sens.h"
#include "motor.h"
#include "motion.h"
#include "comm_mgr.h"
#include "base64.h"

extern ComLogger xLogger;
extern Sensor xSensor;
extern Motor xMotor;
extern Motion xMotion;
 
void CommManager::Init(uint32_t comm_speed) {
  vSemaphoreCreateBinary(xAlarmFree);
  xAlarmQueue = xQueueCreate( ALR_Q_SZ, sizeof( struct Alarm ) );

  if( xAlarmQueue == NULL )
    {
        /* Queue was not created and must not be used. */
        Serial.println("Couldn't create AQ");
        return;
    }

  txAlarm.uAlarmID=0;
    
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
  *msgdbg=0;  
  // check crc
  uint8_t crc=CRC();
  if(buf[pos]=='%') {
    pos++;
    while(isspace(buf[pos])) pos++;
    uint8_t mcrc=(uint8_t)ReadInt();    
    if(crc!=mcrc) {
      Serial3.println("R -1");
      strcpy(msgdbg, "CRC FAIL");
      return false;    
    } else strcpy(msgdbg, "% ");
  }
  
  pos=0;
  verb=buf[0];
  switch(verb) {
    case 'g': 
      verb='G';
    case 'G': // get cmd 
      strcat(msgdbg, "G");
      break;
    case 's':  
      verb='S'; 
    case 'S': // get cmd 
      strcat(msgdbg, "S");
      break;
    default:  
      strcat(msgdbg, "BAD CMD");
      Serial3.println("R -1");
      return false;
  }

  while(isspace(buf[pos]) || buf[pos]==',' ) pos++;     
  
  pos=1;
  reg=ReadInt();
  if(reg==0) {    
    Serial3.println("R -2");      
    strcat(msgdbg, " BAD REG");    
    return false;
  }
  
  strcat(msgdbg, " ");
  itoa_cat(reg, msgdbg);

  vcnt=0;
  if(verb=='G') {    
    switch(reg) {
      case REG_ID:
        vcnt=1;
        val[0]=CM_ID;
        break;
      case REG_STATUS:
        vcnt=1;
        if(MpuDrv::Mpu.Acquire()) {
          val[0]=MpuDrv::Mpu.getStatus();
          MpuDrv::Mpu.Release();
        }
        break;  
      case REG_SENS:
        if(xSensor.Acquire()) {
          vcnt=xSensor.GetNMeas();
          xSensor.Get(val, vcnt);
          xSensor.Release();
        }
        break;
      case REG_ALL:
        // yaw, X, Y
        vcnt=0;
        if(MpuDrv::Mpu.Acquire()) {
          val[0]=MpuDrv::Mpu.getYaw()*180.0/PI;
          vcnt++;
          MpuDrv::Mpu.Release();
        }
        if(xMotion.Acquire()) {
          xMotion.GetCrdCm(val+1); //val[1,2]
          val[3]=xMotion.GetAdvanceCm();
          vcnt+=3;
          xMotion.Release();
        }
        if(xSensor.Acquire()) {
          xSensor.Get(val+vcnt, vcnt);
          vcnt+=xSensor.GetNMeas();          
          xSensor.Release();
        }
        break;  
      case REG_ALARM:
        if( xQueueReceive( xAlarmQueue, &rxAlarm, ( TickType_t ) 1 ) )
        {
          vcnt=7;
          val[0]=rxAlarm.level;
          val[1]=rxAlarm.module;
          val[2]=rxAlarm.code;
          val[3]=rxAlarm.iData[0];
          val[4]=rxAlarm.iData[1];
          val[5]=rxAlarm.iData[2];
          val[6]=rxAlarm.iData[3];
        }  
        break;  
      default:;
        vcnt=0;
    }
    if(!vcnt) strcpy(buf, "R -7");
    else 
    {
      if(vcnt==1) 
      {
        strcpy(buf, "R 0,");
        itoa_cat(val[0], buf);
        strcat(buf, "%");
      } else 
      {
        strcpy(buf, "R 0,[");
        for(uint8_t i=0; i<vcnt; i++) {
          itoa_cat(val[i], buf);
          if(i<vcnt-1) strcat(buf, ",");
        }
        strcat(buf, "]%");
      }    
      crc=CRC();
      itoa_cat(crc, buf);
    }
    Serial3.println(buf);
    strcat(msgdbg, "->");
    strcat(msgdbg, buf);
    return true;
  }

  // set 
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
    strcat(msgdbg, " [@");    
    itoa_cat(vcnt, msgdbg);    
  } else {
    vcnt=1;
    val[0]=ReadInt();
    strcat(msgdbg, " ");
    itoa_cat(val[0], msgdbg);
  }

  int8_t rc=0;
  switch(reg) {    
    //case REG_ID:
    //    break;    
    case REG_MOTOR_POWER:
        if(vcnt<2) { rc=-21; break; }
        xMotion.SetMotors(val[0], val[1]);
        break;
    case REG_MOVE:
        if(vcnt<1) { rc=-21; break; }
        xMotion.Move(val[0]);
        break;    
    case REG_STEER:
        if(vcnt<1) { rc=-21; break; }
        xMotion.Steer(val[0]);
        break;    
    case REG_MOVE_BEAR:
        if(vcnt<1) { rc=-21; break; }
        xMotion.MoveBearing(val[0]);
        break;   
    case REG_RESET:
        // requires one parameter with fixed value of 100
        if(vcnt<1 || val[0]!=100) { rc=-21; break; }        
        vAddAlarm(CM_ALARM, CM_MODULE_SYS, 100);
        xLogger.vAddLogMsg("RST");           
        vTaskDelay(2000);                
        nvic_sys_reset();
        break;             
    default:;        
  }

  
  //Serial3.println("R 0");
  strcpy(buf, "R ");
  itoa_cat(rc, buf);
  Serial3.println(buf);       
  
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
    
const char *CommManager::GetBuffer() {
  return buf;
}

const char *CommManager::GetDbgBuffer() {
  return msgdbg;
}

void CommManager::Complete() {
  bytes=0;
  *buf=0;
  *msgdbg=0;
  verb=reg=vcnt=0;
}

void CommManager::vAddAlarm(uint8_t level, uint8_t module, uint8_t code, int16_t p0, int16_t p1, int16_t p2, int16_t p3)
{
   if ( xSemaphoreTake( xAlarmFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txAlarm.uAlarmID++;     
      txAlarm.level=level;
      txAlarm.module=module;
      txAlarm.code=code;
      txAlarm.iData[0]=p0;
      txAlarm.iData[1]=p1;
      txAlarm.iData[2]=p2;
      txAlarm.iData[3]=p3;
      xQueueSendToBack( xAlarmQueue, ( void * ) &txAlarm, ( TickType_t ) 0 );          
      xSemaphoreGive( xAlarmFree );
    }
}



