
#include "comm_mgr.h"


CommManager cmgr;

void checkAlarm() {
  while(cmgr.Get(9)==0) { // alarm
    int n=cmgr.GetResultCnt();
    if(n) {
      const int16_t *va=cmgr.GetResultVal();
      Serial.print("ALR :");
      
      for(int i=0; i<n; i++) {
        Serial.print(va[i]);    
        Serial.print(" ");
      }
      Serial.print(" in ");
      Serial.println(cmgr.GetLastTimeMs());
    }
  }
}

void doCmd() {
  if(Serial.available()) {
      char c = Serial.read();
      int16_t vals[2];
      int16_t nvals=0;
      int16_t reg=0;
      boolean doSet=true;
      
      switch(c) {
        case 'q' :
        case 'Q' : // reset
          Serial.println("RESET");
          reg=100;
          nvals=1;
          vals[0]=100;
          break;
        case 'd' :
        case 'D' : // drive
          Serial.println("DRIVE MED");
          reg=5;
          nvals=2;
          vals[0]=vals[1]=50;
          break;  
        case 'l' :
        case 'L' : // drive
          Serial.println("DRIVE LEFT");
          reg=5;
          nvals=2;
          vals[0]=50;
          vals[1]=0;
          break;
        case 'r' :
        case 'R' : // drive
          Serial.println("DRIVE RIGHT");
          reg=5;
          nvals=2;
          vals[0]=0;
          vals[1]=50;
          break;  
        case 'h' :
        case 'H' : // drive
          Serial.println("DRIVE HI");
          reg=5;
          nvals=2;
          vals[0]=vals[1]=90;
          break;        
         case 's' :
        case 'S' : // drive
          Serial.println("DRIVE STOP");
          reg=5;
          nvals=2;
          vals[0]=vals[1]=0;
          break;   
        case '1' :  
        case '2' :  
        case '3' :  
        case '4' :  
          Serial.println("GET");
          reg=c-'0';
          doSet=false;          
          break;   
        case 'a' :  
          Serial.println("GET");
          reg=10;
          doSet=false;          
          break;     
        default:;  
      }
      if(reg) {
        int resp;
        if(doSet) resp=cmgr.Set(reg, vals, nvals);
        else resp=cmgr.Get(reg);
        if(resp!=0) { 
          Serial.print("Error ");
          Serial.print(resp);
        }
        else if(!doSet) {
            int n=cmgr.GetResultCnt();
            const int16_t *va=cmgr.GetResultVal();
            Serial.print(n);
            Serial.print(" : ");
  
            for(int i=0; i<n; i++) {
              Serial.print(va[i]);    
              Serial.print(" ");
          }
        }
        Serial.print(" in ");
        Serial.println(cmgr.GetLastTimeMs());
      }
    }
}
uint32_t t;

void setup() {
  delay(2000);
  Serial.begin(115200);
  cmgr.Init();
  t=millis();
}


void loop() {
  //int resp;

  if(millis()-t > 10000) {     
    checkAlarm();
    t=millis();
  }
  
  doCmd();
  
}
