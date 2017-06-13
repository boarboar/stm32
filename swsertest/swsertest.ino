
#include "comm_mgr.h"


/*
 * 
GET
>G 3%214
<R 0,[104,104,105,107,104,375,373,373,376,373]%63
10 : 104 104 105 107 104 375 373 373 376 373  in 7
GET
>G 4%85
<R 0,[-12,-16,75,15,104,104,105,107,376,373,0,0,0,0]%77
14 : -12 -16 75 15 104 104 105 107 376 373 0 0 0 0  in 12
 */
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
        case 'a' :  
          Serial.println("GET");
          reg=c-'0';
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

  if(millis()-t > 5000) {     
    checkAlarm();
    t=millis();
  }
  
  doCmd();
  
  /*
  for(int ireg=1; ireg<=4; ireg++) { 
  delay(5000);
  resp=cmgr.Get(ireg);
  if(resp==0) 
  {
    int n=cmgr.GetResultCnt();
    const int16_t *va=cmgr.GetResultVal();
    Serial.print(n);
    Serial.print(" : ");
  
    for(int i=0; i<n; i++) {
      Serial.print(va[i]);    
      Serial.print(" ");
    }
    Serial.print(" in ");
    Serial.println(cmgr.GetLastTimeMs());
  }
  else 
  { 
    Serial.print("Error ");
    Serial.print(resp);
    Serial.print(" in ");
    Serial.println(cmgr.GetLastTimeMs());
  }
  }  
  */
}
