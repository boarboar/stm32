
#include "comm_mgr.h"

CommManager cmgr;
    
void setup() {
  delay(2000);
  Serial.begin(115200);
  cmgr.Init();
}

void loop() {
  int resp;
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
    Serial.println();
  }
  else 
  { 
    Serial.print("Error ");
    Serial.println(resp);
  }
  }
  /*
  delay(5000);
  int16_t vals[]={35, -65};
  resp=cmgr.Set(2, vals, 2);
  if(resp!=0) { 
    Serial.print("Error ");
    Serial.println(resp);
  }
  */
  
}
