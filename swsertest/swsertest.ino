
#include "comm_mgr.h"

CommManager cmgr;
    
void setup() {
  delay(2000);
  Serial.begin(115200);
  cmgr.Init();
}

void loop() {
  delay(5000);
  cmgr.Command("G 1");
  delay(5000);
  cmgr.Command("S 1,[2,34]");
}
