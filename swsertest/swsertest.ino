
#include "comm_mgr.h"
/*
#include <SoftwareSerial.h>

SoftwareSerial swSer(12, 13, false, 256);

uint8_t crc(char *buf) {
  // check crc
  uint8_t crc=0, i;
  uint8_t pos=0;
  
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
  */

CommManager cmgr;
    
void setup() {
  delay(2000);
  Serial.begin(115200);

  cmgr.Init();

  /*
  swSer.begin(115200);

  while (swSer.available() > 0)  swSer.read();

  Serial.println("\nSoftware serial test started");
*/

}

void loop() {
  delay(5000);
  cmgr.Command("G 1");
  delay(5000);
  cmgr.Command("S 1,[2,34]");

  /*
  while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }
  while (Serial.available() > 0) {
    swSer.write(Serial.read());
  }
*/
/*
delay(5000);
char buf[20];
strcpy(buf, "G 1%");
Serial.println(buf);


  while (swSer.available() > 0)  swSer.read();
  
swSer.print(buf);
swSer.println(crc(buf));
*buf=0;
int i=0;
while (!swSer.available())  {}

while (swSer.available() > 0) {
    buf[i]=swSer.read();
    Serial.write(buf[i]);
    i++;
  }
buf[i]=0;
Serial.println(crc(buf));  

delay(5000);

strcpy(buf, "S 1,[2,34]%");
Serial.println(buf);


  while (swSer.available() > 0)  swSer.read();
  
swSer.print(buf);
swSer.println(crc(buf));

*buf=0;
i=0;

while (!swSer.available())  {}

while (swSer.available() > 0) {
    buf[i]=swSer.read();
    Serial.write(buf[i]);
    i++;
  }
buf[i]=0;
*/

}
