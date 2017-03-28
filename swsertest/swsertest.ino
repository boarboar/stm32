
#include <SoftwareSerial.h>

SoftwareSerial swSer(12, 13, false, 256);

uint8_t docrc(char *buf) {
  // check crc
  uint8_t crc=0, i;
  uint8_t pos=0;
  
  while(buf[pos])
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
    
void setup() {
  delay(2000);
  Serial.begin(115200);
  swSer.begin(115200);

  Serial.println("\nSoftware serial test started");
/*
  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");
*/

}

void loop() {
  /*
  while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }
  while (Serial.available() > 0) {
    swSer.write(Serial.read());
  }
*/
delay(1000);
char buf[20];
strcpy(buf, "G 1");
swSer.print(buf);
swSer.print("%");
swSer.println(docrc(buf));

while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }
  
delay(1000);

strcpy(buf, "S 1,[2,34]");
swSer.print(buf);
swSer.print("%");
swSer.println(docrc(buf));

while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }
}
