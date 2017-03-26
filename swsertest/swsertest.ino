
#include <SoftwareSerial.h>

SoftwareSerial swSer(12, 13, false, 256);

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
delay(10);
swSer.println("G 1");
while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }

swSer.println("S 1,[2,34]");
while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }
}
