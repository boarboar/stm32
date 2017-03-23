// need FT ports!
#define US_IN_PIN   PB13
#define US_OUT_PIN  PB12


void setup() {
  delay(1000);
      Serial.begin(115200); 
    pinMode(US_OUT_PIN, OUTPUT);     
    pinMode(US_IN_PIN, INPUT); 
}

void loop() {
  delay(1000);
  // put your main code here, to run repeatedly:
          digitalWrite(US_OUT_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(US_OUT_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(US_OUT_PIN, LOW);
        uint32_t d=pulseIn(US_IN_PIN, HIGH, 50000);
        // if d==0 ...
        int16_t dist=(int16_t)(d/58);

        Serial.println(dist);
}
