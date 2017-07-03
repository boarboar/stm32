// need FT ports!
#define US_IN_PIN   PB12
#define US_OUT_PIN  PB13

volatile uint32_t di=0;


static void intr() {
  static uint32_t t0=0;
  uint16_t v=digitalRead(US_IN_PIN);
  if(v==HIGH) {
    t0=micros();
    di=0;
  } else {
    di=micros()-t0;
  }
}


void setup() {
  delay(1000);
      Serial.begin(115200); 
    pinMode(US_OUT_PIN, OUTPUT);     
    pinMode(US_IN_PIN, INPUT); 
    attachInterrupt(US_IN_PIN, intr,  CHANGE);
}

void loop() {
  delay(1000);
  // put your main code here, to run repeatedly:
   di=0;
          digitalWrite(US_OUT_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(US_OUT_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(US_OUT_PIN, LOW);
        
        uint32_t d=pulseIn(US_IN_PIN, HIGH, 50000);
        // if d==0 ...
        //int16_t dist=(int16_t)(d/58);

        //Serial.println(dist);
        Serial.print("D="); Serial.print(d);
        Serial.print(" / "); Serial.println(di);
        
}
