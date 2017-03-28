
int count=0;

void isr(void)  {  count++;  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  attachInterrupt(PB10, isr, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial.println(count);
}
