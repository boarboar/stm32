#include <MapleFreeRTOS821.h>
#include "sens.h"
#include "log.h"

extern ComLogger xLogger;

void Sensor::Init(int sens_in_pin, int sens_out_pin, int servo_pin) {
    this->sens_in_pin=sens_in_pin;
    this->sens_out_pin=sens_out_pin;
    vSemaphoreCreateBinary(xSensFree);    
    xServo.attach(servo_pin);
    pinMode(sens_out_pin, OUTPUT);     
    pinMode(sens_in_pin, INPUT); 
    value=-2;
    Serial.println("Sensor OK");
}


void Sensor::Do() {
    digitalWrite(sens_out_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(sens_out_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sens_out_pin, LOW);
    uint32_t d=pulseIn(sens_in_pin, HIGH, 50000);
    if ( xSemaphoreTake( xSensFree, ( portTickType ) 10 ) == pdTRUE )
    {
      if(d>0) value=(int16_t)(d/58);
      else value = -2;
/*
      char buf[32];
        
        
        strcpy(buf, "SENS: ");

        itoa_cat(value, buf);
        
        xLogger.vAddLogMsg(buf);  
        */
      xSemaphoreGive( xSensFree );
    }
}

int16_t Sensor::Get() {  
  int ret=-2;
  if ( xSemaphoreTake( xSensFree, ( portTickType ) 10 ) == pdTRUE )
    {
      ret = value;
      xSemaphoreGive( xSensFree );
    }
  return ret;
}

