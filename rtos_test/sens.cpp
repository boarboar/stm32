#include <MapleFreeRTOS821.h>
#include "sens.h"
#include "log.h"

#define SERVO_NSTEPS  2
#define SERVO_STEP    36
#define SERVO_ZERO_SHIFT    3
#define SERVO_CORR    2
/*
// actual USENS_DIVISOR constant should be 58.138, but we make correction for angle
#define USENS_DIVISOR 57
#define USENS_BASE    3
*/

#define USENS_DIVISOR 58
#define USENS_BASE    0

extern ComLogger xLogger;

void Sensor::Init(int sens_in_pin, int sens_out_pin, int servo_pin) {
    this->sens_in_pin=sens_in_pin;
    this->sens_out_pin=sens_out_pin;
    vSemaphoreCreateBinary(xSensFree);    
    xServo.attach(servo_pin);
    xServo.write(90-SERVO_ZERO_SHIFT);
    pinMode(sens_out_pin, OUTPUT);     
    pinMode(sens_in_pin, INPUT); 
    value=-2;
    sservo_pos=0; //90
    sservo_step=1; 
    running = false;
    Serial.println("Sensor OK");
}

void Sensor::Start() {
    xLogger.vAddLogMsg("Sens module run.");
    running=true;
}
  
void Sensor::DoCycle() {
      
    if((sservo_step>0 && sservo_pos>=SERVO_NSTEPS) || (sservo_step<0 && sservo_pos<=-SERVO_NSTEPS)) sservo_step=-sservo_step;
    sservo_pos+=sservo_step;
    int16_t sservo_angle=90-SERVO_ZERO_SHIFT+sservo_pos*SERVO_STEP+abs(sservo_pos)*SERVO_CORR;
    xServo.write(sservo_angle);
    vTaskDelay(50);
    for(uint16_t sens_step=0; sens_step<2; sens_step++) {  
      int8_t current_sens=-sservo_pos+SERVO_NSTEPS+sens_step*(SERVO_NSTEPS*2+1); 
      // todo
      // pins=..[sens_step]
      // value=..[current_sens]
      digitalWrite(sens_out_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(sens_out_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(sens_out_pin, LOW);
      uint32_t d=pulseIn(sens_in_pin, HIGH, 50000);
      if ( xSemaphoreTake( xSensFree, ( portTickType ) 10 ) == pdTRUE )
      {
        if(d>0) value=(int16_t)(d/USENS_DIVISOR+USENS_BASE);
        else value = -2;
        xSemaphoreGive( xSensFree );
      }
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

