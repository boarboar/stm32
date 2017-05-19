#include <MapleFreeRTOS821.h>
#include "sens.h"
#include "log.h"

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

void Sensor::Init(int servo_pin, int sens_in_pin_0, int sens_out_pin_0, int sens_in_pin_1, int sens_out_pin_1) {
    uint8_t i;
    this->sens_in_pin[0]=sens_in_pin_0;
    this->sens_out_pin[0]=sens_out_pin_0;
    this->sens_in_pin[1]=sens_in_pin_1;
    this->sens_out_pin[1]=sens_out_pin_1;
    vSemaphoreCreateBinary(xSensFree);    
    xServo.attach(servo_pin);
    xServo.write(90-SERVO_ZERO_SHIFT);
    for(i=0; i<2; i++) {
      pinMode(sens_out_pin[i], OUTPUT);           
      pinMode(sens_in_pin[i], INPUT); 
    }
    for(i=0; i<M_SENS_N; i++) 
      value[i]=-2;
    sservo_pos=0; //90
    sservo_step=1; 
    running = false;
    Serial.println("Sensor OK");
}

bool Sensor::Acquire() {
  return xSemaphoreTake( xSensFree, ( portTickType ) 10 ) == pdTRUE;
}


void Sensor::Release() {
  xSemaphoreGive( xSensFree );
}

void Sensor::Start() {
    xLogger.vAddLogMsg("Sens module run.");
    running=true;
}

/*
N=6:  (NSTEPS=1)
      Sens_s |
Servo_p   \  | 0  | 1  |
------------ +----+----+  
  -1         | 2  | 5  |  
------------ +----+----+  
   0         | 1  | 4  |  
------------ +----+----+  
   1         | 0  | 3  |  
------------ +----+----+  

N=10:  (NSTEPS=2)
      Sens_s |
Servo_p   \  | 0  | 1  |
------------ +----+----+  
  -2         | 4  | 9  |  
------------ +----+----+  
  -1         | 3  | 8  |  
------------ +----+----+  
   0         | 2  | 7  |  
------------ +----+----+  
   1         | 1  | 6  |  
------------ +----+----+  
   2         | 0  | 5  |  
------------ +----+----+    
*/

void Sensor::DoCycle() {
    if(!running) return;
      
    if((sservo_step>0 && sservo_pos>=SERVO_NSTEPS) || (sservo_step<0 && sservo_pos<=-SERVO_NSTEPS)) sservo_step=-sservo_step;
    sservo_pos+=sservo_step;
    int16_t sservo_angle=90-SERVO_ZERO_SHIFT+sservo_pos*SERVO_STEP+abs(sservo_pos)*SERVO_CORR;
    xServo.write(sservo_angle);
    vTaskDelay(50);
    for(uint16_t sens_step=0; sens_step<2; sens_step++) {  
      int8_t current_sens=-sservo_pos+SERVO_NSTEPS+sens_step*(SERVO_NSTEPS*2+1); 
      digitalWrite(sens_out_pin[sens_step], LOW);
      delayMicroseconds(2);
      digitalWrite(sens_out_pin[sens_step], HIGH);
      delayMicroseconds(10);
      digitalWrite(sens_out_pin[sens_step], LOW);
      uint32_t d=pulseIn(sens_in_pin[sens_step], HIGH, 40000);

      if(Acquire()) 
      {
        if(d>0) value[current_sens]=(int16_t)(d/USENS_DIVISOR+USENS_BASE);
        else value[current_sens] = -2;
        Release();  
      }
      vTaskDelay(1);
    }
}

int16_t Sensor::GetNMeas() {
  return M_SENS_N;
}

int16_t Sensor::Get() {
  return value[0];
}


void Sensor::Get(int16_t *v, int16_t n) {
  if(n>M_SENS_N) n=M_SENS_N;
  for(int i=0; i<n; i++)
    v[i]=value[i];
}

