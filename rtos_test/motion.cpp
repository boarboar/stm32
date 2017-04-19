#include <MapleFreeRTOS821.h>
#include "motion.h"
#include "log.h"
#include "motor.h"

extern ComLogger xLogger;

// for test
TickType_t xRunTime=0;


void Motion::Init(Motor *m) {
    vSemaphoreCreateBinary(xMotionFree);        
    bReady = false;      
    pxMotor=m;
    Serial.println("Motion init OK");
}

void Motion::Start() {
  xLogger.vAddLogMsg("Motion module ready");
  if ( xSemaphoreTake( xMotionFree, ( portTickType ) 10 ) == pdTRUE )
    { 
      bReady=true;
      xSemaphoreGive( xMotionFree );
    }
  // test
  pxMotor->SetMotors(50, 50);       
  
        xRunTime=xTaskGetTickCount(); 
}

void Motion::DoCycle(float yaw) {
  pxMotor->DoCycle();
  if(!bReady) return;

          // test
          
        if(xTaskGetTickCount()-xRunTime > 5000) {
          xLogger.vAddLogMsg("==Stop");
          pxMotor->SetMotors(0, 0);     
        }
}

void Motion::SetMotors(int8_t dp1, int8_t dp2) // in %%
{
  if(!bReady) return;
  pxMotor->SetMotors(dp1, dp2);     
}

void Motion::Move(int16_t speed)
{
  if(!bReady) return;
  xLogger.vAddLogMsg("MOV V:", speed);
  // setTargSpeed(speed_val)
}

void Motion::Steer(int16_t angle)
{
  if(!bReady) return;
  xLogger.vAddLogMsg("ROT A:", angle);
  // setTargSteering(steer_val)
}

void Motion::MoveBearing(int16_t angle)
{
  if(!bReady) return;
  xLogger.vAddLogMsg("BER A:", angle);
  // setTargBearing(angle_val)
}


