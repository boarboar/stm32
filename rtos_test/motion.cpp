#include <MapleFreeRTOS821.h>
#include "motion.h"
#include "log.h"
#include "motor.h"

extern ComLogger xLogger;

const int16_t bear_pid_gain_p=8;
const int16_t bear_pid_gain_d=120;
const int16_t bear_pid_gain_i=4;
const int16_t bear_pid_gain_div=10;
const int16_t bear_pid_limit_i=100;
const int M_POW_MIN=30; 
const int M_POW_MAX=200;

void Motion::Init(Motor *m) {
  vSemaphoreCreateBinary(xMotionFree);        
  bReady = false;      
  pxMotor=m;
  Reset();
  Serial.println("Motion init OK");
}

void Motion::Start() {
  xLogger.vAddLogMsg("Motion module ready");
  if ( xSemaphoreTake( xMotionFree, ( portTickType ) 10 ) == pdTRUE )
    { 
      bReady=true;
      xSemaphoreGive( xMotionFree );
    }

  xRunTime=xTaskGetTickCount(); 
    
  // test
  pxMotor->SetMotors(50, 50);       
        
}

void Motion::Reset() {
  iTargBearing=0;
  iTargSpeed=0;
  iTargRot=0;
  lPIDCnt=0;
  for(int i=0; i<2; i++ ) {
    lAdvance[i]=0;
    lAdvance0[i]=0;
    fCrd[i]=0.0f;
  }
  err_bearing_p_0=0;
  err_bearing_i=0;
  base_pow=0;
  delta_pow=0;
}

void Motion::DoCycle(float yaw) 
{
  float mov; //mm
  pxMotor->DoCycle();
  if(!bReady) return;
  //uint32_t dt=xTaskGetTickCount()-xRunTime;
  xRunTime=xTaskGetTickCount();       
  pxMotor->GetEncDist(NULL, lAdvance);
  mov=((lAdvance[0]-lAdvance0[0])+(lAdvance[1]-lAdvance0[1]))*0.5f;
  lAdvance0[0]=lAdvance[0];
  lAdvance0[1]=lAdvance[1];
  // integrate
  fCrd[0]+=mov*sin(yaw);
  fCrd[1]+=mov*cos(yaw);

  if(iTargSpeed || iTargRot) 
  { // movement
    if(lPIDCnt>0) 
    {
      int16_t err_bearing_p, err_bearing_d;
      err_bearing_p = (int16_t)(yaw*180.0f/PI-iTargBearing);      
      while(err_bearing_p>180) err_bearing_p-=360;
      while(err_bearing_p<-180) err_bearing_p+=360;   

      if(iTargSpeed) { // straight 
        if(iTargSpeed<0) err_bearing_p=-err_bearing_p;               
        err_bearing_i=err_bearing_i+err_bearing_p;
        // note: it should rather be +err_bearing_p*dt; 
        // or if normed to 100ms: err_bearing_i/100;
        if(err_bearing_i>bear_pid_limit_i) err_bearing_i=bear_pid_limit_i;
        if(err_bearing_i<-bear_pid_limit_i) err_bearing_i=-bear_pid_limit_i;      
        //if(run_dist>=100) //100mm
        //  qsum_err+=err_bearing_p*err_bearing_p;
      } else { // rot
        if((err_bearing_p<0 && iTargRot<0) || (err_bearing_p>0 && iTargRot>0)) { 
          iTargRot=-iTargRot;          
        }
        if(err_bearing_p<0) err_bearing_p=-err_bearing_p;        
        if(err_bearing_p<5) { // at the moment - 5 degrees
          iTargRot=0;          
        }        
      }

      err_bearing_d=err_bearing_p-err_bearing_p_0;      
      if(err_bearing_d>180) err_bearing_d-=360;
      else if(err_bearing_d<-180) err_bearing_d+=360;
      // note: it should rather be (err_bearing_p-err_bearing_p_0)/dt; 
      // or if normed to 100ms: (int32_t)(err_bearing_p-err_bearing_p_0)*100/dt;
        
      err_bearing_p_0=err_bearing_p;
      // use 32 bit math?
      delta_pow=-(int16_t)((err_bearing_p*bear_pid_gain_p+err_bearing_d*bear_pid_gain_d+err_bearing_i*bear_pid_gain_i)/bear_pid_gain_div);

      int16_t cur_pow[2];
      if(iTargSpeed) {              
        cur_pow[0]=base_pow+delta_pow;
        cur_pow[1]=base_pow-delta_pow;       
      } else {
        cur_pow[0]=base_pow-delta_pow;
        cur_pow[1]=base_pow-delta_pow;
      }
      
      for(int i=0; i<2; i++) {
        if(cur_pow[i]<M_POW_MIN) cur_pow[i]=M_POW_MIN; 
        if(cur_pow[i]>M_POW_MAX) cur_pow[i]=M_POW_MAX; 
      // maybe a better idea would be to make limits proportional to the target?
      }
      /*
      if(targ_speed)
        setPowerStraight(targ_speed, cur_pow);      
      else {
        if(!rot_speed) startRotate(0); // stop rotate
        else setPowerRotate(rot_speed, cur_pow);      
      }
      */
    }
    lPIDCnt++;
  }
}

void Motion::SetMotors(int8_t dp1, int8_t dp2) // in %%
{
  if(!bReady) return;
  pxMotor->SetMotors(dp1, dp2);     
}

bool Motion::GetAdvance(uint32_t *dst_dist) 
{
  if(!dst_dist) return false;
  int ret=false;
  if ( xSemaphoreTake( xMotionFree, ( portTickType ) 10 ) == pdTRUE )
    {
      dst_dist[0]=lAdvance[0];
      dst_dist[1]=lAdvance[1];      
      xSemaphoreGive( xMotionFree );
    }
  return ret;
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


