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
const int M_POW_NORM=100;
const int M_SPEED_NORM=200;

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
  fCurrYaw = yaw;
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
      if(iTargSpeed)
        SetPowerStraight(iTargSpeed, cur_pow);      
      else {
        if(!iTargRot) StartRotate(0); // stop rotate
        else SetPowerRotate(iTargRot, cur_pow);              
      }     
    }
    lPIDCnt++;
  }
}

void Motion::Move(int16_t tspeed)
{
  if(!bReady) return;
  xLogger.vAddLogMsg("MOV V:", tspeed);
  if(iTargSpeed!=0 && tspeed==0) {
    // stop moving    
    //Serial.print(F("Stop TV, AVQE=")); Serial.println(getAVQErr());     
  } else if(iTargRot!=0 && tspeed==0) {
    // stop rotating    
    iTargRot=0;
    //Serial.println(F("Stop ROT")); 
  } else if(iTargSpeed!=0 && tspeed!=0) { 
    // start moving
    lPIDCnt=0;
    //qsum_err=0;
    //run_dist=0;
    //Serial.print(F("Start TV=")); Serial.println(tspeed);     
  } 


  AdjustTargBearing(0, true);
  err_bearing_p_0=err_bearing_i=0;    
  iTargSpeed=tspeed*10; //mm
  base_pow=(int32_t)abs(iTargSpeed)*M_POW_NORM/M_SPEED_NORM; // temp
  delta_pow=0;
   
  int16_t cur_pow[2]={base_pow, base_pow};
  //Serial.print(F("STV TV=")); Serial.print(targ_speed); Serial.print(F("POW=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]); 
  SetPowerStraight(iTargSpeed, cur_pow);
  return;
}

void Motion::Steer(int16_t angle)
{
  if(!bReady) return;
  xLogger.vAddLogMsg("ROT A:", angle);
  AdjustTargBearing(angle, true);
  //if(!iTargRot && !iTaregSpeed) return startRotate(M_SPEED_NORM);
}

void Motion::MoveBearing(int16_t angle)
{
  if(!bReady) return;
  xLogger.vAddLogMsg("BER A:", angle);
  AdjustTargBearing(angle, false);
  //if(!iTargRot && !iTaregSpeed) return startRotate(M_SPEED_NORM);
}

void Motion::AdjustTargBearing(int16_t s, bool absolute) {
  iTargBearing = s;
  if(absolute) iTargBearing+=fCurrYaw*180.0/PI;
  if(iTargBearing>180) iTargBearing-=180;
  else if(iTargBearing<-180) iTargBearing+=180;    
}


void Motion::StartRotate(int16_t tspeed) {
  int16_t a = iTargBearing-fCurrYaw*180.0/PI;
  if(a>180) a-=180;
  else if(a<-180) a+=180;    
  
  if(a>1) { 
    iTargRot=tspeed;  
    //Serial.println(F("Start ROT >>"));     
  }
  else if(a<-1) { 
    iTargRot=-tspeed;
    //Serial.println(F("Start ROT <<")); 
    }
  
  err_bearing_p_0=-a;     
  if(err_bearing_p_0<0) err_bearing_p_0=-err_bearing_p_0; 
  err_bearing_i=0;    
  base_pow=(int32_t)abs(iTargRot)*M_POW_NORM/M_SPEED_NORM; // temp
  delta_pow=0;  
  lPIDCnt=0;    
  int16_t cur_pow[2]={base_pow, base_pow};
  //Serial.print(F("STR =")); Serial.print(rot_speed); Serial.print(F("POW=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);  
  SetPowerRotate(iTargRot, cur_pow);    
}

void Motion::SetPowerStraight(int16_t dir, int16_t *p) {
  if(dir>0) SetMotors(p[0], p[1]);
  else SetMotors(-p[0], -p[1]);  
}

void Motion::SetPowerRotate(int16_t dir, int16_t *p) {
  if(dir>0) SetMotors(p[0], -p[1]);
  else SetMotors(-p[0], p[1]);  
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


