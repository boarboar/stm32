#include <MapleFreeRTOS821.h>
#include "motor.h"
#include "log.h"

#define   V_NORM 10000
#define   V_NORM_MAX 30000
#define   V_NORM_PI2 62832L
#define   WHEEL_CHGSTATES 40
#define   WHEEL_RAD_MM   33 // measured 32
#define   CHGST_TO_MM(CNT)  ((uint64_t)(CNT)*V_NORM_PI2*WHEEL_RAD_MM/WHEEL_CHGSTATES/V_NORM)
extern ComLogger xLogger;

static Motor *motor_instance=NULL;  

static void encodeInterrupt_1() { 
  static portBASE_TYPE xHigherPriorityTaskWoken1;
  xHigherPriorityTaskWoken1 = pdFALSE;
  if(motor_instance) motor_instance->encInterrupt(0); 
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );
  }
  
static void encodeInterrupt_2() { 
  static portBASE_TYPE xHigherPriorityTaskWoken2;
  xHigherPriorityTaskWoken2 = pdFALSE;
  if(motor_instance) motor_instance->encInterrupt(1); 
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken2 );
  }

void Motor::Init(int m_1_1_pin, int m_1_2_pin, int m_1_enab_pin, int m_1_enc_pin, int m_2_1_pin, int m_2_2_pin, int m_2_enab_pin, int m_2_enc_pin) {
    vSemaphoreCreateBinary(xMotorFree);    
    
    m[0].pin_1=m_1_1_pin;
    m[0].pin_2=m_1_2_pin;
    m[0].pin_enab=m_1_enab_pin;
    m[0].pin_enc=m_1_enc_pin;

    m[1].pin_1=m_2_1_pin;
    m[1].pin_2=m_2_2_pin;
    m[1].pin_enab=m_2_enab_pin;
    m[1].pin_enc=m_2_enc_pin;
    
    for(int i=0; i<2; i++) {
      pinMode(m[i].pin_1, OUTPUT);     
      pinMode(m[i].pin_2, OUTPUT);
      pinMode(m[i].pin_enab, PWM);
      pinMode(m[i].pin_enc, INPUT_PULLUP);
      m[i].enc_count=0;
      m[i].enc_accum=0;
      //m[i].dist=0;
      m[i].enc_prev_st=digitalRead(m[i].pin_enc);
      m[i].dir=0;
      m[i].power=0;
      Low_Drive(i);
    }
    
    motor_instance = this;

    attachInterrupt(m[0].pin_enc, encodeInterrupt_1, CHANGE);
    attachInterrupt(m[1].pin_enc, encodeInterrupt_2, CHANGE);
    
    //running = false;
      
    Serial.println("Motor OK");
}

void Motor::DoCycle() {
  if ( xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE )
    {
      for(int i=0; i<2; i++) {
        m[i].enc_accum+=m[i].enc_count;
        m[i].enc_count=0;       
      }
      xSemaphoreGive( xMotorFree );
    }
}

// 0-100 
void Motor::SetMotors(int8_t dp1, int8_t dp2) {

  //if(!running) return;
  
  int8_t dp[2];
  int8_t d[2];
  uint16_t p[2];  
  int i;
  dp[0]=dp1;
  dp[1]=dp2;
  
  for(i=0; i<2; i++) {
    if(dp[i]<-100 || dp[i]>100) return;    
    if(dp[i]==0) {d[i]=0; p[i]=0; }
    else if(dp[i]>0) {d[i]=1; p[i]=dp[i]; }
    else {d[i]=-1; p[i]=-dp[i]; }
    p[i]*=655;
  }      
  
  if ( xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE )
    {      
      for(i=0; i<2; i++) {
        if(m[i].dir!=d[i] || m[i].power!=p[i]) {
          m[i].dir=d[i]; 
          m[i].power=p[i];
          Low_Drive(i); 
          xLogger.vAddLogMsg("M:", i, d[i], p[1]);
        }
      }
      xSemaphoreGive( xMotorFree );
    }
}

bool Motor::GetEncDist(uint16_t *dst_enc, uint32_t *dst_dist) {  
  int ret=false;
  if ( xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE )
    {
      for(int i=0; i<2; i++) {
        if(dst_enc) dst_enc[i]=m[i].enc_accum;
        if(dst_dist) dst_dist[i]= (CHGST_TO_MM(m[i].enc_accum));
      }      
      ret = true;
      xSemaphoreGive( xMotorFree );
    }
  return ret;
}


void Motor::encInterrupt(uint16_t i)  {  
  uint8_t v=digitalRead(m[i].pin_enc);
  // separate semaphores
  //if ( xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE )
    {
      // add debounces
      if (v!=m[i].enc_prev_st) 
      {
        m[i].enc_prev_st=v;
        m[i].enc_count++;
      }     
 //     xSemaphoreGive( xMotorFree );
    }
 }

void Motor::Low_Drive(uint8_t i) 
{
  if(m[i].dir==0 || m[i].power==0) {
    pwmWrite(m[i].pin_enab, 0);
    digitalWrite(m[i].pin_1, LOW); digitalWrite(m[i].pin_2, LOW); 
    return;
  }
  else if(m[i].dir==1) {
    digitalWrite(m[i].pin_1, LOW); digitalWrite(m[i].pin_2, HIGH); 
  }
  else {
    digitalWrite(m[i].pin_1, HIGH); digitalWrite(m[i].pin_2, LOW); 
  } 
  pwmWrite(m[i].pin_enab, m[i].power==0);
} 

