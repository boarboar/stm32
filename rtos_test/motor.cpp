#include <MapleFreeRTOS821.h>
#include "motor.h"
#include "log.h"

extern ComLogger xLogger;

static Motor *motor_instance=NULL;  

static void encodeInterrupt_1() { 
  static portBASE_TYPE xHigherPriorityTaskWoken1;
  xHigherPriorityTaskWoken1 = pdFALSE;
  //if(Motor::instance) Motor::instance->encInterrupt(0); 
  if(motor_instance) motor_instance->encInterrupt(0); 
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken1 );
  }
  
static void encodeInterrupt_2() { 
  static portBASE_TYPE xHigherPriorityTaskWoken2;
  xHigherPriorityTaskWoken2 = pdFALSE;
  //if(Motor::instance) Motor::instance->encInterrupt(0); 
  if(motor_instance) motor_instance->encInterrupt(1); 
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken2 );
  }

void Motor::Init(int m_1_1_pin, int m_1_2_pin, int m_1_enab_pin, int m_1_enc_pin, int m_2_1_pin, int m_2_2_pin, int m_2_enab_pin, int m_2_enc_pin) {
    this->m_1_1_pin=m_1_1_pin;
    this->m_1_2_pin=m_1_2_pin;
    this->m_1_enab_pin=m_1_enab_pin;
    //this->m_1_enc_pin=m_1_enc_pin;
    this->enc_pin[0]=m_1_enc_pin;
    this->m_2_1_pin=m_2_1_pin;
    this->m_2_2_pin=m_2_2_pin;
    this->m_2_enab_pin=m_2_enab_pin;
    this->enc_pin[1]=m_2_enc_pin;
    vSemaphoreCreateBinary(xMotorFree);    
    
    pinMode(m_1_1_pin, OUTPUT);     
    pinMode(m_1_2_pin, OUTPUT);     
    pinMode(m_1_enab_pin, OUTPUT);     
    pinMode(m_2_1_pin, OUTPUT);     
    pinMode(m_2_2_pin, OUTPUT);     
    pinMode(m_2_enab_pin, OUTPUT);     
    
    pinMode(m_1_enc_pin, INPUT_PULLUP);     
    pinMode(m_2_enc_pin, INPUT_PULLUP);     

    running = false;

    //instance = this;
    motor_instance = this;
    enc_count[0]=enc_count[1]=0;
    enc_prev[0]=digitalRead(enc_pin[0]);
    enc_prev[1]=digitalRead(enc_pin[1]);
    
    attachInterrupt(enc_pin[0], encodeInterrupt_1, CHANGE);
    attachInterrupt(enc_pin[1], encodeInterrupt_2, CHANGE);
          
    Serial.println("Motor OK");
}

void Motor::Start() {
  xLogger.vAddLogMsg("Motor module run.");
  running=true;
}
  

bool Motor::GetEnc(int16_t *dst) {  
  int ret=false;
  if ( xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE )
    {
      dst[0]=enc_count[0];
      dst[1]=enc_count[1];
      ret = true;
      xSemaphoreGive( xMotorFree );
    }
  return ret;
}


void Motor::encInterrupt(uint16_t i)  {  
  uint8_t v=digitalRead(enc_pin[i]);

  if ( xSemaphoreTake( xMotorFree, ( portTickType ) 10 ) == pdTRUE )
    {
      if (v!=enc_prev[i]) {
        enc_prev[i]=v;
        enc_count[i]++;
      }     
      xSemaphoreGive( xMotorFree );
    }
 }

