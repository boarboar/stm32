
#include <MapleFreeRTOS821.h>
//#include <Servo.h>
#include <Wire.h>
#include "comm_mgr.h"
#include "log.h"
#include "mpu.h"
#include "sens.h"

#define BOARD_LED_PIN PC13

#define SER3_TX_PIN PB10
#define SER3_RX_PIN PB11

#define SCL_PIN PB6
#define SDA_PIN PB7

#define SERVO_1_PIN PA8 //PWM

// need FT ports!
#define US_IN_PIN   PB13
#define US_OUT_PIN  PB12

#define ENC1_IN_PIN  PB8
#define ENC2_IN_PIN  PB9

#define MOTOR_EN_1_PIN PA1
#define MOTOR_EN_2_PIN PA2

CommManager xCommMgr;
ComLogger xLogger;
//Servo xServo;

Sensor xSensor;

xSemaphoreHandle xIMUFree;

int enc_count=0;
uint8_t enc_prev;

void vEncoderISR(void)  {  
  /* Declared static to minimize stack use. */
  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  uint8_t v=digitalRead(ENC1_IN_PIN);
  if (v==enc_prev) return;
  enc_prev=v;

       // should be locked!
 
  enc_count++;  

  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
 }

static void vSerialOutTask(void *pvParameters) {
    Serial.println("Serial Out Task started.");
    for (;;) {
       xLogger.Process();
       vTaskDelay(20);
    }
}

static void vCommTask(void *pvParameters) {
    xLogger.vAddLogMsg("Comm Task started.");
    for (;;) {
        vTaskDelay(10);        
        if(xCommMgr.ReadSerialCommand()) {       
          digitalWrite(BOARD_LED_PIN, HIGH);        
          xLogger.vAddLogMsg(xCommMgr.GetBuffer());    
          xCommMgr.ProcessCommand();
          xLogger.vAddLogMsg(xCommMgr.GetDbgBuffer());  // debug        
          xCommMgr.Complete();
          //xCommMgr.Respond(xCmd.GetResponce());          
          digitalWrite(BOARD_LED_PIN, LOW);
        }        
    }
}

static void vLazyTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    int i;
    int i1, i10, i100;
    xLogger.vAddLogMsg("Lazy Task started.");
    for (;;) {       
      vTaskDelay(1000);
      /*
        i1=i10=i100=0;
        for(i=0; i<1000; i++) {
          xLastWakeTime = xTaskGetTickCount();  
          vTaskDelay(10);
          xLastWakeTime = xTaskGetTickCount()-xLastWakeTime;
          if(xLastWakeTime>100) i100++;
          else if(xLastWakeTime>10) i10++;
          else i1++;
        }
        char buf[32]="T: ";        
        itoa_cat(i1, buf); strcat(buf, " ");
        itoa_cat(i10, buf); strcat(buf, " ");
        itoa_cat(i100, buf);
        xLogger.vAddLogMsg(buf);    
*/

        float yaw=0.0;
        bool mpu_rst=false;
        if ( xSemaphoreTake( xIMUFree, ( portTickType ) 10 ) == pdTRUE )
        {
          if(MpuDrv::Mpu.isNeedReset())  {
            MpuDrv::Mpu.init();
            mpu_rst=true;
          } else yaw = MpuDrv::Mpu.getYaw(); 
          xSemaphoreGive( xIMUFree );
        } 
        
        int val = yaw*180.0/PI;
        char buf[32];
        
        strcpy(buf, "Y: ");
        itoa_cat(val, buf);
        strcat(buf, " C: ");
        itoa_cat(enc_count, buf);
        strcat(buf, " U: ");
        val=xSensor.Get();
        itoa_cat(val, buf);
        
        xLogger.vAddLogMsg(buf);  
    }
}

static void vIMU_Task(void *pvParameters) {
    int16_t mpu_res;    
    xLogger.vAddLogMsg("IMU Task started.");
    MpuDrv::Mpu.init();
    TickType_t xLastWakeTime=xTaskGetTickCount();
    for (;;) { 
      vTaskDelay(3); 
      mpu_res=-20;
      if ( xSemaphoreTake( xIMUFree, ( portTickType ) 10 ) == pdTRUE )
      {
        int16_t mpu_res = MpuDrv::Mpu.cycle(xTaskGetTickCount()-xLastWakeTime);  
        xLastWakeTime=xTaskGetTickCount();
        xSemaphoreGive( xIMUFree );
      }      
      if(mpu_res==2) {
        // IMU settled
        xLogger.vAddLogMsg("Activate motion!");
        // TODO
      }
    }
}

static void vSensorTask(void *pvParameters) {
  /*    
    xServo.write(180);
   */
    
    xLogger.vAddLogMsg("Sensor Task started.");
    for (;;) {
        vTaskDelay(1000);
        xSensor.Do();
        /*
        // move it to its class
        TickType_t t=xTaskGetTickCount();
        digitalWrite(US_OUT_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(US_OUT_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(US_OUT_PIN, LOW);
        uint32_t d=pulseIn(US_IN_PIN, HIGH, 50000);
        // if d==0 ...
        int16_t dist=(int16_t)(d/58);
        t = xTaskGetTickCount() - t;
        char buf[32]="US: ";
        itoa_cat(dist, buf);
        strcat(buf, " ");
        itoa_cat((int)t, buf);
        xLogger.vAddLogMsg(buf);        
        */
    }
}


static void vMotionTask(void *pvParameters) {
    xLogger.vAddLogMsg("Motion Task started.");
    for (;;) { 
      vTaskDelay(50); 
      if ( xSemaphoreTake( xIMUFree, ( portTickType ) 10 ) == pdTRUE )
      {
        MpuDrv::Mpu.process();  
        xSemaphoreGive( xIMUFree );
      }    
      MpuDrv::Mpu.flushAlarms();
    }
}


void setup() {
    delay(5000);
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    
    //pinMode(US_OUT_PIN, OUTPUT);     
    //pinMode(US_IN_PIN, INPUT); 
    Serial.begin(115200); 
    xLogger.Init();
    xCommMgr.Init(115200);
    xSensor.Init(US_IN_PIN, US_OUT_PIN, SERVO_1_PIN); 

    Serial.println("Init Wire...");
    //Wire.begin(SCL_PIN, SDA_PIN);
    Wire.begin();
    //MpuDrv::Mpu.init();
     
    //Serial.println("Init Servo...");
    //xServo.attach(SERVO_1_PIN);  // attaches the servo on pin 9 to the servo object 

    Serial.print("Tick = ");
    Serial.println(portTICK_PERIOD_MS);
    
    Serial.println("Starting...");

    pinMode(ENC1_IN_PIN, INPUT_PULLUP);
    enc_prev=digitalRead(ENC1_IN_PIN);
    attachInterrupt(ENC1_IN_PIN, vEncoderISR, CHANGE);
  
    vSemaphoreCreateBinary(xIMUFree);
    
    xTaskCreate(vSerialOutTask,
                "TaskSO",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL);

    xTaskCreate(vCommTask,
                "TaskCom",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL);
                
    xTaskCreate(vSensorTask,
                "TaskSens",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3, // max
                NULL);
  
    
    xTaskCreate(vLazyTask,
                "TaskLazy",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 0, // min
                NULL);

                                            
    xTaskCreate(vIMU_Task,
                "TaskIMU",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3, // max
                NULL);

    xTaskCreate(vMotionTask,
                "TaskMotion",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2, // med
                NULL);
                            
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


