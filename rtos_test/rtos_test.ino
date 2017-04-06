
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

#define MOTOR_EN_1_PIN PA1
#define MOTOR_EN_2_PIN PA2

// need FT ports!
#define ENC1_IN_PIN  PB8
#define ENC2_IN_PIN  PB9

#define US_IN_1_PIN   PB13
#define US_OUT_1_PIN  PB12

#define US_IN_2_PIN   PB4
#define US_OUT_2_PIN  PB3


CommManager xCommMgr;
ComLogger xLogger;
Sensor xSensor;

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
    //TickType_t xLastWakeTime;
    //int i;
    //int i1, i10, i100;
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
      /*
        if(MpuDrv::Mpu.isNeedReset())  {
            MpuDrv::Mpu.init();
            mpu_rst=true;
          } else 
      */
      
        float yaw=MpuDrv::Mpu.getYaw_safe(); 
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
    for (;;) { 
      vTaskDelay(3); 
      mpu_res = MpuDrv::Mpu.cycle_safe();       
      if(mpu_res==2) {
        // IMU settled
        xLogger.vAddLogMsg("Activate motion!");
        // TODO
      }
    }
}

static void vSensorTask(void *pvParameters) {
    xLogger.vAddLogMsg("Sensor Task started.");
    for (;;) {
        vTaskDelay(1000);
        xSensor.Do();     
    }
}


static void vMotionTask(void *pvParameters) {
    xLogger.vAddLogMsg("Motion Task started.");
    for (;;) { 
      vTaskDelay(50); 
      MpuDrv::Mpu.process_safe();     
      MpuDrv::Mpu.flushAlarms();
    }
}


void setup() {
    delay(5000);
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    
    Serial.begin(115200); 
    xLogger.Init();
    xCommMgr.Init(115200);
    xSensor.Init(US_IN_1_PIN, US_OUT_1_PIN, SERVO_1_PIN); 

    Serial.println("Init Wire...");
    //Wire.begin(SCL_PIN, SDA_PIN);
    Wire.begin();
    MpuDrv::Mpu.init();
     
    Serial.print("Tick = ");
    Serial.println(portTICK_PERIOD_MS);
    
    Serial.println("Starting...");

    pinMode(ENC1_IN_PIN, INPUT_PULLUP);
    enc_prev=digitalRead(ENC1_IN_PIN);
    attachInterrupt(ENC1_IN_PIN, vEncoderISR, CHANGE);
  
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


