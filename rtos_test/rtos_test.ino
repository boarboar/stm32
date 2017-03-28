
#include <MapleFreeRTOS821.h>
#include <Servo.h>
#include <Wire.h>
#include "comm_mgr.h"
#include "log.h"
#include "mpu.h"

#define BOARD_LED_PIN PC13

#define SCL_PIN PB6
#define SDA_PIN PB7

#define SERVO_1_PIN PA8 

// need FT ports!
#define US_IN_PIN   PB13
#define US_OUT_PIN  PB12

CommManager xCommMgr;
ComLogger xLogger;
Servo xServo;

xSemaphoreHandle xIMUFree;

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
          xLogger.vAddLogMsg(xCommMgr.GetDbgBuffer());  // response        
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

        // TODO - test IMU reset flag

        float yaw=0.0;
        if ( xSemaphoreTake( xIMUFree, ( portTickType ) 10 ) == pdTRUE )
        {
          yaw = MpuDrv::Mpu.getYaw();  
          xSemaphoreGive( xIMUFree );
        } 

        int yaw_i = yaw*180.0/PI;
        char buf[20];
        
        strcpy(buf, "YAW: ");
        itoa_cat(yaw_i, buf);
        xLogger.vAddLogMsg(buf);  
    }
}

static void vIMU_Task(void *pvParameters) {
    TickType_t xLastWakeTime;
    int i;
    int i1, i10, i100;
    xLogger.vAddLogMsg("IMU Task started.");
    //xLastWakeTime = xTaskGetTickCount();  
    for (;;) { 
      vTaskDelay(3); 
      if ( xSemaphoreTake( xIMUFree, ( portTickType ) 10 ) == pdTRUE )
      {
        MpuDrv::Mpu.cycle(xTaskGetTickCount()-xLastWakeTime);  
        xSemaphoreGive( xIMUFree );
      }
      //xLastWakeTime = xTaskGetTickCount();  
    }
}

static void vSensorTask(void *pvParameters) {
  /*    
    xServo.write(180);
   */
    
    xLogger.vAddLogMsg("Sensor Task started.");
    for (;;) {
        vTaskDelay(10000);
        //vAddLogMsg("SENS");        
        //uint32_t t0 = millis();  
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
    }
}


void setup() {
    delay(5000);
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(US_OUT_PIN, OUTPUT);     
    pinMode(US_IN_PIN, INPUT); 
    Serial.begin(115200); 
    xLogger.Init();

    xCommMgr.Init(115200);
    
    Serial.println("Init Wire...");
    //Wire.begin(SCL_PIN, SDA_PIN);
    Wire.begin();
    MpuDrv::Mpu.init();
     
    Serial.println("Init Servo...");
    xServo.attach(SERVO_1_PIN);  // attaches the servo on pin 9 to the servo object 

    Serial.print("Tick = ");
    Serial.println(portTICK_PERIOD_MS);
    
    Serial.println("Starting...");

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


