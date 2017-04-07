
#include <MapleFreeRTOS821.h>
//#include <Servo.h>
#include <Wire.h>
#include "comm_mgr.h"
#include "log.h"
#include "mpu.h"
#include "sens.h"
#include "motor.h"

#define BOARD_LED_PIN PC13

// Serial MCU comm
#define SER3_TX_PIN PB10
#define SER3_RX_PIN PB11

// I2C ports fixed
#define SCL_PIN PB6
#define SDA_PIN PB7

// PWM ports (check!)
#define SERVO_1_PIN PA8 
#define MOTOR_EN_1_PIN PA1 
#define MOTOR_EN_2_PIN PA2 

// GPIO
#define MOTOR_OUT_1_1_PIN PA3
#define MOTOR_OUT_1_2_PIN PA4
#define MOTOR_OUT_2_1_PIN PA5
#define MOTOR_OUT_2_2_PIN PA6

// 5v Tolerant ports (check!)
#define MOTOR_ENC_1_PIN  PB8
#define MOTOR_ENC_2_PIN  PB9

#define US_IN_1_PIN   PB13
#define US_OUT_1_PIN  PB12

#define US_IN_2_PIN   PB4
#define US_OUT_2_PIN  PB3


CommManager xCommMgr;
ComLogger xLogger;
Sensor xSensor;
Motor xMotor;

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
    xLogger.vAddLogMsg("Lazy Task started.");
    for (;;) {       
        vTaskDelay(1000);
        float yaw=MpuDrv::Mpu.getYaw_safe(); 
        int val = yaw*180.0/PI;
        int16_t enc[2];
        char buf[32];       
        strcpy(buf, "Y: ");
        itoa_cat(val, buf);        
        if (xMotor.GetEnc(enc)) {
          strcat(buf, " C: ");
          itoa_cat(enc[0], buf);
          strcat(buf, ", ");
          itoa_cat(enc[1], buf);
        }
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
        xSensor.Start();     
        xMotor.Start();     
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
    digitalWrite(BOARD_LED_PIN, HIGH);
    pinMode(BOARD_LED_PIN, OUTPUT);
      
    Serial.begin(115200); 
    Serial.print("Tick = ");
    Serial.println(portTICK_PERIOD_MS);
    
    xLogger.Init();
    xCommMgr.Init(115200);
    xSensor.Init(US_IN_1_PIN, US_OUT_1_PIN, SERVO_1_PIN); 
    xMotor.Init(MOTOR_OUT_1_1_PIN, MOTOR_OUT_1_2_PIN, MOTOR_EN_1_PIN, MOTOR_ENC_1_PIN,
        MOTOR_OUT_2_1_PIN, MOTOR_OUT_2_2_PIN, MOTOR_EN_2_PIN, MOTOR_ENC_2_PIN); 

    Serial.println("Init Wire...");
    //Wire.begin(SCL_PIN, SDA_PIN);
    Wire.begin();
    MpuDrv::Mpu.init();
     
    Serial.println("Starting...");
    digitalWrite(BOARD_LED_PIN, LOW);
       
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
          
    xTaskCreate(vLazyTask,
                "TaskLazy",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 0, // min
                NULL);
                            
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


