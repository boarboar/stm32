
#include <MapleFreeRTOS821.h>
#include <Servo.h>
#include "comm_mgr.h"
#include "log.h"

#define BOARD_LED_PIN PC13

#define SERVO_1_PIN PA8 

// need FT ports!
#define US_IN_PIN   PB13
#define US_OUT_PIN  PB12

CommManager xCommMgr;
ComLogger xLogger;
Servo xServo;

static void vSerialOutTask(void *pvParameters) {
    Serial.println("Serial Out Task started.");
    for (;;) {
       xLogger.Process();
       vTaskDelay(100);
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

static void vTimerTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    int i;
    int i1, i10, i100;
    xLogger.vAddLogMsg("Timer Task started.");
    for (;;) {        
        i1=i10=i100=0;
        for(i=0; i<1000; i++) {
          xLastWakeTime = xTaskGetTickCount();  
          vTaskDelay(10);
          xLastWakeTime = xTaskGetTickCount()-xLastWakeTime;
          if(xLastWakeTime>100) i100++;
          else if(xLastWakeTime>10) i10++;
          else i1++;
        }
        //vTaskDelay(10000);
        //vAddLogMsg("TMR");
        char buf[32]="T: ";
        //char bufn[8];
        //itoa(i1, bufn);
        //strcat(buf, bufn);
        itoa(i1, buf+strlen(buf));
        strcat(buf, " ");
        //itoa(i10, bufn);
        //strcat(buf, bufn);
        itoa(i10, buf+strlen(buf));
        strcat(buf, " ");
        //itoa(i100, bufn);
        //strcat(buf, bufn);
        itoa(i10, buf+strlen(buf));
        xLogger.vAddLogMsg(buf);    
    }
}

static void vRealTimeTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    int i;
    int i1, i10, i100;
    xLogger.vAddLogMsg("Realtime Task started.");
    for (;;) {        
        i1=i10=i100=0;
        for(i=0; i<1000; i++) {
          xLastWakeTime = xTaskGetTickCount();  
          vTaskDelay(10);
          xLastWakeTime = xTaskGetTickCount()-xLastWakeTime;
          if(xLastWakeTime>100) i100++;
          else if(xLastWakeTime>10) i10++;
          else i1++;
        }
        char buf[20]="R: ";
        char bufn[8];
        itoa(i1, bufn);
        strcat(buf, bufn);
        strcat(buf, " ");
        itoa(i10, bufn);
        strcat(buf, bufn);
        strcat(buf, " ");
        itoa(i100, bufn);
        strcat(buf, bufn);
        xLogger.vAddLogMsg(buf);    
    }
}
static void vSensorTask(void *pvParameters) {
  /*
    vTaskDelay(1000);
    vAddLogMsg("SRV1");   
    xServo.write(90);
        vTaskDelay(1000);
    vAddLogMsg("SRV2");   
    xServo.write(0);
        vTaskDelay(1000);
    vAddLogMsg("SRV3");   
    xServo.write(0);
        vTaskDelay(1000);
    vAddLogMsg("SRV4");   
    xServo.write(180);
   */
    
    xLogger.vAddLogMsg("Sensor Task started.");
    for (;;) {
        vTaskDelay(1000);
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
        char bufn[8];
        itoa(dist, bufn);
        strcat(buf, bufn);
        strcat(buf, " ");
        ltoa(t, bufn);
        strcat(buf, bufn);
        xLogger.vAddLogMsg(buf);        
    }
}

void setup() {
    delay(5000);
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(US_OUT_PIN, OUTPUT);     
    pinMode(US_IN_PIN, INPUT); 
    Serial.begin(115200); 
    xCommMgr.Init(115200);
    xLogger.Init();

    Serial.println("Init Servo...");
    xServo.attach(SERVO_1_PIN);  // attaches the servo on pin 9 to the servo object 

    Serial.print("Tick = ");
    Serial.println(portTICK_PERIOD_MS);
    
    Serial.println("Starting...");
    
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
                tskIDLE_PRIORITY + 2, // mid
                NULL);
  
    
    xTaskCreate(vTimerTask,
                "TaskTmr",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 0, // min
                NULL);

                                            
    xTaskCreate(vRealTimeTask,
                "TaskRT",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3, // min
                NULL);
                
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


