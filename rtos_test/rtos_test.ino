
#include <MapleFreeRTOS821.h>
#include <Servo.h>
#include "comm_mgr.h"
#include "log.h"

#define BOARD_LED_PIN PC13

#define SERVO_1_PIN PA8 

// need FT ports!
#define US_IN_PIN   PB13
#define US_OUT_PIN  PB12

/*
struct AMessage
{
    char ucMessageID;
    char ucData[ 20 ];
};

struct AMessage txMessage={0, "a"};
struct AMessage rxMessage={0, ""};
  
QueueHandle_t xLogQueue=NULL;
xSemaphoreHandle xLogFree=NULL;
*/

CommManager xCommMgr;
ComLogger xLogger;
Servo xServo;

/*
static void vAddLogMsg(const char *pucMsg=NULL) {  
  if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg) 
        strncpy(txMessage.ucData, pucMsg, 20);          
      xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
      xSemaphoreGive( xLogFree );
    }
}
*/

static void vSerialOutTask(void *pvParameters) {
    Serial.println("Serial Out Task started.");
    for (;;) {
      /*
      if( xQueueReceive( xLogQueue, &rxMessage, ( TickType_t ) 10 ) )
        {
         digitalWrite(BOARD_LED_PIN, HIGH);
         Serial.print((int)rxMessage.ucMessageID);
         Serial.print(" : ");
         Serial.println(rxMessage.ucData);
         vTaskDelay(100);
         digitalWrite(BOARD_LED_PIN, LOW);
        }        
        */
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
          xCommMgr.Consume();
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
          i1++;
        }
        //vTaskDelay(10000);
        //vAddLogMsg("TMR");
        char buf[32]="T: ";
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
          i1++;
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
        uint32_t t0 = millis();  
        digitalWrite(US_OUT_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(US_OUT_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(US_OUT_PIN, LOW);
        uint32_t d=pulseIn(US_IN_PIN, HIGH, 50000);
        // if d==0 ...
        int16_t dist=(int16_t)(d/58);
        uint32_t t1 = millis() - t0;
        char buf[32]="US: ";
        char bufn[8];
        itoa(dist, bufn);
        strcat(buf, bufn);
        strcat(buf, " ");
        ltoa(t1, bufn);
        strcat(buf, bufn);
        xLogger.vAddLogMsg(buf);        
    }
}

void setup() {
    //delay(5000);
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(US_OUT_PIN, OUTPUT);     
    pinMode(US_IN_PIN, INPUT); 
    Serial.begin(115200); 
    xCommMgr.Init(115200);
    xLogger.Init();

    /*
    vSemaphoreCreateBinary(xLogFree);
    xLogQueue = xQueueCreate( 10, sizeof( struct AMessage ) );

    if( xLogQueue == NULL )
    {
        // Queue was not created and must not be used. 
        Serial.println("Couldn't create LQ");
        return;
    }
*/

    Serial.println("LQ OK");

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


