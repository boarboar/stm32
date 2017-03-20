
#include <MapleFreeRTOS821.h>
#include <Servo.h>
#include "comm_mgr.h"


#define BOARD_LED_PIN PC13

#define SERVO_1_PIN PA8 

// need FT ports!
#define US_IN_PORT PB13
#define US_OUT_PORT  PB12

struct AMessage
{
    char ucMessageID;
    char ucData[ 20 ];
};

struct AMessage txMessage={0, "a"};
struct AMessage rxMessage={0, ""};
  
QueueHandle_t xLogQueue=NULL;
xSemaphoreHandle xLogFree=NULL;

CommManager xCommMgr;
Servo xServo;

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

static void vSerialOutTask(void *pvParameters) {
    Serial.println("Serial Out started.");
    for (;;) {
      if( xQueueReceive( xLogQueue, &rxMessage, ( TickType_t ) 10 ) )
        {
         digitalWrite(BOARD_LED_PIN, HIGH);
         Serial.print((int)rxMessage.ucMessageID);
         Serial.print(" : ");
         Serial.println(rxMessage.ucData);
         vTaskDelay(100);
         digitalWrite(BOARD_LED_PIN, LOW);
        }        
    }
}

static void vCommTask(void *pvParameters) {
    vAddLogMsg("Comm Task started.");
    for (;;) {
        vTaskDelay(10);        
        if(xCommMgr.ReadSerialCommand()) {               
          vAddLogMsg(xCommMgr.GetBuffer());    
          xCommMgr.ProcessCommand();
          vAddLogMsg(xCommMgr.GetBuffer());  // response  
          //xCmd.Process(xCommMgr.GetBuffer());      
          //xCommMgr.Consume();
          //xCommMgr.Respond(xCmd.GetResponce());
        }        
    }
}

static void vTimerTask(void *pvParameters) {
    vAddLogMsg("Timer Task started.");
    for (;;) {
        vTaskDelay(10000);
        vAddLogMsg("TMR");
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
    
    vAddLogMsg("Sensor Task started.");
    for (;;) {
        vTaskDelay(1000);
        vAddLogMsg("SENS");        
        uint32_t t0 = millis();  
  
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);
  uint32_t d=pulseIn(US_IN, HIGH, 50000);
  int16_t dist=(int16_t)(d/58);
  
  uint32_t t1 = millis() - t0;
  Serial.print ("Sqrt calculations took (ms): ");
  Serial.print(t1);
  Serial.print("  Dist: ");
  Serial.println(dist);
  
    }
}

void setup() {
    //delay(5000);
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
      pinMode(US_OUT_PORT, OUTPUT);     
  pinMode(US_IN_PORT, INPUT); 
    Serial.begin(115200); 
    xCommMgr.Init(115200);
    
    vSemaphoreCreateBinary(xLogFree);
    xLogQueue = xQueueCreate( 10, sizeof( struct AMessage ) );

    if( xLogQueue == NULL )
    {
        /* Queue was not created and must not be used. */
        Serial.println("Couldn't create LQ");
        return;
    }

    Serial.println("LQ OK");

    Serial.println("Init Servo...");
    xServo.attach(SERVO_1_PIN);  // attaches the servo on pin 9 to the servo object 
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

    xTaskCreate(vTimerTask,
                "TaskTmr",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 0, // min
                NULL);
                
    xTaskCreate(vSensorTask,
                "TaskSens",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2, // mid
                NULL);
  
                                
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


