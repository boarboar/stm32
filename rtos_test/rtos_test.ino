
#include <MapleFreeRTOS821.h>
#include <Servo.h>
#include "comm_mgr.h"

#define BOARD_LED_PIN PC13

#define SERVO_1_PIN PB12 //???

Servo xServo;

struct AMessage
{
    char ucMessageID;
    char ucData[ 20 ];
};

struct AMessage txMessage={0, "a"};
struct AMessage rxMessage={0, ""};

CommManager xCommMgr;
  
QueueHandle_t xLogQueue=NULL;
xSemaphoreHandle xLogFree=NULL;

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
    for (;;) {
        vTaskDelay(10);        
        if(xCommMgr.ReadSerialCommand()) {               
          //xCommMgr.Consume(txMessage.ucData, 20);
          //Serial3.println("sndq");
          //txMessage.ucMessageID++;          
          //xLogQueueSendToBack( xQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
          vAddLogMsg(xCommMgr.GetBuffer());          
          xCommMgr.Consume();
        }
        
    }
}

static void vTimerTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(10000);
        //txMessage.ucMessageID++;     
        //strcpy(txMessage.ucData, "TMR");
        vAddLogMsg("TMR");
        //xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );      
    }
}

static void vServoTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(1000);
        vAddLogMsg("SRV");        
    }
}

void setup() {
    delay(5000);
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);
    Serial.begin(115200); 
    xCommMgr.Init(115200);
    //Serial3.begin(115200); 

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
    delay(500);
    xServo.write(90);
    delay(500);
    xServo.write(0);
    delay(500);
    xServo.write(180);
    delay(500);
    xServo.write(90);
    
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

    xTaskCreate(vServoTask,
                "TaskSrv",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2, // mid
                NULL);
                                
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


