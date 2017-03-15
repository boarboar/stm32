
#include <MapleFreeRTOS821.h>
#include "comm_mgr.h"

#define BOARD_LED_PIN PC13

struct AMessage
{
    char ucMessageID;
    char ucData[ 20 ];
};

struct AMessage txMessage={0, "a"};
struct AMessage rxMessage={0, ""};

CommManager xCommMgr;
  
QueueHandle_t xQueue=NULL;

static void vSerialOutTask(void *pvParameters) {
    for (;;) {
      if( xQueueReceive( xQueue, &rxMessage, ( TickType_t ) 10 ) )
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
        //vTaskDelay(1000);
        if(xCommMgr.ReadSerialCommand()) {        
          txMessage.ucMessageID++;
          xCommMgr.ReadBuffer(txMessage.ucData, 20);
          //txMessage.ucData[0]++;
          xQueueSendToBack( xQueue, ( void * ) &txMessage, ( TickType_t ) 0 );
        }
    }
}

void setup() {
    delay(5000);
    pinMode(BOARD_LED_PIN, OUTPUT);
    Serial.begin(115200); 
    xCommMgr.Init(115200);
    //Serial3.begin(115200); 

    xQueue = xQueueCreate( 10, sizeof( struct AMessage ) );

    if( xQueue == NULL )
    {
        /* Queue was not created and must not be used. */
        Serial.println("Couldn't create Q");
        return;
    }

    Serial.println("Q OK");

    xTaskCreate(vSerialOutTask,
                "Task1",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL);

    xTaskCreate(vCommTask,
                "Task2",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2, // high
                NULL);
                            
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


