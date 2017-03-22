#include <MapleFreeRTOS821.h>
#include "log.h"

void ComLogger::Init() {
    vSemaphoreCreateBinary(xLogFree);
    xLogQueue = xQueueCreate( 10, sizeof( struct AMessage ) );

    if( xLogQueue == NULL )
    {
        /* Queue was not created and must not be used. */
        Serial.println("Couldn't create LQ");
        return;
    }

    txMessage.ucMessageID=0;
    txMessage.ucData[0]=0;
    
    Serial.println("Logger OK");
}

void ComLogger::vAddLogMsg(const char *pucMsg) {  
  if ( xSemaphoreTake( xLogFree, ( portTickType ) 10 ) == pdTRUE )
    {
      txMessage.ucMessageID++;     
      if(pucMsg) 
        strncpy(txMessage.ucData, pucMsg, 20);          
      xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
      xSemaphoreGive( xLogFree );
    }
}

void ComLogger::Process() {  
  if( xQueueReceive( xLogQueue, &rxMessage, ( TickType_t ) 10 ) )
  {
    Serial.print((int)rxMessage.ucMessageID);
    Serial.print(" : ");
    Serial.println(rxMessage.ucData);
    vTaskDelay(100);         
   }        
}

