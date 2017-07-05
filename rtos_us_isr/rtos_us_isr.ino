
#include <MapleFreeRTOS821.h>

#define BOARD_LED_PIN PC13

#define US_IN_PIN   PB12
#define US_OUT_PIN  PB13

volatile uint32_t di=0;
static TaskHandle_t xTaskToNotify = NULL;


static void intr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  static uint32_t t0=0;
  uint16_t v=digitalRead(US_IN_PIN);
  if(v==HIGH) {
    t0=micros();
    di=0;
  } else {
    di=micros()-t0;
    
    /* At this point xTaskToNotify should not be NULL as a transmission was
    in progress. */
    //configASSERT( xTaskToNotify != NULL );

    /* Notify the task that the transmission is complete. */
    if(xTaskToNotify != NULL )
      vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );

    /* There are no transmissions in progress, so no tasks to notify. */
    xTaskToNotify = NULL;

  }

     /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
 
}

static void vCalcTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(1);
       uint32_t x=0;
        for (uint32_t n = 247583650 ; n > 247400000 ; n--) {
            x += sqrt (n);
        }
       
    }
}

static void vLEDFlashTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(500);
        digitalWrite(BOARD_LED_PIN, LOW);
        vTaskDelay(5000);
        digitalWrite(BOARD_LED_PIN, HIGH);
    }
}

static void vUSTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(1000);

        Serial.println("Test...");
        vTaskDelay(500);
        
        di=0;
        taskENTER_CRITICAL();
        digitalWrite(US_OUT_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(US_OUT_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(US_OUT_PIN, LOW);
        taskEXIT_CRITICAL();
        //uint32_t d=pulseIn(US_IN_PIN, HIGH, 50000);
        // if d==0 ...
        //int16_t dist=(int16_t)(d/58);

        //Serial.println(dist);
        //Serial.print("D="); Serial.print(d);
        //vTaskDelay(40); //!!! so far should be enough

        /* At this point xTaskToNotify should be NULL as no transmission
    is in progress.  A mutex can be used to guard access to the
    peripheral if necessary. */
    //configASSERT( xTaskToNotify == NULL );

    /* Store the handle of the calling task. */
    xTaskToNotify = xTaskGetCurrentTaskHandle();

     // exit_critical here ???

        uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS( 40 ) );

        if( ulNotificationValue == 1 ) {
          Serial.print("D = "); Serial.print(di), Serial.print("  cm = "); Serial.println(di/58);
        }
        else
        {
           xTaskToNotify = NULL;
          Serial.print("Timeout in "); Serial.println( pdMS_TO_TICKS( 40 ) );
        }
             
    }
}

void setup() {
    delay(5000);    
    // initialize the digital pin as an output:
    pinMode(BOARD_LED_PIN, OUTPUT);
    Serial.begin(115200); 
    pinMode(US_OUT_PIN, OUTPUT);     
    pinMode(US_IN_PIN, INPUT); 
    attachInterrupt(US_IN_PIN, intr,  CHANGE);

    Serial.println("Start...");
    
    xTaskCreate(vLEDFlashTask,
                "Task1",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    xTaskCreate(vUSTask,
                "Task2",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    xTaskCreate(vCalcTask,
                "Task3",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3,
                NULL);                        
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


