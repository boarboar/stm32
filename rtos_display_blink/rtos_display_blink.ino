#define USE_SEMAPHORE_DMA1
#include <MapleFreeRTOS821.h>
#include <SPI.h>
#include <Adafruit_GFX_AS.h>    // Core graphics library, with extra fonts.
#include <Adafruit_ILI9341_STM.h> // STM32 DMA Hardware-specific library

//#include <stdlib.h>

#define BOARD_LED_PIN PC13

// need FT ports!
#define US_IN PB13
#define US_OUT  PB12

#define cs   PA3
#define dc   PA1
#define rst  PA2

#define WHITE 0xFFFF
#define BLACK 0x0000
#define RED 0xF800

void itoa(int n, char s[]);
void ltoa(int32_t n, char s[]);
 
//TFT_ILI9163C tft = TFT_ILI9163C(__CS, __DC, __RST);
Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(cs, dc, rst);       // Invoke custom library

xSemaphoreHandle xDisplayFree;


const float sin_d[] = {
  0, 0.17, 0.34, 0.5, 0.64, 0.77, 0.87, 0.94, 0.98, 1, 0.98, 0.94,
  0.87, 0.77, 0.64, 0.5, 0.34, 0.17, 0, -0.17, -0.34, -0.5, -0.64,
  -0.77, -0.87, -0.94, -0.98, -1, -0.98, -0.94, -0.87, -0.77,
  -0.64, -0.5, -0.34, -0.17
};
const float cos_d[] = {
  1, 0.98, 0.94, 0.87, 0.77, 0.64, 0.5, 0.34, 0.17, 0, -0.17, -0.34,
  -0.5, -0.64, -0.77, -0.87, -0.94, -0.98, -1, -0.98, -0.94, -0.87,
  -0.77, -0.64, -0.5, -0.34, -0.17, 0, 0.17, 0.34, 0.5, 0.64, 0.77,
  0.87, 0.94, 0.98
};
const float d = 5;
float cube1_px[] = {
  -d,  d,  d, -d, -d,  d,  d, -d
};
float cube1_py[] = {
  -d, -d,  d,  d, -d, -d,  d,  d
};
float cube1_pz[] = {
  -d, -d, -d, -d,  d,  d,  d,  d
};

float cube1_p2x[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
float cube1_p2y[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

int cube1_r[] = {
  0, 0, 0
};
const float d2 = 10;
float cube2_px[] = {
  -d2,  d2,  d2, -d2, -d2,  d2,  d2, -d2
};
float cube2_py[] = {
  -d2, -d2,  d2,  d2, -d2, -d2,  d2,  d2
};
float cube2_pz[] = {
  -d2, -d2, -d2, -d2,  d2,  d2,  d2,  d2
};

float cube2_p2x[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
float cube2_p2y[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

int cube2_r[] = {
  0, 0, 0
};

uint16 cube1_x, cube1_y, cube2_x, cube2_y, cube1_color, cube2_color;

static void vLEDFlashTask(void *pvParameters) {
  for (;;) {
    vTaskDelay(1000);
    digitalWrite(BOARD_LED_PIN, HIGH);
    vTaskDelay(50);
    digitalWrite(BOARD_LED_PIN, LOW);
  }
}

static void vCube1LoopTask(void *pvParameters) {
  while (1) {
    if ( xSemaphoreTake( xDisplayFree, ( portTickType ) 10 ) == pdTRUE )
    {
      cube(cube1_px, cube1_py, cube1_pz, cube1_p2x, cube1_p2y, cube1_r, &cube1_x, &cube1_y, &cube1_color);
      xSemaphoreGive( xDisplayFree );
        vTaskDelay(15);
    }
  }
}

static void vCube2LoopTask(void *pvParameters) {
  while (1) {
    if ( xSemaphoreTake( xDisplayFree, ( portTickType ) 10 ) == pdTRUE )
    {
      cube(cube2_px, cube2_py, cube2_pz, cube2_p2x, cube2_p2y, cube2_r, &cube2_x, &cube2_y, &cube2_color);
      xSemaphoreGive( xDisplayFree );
        vTaskDelay(40);
    }
  }
}


void cube(float *px, float *py, float *pz, float *p2x, float *p2y, int *r, uint16 *x, uint16 *y, uint16 *color) {

  for (int i = 0; i < 3; i++) {
    tft.drawLine(p2x[i], p2y[i], p2x[i + 1], p2y[i + 1], WHITE);
    tft.drawLine(p2x[i + 4], p2y[i + 4], p2x[i + 5], p2y[i + 5], WHITE);
    tft.drawLine(p2x[i], p2y[i], p2x[i + 4], p2y[i + 4], WHITE);
  }
  tft.drawLine(p2x[3], p2y[3], p2x[0], p2y[0], WHITE);
  tft.drawLine(p2x[7], p2y[7], p2x[4], p2y[4], WHITE);
  tft.drawLine(p2x[3], p2y[3], p2x[7], p2y[7], WHITE);

  r[0] = r[0] + 1;
  r[1] = r[1] + 1;
  if (r[0] == 36) r[0] = 0;
  if (r[1] == 36) r[1] = 0;
  if (r[2] == 36) r[2] = 0;
  for (int i = 0; i < 8; i++)
  {
    float px2 = px[i];
    float py2 = cos_d[r[0]] * py[i] - sin_d[r[0]] * pz[i];
    float pz2 = sin_d[r[0]] * py[i] + cos_d[r[0]] * pz[i];

    float px3 = cos_d[r[1]] * px2 + sin_d[r[1]] * pz2;
    float py3 = py2;
    float pz3 = -sin_d[r[1]] * px2 + cos_d[r[1]] * pz2;

    float ax = cos_d[r[2]] * px3 - sin_d[r[2]] * py3;
    float ay = sin_d[r[2]] * px3 + cos_d[r[2]] * py3;
    float az = pz3 - 190;

    p2x[i] = *x + ax * 500 / az;
    p2y[i] = *y + ay * 500 / az;
  }

  for (int i = 0; i < 3; i++) {
    tft.drawLine(p2x[i], p2y[i], p2x[i + 1], p2y[i + 1], *color);
    tft.drawLine(p2x[i + 4], p2y[i + 4], p2x[i + 5], p2y[i + 5], *color);
    tft.drawLine(p2x[i], p2y[i], p2x[i + 4], p2y[i + 4], *color);
  }
  tft.drawLine(p2x[3], p2y[3], p2x[0], p2y[0], *color);
  tft.drawLine(p2x[7], p2y[7], p2x[4], p2y[4], *color);
  tft.drawLine(p2x[3], p2y[3], p2x[7], p2y[7], *color);
}

static void vSqrtTask(void *pvParameters) {
  while (1){
  if ( xSemaphoreTake( xDisplayFree, ( portTickType ) 10 ) == pdTRUE ) {  
    tft.fillRect(20, 20, 200, 32, WHITE); 
    tft.drawString("Calculating...",20,20,4);
    xSemaphoreGive( xDisplayFree );
  }
  Serial.println ("Starting Sqrt calculations...");
  uint16 x = 0;
  //uint16 ixx[1001];
  // Library Sqrt
  uint32_t t0 = millis();
  /*
  for (uint32_t n = 247583650 ; n > 247400000 ; n--) {
    x = sqrt (n);
  }
  */
  vTaskDelay(1000);
  uint32_t t1 = millis() - t0;
  Serial.print ("Sqrt calculations took (ms): ");
  Serial.println (t1);

  char buf[16];
  ltoa(t1, buf);
    
  if ( xSemaphoreTake( xDisplayFree, ( portTickType ) 10 ) == pdTRUE ) {  
    tft.fillRect(20, 20, 200, 32, WHITE); 
    //tft.drawString("Ready",20,20,4);
    tft.drawString(buf,20,20,4);
    xSemaphoreGive( xDisplayFree );
  }
  //delay (5000);
  vTaskDelay(5000);

  
  }
}

static void vUltraSensTask(void *pvParameters) {
  while (1){
  if ( xSemaphoreTake( xDisplayFree, ( portTickType ) 10 ) == pdTRUE ) {  
    tft.fillRect(20, 20, 200, 32, WHITE); 
    tft.drawString("Measuring...",20,20,4);
    xSemaphoreGive( xDisplayFree );
  }
  Serial.println ("Starting measurement...");
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

  
  char buf[16];
  itoa(dist, buf);

    
  if ( xSemaphoreTake( xDisplayFree, ( portTickType ) 10 ) == pdTRUE ) {  
    tft.fillRect(20, 20, 200, 32, WHITE); 
    //tft.drawString("Ready",20,20,4);
    tft.drawString(buf,20,20,4);
    xSemaphoreGive( xDisplayFree );
  }
  vTaskDelay(500);

  
  }
}

void setup() {
  // initialize the digital pin as an output:
  Serial.begin(9600);
  delay (2000);
  Serial.println ("Running...");
  pinMode(BOARD_LED_PIN, OUTPUT);

  pinMode(US_OUT, OUTPUT);     
  pinMode(US_IN, INPUT);       
  
  tft.begin();
  tft.fillScreen(WHITE);
  tft.setTextColor(BLACK);  
  tft.setTextSize(1);
  tft.setTextColor(BLACK, WHITE);
  tft.drawString("Starting...",20,20,4);
  delay(1000);
  cube1_x = ((tft.width()) / 4);
  cube1_y = ((tft.height()) / 4);
  cube2_x = ((tft.width()) / 2);
  cube2_y = ((tft.height()) / 2);
  cube1_color = BLACK;
  cube2_color = RED;
  vSemaphoreCreateBinary(xDisplayFree);
  xTaskCreate(vLEDFlashTask,
              "Task1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  xTaskCreate(vCube1LoopTask,
              "Cube1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  xTaskCreate(vCube2LoopTask,
              "Cube2",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY+1,
              NULL);
  //
  xTaskCreate(vUltraSensTask,
              "US",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY,
              NULL);
  vTaskStartScheduler();
}

void loop() {
  // Do not write any code here, it would not execute.
}


/* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}  

/* itoa:  convert n to characters in s */
 void itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  


/* itoa:  convert n to characters in s */
 void ltoa(int32_t n, char s[])
 {
     int32_t i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  

