#include <Servo.h>

class Sensor {  
  public:
    void Init(int sens_in_pin, int sens_out_pin, int servo_pin);    
    void Start();
    void DoCycle();    
    int16_t Get();
  protected: 
    int16_t sens_in_pin;
    int16_t sens_out_pin;
    int16_t value;
    int8_t sservo_pos;
    int8_t sservo_step;
    Servo xServo;
    xSemaphoreHandle xSensFree;
    bool running;
};

