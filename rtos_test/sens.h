#include <Servo.h>

class Sensor {  
  public:
    void Init(int sens_in_pin, int sens_out_pin, int servo_pin);    
    void Start();
    void Do();    
    int16_t Get();
  protected: 
    int sens_in_pin;
    int sens_out_pin;
    int value;
    Servo xServo;
    xSemaphoreHandle xSensFree;
    bool running;
};

