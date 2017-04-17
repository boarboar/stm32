#include <Servo.h>

#define SERVO_NSTEPS  2
#define M_SENS_N      (SERVO_NSTEPS*4+2) // number of readings

class Sensor {  
  public:
    void Init(int servo_pin, int sens_in_pin_0, int sens_out_pin_0, int sens_in_pin_1, int sens_out_pin_1);    
    void Start();
    void DoCycle();    
    int16_t Get();
  protected: 
    int16_t sens_in_pin[2];
    int16_t sens_out_pin[2];
    int16_t value[M_SENS_N];
    int8_t sservo_pos;
    int8_t sservo_step;
    Servo xServo;
    xSemaphoreHandle xSensFree;
    bool running;
};

