class Motor;

class Motion {  
  public:
    void Init(Motor *m);    
    void Start();
    void DoCycle(float yaw); 
    void SetMotors(int8_t dp1, int8_t dp2);
    void Move(int16_t speed);
    void Steer(int16_t angle);
    void MoveBearing(int16_t angle);
  protected:     
    xSemaphoreHandle xMotionFree;
    Motor *pxMotor;
    bool bReady;
    
};

