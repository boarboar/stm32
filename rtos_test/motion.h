class Motor;

class Motion {  
  public:
    void Init(Motor *m);    
    void Start();
    void Reset();
    void DoCycle(float yaw); 
    void SetMotors(int8_t dp1, int8_t dp2);
    void Move(int16_t speed);
    void Steer(int16_t angle);
    void MoveBearing(int16_t angle);
    bool GetAdvance(uint32_t *dst_dist);
  protected:     
    xSemaphoreHandle xMotionFree;
    TickType_t xRunTime;
    Motor *pxMotor;
    bool bReady;
    int16_t iTargBearing; // in grads
    int16_t iTargSpeed; //mm_s
    int16_t iTargRot; //mm_s ??
    uint32_t lPIDCnt;
    int16_t err_bearing_p_0, err_bearing_i;
    int16_t delta_pow;
    int16_t base_pow;
    uint32_t lAdvance0[2], lAdvance[2]; // in mm
    float fCrd[2];
};

