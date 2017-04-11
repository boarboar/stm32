class Motor {  
  public:
    void Init(int m_1_1_pin, int m_1_2_pin, int m_1_enab_pin, int m_1_enc_pin, int m_2_1_pin, int m_2_2_pin, int m_2_enab_pin, int m2_enc_pin);    
    void Start();
    void Do(); 
    void SetMotors(int8_t dp1, int8_t dp2);
    bool GetEnc(int16_t *dst);
    void encInterrupt(uint16_t i);
  protected: 
    void Low_Drive(uint8_t i); 
    struct {
      uint16_t pin_1, pin_2, pin_enab, pin_enc;
      int16_t enc_count;
      uint8_t enc_prev_st;
      int8_t dir;
      uint16_t power;
    } m[2];
    xSemaphoreHandle xMotorFree;
    bool running;
    
};

