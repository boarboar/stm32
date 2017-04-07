class Motor {  
  public:
    void Init(int m_1_1_pin, int m_1_2_pin, int m_1_enab_pin, int m_1_enc_pin, int m_2_1_pin, int m_2_2_pin, int m_2_enab_pin, int m2_enc_pin);    
    void Start();
    bool GetEnc(int16_t *dst);
    //static Motor *instance;  
    void encInterrupt(uint16_t i);
  protected: 
    int m_1_1_pin, m_1_2_pin, m_1_enab_pin;
    int m_2_1_pin, m_2_2_pin, m_2_enab_pin;
    
    int16_t enc_pin[2];
    int16_t enc_count[2];
    int8_t enc_prev[2];
    xSemaphoreHandle xMotorFree;
    bool running;
    
};

