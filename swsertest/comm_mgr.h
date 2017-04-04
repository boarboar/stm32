
#define CM_BUF_SIZE 64
#define CM_NVAL     2

class CommManager {
  public:
    void Init(uint32_t comm_speed=115200, int16_t timeout=100);
    bool Command(char *cmd);
    bool Get(uint16_t reg);
    bool Set(uint16_t reg, int16_t *va, uint16_t nval);
    
  protected:
    uint8_t CRC();
    int16_t ReadInt() ;    
    
    char buf[CM_BUF_SIZE];
    uint8_t bytes;
    uint8_t pos;
    uint16_t timeout;

    int16_t resp_val;
    uint8_t verb;
    uint8_t reg;
    uint8_t vcnt;
    uint16_t val[CM_NVAL];
   
};





