
#define CM_BUF_SIZE 64
#define CM_NVAL     2

class CommManager {
  public:
    void Init(uint32_t comm_speed=115200, int16_t timeout=100);
    int Command(char *cmd);
    int Get(uint16_t reg);
    int Set(uint16_t reg, int16_t *va, uint16_t nval);
    int GetResultCnt();
    const int16_t *GetResultVal();
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
    int16_t val[CM_NVAL];
   
};





