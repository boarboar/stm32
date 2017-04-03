
#define CM_BUF_SIZE 64
#define CM_NVAL     2

class CommManager {
  public:
    void Init(uint32_t comm_speed=115200);
    bool Command(char *cmd, int16_t timeout=100);
    
  protected:
    uint8_t CRC();
    int16_t ReadInt() ;    
    
    char buf[CM_BUF_SIZE];

    uint8_t bytes;
    uint8_t pos;
};





