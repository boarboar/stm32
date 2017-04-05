
#define CM_BUF_SIZE 64
#define CM_NVAL     2

#define CM_ID 94

class CommManager {
  public:
    void Init(uint32_t comm_speed);
    boolean ReadSerialCommand();
    void Complete();
    const char *GetBuffer();
    const char *GetDbgBuffer();
    boolean ProcessCommand();    
  protected:
    enum Regs {REG_None=0, REG_ID=1, REG_Pow=2};
    uint8_t CRC();
    int16_t ReadInt() ;        
    char buf[CM_BUF_SIZE];
    char msgdbg[CM_BUF_SIZE];
    uint8_t bytes;
    uint8_t pos;
    uint8_t verb;
    uint8_t reg;
    uint8_t vcnt;
    uint16_t val[CM_NVAL];
};





