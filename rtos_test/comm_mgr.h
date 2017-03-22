
#define CM_BUF_SIZE 64
#define CM_NVAL     2

class CommManager {
  public:
    void Init(uint32_t comm_speed);
    boolean ReadSerialCommand();
    /*
    void StartParse(){ pos=0; }
    void Reset(){ bytes=0; buf[0]=0;}
    boolean Match(const char *cmd);
    //boolean ReadInt(int16_t *val) ;
    char ReadChar() ;
    */
    void Consume();
    const char *GetBuffer();
    const char *GetDbgBuffer();
    boolean ProcessCommand();
    int16_t ReadInt() ;    
  protected:
    char buf[CM_BUF_SIZE];
    char rsp[CM_BUF_SIZE];
    char bufn[16];
    uint8_t bytes;
    uint8_t pos;
    uint8_t verb;
    uint8_t reg;
    uint8_t vcnt;
    uint16_t val[CM_NVAL];
};

void itoa(int n, char s[]);
void ltoa(int32_t n, char s[]);


