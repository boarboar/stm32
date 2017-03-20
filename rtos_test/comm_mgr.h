
#define BUF_SIZE 64

class CommManager {
  public:
    void Init(uint32_t comm_speed);
    boolean ReadSerialCommand();
    /*
    void StartParse(){ pos=0; }
    void Reset(){ bytes=0; buf[0]=0;}
    boolean Match(const char *cmd);
    //boolean ReadInt(int16_t *val) ;
    int16_t ReadInt() ;
    char ReadChar() ;
    */
    //void Consume(char *pcBuf, uint16_t uLen);
    const char *GetBuffer();
    boolean ProcessCommand();
    //void Consume();    
    //void Respond(const char *rsp);    
  protected:
    char buf[BUF_SIZE];
    uint8_t bytes;
    uint8_t pos;
};

