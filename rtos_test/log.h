
#define CLOG_MSG_SZ 32
#define CLOG_Q_SZ 16

class ComLogger {  
  public:
    void Init();    
    void vAddLogMsg(const char *pucMsg=NULL);
    void Process();
  protected:
  
  struct AMessage
  {
    char ucMessageID;
    char ucData[ CLOG_MSG_SZ ];
  };
  struct AMessage txMessage;
  struct AMessage rxMessage;
  QueueHandle_t xLogQueue;
  xSemaphoreHandle xLogFree;
};

void itoa(int n, char s[]);
void ltoa(int32_t n, char s[]);

inline void itoa_cat(int n, char s[]) { itoa(n, s+strlen(s)); }
inline void ltoa_cat(int n, char s[]) { ltoa(n, s+strlen(s)); }
