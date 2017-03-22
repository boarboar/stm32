
class ComLogger {  
  public:
    void Init();    
    void vAddLogMsg(const char *pucMsg=NULL);
    void Process();
  protected:
  
  struct AMessage
  {
    char ucMessageID;
    char ucData[ 20 ];
  };
  struct AMessage txMessage;
  struct AMessage rxMessage;
  QueueHandle_t xLogQueue;
  xSemaphoreHandle xLogFree;
};


