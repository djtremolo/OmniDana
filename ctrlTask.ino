#include "OmniDanaCommon.h"
#include "ctrlTask.h"
#include "ioInterface.h"


#define MAX_PAYLOAD_LENGTH  16
#define HEADER_LEN          6
#define FOOTER_LEN          4
#define MAX_BUF_LEN         (HEADER_LEN+MAX_PAYLOAD_LENGTH+FOOTER_LEN)

#define MAX_STEPS_IN_SEQUENCE   20



typedef enum 
{
  PRESS_SHORT,
  PRESS_MEDIUM,
  PRESS_LONG
} buttonPress_t;


typedef struct
{
  buttonKey_t button;
  buttonPress_t pressLength;  
  uint8_t repeat;
  bool waitForFeedback;
} sequenceStep_t;

sequenceStep_t keySequence[MAX_STEPS_IN_SEQUENCE];
uint8_t keySequenceStepCount;



void ctrlTaskInitialize(MessageBufferHandle_t msgBuf, uint8_t priority);

void ctrlTaskInitialize(MessageBufferHandle_t msgBuf, uint8_t priority)
{


  xTaskCreate(
    ctrlTask
    ,  (const portCHAR *)"ctrlTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)msgBuf
    ,  priority  // priority
    ,  NULL );
}



void ctrlTask( void *pvParameters )
{
  MessageBufferHandle_t msgBuf = (MessageBufferHandle_t)pvParameters;

  IoInterfaceSetupPins();

  while(1)
  {
    uint8_t msg[MAX_BUF_LEN];
    size_t receivedBytes;

    receivedBytes = xMessageBufferReceive(msgBuf, (void*)msg, sizeof(msg), portMAX_DELAY);

    if(receivedBytes > 0 )
    { 
      Serial.print("ctrlTask received ");
      Serial.print(receivedBytes, DEC);
      Serial.println(" bytes.");
    }
  }
}
