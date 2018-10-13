#include "OmniDanaCommon.h"
#include "ControlTask.h"

#define MAX_PAYLOAD_LENGTH  10
#define HEADER_LEN          6
#define FOOTER_LEN          4
#define MAX_BUF_LEN         (HEADER_LEN+MAX_PAYLOAD_LENGTH+FOOTER_LEN)

void ControlInitialize(MessageBufferHandle msgBuf)
{
  xTaskCreate(
    ControlTask
    ,  (const portCHAR *)"ControlTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)msgBuf
    ,  2  // priority
    ,  NULL );
}



void ControlTask( void *pvParameters )
{
  MessageBufferHandle msgBuf = (MessageBufferHandle)pvParameters;

  Serial.print("ControlTask: msgBuf=");
  Serial.print((unsigned int)msgBuf, HEX);
  Serial.println(".");

  while(1)
  {
    uint8_t msg[20];
    size_t receivedBytes;

    receivedBytes = xMessageBufferReceive(msgBuf, (void*)msg, sizeof(msg), portMAX_DELAY);

    if(receivedBytes > 0 )
    {
      DumpMessage("ControlTask: Received ", " bytes: ", msg, receivedBytes);
    }
  }
}
