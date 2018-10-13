#include "OmniDanaCommon.h"
#include "ctrlTask.h"

#define MAX_PAYLOAD_LENGTH  16
#define HEADER_LEN          6
#define FOOTER_LEN          4
#define MAX_BUF_LEN         (HEADER_LEN+MAX_PAYLOAD_LENGTH+FOOTER_LEN)

void ctrlTaskInitialize(MessageBufferHandle_t msgBuf);

void ctrlTaskInitialize(MessageBufferHandle_t msgBuf)
{
  xTaskCreate(
    ctrlTask
    ,  (const portCHAR *)"ctrlTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)msgBuf
    ,  2  // priority
    ,  NULL );
}



void ctrlTask( void *pvParameters )
{
  MessageBufferHandle_t msgBuf = (MessageBufferHandle_t)pvParameters;

  Serial.print("ctrlTask: msgBuf=");
  Serial.print((unsigned int)msgBuf, HEX);
  Serial.println(".");

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
