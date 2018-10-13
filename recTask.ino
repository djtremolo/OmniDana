#include "OmniDanaCommon.h"

#include "recTask.h"

#define POLLING_TIME_MS     1000



void ReceiverTask( void *pvParameters );
void ReceiverInitialize(MessageBufferHandle_t msgBuf);


void ReceiverInitialize(MessageBufferHandle_t msgBuf)
{
  xTaskCreate(
    ReceiverTask
    ,  (const portCHAR *)"ReceiverTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)msgBuf
    ,  2  // priority
    ,  NULL );
}


void ReceiverTask( void *pvParameters )
{
  MessageBufferHandle_t msgBuf = (MessageBufferHandle_t)pvParameters;
  uint8_t sendBuf[] = { 0xBE, 0xEF, 0xAB, 0xCD, 0xEF};
  const TickType_t timeout = pdMS_TO_TICKS( 1000 );


  Serial.print("ReceiverTask: msgBuf=");
  Serial.print((unsigned int)msgBuf, HEX);
  Serial.println(".");

  while(1)
  {
    size_t len = 5;
    size_t bytesSent;
    
    bytesSent = xMessageBufferSend(msgBuf, (void*)sendBuf, len, timeout);

    Serial.print("ReceiverTask: Sent ");
    Serial.print(bytesSent, DEC);
    Serial.println(" bytes.");

    if ( bytesSent != len )
    {
      Serial.println("ReceiverTask: TX BUF FULL");
    }


    vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS ); // wait for one second
  }
}
