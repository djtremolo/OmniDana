#include "OmniDanaCommon.h"
//#include "BTReceiverTask.h"

#define POLLING_TIME_MS     1000



void BTReceiverTask( void *pvParameters );


void BTReceiverInitialize(MessageBufferHandle msgBuf)
{
  xTaskCreate(
    BTReceiverTask
    ,  (const portCHAR *)"BTReceiverTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)msgBuf
    ,  2  // priority
    ,  NULL );
}


void BTReceiverTask( void *pvParameters )
{
  MessageBufferHandle msgBuf = (MessageBufferHandle)pvParameters;
  uint8_t sendBuf[] = { 0xBE, 0xEF, 0xAB, 0xCD, 0xEF};
  const TickType_t timeout = pdMS_TO_TICKS( 1000 );


  Serial.print("BTReceiverTask: msgBuf=");
  Serial.print((unsigned int)msgBuf, HEX);
  Serial.println(".");

  while(1)
  {
    size_t len = 5;
    size_t bytesSent;
    
    bytesSent = xMessageBufferSend(msgBuf, (void*)sendBuf, len, timeout);

    Serial.print("BTReceiverTask: Sent ");
    Serial.print(bytesSent, DEC);
    Serial.println(" bytes.");

    if ( bytesSent != len )
    {
      Serial.println("BTReceiverTask: TX BUF FULL");
    }


    vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS ); // wait for one second
  }
}
