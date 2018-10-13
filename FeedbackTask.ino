#include "OmniDanaCommon.h"
#include "FeedbackTask.h"

#define POLLING_TIME_MS     1000

void FeedbackInitialize(MessageBufferHandle msgBuf)
{
  xTaskCreate(
    FeedbackTask
    ,  (const portCHAR *)"FeedbackTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)msgBuf
    ,  2  // priority
    ,  NULL );
}


void FeedbackTask( void *pvParameters )
{
  MessageBufferHandle msgBuf = (MessageBufferHandle)pvParameters;

  Serial.print("FeedbackTask: msgBuf=");
  Serial.print((unsigned int)msgBuf, HEX);
  Serial.println(".");

  while(1)
  {
    Serial.println("FeedbackTask");

    vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS ); // wait for one second
  }
}
