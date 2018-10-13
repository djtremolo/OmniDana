#include "OmniDanaCommon.h"
#include "fbTask.h"

#define POLLING_TIME_MS     1000

void FeedbackInitialize(MessageBufferHandle_t msgBuf)
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
  MessageBufferHandle_t msgBuf = (MessageBufferHandle_t)pvParameters;

  Serial.print("FeedbackTask: msgBuf=");
  Serial.print((unsigned int)msgBuf, HEX);
  Serial.println(".");

  while(1)
  {
    Serial.println("FeedbackTask");

    vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS ); // wait for one second
  }
}
