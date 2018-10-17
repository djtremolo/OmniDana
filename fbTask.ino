#include "common.h"
#include "fbTask.h"

#define POLLING_TIME_MS     1000

void fbTaskInitialize(MessageBufferHandle_t msgBuf)
{
  xTaskCreate(
    fbTask
    ,  (const portCHAR *)"fbTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)msgBuf
    ,  2  // priority
    ,  NULL );
}


void fbTask( void *pvParameters )
{
  MessageBufferHandle_t msgBuf = (MessageBufferHandle_t)pvParameters;

  Serial.print("fbTask: msgBuf=");
  Serial.print((unsigned int)msgBuf, HEX);
  Serial.println(".");

  while(1)
  {
    Serial.println("fbTask");

    vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS ); // wait for one second
  }
}
