#include "common.h"
#include "fbTask.h"

#define POLLING_TIME_MS     1000

void fbTaskInitialize(OmniDanaContext_t *ctx)
{
  xTaskCreate(
    fbTask
    ,  (const portCHAR *)"fbTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)ctx
    ,  FB_TASK_PRIORITY  // priority
    ,  NULL );
}


void fbTask( void *pvParameters )
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;

  while(1)
  {
    Serial.println("fbTask");

    vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS ); // wait for one second
  }
}
