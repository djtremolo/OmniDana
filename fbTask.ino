#include "common.h"
#include "fbTask.h"

#define POLLING_TIME_MS     1000

void fbTaskInitialize(OmniDanaContext_t *ctx)
{
  Serial.println(F("fbTaskInitialize"));

  xTaskCreate(
    fbTask
    ,  (const portCHAR *)"fbTask"   // A name just for humans
    ,  100  // Stack size
    ,  (void*)ctx
    ,  FB_TASK_PRIORITY  // priority
    ,  NULL );
}


void fbTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;

  #if DEBUG_PRINT
  Serial.print(F("fbTask: starting with ctx = "));
  Serial.print((uint16_t)ctx, HEX);
  Serial.print(F(", ctx->fbToCtrlBuffer = "));
  Serial.print((uint16_t)(ctx->fbToCtrlBuffer), HEX);
  Serial.println();
  #endif

  while(1)
  {
    //Serial.println(F("fbTask"));

    //clockUpdate();  /*do this somewhere*/

    vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS ); // wait for one second
  }
}
