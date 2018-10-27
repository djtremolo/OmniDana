#include "common.h"
#include "ctrlTask.h"
#include "ioInterface.h"

#define DEBUG_PRINT             true
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

static void handleTreatment(OmniDanaContext_t *ctx, TreatmentMessage_t *tr);
static void ctrlTask(void *pvParameters);



void ctrlTaskInitialize(OmniDanaContext_t *ctx);

void ctrlTaskInitialize(OmniDanaContext_t *ctx)
{
  Serial.println(F("ctrlTaskInitialize"));

  xTaskCreate(
    ctrlTask
    ,  (const portCHAR *)"ctrlTask"   // A name just for humans
    ,  100  // Stack size
    ,  (void*)ctx
    ,  CTRL_TASK_PRIORITY  // priority
    ,  NULL );
}



static void ctrlTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;

  #if DEBUG_PRINT
  Serial.print(F("ctrlTask: starting with ctx = "));
  Serial.print((uint16_t)ctx, HEX);
  Serial.print(F(", ctx->commToCtrlBuffer = "));
  Serial.print((uint16_t)(ctx->commToCtrlBuffer), HEX);
  Serial.println();
  #endif

/*
while(1)
{
  vTaskDelay( 1000 / portTICK_PERIOD_MS );
}
*/

  IoInterfaceSetupPins();

  while(1)
  {
    TreatmentMessage_t treatment;

    #if DEBUG_PRINT
    Serial.println(F("ctrlTask: listening for treatments."));
    #endif

    int recBytes = xMessageBufferReceive(ctx->commToCtrlBuffer, (void*)&treatment, sizeof(TreatmentMessage_t), portMAX_DELAY);

    if(recBytes == sizeof(TreatmentMessage_t))
    {
      #if DEBUG_PRINT
      Serial.flush();
      Serial.println(F("ctrlTask: treatment received!"));
      #endif
      handleTreatment(ctx, &treatment);
    }

BlinkLed(10);

    /*and continue waiting for next...*/
  }
}


static void handleTreatment(OmniDanaContext_t *ctx, TreatmentMessage_t *tr)
{
  #if DEBUG_PRINT
  Serial.println(F("handleTreatment:"));
  #endif
}
