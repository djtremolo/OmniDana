#include "common.h"
#include "ctrlTask.h"
#include "ioInterface.h"


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



void ctrlTaskInitialize(OmniDanaContext_t *ctx);

void ctrlTaskInitialize(OmniDanaContext_t *ctx)
{
  xTaskCreate(
    ctrlTask
    ,  (const portCHAR *)"ctrlTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)ctx
    ,  CTRL_TASK_PRIORITY  // priority
    ,  NULL );
}



void ctrlTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;

  IoInterfaceSetupPins();

  while(1)
  {
    DanaMessage_t dMsg;

    int recBytes = xMessageBufferReceive(ctx->commToCtrlBuffer, (void*)&dMsg, sizeof(DanaMessage_t), portMAX_DELAY);

    if(recBytes == sizeof(DanaMessage_t))
    {
      handleDanaCommand(ctx, &dMsg);

    }

    /*and continue waiting for next...*/
  }
}


static int handleDanaCommand(OmniDanaContext_t *ctx, DanaMessage_t *dMsg)
{
  int ret = -1;
  switch(dMsg->cmd)
  {
    default:
      //Serial.println("handleDanaCommand: unknown command");
      break;
  }
  return ret;
}


