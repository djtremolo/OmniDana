#include "common.h"
#include "ctrlTask.h"
#include "ioInterface.h"

#define MAX_STEPS_IN_SEQUENCE     10

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

TimerHandle_t tickTimer;
SemaphoreHandle_t tickSemaphore;


static void handleTreatment(OmniDanaContext_t *ctx, TreatmentMessage_t *tr);
static void ctrlTask(void *pvParameters);
static void ctrlTaskTimerTick(TimerHandle_t xTimer);

void ctrlTaskInitialize(OmniDanaContext_t *ctx);

void ctrlTaskInitialize(OmniDanaContext_t *ctx)
{
  Serial.println(F("ctrlTaskInitialize"));


  tickTimer = xTimerCreate
                 ( (const portCHAR *)"ctrlTaskTimer",
                   1000 / portTICK_PERIOD_MS,     //one second
                   pdTRUE,
                   (void*)0,
                   ctrlTaskTimerTick );

  if(tickTimer == NULL)
  {
    Serial.println(F("xTimerCreate failed"));
  }



  tickSemaphore = xSemaphoreCreateBinary();
  if(tickSemaphore == NULL)
  {
    Serial.println(F("xSemaphoreCreateBinary failed"));
  }



  if(xTaskCreate(
      ctrlTask, 
      (const portCHAR *)"ctrlTask",
      100,
      (void *)ctx, 
      CTRL_TASK_PRIORITY,
      NULL) != pdPASS)
  {
    Serial.println(F("xTaskCreate failed"));
  }
}

static void ctrlTaskTimerTick(TimerHandle_t xTimer)
{
  (void)xTimer;

  //Serial.println("Tick");
  xSemaphoreGive(tickSemaphore);
}


static void ctrlTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t *)pvParameters;

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

  xTimerStart(tickTimer, 0);

  while (1)
  {
    TreatmentMessage_t treatment;

    /*wait for a tick*/
    xSemaphoreTake(tickSemaphore, portMAX_DELAY);

    //Serial.println("TICK");

    int recBytes = xMessageBufferReceive(ctx->commToCtrlBuffer, (void *)&treatment, sizeof(TreatmentMessage_t), 0);

    if (recBytes == sizeof(TreatmentMessage_t))
    {
#if DEBUG_PRINT
      Serial.flush();
      Serial.println(F("ctrlTask: treatment received!"));
#endif
      handleTreatment(ctx, &treatment);
    }

    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);

    /*and continue waiting for next...*/
  }
}

static void handleTreatment(OmniDanaContext_t *ctx, TreatmentMessage_t *tr)
{
  (void)ctx;
  (void)tr;
#if DEBUG_PRINT
  Serial.println(F("handleTreatment:"));
#endif
}
