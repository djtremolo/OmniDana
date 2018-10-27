#include "common.h"
#include "commTask.h"

#define DEBUG_PRINT         true

#define MAX_PAYLOAD_LENGTH  16
#define HEADER_LEN          6
#define FOOTER_LEN          4
#define MAX_BUF_LEN         (HEADER_LEN+MAX_PAYLOAD_LENGTH+FOOTER_LEN)



typedef enum
{
  CMD_PUMP_STATUS = 0x1234,
} danaCommand_t;

void commTaskInitialize(OmniDanaContext_t *ctx);

static int handleLowLevelMessage(OmniDanaContext_t *ctx, uint8_t *buf, size_t bufLen);
static int handlePayload(OmniDanaContext_t *ctx, DanaMessage_t *dMsg);
static int danaMsgStatus(OmniDanaContext_t *ctx, uint8_t *buf, uint8_t len);


void commTaskInitialize(OmniDanaContext_t *ctx)
{
  Serial.println(F("commTaskInitialize"));

  xTaskCreate(
    commTask
    ,  (const portCHAR *)"commTask"   // A name just for humans
    ,  100  // Stack size
    ,  (void*)ctx
    ,  COMM_TASK_PRIORITY  // priority
    ,  NULL );
}

static void commTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;
  int receivedBytes;

  #if DEBUG_PRINT
  Serial.print(F("commTask: starting with ctx = "));
  Serial.print((uint16_t)ctx, HEX);
  Serial.print(F(", ctx->recToCommBuffer = "));
  Serial.print((uint16_t)(ctx->recToCommBuffer), HEX);
  Serial.println();
  #endif

/*
while(1)
{
  vTaskDelay( 1000 / portTICK_PERIOD_MS );
}
*/
  while(1)
  {
    receivedBytes = xMessageBufferReceive(ctx->recToCommBuffer, (void*)ctx->commTaskMsg, sizeof(DanaMessage_t), portMAX_DELAY);

  #if DEBUG_PRINT
    Serial.println(F("commTask: received msg"));
  #endif

    if(receivedBytes == sizeof(DanaMessage_t) )
    { 
      /*handle received message*/
      if(handlePayload(ctx, ctx->commTaskMsg))
      {
        /*failed*/
  #if DEBUG_PRINT
        Serial.println(F("commTask:handlePayload failed!"));
  #endif
      }
    }

    /*and continue waiting for next...*/
  }
}


static int handlePayload(OmniDanaContext_t *ctx, DanaMessage_t *dMsg)
{
  int ret = -1;
  
  if(dMsg && (dMsg->length >= 2))
  {
    uint16_t cmd = ((uint16_t)dMsg->buf[0] << 8) | ((uint16_t)dMsg->buf[1]);
    uint8_t *buf = &(dMsg->buf[2]); 
    uint8_t len = dMsg->length - 2;

    switch(cmd)
    {
      case CMD_PUMP_STATUS:
        ret = danaMsgStatus(ctx, buf, len);
        break;

      default:
  #if DEBUG_PRINT
        Serial.print(F("commTask:handlePayload: Not implemented! cmd = "));
        Serial.print(cmd, HEX);
        Serial.print(F(", data=["));
        for(uint8_t i=0; i<len; i++)
        {
          if(buf[i] < 16) Serial.print(F("0"));
          Serial.print(buf[i], HEX);
          if(i < len-1) Serial.print(F(" "));
        }
        Serial.println(F("]."));
#endif
        /*DEBUG. Send treatment anyway*/
#if 1
        TreatmentMessage_t tr;
        tr.treatment = TREATMENT_BOLUS;
        tr.param1 = 340;  /*units*/

  #if DEBUG_PRINT
          Serial.println(F("commTask:handlePayload: trying to send treatment to ctrlTask."));
          Serial.flush();
  #endif

        size_t sentBytes = xMessageBufferSend(ctx->commToCtrlBuffer, &tr, sizeof(TreatmentMessage_t), 0);  //infinite
        if(sentBytes != sizeof(TreatmentMessage_t))
        {
  #if DEBUG_PRINT
          Serial.flush();
          Serial.println(F("commTask:handlePayload: sending to ctrlTask failed."));
  #endif
        }

  #if DEBUG_PRINT
        else
        {
          Serial.println(F("commTask:handlePayload: sent successfully!"));
        }
  #endif

#endif
        break;
    }
  }
  return ret;
}

static int danaMsgStatus(OmniDanaContext_t *ctx, uint8_t *buf, uint8_t len)
{
  int ret = -1;

#if DEBUG_PRINT
  Serial.println(F("MsgStatus"));
#endif

  //send treatment request to ctx->commToCtrlBuffer
  //send response to ctx->commToRecBuffer

  /*ok*/
  ret = 0;

  return ret;
}



