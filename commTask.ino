#include "common.h"
#include "commTask.h"

#define MAX_PAYLOAD_LENGTH  16
#define HEADER_LEN          6
#define FOOTER_LEN          4
#define MAX_BUF_LEN         (HEADER_LEN+MAX_PAYLOAD_LENGTH+FOOTER_LEN)

#define START_CHAR          0x7E
#define STOP_CHAR           0x2E


typedef enum
{
  CMD_PUMP_STATUS = 0x1234,
} danaCommand_t;

void commTaskInitialize(OmniDanaContext_t *ctx);

static int handleLowLevelMessage(OmniDanaContext_t *ctx, uint8_t *buf, size_t bufLen);
static int handlePayload(uint16_t cmd, uint8_t *buf, uint8_t len);
static void danaMsgStatus(uint8_t *buf, uint8_t len);


void commTaskInitialize(OmniDanaContext_t *ctx)
{
  xTaskCreate(
    ctrlTask
    ,  (const portCHAR *)"commTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)ctx
    ,  COMM_TASK_PRIORITY  // priority
    ,  NULL );
}



void commTask( void *pvParameters )
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;
  uint8_t msg[20];
  int receivedBytes;

  while(1)
  {
    receivedBytes = xMessageBufferReceive(ctx->recToCommBuffer, (void*)msg, sizeof(msg), portMAX_DELAY);

    if(receivedBytes > 0 )
    { 
      Serial.print("commTask received ");
      Serial.print(receivedBytes, DEC);
      Serial.println(" bytes.");
    }

    /*handle received message*/
    if(handleLowLevelMessage(ctx, &msg[0], receivedBytes))
    {
      /*failed*/
      Serial.println("handleLowLevelMessage failed!");
    }

    /*and continue waiting for next...*/
  }
}



static int handleLowLevelMessage(OmniDanaContext_t *ctx, uint8_t *buf, size_t bufLen)
{
  int ret = -1;
  
  /*sanity check*/
  if((buf[0] == START_CHAR) && (buf[1] == START_CHAR))
  {
    if(buf[3] == 0xF1)    /*fixed byte*/
    {
      uint8_t payloadLen = buf[2] - 3;
      uint8_t *payload = &buf[6];
      uint16_t cmd = (buf[4]<<8 | buf[5]);

      uint8_t footerOffset = payloadLen + 3;   /*crc hi*/

      if(bufLen >= (payloadLen +6))
      {
        if((buf[footerOffset+2] == STOP_CHAR) && (buf[footerOffset+3] == STOP_CHAR))
        {
          ret = handlePayload(ctx, cmd, payload, payloadLen);
        }
      }
    }
  }

  return ret;
}

static int handlePayload(OmniDanaContext_t *ctx, uint16_t cmd, uint8_t *buf, uint8_t len)
{
  int ret = -1;
  
  switch(cmd)
  {
    case CMD_PUMP_STATUS:
      ret = danaMsgStatus(ctx, buf, len);
      break;

    default:
      Serial.print("handleMessage: Not implemented! cmd = ");
      Serial.print(cmd, HEX);
      Serial.print(", data=[");
      for(int i=0; i<len; i++)
      {
        Serial.print(buf[i], DEC);
      }
      Serial.println(".");
      break;
  }

  return ret;
}

static int danaMsgStatus(OmniDanaContext_t *ctx, uint8_t *buf, uint8_t len)
{
  int ret = -1;
  Serial.println("MsgStatus");

  //send treatment request to ctx->commToCtrlBuffer
  //send response to ctx->commToRecBuffer

  /*ok*/
  ret = 0;

  return ret;
}



