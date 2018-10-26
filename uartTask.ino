#include "common.h"
#include "uartTask.h"
#include "util/crc16.h"

#define START_CHAR                  0xA5
#define STOP_CHAR                   0x5A

#define DEBUG_PRINT                 false
#define POLLING_TIME_MS             100

#define IDLE_COUNTER_MAX            100

typedef enum
{
  MSG_STATE_START = 0,
  MSG_STATE_LEN,
  MSG_STATE_CMD,
  MSG_STATE_PAYLOAD,
  MSG_STATE_CRC,
  MSG_STATE_STOP,
  MSG_STATE_FINISHED
} danaFrameState_t;


typedef struct
{
  DanaMessage_t danaMsg; 
  uint8_t field_len; 
  danaFrameState_t state;
  uint8_t stateRoundsLeft;
  uint16_t crcCalculated;
  uint16_t crcReceived;
  bool isReady;
  bool isBusy;
  uint16_t idleCounter;
} danaFrame_t;


void uartTaskInitialize(OmniDanaContext_t *ctx);

static void uartTask(void *pvParameters);
static bool sendToAAPS(OmniDanaContext_t *ctx, danaFrame_t *frame);
static bool receiveFromAAPS(OmniDanaContext_t *ctx, danaFrame_t *frame);
static void incomingFrameInitialize(danaFrame_t *frame);
static int incomingFrameSendToCommTask(MessageBufferHandle_t msgBuf, danaFrame_t *frame);
static void incomingFrameFollow(danaFrame_t *frame, uint8_t inByte);
static int createOutMessage(uint8_t *rawBuf, DanaMessage_t *msg);


void uartTaskInitialize(OmniDanaContext_t *ctx)
{
  Serial.println("uartTask: starting");

  xTaskCreate(
    uartTask
    ,  (const portCHAR *)"uartTask"   // A name just for humans
    ,  256  // Stack size
    ,  (void*)ctx
    ,  UART_TASK_PRIORITY
    ,  NULL );
}

bool ledState = false;
int cntr = 0;

static void uartTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;
  danaFrame_t myDanaFrame;  /*move this from stack to heap if this becomes too big.*/
  danaFrame_t *frame = &myDanaFrame;

  /*start from scratch by resetting the incoming frame handler*/
  incomingFrameInitialize(frame);

  /*start receiving from uart.*/
  while(1)
  {
    bool somethingHappened = false;

    /*Phase 1: if receiver is not busy, send if commTask has something to send*/
    somethingHappened |= sendToAAPS(ctx, frame);

    /*Phase 2: receive if there's something to receive*/
    somethingHappened |= receiveFromAAPS(ctx, frame);

    /*If nothing happened, let's rest a bit.*/
    if(!somethingHappened)
    {
      vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS );
    }

  }
}



static bool sendToAAPS(OmniDanaContext_t *ctx, danaFrame_t *frame)
{
  bool ret = false;

  /*do not send while we are receiving. TODO: Check if this is really needed? BTLE should be full duplex?*/
  if(frame->isBusy == false)
  {
    DanaMessage_t dMsg;
    
    size_t len = xMessageBufferReceive(ctx->commToRecBuffer, (void*)&dMsg, sizeof(DanaMessage_t), 0);
    if(len == sizeof(DanaMessage_t))
    {
      uint8_t outMsg[DANA_MAX_BUF_LEN];
      int outLen;

      outLen = createOutMessage(outMsg, &dMsg);

      if(outLen > 0)
      {
        size_t sentBytes = Serial.write(outMsg, outLen);

        if(sentBytes == outLen)
        {
          ret = true;
        }
      }
    }
  }

  return ret;
}

static bool receiveFromAAPS(OmniDanaContext_t *ctx, danaFrame_t *frame)
{
  bool ret = false;

  size_t bytesAvailable = Serial.available();

  /*consume bytes available on uart, but do not go to next frame yet*/
  while((bytesAvailable--) > 0)
  {
    uint8_t inByte = Serial.read();

//    digitalWrite(LED_BUILTIN, ledState);
//    ledState = !ledState;


Serial.print(inByte, HEX);



    /*follow frame structure*/
    incomingFrameFollow(frame, inByte);

    /*when follower has marked the frame complete, let's send it to
    commTask*/
    if(frame->isReady)
    {
      /*send full frame (with header+payload+footer) to commTask*/
      incomingFrameSendToCommTask(ctx->recToCommBuffer, frame);

      /*after sending, let's restart receiving*/
      incomingFrameInitialize(frame);

      /*return true when we finished with something. This will cause the task to rest for a while.*/
      ret = true;

      /*done for this frame, exit while loop*/
      break;
    }
  }

  /*check if the message never finished... cancel after some rounds*/
  if(frame->idleCounter++ >= IDLE_COUNTER_MAX)
  {
    incomingFrameInitialize(frame);
    ret = true;
  }

  return ret;
}

static int createOutMessage(uint8_t *rawBuf, DanaMessage_t *msg)
{
  int ret = -1;

  if(msg && msg->length <= DANA_MAX_PAYLOAD_LENGTH)
  {
    uint16_t crc = 0;
    int idx = 0;
    uint8_t tmp;

    /*start*/
    rawBuf[idx++] = START_CHAR;
    rawBuf[idx++] = START_CHAR;
    
    /*len*/
    rawBuf[idx++] = ((uint8_t)msg->length) + 2;

    /*type*/
    tmp = (uint8_t)((msg->cmd >> 8) & 0xFF);
    rawBuf[idx++] = tmp;
    crc = _crc16_update(crc, tmp);  

    /*code*/
    tmp = (uint8_t)(msg->cmd & 0xFF);
    rawBuf[idx++] = tmp;
    crc = _crc16_update(crc, tmp);  

    /*params*/
    for(uint8_t i = 0; i < msg->length; i++)
    {
      tmp = msg->buf[i];
      rawBuf[idx++] = tmp;
      crc = _crc16_update(crc, tmp);  
    }

    /*crc1*/
    rawBuf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    /*crc2*/
    rawBuf[idx++] = (uint8_t)(crc & 0xFF);

    /*stop*/
    rawBuf[idx++] = STOP_CHAR;
    rawBuf[idx++] = STOP_CHAR;

    ret = idx;
  }

  return ret;
}



static int incomingFrameSendToCommTask(MessageBufferHandle_t msgBuf, danaFrame_t *frame)
{
  int ret = 0;
  DanaMessage_t *dMsg = &(frame->danaMsg);
  uint8_t bytesSent = xMessageBufferSend(msgBuf, (void*)dMsg->buf, sizeof(DanaMessage_t), portMAX_DELAY);

#if DEBUG_PRINT
  Serial.print("incomingFrameSendToCommTask: sent ");
  Serial.print(bytesSent, DEC);
  Serial.println(" bytes.");
#endif

  if(bytesSent != sizeof(DanaMessage_t))
  {
#if DEBUG_PRINT
    Serial.println("uartTask: TX BUF FULL");
#endif
    ret = -1;
  }

  return ret;
}

// A5 A5 LEN TYPE CODE PARAMS CHECKSUM1 CHECKSUM2 5A 5A
//           ^---- LEN -----^
// total packet length 2 + 1 + readBuffer[2] + 2 + 2

static void incomingFrameFollow(danaFrame_t *frame, uint8_t inByte)
{
  bool error = true;
  DanaMessage_t *dMsg;

  switch(frame->state)
  {
    case MSG_STATE_START:
      if(inByte == START_CHAR)
      {
        /*receiving started - mark busy to disable sender*/
        frame->isBusy = true;

        if(--(frame->stateRoundsLeft) == 0)
        {
          /*last round -> advance to next state*/
          frame->state = MSG_STATE_LEN;
          /*no stateRoundsLeft needed for this state*/
        }
        error = false;
      }
      break;

    case MSG_STATE_LEN:
      if(inByte <= DANA_MAX_LEN_FIELD)
      {
        frame->field_len = inByte;

        /*advance to next state*/
        frame->state = MSG_STATE_CMD;
        frame->stateRoundsLeft = 2;

        /*ok!*/
        error = false;
      }
      break;

    case MSG_STATE_CMD:
      /*append in CRC*/
      frame->crcCalculated = _crc16_update(frame->crcCalculated, inByte);

      /*set command word*/
      dMsg = &(frame->danaMsg);

      dMsg->cmd <<= 8;
      dMsg->cmd |= inByte;

      if(--(frame->stateRoundsLeft) == 0)
      {
        /*last round -> advance to next state*/
        frame->state = MSG_STATE_PAYLOAD;
        frame->stateRoundsLeft = frame->field_len - DANA_HEADER_BYTES_AFTER_LEN;
      }
      error = false;
      break;

    case MSG_STATE_PAYLOAD:
      /*append in CRC*/
      frame->crcCalculated = _crc16_update(frame->crcCalculated, inByte);

      /*append to payload buffer*/
      dMsg = &(frame->danaMsg);

      if(dMsg->length < (DANA_MAX_PAYLOAD_LENGTH-1))
      {
        dMsg->buf[dMsg->length++] = inByte;

        if(--(frame->stateRoundsLeft) == 0)
        {
          /*last round -> advance to next state*/
          frame->state = MSG_STATE_CRC;
          frame->stateRoundsLeft = 2;
        }
        error = false;
      }
      break;

    case MSG_STATE_CRC:
      /*store the incoming CRC in two parts.
      On first byte, the crcReceived==0, so shifting does not matter. We just or the low part.
      On second byte, we shift left and OR with low part.*/
      frame->crcReceived <<= 8;
      frame->crcReceived |= inByte;

      /*let's assume the receiving will be OK*/
      error = false;

      if(--(frame->stateRoundsLeft) == 0)
      {
        if(frame->crcReceived == frame->crcCalculated)
        {
          /*Checksum OK!*/
          /*last round -> advance to next state*/
          frame->state = MSG_STATE_STOP;
          frame->stateRoundsLeft = 2;
        }
        else
        {
          /*CRC failed - drop frame*/
          error = true;
        }
      }
      break;

    case MSG_STATE_STOP:
      if(inByte == STOP_CHAR)
      {
        if(--(frame->stateRoundsLeft) == 0)
        {
          /*last round -> advance to next state*/
          frame->state = MSG_STATE_FINISHED;
          /*no stateRoundsLeft needed for this state*/

          /*mark frame ready to be sent to ControlTask*/
          frame->isReady = true;
        }
        error = false;
      }
      break;

    case MSG_STATE_FINISHED:
      /*idle... we shouldn't get here at anytime, but it wouldn't be an error.
      We just drop all the bytes.*/
      error = false;
      break;

    default:
      /*out of sync*/
      break;
  }

  if(error)
  {
    /*restart listening from start*/
    incomingFrameInitialize(frame);
  }
}

static void incomingFrameInitialize(danaFrame_t *frame)
{
  memset(frame, 0, sizeof(danaFrame_t));

  frame->state = MSG_STATE_START;
  frame->stateRoundsLeft = 2;
}


