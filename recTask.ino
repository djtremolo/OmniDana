#include "common.h"
#include "recTask.h"
#include "util/crc16.h"


#define DEBUG_PRINT             true
#define POLLING_TIME_MS         10
#define INCOMING_FRAME_MAX      20

#define MAX_PAYLOAD_LENGTH      16
#define HEADER_LEN              6
#define FOOTER_LEN              4
#define HEADER_BYTES_AFTER_LEN  3

#define MAX_BUF_LEN             (HEADER_LEN+MAX_PAYLOAD_LENGTH+FOOTER_LEN)
#define MAX_LEN_FIELD           (HEADER_BYTES_AFTER_LEN+MAX_PAYLOAD_LENGTH)


typedef enum
{
  MSG_STATE_START = 0,
  MSG_STATE_LEN,
  MSG_STATE_F1,
  MSG_STATE_CMD,
  MSG_STATE_PAYLOAD,
  MSG_STATE_CRC,
  MSG_STATE_STOP,
  MSG_STATE_FINISHED
} danaFrameState_t;


typedef struct
{
  uint8_t buf[INCOMING_FRAME_MAX];
  uint8_t field_len; 
  uint8_t currentPosition;
  danaFrameState_t state;
  uint8_t stateRoundsLeft;
  uint16_t crcCalculated;
  uint16_t crcReceived;
  bool isReady;
  bool isBusy;
} danaFrame_t;


void recTask( void *pvParameters );
void recTaskInitialize(OmniDanaContext_t *ctx);

static void incomingFrameInitialize(danaFrame_t *frame);
static int incomingFrameSendToControlTask(MessageBufferHandle_t msgBuf, danaFrame_t *frame);
static void incomingFrameFollow(danaFrame_t *frame, uint8_t inByte);


void recTaskInitialize(OmniDanaContext_t *ctx)
{
  xTaskCreate(
    recTask
    ,  (const portCHAR *)"recTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)ctx
    ,  REC_TASK_PRIORITY
    ,  NULL );
}


void recTask( void *pvParameters )
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;
  danaFrame_t myDanaFrame;  /*move this from stack to heap if this becomes too big.*/
  danaFrame_t *frame = &myDanaFrame;

  /*start from scratch by resetting the incoming frame handler*/
  incomingFrameInitialize(frame);

  /*start receiving from uart.*/
  while(1)
  {
    /*Phase 1: if receiver is not busy, send if commTask has something to send*/
    if(frame->isBusy == false)
    {
      uint8_t txMsg[INCOMING_FRAME_MAX];
      
      size_t txLen = xMessageBufferReceive(ctx->commToRecBuffer, (void*)txMsg, sizeof(txMsg), 0);
      if(txLen > 0)
      {
        size_t sentBytes;
        sentBytes = Serial.write(txMsg, txLen);
      }
    }
    
    /*Phase 2: receive if there's something to receive*/
    size_t bytesAvailable = Serial.available();

    /*consume bytes available on uart, but do not go to next frame yet*/
    while((bytesAvailable--) > 0)
    {
      uint8_t inByte = Serial.read();

    //  Serial.print(inByte, HEX);
    //  Serial.print(" ");

      /*follow frame structure*/
      incomingFrameFollow(frame, inByte);

      /*when follower has marked the frame complete, let's send it to
      Control Task*/
      if(frame->isReady)
      {
        /*send full frame (with header+payload+footer) to Control Task*/
        incomingFrameSendToControlTask(ctx->recToCommBuffer, frame);

        /*after sending, let's restart receiving*/
        incomingFrameInitialize(frame);

        /*done for this frame, exit while loop*/
        break;
      }
    }

    /*delay for some time. It would be better to trig the task at each millisecond tick.*/
    vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS );
  }
}




static int incomingFrameSendToControlTask(MessageBufferHandle_t msgBuf, danaFrame_t *frame)
{
  int ret = 0;
  uint8_t bytesSent = xMessageBufferSend(msgBuf, (void*)frame->buf, frame->currentPosition, portMAX_DELAY);

#if DEBUG_PRINT
  Serial.print("sendToControlTask: sent ");
  Serial.print(bytesSent, DEC);
  Serial.println(" bytes.");
#endif

  if(bytesSent != frame->currentPosition)
  {
#if DEBUG_PRINT
    Serial.println("recTask: TX BUF FULL");
#endif
    ret = -1;
  }

  return ret;
}


static void incomingFrameFollow(danaFrame_t *frame, uint8_t inByte)
{
  bool error = true;

  if(frame->currentPosition < INCOMING_FRAME_MAX)
  {
    /*store byte*/
    frame->buf[frame->currentPosition++] = inByte;

    switch(frame->state)
    {
      case MSG_STATE_START:
        if(inByte == 0x7E)
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
        if(inByte <= MAX_LEN_FIELD)
        {
          frame->field_len = inByte;

          /*advance to next state*/
          frame->state = MSG_STATE_F1;
          /*no stateRoundsLeft needed for this state*/

          /*ok!*/
          error = false;
        }
        break;

      case MSG_STATE_F1:
        if(inByte == 0xF1)
        {
          /*append in CRC*/
          frame->crcCalculated = _crc16_update(frame->crcCalculated, inByte);

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

        if(--(frame->stateRoundsLeft) == 0)
        {
          /*last round -> advance to next state*/
          frame->state = MSG_STATE_PAYLOAD;
          frame->stateRoundsLeft = frame->field_len - HEADER_BYTES_AFTER_LEN;
        }
        error = false;
        break;

      case MSG_STATE_PAYLOAD:
        /*append in CRC*/
        frame->crcCalculated = _crc16_update(frame->crcCalculated, inByte);

        if(--(frame->stateRoundsLeft) == 0)
        {
          /*last round -> advance to next state*/
          frame->state = MSG_STATE_CRC;
          frame->stateRoundsLeft = 2;
        }
        error = false;
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
        if(inByte == 0x2E)
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


