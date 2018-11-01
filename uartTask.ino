#include "common.h"
#include "uartTask.h"
#include "Crc16.h"
#include "msgBuf.h"



#define START_CHAR                  0xA5
#define STOP_CHAR                   0x5A

#define DEBUG_PRINT                 true
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



#define TYPE_ENCRYPTION_REQUEST                                         0x01
#define TYPE_ENCRYPTION_RESPONSE                                        0x02
#define TYPE_COMMAND                                                    0xA1
#define TYPE_RESPONSE                                                   0xB2
#define TYPE_NOTIFY                                                     0xC3
#define OPCODE_ENCRYPTION__PUMP_CHECK                                   0x00
#define OPCODE_ENCRYPTION__CHECK_PASSKEY                                0xD0
#define OPCODE_ENCRYPTION__PASSKEY_REQUEST                              0xD1
#define OPCODE_ENCRYPTION__PASSKEY_RETURN                               0xD2
#define OPCODE_ENCRYPTION__TIME_INFORMATION                             0x01
#define OPCODE_NOTIFY__DELIVERY_COMPLETE                                0x01
#define OPCODE_NOTIFY__DELIVERY_RATE_DISPLAY                            0x02
#define OPCODE_NOTIFY__ALARM                                            0x03
#define OPCODE_NOTIFY__MISSED_BOLUS_ALARM                               0x04
#define OPCODE_REVIEW__INITIAL_SCREEN_INFORMATION                       0x02
#define OPCODE_REVIEW__DELIVERY_STATUS                                  0x03
#define OPCODE_REVIEW__GET_PASSWORD                                     0x04
#define OPCODE_REVIEW__BOLUS_AVG                                        0x10
#define OPCODE_REVIEW__BOLUS                                            0x11
#define OPCODE_REVIEW__DAILY                                            0x12
#define OPCODE_REVIEW__PRIME                                            0x13
#define OPCODE_REVIEW__REFILL                                           0x14
#define OPCODE_REVIEW__BLOOD_GLUCOSE                                    0x15
#define OPCODE_REVIEW__CARBOHYDRATE                                     0x16
#define OPCODE_REVIEW__TEMPORARY                                        0x17
#define OPCODE_REVIEW__SUSPEND                                          0x18
#define OPCODE_REVIEW__ALARM                                            0x19
#define OPCODE_REVIEW__BASAL                                            0x1A
#define OPCODE_REVIEW__ALL_HISTORY                                      0x1F
#define OPCODE_REVIEW__GET_SHIPPING_INFORMATION                         0x20
#define OPCODE_REVIEW__GET_PUMP_CHECK                                   0x21
#define OPCODE_REVIEW__GET_USER_TIME_CHANGE_FLAG                        0x22
#define OPCODE_REVIEW__SET_USER_TIME_CHANGE_FLAG_CLEAR                  0x23
#define OPCODE_REVIEW__GET_MORE_INFORMATION                             0x24
#define OPCODE_REVIEW__SET_HISTORY_UPLOAD_MODE                          0x25
#define OPCODE_REVIEW__GET_TODAY_DELIVERY_TOTAL                         0x26
#define OPCODE_BOLUS__GET_STEP_BOLUS_INFORMATION                        0x40
#define OPCODE_BOLUS__GET_EXTENDED_BOLUS_STATE                          0x41
#define OPCODE_BOLUS__GET_EXTENDED_BOLUS                                0x42
#define OPCODE_BOLUS__GET_DUAL_BOLUS                                    0x43
#define OPCODE_BOLUS__SET_STEP_BOLUS_STOP                               0x44
#define OPCODE_BOLUS__GET_CARBOHYDRATE_CALCULATION_INFORMATION          0x45
#define OPCODE_BOLUS__GET_EXTENDED_MENU_OPTION_STATE                    0x46
#define OPCODE_BOLUS__SET_EXTENDED_BOLUS                                0x47
#define OPCODE_BOLUS__SET_DUAL_BOLUS                                    0x48
#define OPCODE_BOLUS__SET_EXTENDED_BOLUS_CANCEL                         0x49
#define OPCODE_BOLUS__SET_STEP_BOLUS_START                              0x4A
#define OPCODE_BOLUS__GET_CALCULATION_INFORMATION                       0x4B
#define OPCODE_BOLUS__GET_BOLUS_RATE                                    0x4C
#define OPCODE_BOLUS__SET_BOLUS_RATE                                    0x4D
#define OPCODE_BOLUS__GET_CIR_CF_ARRAY                                  0x4E
#define OPCODE_BOLUS__SET_CIR_CF_ARRAY                                  0x4F
#define OPCODE_BOLUS__GET_BOLUS_OPTION                                  0x50
#define OPCODE_BOLUS__SET_BOLUS_OPTION                                  0x51
#define OPCODE_BASAL__SET_TEMPORARY_BASAL                               0x60
#define OPCODE_BASAL__TEMPORARY_BASAL_STATE                             0x61
#define OPCODE_BASAL__CANCEL_TEMPORARY_BASAL                            0x62
#define OPCODE_BASAL__GET_PROFILE_NUMBER                                0x63
#define OPCODE_BASAL__SET_PROFILE_NUMBER                                0x64
#define OPCODE_BASAL__GET_PROFILE_BASAL_RATE                            0x65
#define OPCODE_BASAL__SET_PROFILE_BASAL_RATE                            0x66
#define OPCODE_BASAL__GET_BASAL_RATE                                    0x67
#define OPCODE_BASAL__SET_BASAL_RATE                                    0x68
#define OPCODE_BASAL__SET_SUSPEND_ON                                    0x69
#define OPCODE_BASAL__SET_SUSPEND_OFF                                   0x6A
#define OPCODE_OPTION__GET_PUMP_TIME                                    0x70
#define OPCODE_OPTION__SET_PUMP_TIME                                    0x71
#define OPCODE_OPTION__GET_USER_OPTION                                  0x72
#define OPCODE_OPTION__SET_USER_OPTION                                  0x73
#define OPCODE_BASAL__APS_SET_TEMPORARY_BASAL                           0xC1
#define OPCODE__APS_HISTORY_EVENTS                                      0xC2
#define OPCODE__APS_SET_EVENT_HISTORY                                   0xC3
#define OPCODE_ETC__SET_HISTORY_SAVE                                    0xE0
#define OPCODE_ETC__KEEP_CONNECTION                                     0xFF


typedef struct
{
  DanaMessage_t danaMsg; 
  uint8_t field_len; 
  danaFrameState_t state;
  uint8_t stateRoundsLeft;
  uint16_t crcReceived;
  bool isReady;
  bool isBusy;
  uint16_t idleCounter;
} danaFrame_t;


void uartTaskInitialize(OmniDanaContext_t *ctx);

static void uartTask(void *pvParameters);
static void sendToAAPS(OmniDanaContext_t *ctx, uint8_t *buf, int len);
static bool receiveFromAAPS(OmniDanaContext_t *ctx, danaFrame_t *frame);
static void incomingFrameInitialize(danaFrame_t *frame);
static int incomingFrameSendToCommTask(MessageBufferHandle_t msgBuf, danaFrame_t *frame);
static void incomingFrameFollow(danaFrame_t *frame, uint8_t inByte);
static int createOutMessage(uint8_t *rawBuf, uint8_t type, uint8_t code, uint8_t *plBuf, uint8_t plLen);

static int handlePayload(OmniDanaContext_t *ctx, DanaMessage_t *dMsg);

static int handleTypeEncryptionRequest(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len);
static int handleTypeCommand(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len);
static int handleTypeNotify(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len);
static int handleTypeResponse(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len);


void uartTaskInitialize(OmniDanaContext_t *ctx)
{
  Serial.println(F("uartTaskInitialize"));

  xTaskCreate(
    uartTask
    ,  (const portCHAR *)"uartTask"   // A name just for humans
    ,  300  // Stack size
    ,  (void*)ctx
    ,  UART_TASK_PRIORITY
    ,  NULL );
}

static void uartTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t*)pvParameters;
  danaFrame_t myDanaFrame;  /*move this from stack to heap if this becomes too big.*/
  danaFrame_t *frame = &myDanaFrame;

  #if DEBUG_PRINT
  Serial.print(F("uartTask: starting with ctx = "));
  Serial.print((uint16_t)ctx, HEX);
  Serial.println(".");
  #endif

  /*start from scratch by resetting the incoming frame handler*/
  incomingFrameInitialize(frame);

  /*start receiving from uart.*/
  while(1)
  {
    bool somethingHappened = false;

    /*receive if there's something to receive*/
    somethingHappened |= receiveFromAAPS(ctx, frame);

    /*If nothing happened, let's rest a bit.*/
    if(!somethingHappened)
    {
      vTaskDelay( POLLING_TIME_MS / portTICK_PERIOD_MS );
    }

  }
}
/*
static bool sendToAAPS(OmniDanaContext_t *ctx, danaFrame_t *frame)
{
  bool ret = false;

  //do not send while we are receiving. TODO: Check if this is really needed? BTLE should be full duplex?
  if(frame->isBusy == false)
  {
    
    size_t len = xMessageBufferReceive(ctx->commToRecBuffer, (void*)ctx->uartTaskMsg, sizeof(DanaMessage_t), 0);
    if(len == sizeof(DanaMessage_t))
    {
      int outLen;

      outLen = createOutMessage(ctx->uartTaskRawMsg, ctx->uartTaskMsg);

      if(outLen > 0)
      {
        size_t sentBytes = Serial.write(ctx->uartTaskRawMsg, outLen);

        if(sentBytes == outLen)
        {
          ret = true;
        }
      }
    }
  }

  return ret;
}
*/


static bool receiveFromAAPS(OmniDanaContext_t *ctx, danaFrame_t *frame)
{
  bool ret = false;

  size_t bytesAvailable = Serial.available();

  /*consume bytes available on uart, but do not go to next frame yet*/
  while((bytesAvailable--) > 0)
  {
    uint8_t inByte = Serial.read();

    #if DEBUG_PRINT
    if(inByte<16)
      Serial.print(F("0"));
    Serial.print(inByte, HEX);
    Serial.print(F(" "));
    #endif

    /*follow frame structure*/
    incomingFrameFollow(frame, inByte);

    /*when follower has marked the frame complete, let's send it to
    commTask*/
    if(frame->isReady)
    {
      #if DEBUG_PRINT
      Serial.println(F("receiveFromAAPS: Incoming frame ready!"));
      #endif

      if(handlePayload(ctx, &(frame->danaMsg)) != 0)
      {
        #if DEBUG_PRINT
        Serial.println(F("receiveFromAAPS: handlePayload failed"));
        #endif
      }

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

static int createOutMessage(uint8_t *rawBuf, uint8_t type, uint8_t code, uint8_t *plBuf, uint8_t plLen)
{
  int ret = -1;

  if(plBuf && plLen <= DANA_MAX_PAYLOAD_LENGTH)
  {
    Crc16 crc;
    int idx = 0;
    uint8_t tmp;
    uint8_t lenField = plLen + 2;

    /*start*/
    rawBuf[idx++] = START_CHAR;
    rawBuf[idx++] = START_CHAR;
    
    /*len*/
    rawBuf[idx++] = lenField;

    /*cmd1: type*/
    rawBuf[idx++] = type;

    /*cmd1: cmd*/
    rawBuf[idx++] = code;

    /*payload: params*/
    for(uint8_t i = 0; i < plLen; i++)
    {
      rawBuf[idx++] = plBuf[i];
    }

    crc.clearCrc();
    uint16_t crcValue = crc.XModemCrc(rawBuf, 3, lenField);

    /*crc1*/
    rawBuf[idx++] = (uint8_t)((crcValue >> 8) & 0xFF);

    /*crc2*/
    rawBuf[idx++] = (uint8_t)(crcValue & 0xFF);

    /*stop*/
    rawBuf[idx++] = STOP_CHAR;
    rawBuf[idx++] = STOP_CHAR;

    ret = idx;
  }

  return ret;
}

/*

static int incomingFrameSendToCommTask(MessageBufferHandle_t msgBuf, danaFrame_t *frame)
{
  int ret = 0;
  DanaMessage_t *dMsg = &(frame->danaMsg);
  uint8_t bytesSent = xMessageBufferSend(msgBuf, (void*)dMsg->buf, sizeof(DanaMessage_t), portMAX_DELAY);

#if DEBUG_PRINT
  Serial.print(F("incomingFrameSendToCommTask: sent "));
  Serial.print(bytesSent, DEC);
  Serial.println(F(" bytes."));
#endif

  if(bytesSent != sizeof(DanaMessage_t))
  {
#if DEBUG_PRINT
    Serial.println(F("uartTask: TX BUF FULL"));
#endif
    ret = -1;
  }

  return ret;
}
*/
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
      /*set command word*/
      dMsg = &(frame->danaMsg);

      dMsg->buf[dMsg->length++] = inByte;

      if(--(frame->stateRoundsLeft) == 0)
      {
        /*last round -> advance to next state*/
        frame->state = MSG_STATE_PAYLOAD;
        frame->stateRoundsLeft = frame->field_len - DANA_HEADER_BYTES_AFTER_LEN;
      }
      error = false;
      break;

    case MSG_STATE_PAYLOAD:
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
        Crc16 crc;

        dMsg = &(frame->danaMsg);


        /*calculate crc*/
        crc.clearCrc();
        uint16_t crcValue = crc.XModemCrc(dMsg->buf, 0, dMsg->length);

        if(frame->crcReceived == crcValue)
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

          Serial.print(F("CRC FAIL: received: "));
          Serial.print(frame->crcReceived, HEX);
          Serial.print(F(", calculated: "));
          Serial.println(crcValue, HEX);
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



/*Incoming message handling*/

static int handlePayload(OmniDanaContext_t *ctx, DanaMessage_t *dMsg)
{
  int ret = -1;
  
  if(dMsg && (dMsg->length >= 2))
  {
    uint8_t *buf = &(dMsg->buf[2]); 
    uint8_t len = dMsg->length - 2;

  #if DEBUG_PRINT
  Serial.print(F("handleTypeEncryptionRequest, type = "));
  Serial.print(dMsg->buf[0], HEX);
  Serial.print(F(", opcode = "));
  Serial.print(dMsg->buf[1], HEX);
  Serial.println(F("."));
  #endif


    switch(dMsg->buf[0])
    {
      case TYPE_ENCRYPTION_REQUEST:
        ret = handleTypeEncryptionRequest(ctx, dMsg->buf[1], buf, len);
        break;
      case TYPE_COMMAND:
        ret = handleTypeCommand(ctx, dMsg->buf[1], buf, len);
        break;
      case TYPE_NOTIFY:
        ret = handleTypeNotify(ctx, dMsg->buf[1], buf, len);
        break;
      case TYPE_RESPONSE:
        ret = handleTypeResponse(ctx, dMsg->buf[1], buf, len);    /*commands sent as responses by AAPS.. a bit strange*/
        break;

      default:
  #if DEBUG_PRINT
        Serial.print(F("handlePayload: Not implemented! type = "));
        Serial.print(dMsg->buf[0], HEX);
        Serial.print(F(", data=["));
        for(uint8_t i=0; i<len; i++)
        {
          if(buf[i] < 16) Serial.print(F("0"));
          Serial.print(buf[i], HEX);
          if(i < len-1) Serial.print(F(" "));
        }
        Serial.println(F("]."));
  #endif
        break;
    }
  }
  return ret;
}

#define ENC_MAX_PAYLOAD_ALLOCATION    10

static int handleTypeEncryptionRequest(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len)
{
  int ret = -1;
  uint8_t rawBuf[DANA_RAW_MSG_LEN(ENC_MAX_PAYLOAD_ALLOCATION)];
  uint8_t tempBuf[ENC_MAX_PAYLOAD_ALLOCATION];
  int outLen;

  #if DEBUG_PRINT
  Serial.print(F("handleTypeEncryptionRequest, code = "));
  Serial.print(code, HEX);
  Serial.println(F("."));
  #endif

  switch(code)
  {






    case OPCODE_ENCRYPTION__PUMP_CHECK:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeEncryptionRequest: OPCODE_ENCRYPTION__PUMP_CHECK"));
      #endif
    
      outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OPCODE_ENCRYPTION__PUMP_CHECK, "OK", 2);
  
      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;
      break;
    case OPCODE_ENCRYPTION__CHECK_PASSKEY:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeEncryptionRequest: OPCODE_ENCRYPTION__CHECK_PASSKEY"));
      #endif

      (void)msgGetU8(&buf);   /*consume, length not needed yet*/

ctx->pump.pass = 7493;

      tempBuf[0] = 1; /*failure by default*/
      if(msgGetU16(&buf) == ((ctx->pump.pass) ^ 3463))
      {
        tempBuf[0] = 0; /*OK*/        
      }


      outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OPCODE_ENCRYPTION__CHECK_PASSKEY, tempBuf, 1);
  
      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;

      break;
    case OPCODE_ENCRYPTION__PASSKEY_REQUEST:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeEncryptionRequest: OPCODE_ENCRYPTION__PASSKEY_REQUEST"));
      #endif

      uint8_t *pairingKey = tempBuf;

      if(!ctx->pump.pairingRequested)
      {
        msgPutU16(&pairingKey, (ctx->pump.pass) ^ 3463);

        /*TODO: should send pairing key*/

        outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OPCODE_ENCRYPTION__PASSKEY_RETURN, tempBuf, msgLen(tempBuf, pairingKey)); /*0 = ok, continue*/
        ctx->pump.pairingRequested = true;    
      }
      else
      {

ctx->pump.pass = 7493;

        msgPutU16(&pairingKey, (ctx->pump.pass)^3463);

        outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OPCODE_ENCRYPTION__TIME_INFORMATION, tempBuf, msgLen(tempBuf, pairingKey)); 
      }

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);



      /*mark ok*/
      ret = 0;

      break;
    case OPCODE_ENCRYPTION__TIME_INFORMATION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeEncryptionRequest: OPCODE_ENCRYPTION__TIME_INFORMATION"));
      #endif
    
      uint8_t *b = tempBuf;

ctx->pump.pass = 7493;


      msgPutU16(&b, (ctx->pump.pass)^3463);

      outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OPCODE_ENCRYPTION__TIME_INFORMATION, tempBuf, msgLen(tempBuf, b)); /*0=OK, no need to request again*/
  
      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;
      break;

    default:
      #if DEBUG_PRINT
      Serial.print(F("handleTypeEncryptionRequest: unimplemented code "));
      Serial.print(code, HEX);
      Serial.println(F("."));
      #endif
      break;
  }

  return ret;
}

static int handleTypeCommand(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len)
{
  int ret = -1;

  switch(code)
  {
    default:
      #if DEBUG_PRINT
      Serial.print(F("handleTypeCommand: unimplemented code "));
      Serial.print(code, HEX);
      Serial.println(F("."));
      #endif
      break;
  }

  return ret;
}


static int handleTypeNotify(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len)
{
  int ret = -1;

  #if DEBUG_PRINT
  Serial.println(F("handleTypeEncryptionRequest"));
  #endif

  switch(code)
  {
    case OPCODE_NOTIFY__DELIVERY_COMPLETE:
      #if DEBUG_PRINT
      Serial.println(F("OPCODE_NOTIFY__DELIVERY_COMPLETE"));
      #endif
      break;

    case OPCODE_NOTIFY__DELIVERY_RATE_DISPLAY:
      #if DEBUG_PRINT
      Serial.println(F("OPCODE_NOTIFY__DELIVERY_RATE_DISPLAY"));
      #endif
      break;

    case OPCODE_NOTIFY__ALARM:
      #if DEBUG_PRINT
      Serial.println(F("OPCODE_NOTIFY__ALARM"));
      #endif
      break;

    case OPCODE_NOTIFY__MISSED_BOLUS_ALARM:
      #if DEBUG_PRINT
      Serial.println(F("OPCODE_NOTIFY__MISSED_BOLUS_ALARM"));
      #endif
      break;

    default:
      #if DEBUG_PRINT
      Serial.print(F("handleTypeNotify: unimplemented code "));
      Serial.print(code, HEX);
      Serial.println(F("."));
      #endif
      break;
  }

  return ret;
}





#define RESP_MAX_PAYLOAD_ALLOCATION    32

static int handleTypeResponse(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len)
{
  int ret = -1;
  uint8_t rawBuf[DANA_RAW_MSG_LEN(RESP_MAX_PAYLOAD_ALLOCATION)];
  uint8_t tempBuf[RESP_MAX_PAYLOAD_ALLOCATION];
  uint8_t *msgPtr = tempBuf;
  uint8_t *rPtr = &(buf[2]);
  int outLen;

  switch(code)
  {

    /*normal commands seem to come in as encryption messages*/
    case OPCODE_REVIEW__INITIAL_SCREEN_INFORMATION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__INITIAL_SCREEN_INFORMATION"));
      #endif
      msgPutU8(&msgPtr, ctx->pump.status);

ctx->pump.dailyTotalUnits = 26.67;
ctx->pump.maxDailyTotalUnits = 90.67;

ctx->pump.currentBasal = 1.5;
ctx->pump.iob = 2.3;
ctx->pump.batteryRemaining = 89;
ctx->pump.reservoirRemainingUnits = 70.7;


      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.dailyTotalUnits * 100.0));
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.maxDailyTotalUnits * 100.0));
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.reservoirRemainingUnits * 100.0));
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.currentBasal * 100.0));

      msgPutU8(&msgPtr, ctx->pump.tempBasalPercent);
      msgPutU8(&msgPtr, ctx->pump.batteryRemaining);

      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.extendedBolusAbsoluteRate * 100.0));
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.iob * 100.0));


      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_REVIEW__INITIAL_SCREEN_INFORMATION, tempBuf, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;
      break;


#if 0
    case OPCODE_REVIEW__DELIVERY_STATUS:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__DELIVERY_STATUS"));
      #endif
      break;

    case OPCODE_REVIEW__GET_PASSWORD:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__GET_PASSWORD"));
      #endif
      break;

    case OPCODE_REVIEW__BOLUS_AVG:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__BOLUS_AVG"));
      #endif
      break;

    case OPCODE_REVIEW__BOLUS:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__BOLUS"));
      #endif
      break;

    case OPCODE_REVIEW__DAILY:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__DAILY"));
      #endif
      break;

    case OPCODE_REVIEW__PRIME:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__PRIME"));
      #endif
      break;

    case OPCODE_REVIEW__REFILL:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__REFILL"));
      #endif
      break;

    case OPCODE_REVIEW__BLOOD_GLUCOSE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__BLOOD_GLUCOSE"));
      #endif
      break;

    case OPCODE_REVIEW__CARBOHYDRATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__CARBOHYDRATE"));
      #endif
      break;

    case OPCODE_REVIEW__TEMPORARY:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__TEMPORARY"));
      #endif
      break;

    case OPCODE_REVIEW__SUSPEND:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__SUSPEND"));
      #endif
      break;

    case OPCODE_REVIEW__ALARM:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__ALARM"));
      #endif
      break;

    case OPCODE_REVIEW__BASAL:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__BASAL"));
      #endif
      break;

    case OPCODE_REVIEW__ALL_HISTORY:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__ALL_HISTORY"));
      #endif
      break;
#endif
    case OPCODE_REVIEW__GET_SHIPPING_INFORMATION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__GET_SHIPPING_INFORMATION"));
      #endif

      

      for(int i=0; i<10; i++)
      {
        uint8_t s[] = "1234567890";
        ctx->pump.serialNumber[i] = s[i];
        msgPutU8(&msgPtr, ctx->pump.serialNumber[i]);
      }

      for(int i=0; i<3; i++)
      {
        uint8_t s[] = {2018-1900, 11-1, 1 };
        ctx->pump.shippingDate[i] = s[i];
        msgPutU8(&msgPtr, ctx->pump.shippingDate[i]);
      }
      for(int i=0; i<3; i++)
      {
        uint8_t s[] = "FIN";
        ctx->pump.shippingCountry[i] = s[i];
        msgPutU8(&msgPtr, ctx->pump.shippingCountry[i]);
      }

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_REVIEW__GET_SHIPPING_INFORMATION, tempBuf, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;


      break;
    case OPCODE_REVIEW__GET_PUMP_CHECK:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__GET_PUMP_CHECK"));
      #endif

      ctx->pump.model = 1;
      ctx->pump.protocol = 2;
      ctx->pump.productCode = 3;

      msgPutU8(&msgPtr, ctx->pump.model);
      msgPutU8(&msgPtr, ctx->pump.protocol);
      msgPutU8(&msgPtr, ctx->pump.productCode);

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_REVIEW__GET_PUMP_CHECK, tempBuf, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;

      break;

#if 0

    case OPCODE_REVIEW__GET_USER_TIME_CHANGE_FLAG:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__GET_USER_TIME_CHANGE_FLAG"));
      #endif
      break;

    case OPCODE_REVIEW__SET_USER_TIME_CHANGE_FLAG_CLEAR:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__SET_USER_TIME_CHANGE_FLAG_CLEAR"));
      #endif
      break;

    case OPCODE_REVIEW__GET_MORE_INFORMATION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__GET_MORE_INFORMATION"));
      #endif
      break;

    case OPCODE_REVIEW__SET_HISTORY_UPLOAD_MODE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__SET_HISTORY_UPLOAD_MODE"));
      #endif
      break;

    case OPCODE_REVIEW__GET_TODAY_DELIVERY_TOTAL:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_REVIEW__GET_TODAY_DELIVERY_TOTAL"));
      #endif
      break;
#endif

    case OPCODE_BOLUS__GET_STEP_BOLUS_INFORMATION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_STEP_BOLUS_INFORMATION"));
      #endif


ctx->pump.bolusType = 0;
ctx->pump.initialBolusAmount = 4.0;
ctx->pump.lastBolusTimeHour = 18;
ctx->pump.lastBolusTimeMinute = 47;
ctx->pump.lastBolusAmount = 5.0;
ctx->pump.maxBolus = 24;
ctx->pump.bolusStep = 0.10;


      msgPutU8(&msgPtr, ctx->pump.error);

      msgPutU8(&msgPtr, ctx->pump.bolusType);
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.initialBolusAmount * 100.0));
      msgPutU8(&msgPtr, ctx->pump.lastBolusTimeHour);
      msgPutU8(&msgPtr, ctx->pump.lastBolusTimeMinute);
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.lastBolusAmount * 100.0));
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.maxBolus * 100.0));
      msgPutU8(&msgPtr, (uint8_t)(ctx->pump.bolusStep * 100.0));


      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_BOLUS__GET_STEP_BOLUS_INFORMATION, tempBuf, msgLen(tempBuf, msgPtr));

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;



      break;


    case OPCODE_BOLUS__GET_EXTENDED_BOLUS_STATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_EXTENDED_BOLUS_STATE"));
      #endif

      msgPutU8(&msgPtr, ctx->pump.error);
      msgPutU8(&msgPtr, ctx->pump.isExtendedInProgress ? 0x01 : 0x00);
      msgPutU8(&msgPtr, ctx->pump.extendedBolusMinutes / 30);
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.extendedBolusAbsoluteRate * 100.0));
      msgPutU16(&msgPtr, ctx->pump.extendedBolusSoFarInMinutes);
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.extendedBolusDeliveredSoFar * 100.0));

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_BOLUS__GET_EXTENDED_BOLUS_STATE, tempBuf, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;
      break;




#if 0
    case OPCODE_BOLUS__GET_EXTENDED_BOLUS:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_EXTENDED_BOLUS"));
      #endif
      break;

    case OPCODE_BOLUS__GET_DUAL_BOLUS:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_DUAL_BOLUS"));
      #endif
      break;

    case OPCODE_BOLUS__SET_STEP_BOLUS_STOP:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__SET_STEP_BOLUS_STOP"));
      #endif
      break;

    case OPCODE_BOLUS__GET_CARBOHYDRATE_CALCULATION_INFORMATION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_CARBOHYDRATE_CALCULATION_INFORMATION"));
      #endif
      break;

    case OPCODE_BOLUS__GET_EXTENDED_MENU_OPTION_STATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_EXTENDED_MENU_OPTION_STATE"));
      #endif
      break;

    case OPCODE_BOLUS__SET_EXTENDED_BOLUS:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__SET_EXTENDED_BOLUS"));
      #endif
      break;

    case OPCODE_BOLUS__SET_DUAL_BOLUS:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__SET_DUAL_BOLUS"));
      #endif
      break;

    case OPCODE_BOLUS__SET_EXTENDED_BOLUS_CANCEL:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__SET_EXTENDED_BOLUS_CANCEL"));
      #endif
      break;

    case OPCODE_BOLUS__SET_STEP_BOLUS_START:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__SET_STEP_BOLUS_START"));
      #endif
      break;
#endif
    case OPCODE_BOLUS__GET_CALCULATION_INFORMATION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_CALCULATION_INFORMATION"));
      #endif



ctx->pump.currentTarget = 5;
ctx->pump.currentCIR = 20;
ctx->pump.currentCF = 6;
ctx->pump.iob = 4.5;
ctx->pump.units = UNITS_MMOL;


      msgPutU8(&msgPtr, ctx->pump.error);
      msgPutU16(&msgPtr, 0);    //currentBG
      msgPutU16(&msgPtr, 0);    //carbohydrate
      msgPutU16(&msgPtr, ctx->pump.currentTarget);
      msgPutU16(&msgPtr, ctx->pump.currentCIR);
      msgPutU16(&msgPtr, ctx->pump.currentCF);
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.iob * 100.0));
      msgPutU8(&msgPtr, ctx->pump.units);


      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_BOLUS__GET_CALCULATION_INFORMATION, tempBuf, msgLen(tempBuf, msgPtr));

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;
      break;
#if 0
    case OPCODE_BOLUS__GET_BOLUS_RATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_BOLUS_RATE"));
      #endif
      break;

    case OPCODE_BOLUS__SET_BOLUS_RATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__SET_BOLUS_RATE"));
      #endif
      break;
#endif
    case OPCODE_BOLUS__GET_CIR_CF_ARRAY:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_CIR_CF_ARRAY"));
      #endif
     


ctx->pump.language = 0;
ctx->pump.units = UNITS_MMOL;
ctx->pump.morningCIR = 20;
ctx->pump.afternoonCIR = 21;
ctx->pump.eveningCIR = 22;
ctx->pump.nightCIR = 23;
ctx->pump.morningCF = 6;
ctx->pump.afternoonCF = 7;
ctx->pump.eveningCF = 6;
ctx->pump.nightCF = 5;




      msgPutU8(&msgPtr, ctx->pump.language);
      msgPutU8(&msgPtr, ctx->pump.units);

      msgPutU16(&msgPtr, ctx->pump.morningCIR);
      msgPutU16(&msgPtr, ctx->pump.afternoonCIR);
      msgPutU16(&msgPtr, 0);
      msgPutU16(&msgPtr, ctx->pump.eveningCIR);
      msgPutU16(&msgPtr, 0);
      msgPutU16(&msgPtr, ctx->pump.nightCIR);


      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.morningCF * 100.0));
      msgPutU16(&msgPtr, 0);
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.afternoonCF * 100.0));
      msgPutU16(&msgPtr, 0);
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.eveningCF * 100.0));
      msgPutU16(&msgPtr, 0);
      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.nightCF * 100.0));




      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_BOLUS__GET_CIR_CF_ARRAY, tempBuf, msgLen(tempBuf, msgPtr));

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;

      break;
#if 0
    case OPCODE_BOLUS__SET_CIR_CF_ARRAY:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__SET_CIR_CF_ARRAY"));
      #endif
      break;
#endif
    case OPCODE_BOLUS__GET_BOLUS_OPTION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__GET_BOLUS_OPTION"));
      #endif

  ctx->pump.isExtendedBolusEnabled = 1;
  ctx->pump.bolusCalculationOption = 0;
  ctx->pump.missedBolusConfig = 0;

      msgPutU8(&msgPtr, ctx->pump.isExtendedBolusEnabled);
      msgPutU8(&msgPtr, ctx->pump.bolusCalculationOption);
      msgPutU8(&msgPtr, ctx->pump.missedBolusConfig);

      for(int i=0; i<16; i++)
        msgPutU8(&msgPtr, 0);


      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_BOLUS__GET_BOLUS_OPTION, tempBuf, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;



      break;
#if 0
    case OPCODE_BOLUS__SET_BOLUS_OPTION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BOLUS__SET_BOLUS_OPTION"));
      #endif
      break;

    case OPCODE_BASAL__SET_TEMPORARY_BASAL:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__SET_TEMPORARY_BASAL"));
      #endif
      break;
#endif
    case OPCODE_BASAL__TEMPORARY_BASAL_STATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__TEMPORARY_BASAL_STATE"));
      #endif

      msgPutU8(&msgPtr, ctx->pump.error);

      msgPutU8(&msgPtr, ctx->pump.isTempBasalInProgress);
      msgPutU8(&msgPtr, ctx->pump.tempBasalPercent);
      msgPutU8(&msgPtr, ctx->pump.tempBasalDurationHour);   /*TODO: 150==15min, 160==30min, otherwise hour*3600*/
      msgPutU16(&msgPtr, ctx->pump.tempBasalRunningMin);


      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_BASAL__TEMPORARY_BASAL_STATE, tempBuf, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;
      break;
#if 0 
    case OPCODE_BASAL__CANCEL_TEMPORARY_BASAL:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__CANCEL_TEMPORARY_BASAL"));
      #endif
      break;
#endif
    case OPCODE_BASAL__GET_PROFILE_NUMBER:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__GET_PROFILE_NUMBER"));
      #endif

ctx->pump.activeProfile = 2;

      msgPutU8(&msgPtr, ctx->pump.activeProfile);

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_BASAL__GET_PROFILE_NUMBER, tempBuf, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;

      break;

#if 0
    case OPCODE_BASAL__SET_PROFILE_NUMBER:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__SET_PROFILE_NUMBER"));
      #endif
      break;

    case OPCODE_BASAL__GET_PROFILE_BASAL_RATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__GET_PROFILE_BASAL_RATE"));
      #endif
      break;

    case OPCODE_BASAL__SET_PROFILE_BASAL_RATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__SET_PROFILE_BASAL_RATE"));
      #endif
      break;
#endif
    case OPCODE_BASAL__GET_BASAL_RATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__GET_BASAL_RATE"));
      #endif


ctx->pump.maxBasal = 4.0;
ctx->pump.basalStep = 0.1;  /*float as u8*/


      msgPutU16(&msgPtr, (uint16_t)(ctx->pump.maxBasal * 100.0));
      msgPutU8(&msgPtr, (uint8_t)(ctx->pump.basalStep * 100.0));

      for(int i=0; i<24; i++)
      {
        msgPutU16(&msgPtr, 0);  /*pump profile basals - not supported, so send zero*/
      }

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_BASAL__GET_BASAL_RATE, tempBuf, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;


      break;
#if 0
    case OPCODE_BASAL__SET_BASAL_RATE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__SET_BASAL_RATE"));
      #endif
      break;

    case OPCODE_BASAL__SET_SUSPEND_ON:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__SET_SUSPEND_ON"));
      #endif
      break;

    case OPCODE_BASAL__SET_SUSPEND_OFF:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__SET_SUSPEND_OFF"));
      #endif
      break;
#endif
    case OPCODE_OPTION__GET_PUMP_TIME:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_OPTION__GET_PUMP_TIME"));
      #endif
     
      time_t t = now(); // store the current time in time variable t
      msgPutU8(&msgPtr, (uint8_t)(year(t)-2000));
      msgPutU8(&msgPtr, (uint8_t)month(t));
      msgPutU8(&msgPtr, (uint8_t)day(t));
      msgPutU8(&msgPtr, (uint8_t)hour(t));
      msgPutU8(&msgPtr, (uint8_t)minute(t));
      msgPutU8(&msgPtr, (uint8_t)second(t));

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_OPTION__GET_PUMP_TIME, tempBuf, msgLen(tempBuf, msgPtr));

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;
      break;

    case OPCODE_OPTION__SET_PUMP_TIME:
      #if DEBUG_PRINT
      Serial.println("handleTypeResponse: OPCODE_OPTION__SET_PUMP_TIME");   /*F removed!*/
      #endif

      if(msgGetU8(&rPtr) == 6)
      {
        uint16_t year = msgGetU8(&rPtr) + 2000;
        uint8_t month = msgGetU8(&rPtr);
        uint8_t day = msgGetU8(&rPtr);
        uint8_t hour = msgGetU8(&rPtr);
        uint8_t minute = msgGetU8(&rPtr);
        uint8_t second = msgGetU8(&rPtr);

        setTime(hour, minute, second, day, month, year);
      }

      msgPutU8(&msgPtr, 0); /*time set ok*/

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OPCODE_OPTION__SET_PUMP_TIME, tempBuf, msgLen(tempBuf, msgPtr));

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);
      
      /*mark ok*/
      ret = 0;
      break;

#if 0
    case OPCODE_OPTION__GET_USER_OPTION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_OPTION__GET_USER_OPTION"));
      #endif
      break;

    case OPCODE_OPTION__SET_USER_OPTION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_OPTION__SET_USER_OPTION"));
      #endif
      break;

    case OPCODE_BASAL__APS_SET_TEMPORARY_BASAL:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_BASAL__APS_SET_TEMPORARY_BASAL"));
      #endif
      break;

    case OPCODE__APS_HISTORY_EVENTS:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE__APS_HISTORY_EVENTS"));
      #endif
      break;

    case OPCODE__APS_SET_EVENT_HISTORY:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE__APS_SET_EVENT_HISTORY"));
      #endif
      break;

    case OPCODE_ETC__SET_HISTORY_SAVE:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_ETC__SET_HISTORY_SAVE"));
      #endif
      break;

    case OPCODE_ETC__KEEP_CONNECTION:
      #if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OPCODE_ETC__KEEP_CONNECTION"));
      #endif
      break;





#endif






    default:
      #if DEBUG_PRINT
      Serial.print(F("handleTypeResponse: unimplemented code "));
      Serial.print(code, HEX);
      Serial.println(F("."));
      #endif
      break;
  }

  return ret;
}



static void sendToAAPS(OmniDanaContext_t *ctx, uint8_t *buf, int len)
{
  (void)ctx;

  if(buf && (len > 0))
  {
//    vTaskSuspendAll();
    Serial.flush();
    Serial.write(buf, (size_t)len);
    Serial.flush();
//    vTaskResumeAll();
  }
}

#if 0

static int danaMsgPumpCheck(OmniDanaContext_t *ctx, uint8_t *buf, uint8_t len)
{
  int ret = -1;
  uint8_t rawBuf[DANA_RAW_MSG_LEN(2)];  /*payload = 2bytes*/

  int outLen = createOutMessage(rawBuf, RSP_PUMP_CHECK, "OK", 2);
  
  /*send if a response was created*/
  sendToAAPS(ctx, rawBuf, outLen);

  return ret;
}

static int danaMsgPumpPasskeyRequest(OmniDanaContext_t *ctx, uint8_t *buf, uint8_t len)
{
  int ret = -1;
  uint8_t rawBuf[DANA_RAW_MSG_LEN(2)];  /*payload = 2bytes*/

  int outLen = createOutMessage(rawBuf, RSP_PUMP_PASSKEY_REQUEST, "11", 2);   /*just non zero is fine to ask for pairing request*/
  sendToAAPS(ctx, rawBuf, outLen);

  /*wait here?*/

  outLen = createOutMessage(rawBuf, RSP_PUMP_CHECK_PASSKEY_RETURN, "ak", 2);
  sendToAAPS(ctx, rawBuf, outLen);

  return ret;
}
#endif