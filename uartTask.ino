#include "common.h"
#include "uartTask.h"
#include "Crc16.h"
#include "msgBuf.h"

#define START_CHAR 0xA5
#define STOP_CHAR 0x5A

#define DEBUG_PRINT false
#define ERROR_PRINT true
#define IDLE_COUNTER_MAX 100

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

#define TYPE_ENCRYPTION_REQUEST 0x01
#define TYPE_ENCRYPTION_RESPONSE 0x02
#define TYPE_COMMAND 0xA1
#define TYPE_RESPONSE 0xB2
#define TYPE_NOTIFY 0xC3
#define OE_PUMP_CHECK 0x00
#define OE_CHECK_PASSKEY 0xD0
#define OE_PASSKEY_REQUEST 0xD1
#define OE_PASSKEY_RETURN 0xD2
#define OE_TIME_INFORMATION 0x01
#define ON_DELIVERY_COMPLETE 0x01
#define ON_DELIVERY_RATE_DISPLAY 0x02
#define ON_ALARM 0x03
#define ON_MISSED_BOLUS_ALARM 0x04
#define OR_INITIAL_SCREEN_INFORMATION 0x02
#define OR_DELIVERY_STATUS 0x03
#define OR_GET_PASSWORD 0x04
#define OR_BOLUS_AVG 0x10
#define OR_BOLUS 0x11
#define OR_DAILY 0x12
#define OR_PRIME 0x13
#define OR_REFILL 0x14
#define OR_BLOOD_GLUCOSE 0x15
#define OR_CARBOHYDRATE 0x16
#define OR_TEMPORARY 0x17
#define OR_SUSPEND 0x18
#define OR_ALARM 0x19
#define OR_BASAL 0x1A
#define OR_ALL_HISTORY 0x1F
#define OR_GET_SHIPPING_INFORMATION 0x20
#define OR_GET_PUMP_CHECK 0x21
#define OR_GET_USER_TIME_CHANGE_FLAG 0x22
#define OR_SET_USER_TIME_CHANGE_FLAG_CLEAR 0x23
#define OR_GET_MORE_INFORMATION 0x24
#define OR_SET_HISTORY_UPLOAD_MODE 0x25
#define OR_GET_TODAY_DELIVERY_TOTAL 0x26
#define OBO_GET_STEP_BOLUS_INFORMATION 0x40
#define OBO_GET_EXTENDED_BOLUS_STATE 0x41
#define OBO_GET_EXTENDED_BOLUS 0x42
#define OBO_GET_DUAL_BOLUS 0x43
#define OBO_SET_STEP_BOLUS_STOP 0x44
#define OBO_GET_CARBOHYDRATE_CALCULATION_INFORMATION 0x45
#define OBO_GET_EXTENDED_MENU_OPTION_STATE 0x46
#define OBO_SET_EXTENDED_BOLUS 0x47
#define OBO_SET_DUAL_BOLUS 0x48
#define OBO_SET_EXTENDED_BOLUS_CANCEL 0x49
#define OBO_SET_STEP_BOLUS_START 0x4A
#define OBO_GET_CALCULATION_INFORMATION 0x4B
#define OBO_GET_BOLUS_RATE 0x4C
#define OBO_SET_BOLUS_RATE 0x4D
#define OBO_GET_CIR_CF_ARRAY 0x4E
#define OBO_SET_CIR_CF_ARRAY 0x4F
#define OBO_GET_BOLUS_OPTION 0x50
#define OBO_SET_BOLUS_OPTION 0x51
#define OBA_SET_TEMPORARY_BASAL 0x60
#define OBA_TEMPORARY_BASAL_STATE 0x61
#define OBA_CANCEL_TEMPORARY_BASAL 0x62
#define OBA_GET_PROFILE_NUMBER 0x63
#define OBA_SET_PROFILE_NUMBER 0x64
#define OBA_GET_PROFILE_BASAL_RATE 0x65
#define OBA_SET_PROFILE_BASAL_RATE 0x66
#define OBA_GET_BASAL_RATE 0x67
#define OBA_SET_BASAL_RATE 0x68
#define OBA_SET_SUSPEND_ON 0x69
#define OBA_SET_SUSPEND_OFF 0x6A
#define OO_GET_PUMP_TIME 0x70
#define OO_SET_PUMP_TIME 0x71
#define OO_GET_USER_OPTION 0x72
#define OO_SET_USER_OPTION 0x73
#define OBA_APS_SET_TEMPORARY_BASAL 0xC1
#define OA_HISTORY_EVENTS 0xC2
#define OA_SET_EVENT_HISTORY 0xC3
#define OETC_SET_HISTORY_SAVE 0xE0
#define OETC_KEEP_CONNECTION 0xFF

#define DANARS_HISTORY_EVENT_RECORD_TEMPSTART                   1
#define DANARS_HISTORY_EVENT_RECORD_TEMPSTOP                    2
#define DANARS_HISTORY_EVENT_RECORD_EXTENDEDSTART               3
#define DANARS_HISTORY_EVENT_RECORD_EXTENDEDSTOP                4
#define DANARS_HISTORY_EVENT_RECORD_BOLUS                       5
#define DANARS_HISTORY_EVENT_RECORD_DUALBOLUS                   6
#define DANARS_HISTORY_EVENT_RECORD_DUALEXTENDEDSTART           7
#define DANARS_HISTORY_EVENT_RECORD_DUALEXTENDEDSTOP            8
#define DANARS_HISTORY_EVENT_RECORD_SUSPENDON                   9
#define DANARS_HISTORY_EVENT_RECORD_SUSPENDOFF                  10
#define DANARS_HISTORY_EVENT_RECORD_REFILL                      11
#define DANARS_HISTORY_EVENT_RECORD_PRIME                       12
#define DANARS_HISTORY_EVENT_RECORD_PROFILECHANGE               13
#define DANARS_HISTORY_EVENT_RECORD_CARBS                       14
#define DANARS_HISTORY_EVENT_RECORD_PRIMECANNULA                15



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
static void incomingFrameFollow(danaFrame_t *frame, uint8_t inByte);
static int createOutMessage(uint8_t *rawBuf, uint8_t type, uint8_t code, uint8_t *plBuf, uint8_t plLen);

static int handlePayload(OmniDanaContext_t *ctx, DanaMessage_t *dMsg);

static int handleTypeEncryptionRequest(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len);
static int handleTypeCommand(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len);
static int handleTypeNotify(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len);
static int handleTypeResponse(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len);

static int notifyDeliveryRateDisplay(OmniDanaContext_t *ctx, uint8_t *rawBuf, uint16_t deliveredInsulin, bool done);


static uint8_t rawBuf[DANA_RAW_MSG_LEN(DANA_MAX_PAYLOAD_LENGTH)];


void uartTaskInitialize(OmniDanaContext_t *ctx)
{
  Serial.println(F("uartTaskInitialize"));

  xTaskCreate(
      uartTask, 
      (const portCHAR *)"uartTask",
      220,
      (void *)ctx, 
      UART_TASK_PRIORITY, 
      NULL);
}

static void uartTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t *)pvParameters;
  danaFrame_t myDanaFrame; /*move this from stack to heap if this becomes too big.*/
  danaFrame_t *frame = &myDanaFrame;

#if DEBUG_PRINT
  Serial.print(F("uartTask: starting with ctx = "));
  Serial.print((uint16_t)ctx, HEX);
  Serial.println(".");
#endif

  /*start from scratch by resetting the incoming frame handler*/
  incomingFrameInitialize(frame);

  /*start receiving from uart.*/
  while (1)
  {
    bool somethingHappened = false;

    /*receive if there's something to receive*/
    somethingHappened |= receiveFromAAPS(ctx, frame);

    /*If nothing happened, let's rest a bit.*/
    if (!somethingHappened)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

static bool receiveFromAAPS(OmniDanaContext_t *ctx, danaFrame_t *frame)
{
  bool ret = false;

  size_t bytesAvailable = Serial.available();

  /*consume bytes available on uart, but do not go to next frame yet*/
  while ((bytesAvailable--) > 0)
  {
    uint8_t inByte = Serial.read();

#if DEBUG_PRINT
    if (inByte < 16)
      Serial.print(F("0"));
    Serial.print(inByte, HEX);
    Serial.print(F(" "));
#endif

    /*follow frame structure*/
    incomingFrameFollow(frame, inByte);

    /*when follower has marked the frame complete, let's send it to
    commTask*/
    if (frame->isReady)
    {
#if DEBUG_PRINT
      Serial.println(F("\r\nreceiveFromAAPS: Incoming frame ready!"));
#endif

      if (handlePayload(ctx, &(frame->danaMsg)) != 0)
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
  if (frame->idleCounter++ >= IDLE_COUNTER_MAX)
  {
    incomingFrameInitialize(frame);
    ret = true;
  }

  return ret;
}

static int createOutMessage(uint8_t *dBuf, uint8_t type, uint8_t code, uint8_t *plBuf, uint8_t plLen)
{
  int ret = -1;

  if(plLen <= DANA_MAX_PAYLOAD_LENGTH)
  {
    Crc16 crc;
    int idx = 0;
    uint8_t lenField = plLen + 2;

    /*start*/
    dBuf[idx++] = START_CHAR;
    dBuf[idx++] = START_CHAR;

    /*len*/
    dBuf[idx++] = lenField;

    /*cmd1: type*/
    dBuf[idx++] = type;

    /*cmd1: cmd*/
    dBuf[idx++] = code;

    /*payload: params*/
    if(plBuf != NULL)
    {
      /*user data comes from external buffer*/
      for (uint8_t i = 0; i < plLen; i++)
      {
        dBuf[idx++] = plBuf[i];
      }
    }
    else
    {
      /*user data is already at the correct position in dBuf. Just jumping to next phase.*/
      idx += plLen;
    }

    crc.clearCrc();
    uint16_t crcValue = crc.XModemCrc(dBuf, 3, lenField);

    /*crc1*/
    dBuf[idx++] = (uint8_t)((crcValue >> 8) & 0xFF);

    /*crc2*/
    dBuf[idx++] = (uint8_t)(crcValue & 0xFF);

    /*stop*/
    dBuf[idx++] = STOP_CHAR;
    dBuf[idx++] = STOP_CHAR;

    ret = idx;
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

  switch (frame->state)
  {
  case MSG_STATE_START:
    if (inByte == START_CHAR)
    {
      /*receiving started - mark busy to disable sender*/
      frame->isBusy = true;

      if (--(frame->stateRoundsLeft) == 0)
      {
        /*last round -> advance to next state*/
        frame->state = MSG_STATE_LEN;
        /*no stateRoundsLeft needed for this state*/
      }
      error = false;
    }
    break;

  case MSG_STATE_LEN:
    if (inByte <= DANA_MAX_LEN_FIELD)
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

    if (--(frame->stateRoundsLeft) == 0)
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

    if (dMsg->length < (DANA_MAX_PAYLOAD_LENGTH - 1))
    {
      dMsg->buf[dMsg->length++] = inByte;

      if (--(frame->stateRoundsLeft) == 0)
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

    if (--(frame->stateRoundsLeft) == 0)
    {
      Crc16 crc;

      dMsg = &(frame->danaMsg);

      /*calculate crc*/
      crc.clearCrc();
      uint16_t crcValue = crc.XModemCrc(dMsg->buf, 0, dMsg->length);

      if (frame->crcReceived == crcValue)
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
    if (inByte == STOP_CHAR)
    {
      if (--(frame->stateRoundsLeft) == 0)
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

  if (error)
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

  if (dMsg && (dMsg->length >= 2))
  {

#if DEBUG_PRINT
    Serial.print(F("handlePayload, type = "));
    Serial.print(dMsg->buf[0], HEX);
    Serial.print(F(", opcode = "));
    Serial.print(dMsg->buf[1], HEX);
    Serial.print(F(", len = "));
    Serial.print(dMsg->length, DEC);
    Serial.println(F("."));
#endif

    uint8_t *payLoadBuf = &(dMsg->buf[2]);
    uint8_t payLoadLen = dMsg->length - 2;

    switch (dMsg->buf[0])
    {
    case TYPE_ENCRYPTION_REQUEST:
      ret = handleTypeEncryptionRequest(ctx, dMsg->buf[1], payLoadBuf, payLoadLen);
      break;
    case TYPE_COMMAND:
      ret = handleTypeCommand(ctx, dMsg->buf[1], payLoadBuf, payLoadLen);
      break;
    case TYPE_NOTIFY:
      ret = handleTypeNotify(ctx, dMsg->buf[1], payLoadBuf, payLoadLen);
      break;
    case TYPE_RESPONSE:
      ret = handleTypeResponse(ctx, dMsg->buf[1], payLoadBuf, payLoadLen); /*commands sent as responses by AAPS.. a bit strange*/

      Serial.println(F("TYPE_RESPONSE done"));

      break;

    default:
#if DEBUG_PRINT
      Serial.print(F("handlePayload: Not implemented! type = "));
      Serial.print(dMsg->buf[0], HEX);
      Serial.print(F(", opcode = "));
      Serial.print(dMsg->buf[1], HEX);
      Serial.print(F(", data=["));
      for (uint8_t i = 0; i < dMsg->length; i++)
      {
        if (dMsg->buf[i] < 16)
          Serial.print(F("0"));
        Serial.print(dMsg->buf[i], HEX);
        if (i < (dMsg->length - 1))
          Serial.print(F(" "));
      }
      Serial.println(F("]."));
#endif
      break;
    }
  }
  return ret;
}



static int handleTypeEncryptionRequest(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len)
{
  int ret = -1;
//  uint8_t rawBuf[DANA_RAW_MSG_LEN(DANA_MAX_PAYLOAD_LENGTH)];
  uint8_t *tempBuf = &(rawBuf[5]); 
  uint8_t *msgPtr = tempBuf;
  int outLen;
  uint16_t tmpu16;
  uint8_t tmpu8;

  (void)len;

#if DEBUG_PRINT
  Serial.print(F("handleTypeEncryptionRequest, code = "));
  Serial.print(code, HEX);
  Serial.println(F(".JEE"));
#endif

  switch (code)
  {
  case OE_PUMP_CHECK:
#if DEBUG_PRINT
    Serial.println(F("handleTypeEncryptionRequest: OE_PUMP_CHECK"));
#endif

    msgPutU8(&msgPtr, 'O');
    msgPutU8(&msgPtr, 'K');

    outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OE_PUMP_CHECK, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;
    break;

  case OE_CHECK_PASSKEY:
#if DEBUG_PRINT
    Serial.println(F("handleTypeEncryptionRequest: OE_CHECK_PASSKEY"));
#endif

    // /*pairing NOT requested: add non zero response if pairing is not yet done*/

    /*ignore lenght*/
    (void)msgGetU8(&buf);

    tmpu16 = msgGetU16(&buf);

    tmpu8 = 1;  //result fail by default
    if (tmpu16 == ((ctx->pump.pass) ^ 0x3463))
    {
      /*pairing ok, continue to time info*/
      tmpu8 = 0;
    }

    msgPutU8(&msgPtr, tmpu8);

    outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OE_CHECK_PASSKEY, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;

  case OE_PASSKEY_REQUEST:
#if DEBUG_PRINT
    Serial.println(F("handleTypeEncryptionRequest: OE_PASSKEY_REQUEST"));
#endif

    /*send OK response*/
    msgPutU8(&msgPtr, 0);

    outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OE_PASSKEY_REQUEST, NULL, msgLen(tempBuf, msgPtr));
    ctx->pump.pairingRequested = true;

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    msgPtr = tempBuf;

    /*and then send OK : pairing button pressed*/
    msgPutU16(&msgPtr, (ctx->pump.pass) ^ 0x3463);

    outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OE_PASSKEY_RETURN, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;

  case OE_TIME_INFORMATION:
#if DEBUG_PRINT
    Serial.println(F("handleTypeEncryptionRequest: OE_TIME_INFORMATION"));
#endif

    msgPutU16(&msgPtr, (ctx->pump.pass) ^ 3463);

    outLen = createOutMessage(rawBuf, TYPE_ENCRYPTION_RESPONSE, OE_TIME_INFORMATION, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;
    break;

  default:
#if ERROR_PRINT
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

  (void)ctx;
  (void)buf;
  (void)len;

  switch (code)
  {
  default:
#if ERROR_PRINT
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

  (void)ctx;
  (void)buf;
  (void)len;

#if DEBUG_PRINT
  Serial.println(F("handleTypeEncryptionRequest"));
#endif

  switch (code)
  {
  case ON_DELIVERY_COMPLETE:
#if DEBUG_PRINT
    Serial.println(F("ON_DELIVERY_COMPLETE"));
#endif
    break;

  case ON_DELIVERY_RATE_DISPLAY:
#if DEBUG_PRINT
    Serial.println(F("ON_DELIVERY_RATE_DISPLAY"));
#endif
    break;

  case ON_ALARM:
#if DEBUG_PRINT
    Serial.println(F("ON_ALARM"));
#endif
    break;

  case ON_MISSED_BOLUS_ALARM:
#if DEBUG_PRINT
    Serial.println(F("ON_MISSED_BOLUS_ALARM"));
#endif
    break;

  default:
#if ERROR_PRINT
    Serial.print(F("handleTypeNotify: unimplemented code "));
    Serial.print(code, HEX);
    Serial.println(F("."));
#endif
    break;
  }

  return ret;
}

static int notifyDeliveryRateDisplay(OmniDanaContext_t *ctx, uint8_t *dBuf, uint16_t deliveredInsulin, bool done)
{
  int ret = -1;
  if(dBuf)
  {
    uint8_t *tempBuf = &(dBuf[5]);
    uint8_t *msgPtr = tempBuf;
    int outLen;

    msgPutU16(&msgPtr, (uint16_t)deliveredInsulin);

    outLen = createOutMessage(dBuf, TYPE_NOTIFY, (done ? ON_DELIVERY_COMPLETE : ON_DELIVERY_RATE_DISPLAY), NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, dBuf, outLen);

    ret = 0;
  }
  return ret;
}


static int handleTypeResponse(OmniDanaContext_t *ctx, uint8_t code, uint8_t *buf, uint8_t len)
{
  int ret = -1;
  uint8_t *tempBuf = &(rawBuf[5]);
  uint8_t *msgPtr = tempBuf;
  int outLen;

  (void)len;

  switch (code)
  {

  /*normal commands seem to come in as encryption messages*/
  case OR_INITIAL_SCREEN_INFORMATION:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OR_INITIAL_SCREEN_INFORMATION"));
#endif
    msgPutU8(&msgPtr, ctx->pump.status);

    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.dailyTotalUnits * 100.0));
    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.maxDailyTotalUnits * 100.0));
    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.reservoirRemainingUnits * 100.0));
    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.currentBasal * 100.0));

    msgPutU8(&msgPtr, ctx->pump.tempBasalPercent);
    msgPutU8(&msgPtr, ctx->pump.batteryRemaining);

    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.extendedBolusAbsoluteRate * 100.0));
    msgPutU16(&msgPtr, 0);//(uint16_t)(ctx->pump.iob * 100.0));

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OR_INITIAL_SCREEN_INFORMATION, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;
    break;

#if 0
    case OR_DELIVERY_STATUS:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_DELIVERY_STATUS"));
#endif
      break;

    case OR_GET_PASSWORD:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_GET_PASSWORD"));
#endif
      break;

    case OR_BOLUS_AVG:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_BOLUS_AVG"));
#endif
      break;

    case OR_BOLUS:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_BOLUS"));
#endif
      break;

    case OR_DAILY:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_DAILY"));
#endif
      break;

    case OR_PRIME:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_PRIME"));
#endif
      break;

    case OR_REFILL:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_REFILL"));
#endif
      break;

    case OR_BLOOD_GLUCOSE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_BLOOD_GLUCOSE"));
#endif
      break;

    case OR_CARBOHYDRATE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_CARBOHYDRATE"));
#endif
      break;

    case OR_TEMPORARY:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_TEMPORARY"));
#endif
      break;

    case OR_SUSPEND:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_SUSPEND"));
#endif
      break;

    case OR_ALARM:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_ALARM"));
#endif
      break;

    case OR_BASAL:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_BASAL"));
#endif
      break;

    case OR_ALL_HISTORY:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_ALL_HISTORY"));
#endif
      break;
  case OR_GET_SHIPPING_INFORMATION:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OR_GET_SHIPPING_INFORMATION"));
#endif

    for (int i = 0; i < 10; i++)
    {
      msgPutU8(&msgPtr, ctx->pump.serialNumber[i]);
    }

    for (int i = 0; i < 3; i++)
    {
      msgPutU8(&msgPtr, ctx->pump.shippingDate[i]);
    }
    for (int i = 0; i < 3; i++)
    {
      msgPutU8(&msgPtr, ctx->pump.shippingCountry[i]);
    }

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OR_GET_SHIPPING_INFORMATION, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;
#endif

  case OR_GET_PUMP_CHECK:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OR_GET_PUMP_CHECK"));
#endif

    msgPutU8(&msgPtr, ctx->pump.model);
    msgPutU8(&msgPtr, ctx->pump.protocol);
    msgPutU8(&msgPtr, ctx->pump.productCode);

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OR_GET_PUMP_CHECK, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;

#if 0

    case OR_GET_USER_TIME_CHANGE_FLAG:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_GET_USER_TIME_CHANGE_FLAG"));
#endif
      break;

    case OR_SET_USER_TIME_CHANGE_FLAG_CLEAR:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_SET_USER_TIME_CHANGE_FLAG_CLEAR"));
#endif
      break;

    case OR_GET_MORE_INFORMATION:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_GET_MORE_INFORMATION"));
#endif
      break;

    case OR_SET_HISTORY_UPLOAD_MODE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_SET_HISTORY_UPLOAD_MODE"));
#endif
      break;

    case OR_GET_TODAY_DELIVERY_TOTAL:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OR_GET_TODAY_DELIVERY_TOTAL"));
#endif
      break;
#endif

  case OBO_GET_STEP_BOLUS_INFORMATION:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OBO_GET_STEP_BOLUS_INFORMATION"));
#endif

    msgPutU8(&msgPtr, ctx->pump.error);

    msgPutU8(&msgPtr, ctx->pump.bolusType);
    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.initialBolusAmount * 100.0));
    msgPutU8(&msgPtr, ctx->pump.lastBolusTimeHour);
    msgPutU8(&msgPtr, ctx->pump.lastBolusTimeMinute);
    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.lastBolusAmount * 100.0));
    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.maxBolus * 100.0));
    msgPutU8(&msgPtr, (uint8_t)(ctx->pump.bolusStep * 100.0));

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_GET_STEP_BOLUS_INFORMATION, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;

  case OBO_GET_EXTENDED_BOLUS_STATE:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OBO_GET_EXTENDED_BOLUS_STATE"));
#endif

    msgPutU8(&msgPtr, ctx->pump.error);
    msgPutU8(&msgPtr, ctx->pump.isExtendedInProgress ? 0x01 : 0x00);


  Serial.print(F("isExtendedInProgress="));
if(ctx->pump.isExtendedInProgress)
  Serial.println(F("true"));
else
  Serial.println(F("false"));


    msgPutU8(&msgPtr, ctx->pump.extendedBolusMinutes / 30);
    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.extendedBolusAbsoluteRate * 100.0));
    msgPutU16(&msgPtr, ctx->pump.extendedBolusSoFarInMinutes);
    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.extendedBolusDeliveredSoFar * 100.0));

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_GET_EXTENDED_BOLUS_STATE, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;
    break;

#if 0
    case OBO_GET_EXTENDED_BOLUS:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_GET_EXTENDED_BOLUS"));
#endif
      break;

    case OBO_GET_DUAL_BOLUS:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_GET_DUAL_BOLUS"));
#endif
      break;
#endif



    case OBO_SET_STEP_BOLUS_STOP:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_SET_STEP_BOLUS_STOP"));
#endif


      msgPutU8(&msgPtr, 0);   //result == OK

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_SET_STEP_BOLUS_STOP, NULL, msgLen(tempBuf, msgPtr));

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);



      break;


#if 0
    case OBO_GET_CARBOHYDRATE_CALCULATION_INFORMATION:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_GET_CARBOHYDRATE_CALCULATION_INFORMATION"));
#endif
      break;

    case OBO_GET_EXTENDED_MENU_OPTION_STATE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_GET_EXTENDED_MENU_OPTION_STATE"));
#endif
      break;
#endif

    case OBO_SET_EXTENDED_BOLUS:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_SET_EXTENDED_BOLUS"));
#endif





      msgGetU8(&buf);

      { //variable scope
        uint16_t amountU16 = msgGetU16(&buf);
        uint8_t timeInHalfHours = msgGetU8(&buf);

#if DEBUG_PRINT
        Serial.print(F("extended = "));
        Serial.print(amountU16, DEC);
        Serial.print(F("/100 u, time="));
        Serial.print(timeInHalfHours*30, DEC);
        Serial.println(F("minutes."));
#endif



        ctx->pump.extendedBolusStartTime = now();

        ctx->pump.isExtendedInProgress = true;
        ctx->pump.extendedBolusMinutes = timeInHalfHours * 30;
        ctx->pump.extendedBolusAmount = ((float)amountU16) / 100.0;
        //float h = ((float)timeInHalfHours) / 2.0;

        ctx->pump.extendedBolusAbsoluteRate =  ctx->pump.extendedBolusAmount;  //ZAIM pitäis kai olla jaettu tunneilla?  / h;

        ctx->pump.extendedBolusSoFarInMinutes = 0;
        ctx->pump.extendedBolusDeliveredSoFar = 0;




        TreatmentMessage_t tr;
        tr.treatment = TREATMENT_EXTENDED_BOLUS_START;
        tr.param1 = amountU16;  /*total*/
        tr.param2 = 0;  /*now*/
        tr.param3 = (uint16_t)timeInHalfHours;
        xMessageBufferSend(ctx->commToCtrlBuffer, &tr, sizeof(TreatmentMessage_t), 0);








        msgPutU8(&msgPtr, 0);   //result == OK

        outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_SET_EXTENDED_BOLUS, NULL, msgLen(tempBuf, msgPtr));

        /*send if a response was created*/
        sendToAAPS(ctx, rawBuf, outLen);


#if 0

        /*1u: 20 clicks, takes 39sec -> 1 click takes 2 seconds. 1 click == 0.05u -> amountU16 = 5. Required clicks = amountU16 / 5*/
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        uint16_t numberOfClicks = amountU16 / 5;
        for(uint16_t x=0; x<numberOfClicks; x++)
        {
          notifyDeliveryRateDisplay(ctx, rawBuf, (x+1)*5, (x==(numberOfClicks-1)));
          vTaskDelay(2000 / portTICK_PERIOD_MS);
          ctx->pump.iob += ctx->pump.bolusStep;
        }


#endif

      }

      /*mark ok*/
      ret = 0;
      break;















      break;

#if 0

    case OBO_SET_DUAL_BOLUS:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_SET_DUAL_BOLUS"));
#endif
      break;
#endif

    case OBO_SET_EXTENDED_BOLUS_CANCEL:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_SET_EXTENDED_BOLUS_CANCEL"));
#endif
      if(1)//ctx->pump.isExtendedInProgress)
      {

        ctx->pump.extendedBolusStopTime = now();

        ctx->pump.isExtendedInProgress = false;




        TreatmentMessage_t tr;
        tr.treatment = TREATMENT_EXTENDED_BOLUS_STOP;
        tr.param1 = 0;
        tr.param2 = 0;
        tr.param3 = 0;
        xMessageBufferSend(ctx->commToCtrlBuffer, &tr, sizeof(TreatmentMessage_t), 0);





        msgPutU8(&msgPtr, 0);   //result == OK

      }
      else
      {
        msgPutU8(&msgPtr, 1);   //result == failed
      }
  
      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_SET_EXTENDED_BOLUS_CANCEL, NULL, msgLen(tempBuf, msgPtr));

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);

      ret = 0;
      break;

    case OBO_SET_STEP_BOLUS_START:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_SET_STEP_BOLUS_START"));
#endif

      msgGetU8(&buf);

      { //variable scope
        uint16_t amountU16 = msgGetU16(&buf);
        uint8_t speed = msgGetU8(&buf);

        (void)speed;

#if DEBUG_PRINT
        Serial.print(F("amount = "));
        Serial.print(amountU16, DEC);
        Serial.println(F("/100 u."));
#endif

        TreatmentMessage_t tr;
        tr.treatment = TREATMENT_BOLUS_START;
        tr.param1 = amountU16;
        tr.param2 = 0;
        tr.param3 = 0;
        xMessageBufferSend(ctx->commToCtrlBuffer, &tr, sizeof(TreatmentMessage_t), 0);




        time_t t = now();

        ctx->pump.initialBolusAmount = 0.0;
        ctx->pump.lastBolusTimeHour = (uint8_t)hour(t);
        ctx->pump.lastBolusTimeMinute = (uint8_t)minute(t);
        ctx->pump.lastBolusAmount = (((float)amountU16)/100.0);





        msgPutU8(&msgPtr, 0);   //result == OK

        outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_SET_STEP_BOLUS_START, NULL, msgLen(tempBuf, msgPtr));

        /*send if a response was created*/
        sendToAAPS(ctx, rawBuf, outLen);




        /*1u: 20 clicks, takes 39sec -> 1 click takes 2 seconds. 1 click == 0.05u -> amountU16 = 5. Required clicks = amountU16 / 5*/
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        uint16_t numberOfClicks = amountU16 / 5;
        for(uint16_t x=0; x<numberOfClicks; x++)
        {
          notifyDeliveryRateDisplay(ctx, rawBuf, (x+1)*5, (x==(numberOfClicks-1)));
          vTaskDelay(2000 / portTICK_PERIOD_MS);
          //ctx->pump.iob += ctx->pump.bolusStep;
        }




      }

      /*mark ok*/
      ret = 0;
      break;

#if 0
  case OBO_GET_CALCULATION_INFORMATION:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OBO_GET_CALCULATION_INFORMATION"));
#endif

    ctx->pump.currentTarget = 5;
    ctx->pump.currentCIR = 20;
    ctx->pump.currentCF = 6;
    ctx->pump.units = UNITS_MMOL;

    msgPutU8(&msgPtr, ctx->pump.error);
    msgPutU16(&msgPtr, 0); //currentBG
    msgPutU16(&msgPtr, 0); //carbohydrate
    msgPutU16(&msgPtr, 0);//ctx->pump.currentTarget);
    msgPutU16(&msgPtr, 0);//ctx->pump.currentCIR);
    msgPutU16(&msgPtr, 0);//ctx->pump.currentCF);
    msgPutU16(&msgPtr, 0); //(uint16_t)(ctx->pump.iob * 100.0));
    msgPutU8(&msgPtr, ctx->pump.units);

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_GET_CALCULATION_INFORMATION, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;
    break;

    case OBO_GET_BOLUS_RATE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_GET_BOLUS_RATE"));
#endif
      break;

    case OBO_SET_BOLUS_RATE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_SET_BOLUS_RATE"));
#endif
      break;

  case OBO_GET_CIR_CF_ARRAY:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OBO_GET_CIR_CF_ARRAY"));
#endif

    msgPutU8(&msgPtr, ctx->pump.language);
    msgPutU8(&msgPtr, ctx->pump.units);

    msgPutU16(&msgPtr, ctx->pump.morningCIR);
    msgPutU16(&msgPtr, 0);
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

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_GET_CIR_CF_ARRAY, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;

    case OBO_SET_CIR_CF_ARRAY:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_SET_CIR_CF_ARRAY"));
#endif
      break;
#endif
  case OBO_GET_BOLUS_OPTION:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OBO_GET_BOLUS_OPTION"));
#endif

    ctx->pump.isExtendedBolusEnabled = 1;
    ctx->pump.bolusCalculationOption = 0;
    ctx->pump.missedBolusConfig = 0;

    msgPutU8(&msgPtr, ctx->pump.isExtendedBolusEnabled);
    msgPutU8(&msgPtr, ctx->pump.bolusCalculationOption);
    msgPutU8(&msgPtr, ctx->pump.missedBolusConfig);

    for (int i = 0; i < 16; i++)
      msgPutU8(&msgPtr, 0);

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBO_GET_BOLUS_OPTION, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;
#if 0
    case OBO_SET_BOLUS_OPTION:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBO_SET_BOLUS_OPTION"));
#endif
      break;
#endif


    case OBA_SET_TEMPORARY_BASAL:
      if (msgGetU8(&buf) == 5)
      {
//#if DEBUG_PRINT
        Serial.println(F("handleTypeResponse: OBA_SET_TEMPORARY_BASAL"));
//#endif
        uint16_t newBasalRate = msgGetU16(&buf);
        uint16_t newBasalRateInPercent = msgGetU16(&buf);
        uint8_t basalDurationMinutes = msgGetU8(&buf);

        //uint16_t newBasalRate = (currentBasalRate*newBasalRateInPercent)/100;

        ctx->pump.tempBasalStartTime = now();

        ctx->pump.isTempBasalInProgress = 1;
        ctx->pump.tempBasalPercent = newBasalRateInPercent;
        
        ctx->pump.tempBasalDurationMinutes = basalDurationMinutes;
        ctx->pump.tempBasalRunningMin = 0;

#if 1
Serial.print(F("Set TempBasal:"));
Serial.print(newBasalRate, DEC);
Serial.print(F("u/hr,"));
Serial.print(ctx->pump.tempBasalPercent, DEC);
Serial.print(F("%,"));
Serial.print(ctx->pump.tempBasalDurationMinutes, DEC);
Serial.println(F("min"));
#endif


        msgPutU8(&msgPtr, 0);   //OK

        outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBA_SET_TEMPORARY_BASAL, NULL, msgLen(tempBuf, msgPtr)); 

        /*send if a response was created*/
        sendToAAPS(ctx, rawBuf, outLen);



        TreatmentMessage_t tr;
        tr.treatment = TREATMENT_TEMPORARY_BASAL_RATE_START;
        tr.param1 = newBasalRate;
        tr.param2 = basalDurationMinutes;
        tr.param3 = 0;
        xMessageBufferSend(ctx->commToCtrlBuffer, &tr, sizeof(TreatmentMessage_t), 0);

        /*mark ok*/
        ret = 0;

      }
      break;


  case OBA_TEMPORARY_BASAL_STATE:
#if 1//DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OBA_TEMPORARY_BASAL_STATE"));
#endif
    msgPutU8(&msgPtr, ctx->pump.error);

    msgPutU8(&msgPtr, ctx->pump.isTempBasalInProgress);
    msgPutU16(&msgPtr, ctx->pump.tempBasalPercent);
    msgPutU8(&msgPtr, ctx->pump.tempBasalDurationMinutes);
    msgPutU16(&msgPtr, ctx->pump.tempBasalRunningMin);




    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBA_TEMPORARY_BASAL_STATE, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;
    break;
  case OBA_CANCEL_TEMPORARY_BASAL:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBA_CANCEL_TEMPORARY_BASAL"));
#endif
      ctx->pump.isTempBasalInProgress = 0;
      ctx->pump.tempBasalStopTime = now();

      msgPutU8(&msgPtr, 0); //OK

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBA_CANCEL_TEMPORARY_BASAL, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);

      /*mark ok*/
      ret = 0;


      break;

  case OBA_GET_PROFILE_NUMBER:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OBA_GET_PROFILE_NUMBER"));
#endif

    msgPutU8(&msgPtr, ctx->pump.activeProfile);

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBA_GET_PROFILE_NUMBER, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;


    case OBA_SET_PROFILE_NUMBER:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBA_SET_PROFILE_NUMBER"));
#endif

      (void)msgGetU8(&buf);   //skip len

      ctx->pump.activeProfile = msgGetU8(&buf);

      msgPutU8(&msgPtr, 0); //OK

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBA_SET_PROFILE_NUMBER, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);

      /*mark ok*/
      ret = 0;


      break;
#if 0
    case OBA_GET_PROFILE_BASAL_RATE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBA_GET_PROFILE_BASAL_RATE"));
#endif
      break;
#endif

    case OBA_SET_PROFILE_BASAL_RATE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBA_SET_PROFILE_BASAL_RATE"));
#endif

      (void)msgGetU8(&buf);   //skip len

      ctx->pump.activeProfile = msgGetU8(&buf);
      
      for(int i=0; i<24; i++)
      {
        //ctx->pump.profileBasal[i] = 
        (void)msgGetU16(&buf);
      }

      msgPutU8(&msgPtr, 0); //OK

      outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBA_SET_PROFILE_BASAL_RATE, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

      /*send if a response was created*/
      sendToAAPS(ctx, rawBuf, outLen);

      /*mark ok*/
      ret = 0;
      break;

  case OBA_GET_BASAL_RATE:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OBA_GET_BASAL_RATE"));
#endif

    ctx->pump.maxBasal = 4.0;
    ctx->pump.basalStep = 0.01; /*float as u8*/   /*MUST be 0.01, otherwise there will be a notification about this*/

    msgPutU16(&msgPtr, (uint16_t)(ctx->pump.maxBasal * 100.0));
    msgPutU8(&msgPtr, (uint8_t)(ctx->pump.basalStep * 100.0));

    for (int i = 0; i < 24; i++)
    {
      msgPutU16(&msgPtr, 0);    //ctx->pump.profileBasal[i]);
    }

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBA_GET_BASAL_RATE, NULL, msgLen(tempBuf, msgPtr)); /*0=OK, no need to request again*/

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;

    break;
#if 0
    case OBA_SET_BASAL_RATE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBA_SET_BASAL_RATE"));
#endif
      break;

    case OBA_SET_SUSPEND_ON:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBA_SET_SUSPEND_ON"));
#endif
      break;

    case OBA_SET_SUSPEND_OFF:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OBA_SET_SUSPEND_OFF"));
#endif
      break;
#endif
  case OO_GET_PUMP_TIME:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OO_GET_PUMP_TIME"));
#endif

    {                   // for scope of t
      time_t t = now(); // store the current time in time variable t
      msgPutU8(&msgPtr, (uint8_t)(year(t) - 2000));
      msgPutU8(&msgPtr, (uint8_t)month(t));
      msgPutU8(&msgPtr, (uint8_t)day(t));
      msgPutU8(&msgPtr, (uint8_t)hour(t));
      msgPutU8(&msgPtr, (uint8_t)minute(t));
      msgPutU8(&msgPtr, (uint8_t)second(t));
    }

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OO_GET_PUMP_TIME, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;
    break;

  case OO_SET_PUMP_TIME:
#if DEBUG_PRINT
    Serial.println(F("handleTypeResponse: OO_SET_PUMP_TIME"));
#endif

    if (msgGetU8(&buf) == 6)
    {
      uint16_t year = msgGetU8(&buf) + 2000;
      uint8_t month = msgGetU8(&buf);
      uint8_t day = msgGetU8(&buf);
      uint8_t hour = msgGetU8(&buf);
      uint8_t minute = msgGetU8(&buf);
      uint8_t second = msgGetU8(&buf);

#if DEBUG_PRINT
      Serial.println(F("setTime"));
#endif

      setTime(hour, minute, second, day, month, year);
    }

    msgPutU8(&msgPtr, 0); /*time set ok*/

    outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OO_SET_PUMP_TIME, NULL, msgLen(tempBuf, msgPtr));

    /*send if a response was created*/
    sendToAAPS(ctx, rawBuf, outLen);

    /*mark ok*/
    ret = 0;
    break;

#if 0
    case OO_GET_USER_OPTION:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OO_GET_USER_OPTION"));
#endif
      break;

    case OO_SET_USER_OPTION:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OO_SET_USER_OPTION"));
#endif
      break;

#endif
    case OBA_APS_SET_TEMPORARY_BASAL:
        if (msgGetU8(&buf) == 5)
        {
//#if DEBUG_PRINT
        Serial.println(F("handleTypeResponse: OBA_APS_SET_TEMPORARY_BASAL"));
//#endif
        uint16_t newBasalRate = msgGetU16(&buf);
        uint16_t newBasalRateInPercent = msgGetU16(&buf);
        uint8_t basalDurationMinutes = msgGetU8(&buf);

        ctx->pump.tempBasalStartTime = now();

        ctx->pump.isTempBasalInProgress = 1;
        ctx->pump.tempBasalPercent = newBasalRateInPercent;
        
        ctx->pump.tempBasalDurationMinutes = basalDurationMinutes;
        ctx->pump.tempBasalRunningMin = 0;


#if 1
Serial.print(F("APS Set TempBasal:"));
Serial.print(newBasalRate, DEC);
Serial.print(F("u/hr,"));
Serial.print(ctx->pump.tempBasalPercent, DEC);
Serial.print(F("%,"));
Serial.print(ctx->pump.tempBasalDurationMinutes, DEC);
Serial.println(F("min"));
#endif

        msgPutU8(&msgPtr, 0);   //OK

        outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OBA_APS_SET_TEMPORARY_BASAL, NULL, msgLen(tempBuf, msgPtr)); 

        /*send if a response was created*/
        sendToAAPS(ctx, rawBuf, outLen);

        TreatmentMessage_t tr;
        tr.treatment = TREATMENT_TEMPORARY_BASAL_RATE_START;
        tr.param1 = newBasalRate;
        tr.param2 = basalDurationMinutes;
        tr.param3 = 0;
        xMessageBufferSend(ctx->commToCtrlBuffer, &tr, sizeof(TreatmentMessage_t), 0);

        /*mark ok*/
        ret = 0;

      }
      break;



    case OA_HISTORY_EVENTS:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OA_HISTORY_EVENTS"));
#endif
      if (msgGetU8(&buf) == 6)
      {
        bool moreToCome = true;
        time_t eventsSince = msgGetTimeDate(&buf);

    /*TODO: Separate this code into a sub func*/


/*
#define DANARS_HISTORY_EVENT_RECORD_EXTENDEDSTART               3
#define DANARS_HISTORY_EVENT_RECORD_EXTENDEDSTOP                4

*/


        while(moreToCome)
        {
          uint8_t *msgPtr = tempBuf;
          bool ebStartToBeSent = (ctx->pump.extendedBolusStartTime > ctx->pump.extendedBolusStartLastReported);
          bool ebStopToBeSent = (ctx->pump.extendedBolusStopTime > ctx->pump.extendedBolusStopLastReported);
          bool tbStartToBeSent = (ctx->pump.tempBasalStartTime > ctx->pump.tempBasalStartLastReported);
          bool tbStopToBeSent = (ctx->pump.tempBasalStopTime > ctx->pump.tempBasalStopLastReported);

Serial.print(F("OA_HISTORY_EVENTS("));
Serial.print(ebStartToBeSent, DEC);
Serial.print(F(","));
Serial.print(ebStopToBeSent, DEC);
Serial.print(F(":"));
Serial.print(tbStartToBeSent, DEC);
Serial.print(F(","));
Serial.print(tbStopToBeSent, DEC);
Serial.println(F(")"));

          if((ctx->pump.extendedBolusStartTime >= eventsSince) && ebStartToBeSent)
          {
            /*mark this as sent*/
            ctx->pump.extendedBolusStartLastReported = ctx->pump.extendedBolusStartTime;

  Serial.println(F("report extendedBolus start"));

            msgPutU8(&msgPtr, DANARS_HISTORY_EVENT_RECORD_EXTENDEDSTART);

            msgPutTimeDate(&msgPtr, ctx->pump.extendedBolusStartTime);
            
            msgPutU16InvertedOrder(&msgPtr, (uint16_t)(ctx->pump.extendedBolusAmount * 100.0));
            msgPutU16InvertedOrder(&msgPtr, ctx->pump.extendedBolusMinutes);

          }
          else if((ctx->pump.extendedBolusStopTime >= eventsSince) && ebStopToBeSent)
          {
            /*mark this as sent*/
            ctx->pump.extendedBolusStopLastReported = ctx->pump.extendedBolusStopTime;

  Serial.println(F("report extendedBolus stop"));

            msgPutU8(&msgPtr, DANARS_HISTORY_EVENT_RECORD_EXTENDEDSTOP);

            msgPutTimeDate(&msgPtr, ctx->pump.extendedBolusStopTime);
            
            msgPutU16InvertedOrder(&msgPtr, (uint16_t)(ctx->pump.extendedBolusAmount * 100.0));   /*TODO: change to real delivered*/
            msgPutU16InvertedOrder(&msgPtr, ctx->pump.extendedBolusMinutes);                    /*TODO: change to real time before stopped*/

            ctx->pump.extendedBolusMinutes = 0;
            ctx->pump.extendedBolusAmount = 0.0;
            ctx->pump.extendedBolusAbsoluteRate =  0.0;

            ctx->pump.extendedBolusSoFarInMinutes = 0;
            ctx->pump.extendedBolusDeliveredSoFar = 0;
          }
          else if((ctx->pump.tempBasalStartTime >= eventsSince) && tbStartToBeSent)
          {
            /*mark this as sent*/
            ctx->pump.tempBasalStartLastReported = ctx->pump.tempBasalStartTime;

  Serial.println(F("report tempbasal start"));

            msgPutU8(&msgPtr, DANARS_HISTORY_EVENT_RECORD_TEMPSTART);

            msgPutTimeDate(&msgPtr, ctx->pump.tempBasalStartTime);
            
            msgPutU16InvertedOrder(&msgPtr, ctx->pump.tempBasalPercent); /*TODO*/
            msgPutU16InvertedOrder(&msgPtr, ctx->pump.tempBasalDurationMinutes);

          }
          else if((ctx->pump.tempBasalStopTime >= eventsSince) && tbStopToBeSent)
          {
            /*mark this as sent*/
            ctx->pump.tempBasalStopLastReported = ctx->pump.tempBasalStopTime;

  Serial.println(F("report tempbasal stop"));

            msgPutU8(&msgPtr, DANARS_HISTORY_EVENT_RECORD_TEMPSTOP);

            msgPutTimeDate(&msgPtr, ctx->pump.tempBasalStopTime);

            msgPutU16InvertedOrder(&msgPtr, ctx->pump.tempBasalPercent); /*TODO*/
            msgPutU16InvertedOrder(&msgPtr, ctx->pump.tempBasalDurationMinutes);
          }
          else
          {
            msgPutU8(&msgPtr, 0xFF); /*last record, will be skipped*/
            moreToCome = false;
          }

          outLen = createOutMessage(rawBuf, TYPE_RESPONSE, OA_HISTORY_EVENTS, NULL, msgLen(tempBuf, msgPtr));

          /*send if a response was created*/
          sendToAAPS(ctx, rawBuf, outLen);
        }
      }
      /*mark ok*/
      ret = 0;
      break;

#if 0

    case OA_SET_EVENT_HISTORY:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OA_SET_EVENT_HISTORY"));
#endif
      break;

    case OETC_SET_HISTORY_SAVE:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OETC_SET_HISTORY_SAVE"));
#endif
      break;

    case OETC_KEEP_CONNECTION:
#if DEBUG_PRINT
      Serial.println(F("handleTypeResponse: OETC_KEEP_CONNECTION"));
#endif
      break;

#endif

  default:
#if ERROR_PRINT
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

  if (buf && (len > 0))
  {
    //    vTaskSuspendAll();
    Serial.flush();
    Serial.write(buf, (size_t)len);
    Serial.flush();

#if DEBUG_PRINT
    vTaskDelay(10 / portTICK_PERIOD_MS);
    Serial.println("");
    Serial.print("sendToAAPS: [");
    for (int i = 0; i < len; i++)
    {
      uint8_t b = buf[i];
      if (b < 16)
        Serial.print("0");
      Serial.print(b, HEX);
      if (i < len - 1)
        Serial.print(" ");
    }
    Serial.println("].");
#endif
    //    vTaskResumeAll();
  }
}

