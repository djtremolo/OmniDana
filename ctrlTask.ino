#include "common.h"
#include "ctrlTask.h"
#include "ioInterface.h"


#define MAX_PAYLOAD_LENGTH  16
#define HEADER_LEN          6
#define FOOTER_LEN          4
#define MAX_BUF_LEN         (HEADER_LEN+MAX_PAYLOAD_LENGTH+FOOTER_LEN)

#define MAX_STEPS_IN_SEQUENCE   20

typedef enum
{
  CMD_MEALINS_STOP = 0x0101,                      //	danaMsgBolusStop
  CMD_MEALINS_START_DATA = 0x0102,                //	danaMsgBolusStart
  CMD_MEALINS_START_DATA_SPEED = 0x0104,          //	danaMsgBolusStartWithSpeed
  CMD_PUMP_THIS_REMAINDER_MEAL_INS = 0x0202,      //	danaMsgBolusProgress
  CMD_PUMP_CALCULATION_SETTING = 0x0204,          //	danaMsgStatusProfile
  CMD_PUMP_EXERCISE_MODE = 0x0205,                //	danaMsgStatusTempBasal
  CMD_PUMP_EXPANS_INS_I = 0x0207,                 //	danaMsgStatusBolusExtended
  CMD_PUMP_INITVIEW_I = 0x020a,                   //	danaMsgStatusBasic
  CMD_PUMP_STATUS = 0x020b,                       //	danaMsgStatus
  CMD_PUMPINIT_TIME_INFO = 0x0301,                //	danaMsgInitConnStatusTime
  CMD_PUMPINIT_BOLUS_INFO = 0x0302,               //	danaMsgInitConnStatusBolus
  CMD_PUMPINIT_INIT_INFO = 0x0303,                //	danaMsgInitConnStatusBasic
  CMD_PUMPINIT_OPTION = 0x0304,                   //	danaMsgInitConnStatusOption
  CMD_PUMPSET_EXERCISE_S = 0x0401,                //	danaMsgSetTempBasalStart
  CMD_PUMPSET_HIS_S = 0x0402,                     //	danaMsgSetCarbsEntry
  CMD_PUMPSET_EXERCISE_STOP = 0x0403,             //	danaMsgSetTempBasalStop
  CMD_PUMPSET_EXPANS_INS_STOP = 0x0406,           //	danaMsgSetExtendedBolusStop
  CMD_PUMPSET_EXPANS_INS_S = 0x0407,              //	danaMsgSetExtendedBolusStart
  CMD_PUMPOWAY_SYSTEM_STATUS = 0x0601,            //	danaMsgError
  CMD_CONNECT = 0x3001,                           //	danaMsgPCCommStart
  CMD_DISCONNECT = 0x3002,                        //	danaMsgPCCommStop
  CMD_HISTORY_MEAL_INS = 0x3101,                  //	danaMsgHistoryBolus
  CMD_HISTORY_DAY_INS = 0x3102,                   //	danaMsgHistoryDailyInsulin
  CMD_HISTORY_GLUCOSE = 0x3104,                   //	danaMsgHistoryGlucose
  CMD_HISTORY_ALARM = 0x3105,                     //	danaMsgHistoryAlarm
  CMD_HISTORY_ERROR = 0x3106,                     //	danaMsgHistoryError
  CMD_HISTORY_CARBOHY = 0x3107,                   //	danaMsgHistoryCarbo
  CMD_HISTORY_REFILL = 0x3108,                    //	danaMsgHistoryRefill
  CMD_HISTORY_SUSPEND = 0x3109,                   //	danaMsgHistorySuspend
  CMD_HISTORY_BASAL_HOUR = 0x310a,                //	danaMsgHistoryBasalHour
  CMD_HISTORY_STOP = 0x31f1,                      //	danaMsgHistoryDone
  CMD_SETTING_V_BASAL_INS_I = 0x3202,             //	danaMsgSettingBasal
  CMD_SETTING_V_MEAL_SETTING_I = 0x3203,          //	danaMsgSettingMeal
  CMD_SETTING_V_CCC_I = 0x3204,                   //	danaMsgSettingProfileRatios
  CMD_SETTING_V_MAX_VALUE_I = 0x3205,             //	danaMsgSettingMaxValues
  CMD_SETTING_V_BASAL_PROFILE_ALL = 0x3206,       //	danaMsgSettingBasalProfileAll
  CMD_SETTING_V_SHIPPING_I = 0x3207,              //	danaMsgSettingShippingInfo
  CMD_SETTING_V_GLUCOSEandEASY = 0x3209,          //	danaMsgSettingGlucose
  CMD_SETTING_V_TIME_I = 0x320a,                  //	danaMsgSettingPumpTime
  CMD_SETTING_V_USER_OPTIONS = 0x320b,            //	danaMsgSettingUserOptions
  CMD_SETTING_V_PROFILE_NUMBER = 0x320c,          //	danaMsgSettingActiveProfile
  CMD_SETTING_V_CIR_CF_VALUE = 0x320d,            //	danaMsgSettingProfileRatiosAll
  CMD_SETTING_BASAL_INS_S = 0x3302,               //	danaMsgSetSingleBasalProfile
  CMD_SETTING_BASAL_PROFILE_S = 0x3306,           //	danaMsgSetBasalProfile
  CMD_SETTING_PROFILE_NUMBER_S = 0x330c,          //	danaMsgSetActivateBasalProfile
  CMD_HISTORY_ALL_DONE = 0x41f1,                  //	danaMsgHistoryAllDone
  CMD_HISTORY_ALL = 0x41f2,                       //	danaMsgHistoryAll
  CMD_HISTORY_NEW_DONE = 0x42f1,                  //	danaMsgHistoryNewDone
  CMD_HISTORY_NEW = 0x42f2,                       //	danaMsgHistoryNew
  CMD_PUMP_CHECK_VALUE = 0xF0F1                   //	danaMsgCheckValue
} danaCommand_t;

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



void ctrlTaskInitialize(MessageBufferHandle_t msgBuf, uint8_t priority);

void ctrlTaskInitialize(MessageBufferHandle_t msgBuf, uint8_t priority)
{


  xTaskCreate(
    ctrlTask
    ,  (const portCHAR *)"ctrlTask"   // A name just for humans
    ,  128  // Stack size
    ,  (void*)msgBuf
    ,  priority  // priority
    ,  NULL );
}



void ctrlTask( void *pvParameters )
{
  MessageBufferHandle_t msgBuf = (MessageBufferHandle_t)pvParameters;

  IoInterfaceSetupPins();

  while(1)
  {
    uint8_t msg[MAX_BUF_LEN];
    size_t receivedBytes;

    receivedBytes = xMessageBufferReceive(msgBuf, (void*)msg, sizeof(msg), portMAX_DELAY);

    if(receivedBytes > 0 )
    { 
      Serial.print("ctrlTask received ");
      Serial.print(receivedBytes, DEC);
      Serial.println(" bytes.");
    }

    /*handle received message*/
    if(handleLowLevelMessage(&msg[0], receivedBytes))
    {
      /*failed*/
      Serial.println("handleLowLevelMessage failed!");
    }

    /*and continue waiting for next...*/
  }
}



static int handleLowLevelMessage(uint8_t *buf, size_t bufLen)
{
  int ret = -1;
  
  /*sanity check*/
  if((buf[0] == 0x7E) && (buf[1] == 0x7E))
  {
    if(buf[3] == 0xF1)
    {
      uint8_t payloadLen = buf[2] - 3;
      uint8_t *payload = &buf[6];
      uint16_t cmd = (buf[4]<<8 | buf[5]);

      uint8_t footerOffset = payloadLen + 3;   /*crc hi*/

      if(bufLen >= (payloadLen +6))
      {
        if((buf[footerOffset+2] == 0x2E) && (buf[footerOffset+3] == 0x2E))
        {
          ret = handlePayload(cmd, payload, payloadLen);
        }
      }
    }
  }

  return ret;
}

static int handlePayload(uint16_t cmd, uint8_t *buf, uint8_t len)
{
  int ret = -1;
  
  switch(cmd)
  {
    case CMD_PUMP_STATUS:
      danaMsgStatus(buf, len);
      ret = 0;
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

static void danaMsgStatus(uint8_t *buf, uint8_t len)
{
  Serial.println("MsgStatus");
}


